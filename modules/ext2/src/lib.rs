#![no_std]
#![allow(unsafe_op_in_unsafe_fn)]

use core::sync::atomic::{AtomicBool, Ordering};

use mochi_cext_abi::{
    EINVAL, EISDIR, ENOENT, ENOSYS, ENOTDIR, MCX_CEXT_ABI, MCX_LOG_INFO, McxBuffer,
    McxDiskOps, McxFsOps, McxKernelApi, McxPath,
};

const EXT2_MAGIC: u16 = 0xef53;
const ROOT_INO: u32 = 2;
const S_IFDIR: u16 = 0x4000;
const S_IFREG: u16 = 0x8000;
const MAX_BLOCK_SIZE: usize = 4096;
const SECTOR_SIZE: usize = 512;

#[repr(C)]
#[derive(Clone, Copy)]
struct Superblock {
    block_size: u32,
    inode_size: u16,
    inodes_per_group: u32,
}

#[repr(C)]
#[derive(Clone, Copy)]
struct GroupDesc {
    inode_table: u32,
}

#[repr(C)]
#[derive(Clone, Copy)]
struct Inode {
    mode: u16,
    size: u32,
    blocks: [u32; 15],
}

struct State {
    disk_ops: *const McxDiskOps,
    mounted: bool,
    disk_id: u32,
    sb: Superblock,
}

static READY: AtomicBool = AtomicBool::new(false);
static mut STATE: State = State {
    disk_ops: core::ptr::null(),
    mounted: false,
    disk_id: 0,
    sb: Superblock {
        block_size: 0,
        inode_size: 0,
        inodes_per_group: 0,
    },
};

fn log_bytes(bytes: &[u8]) {
    unsafe {
        let api = KERNEL_API;
        if !api.is_null() {
            ((*api).log)(MCX_LOG_INFO, bytes.as_ptr(), bytes.len());
        }
    }
}

static mut KERNEL_API: *const McxKernelApi = core::ptr::null();

fn path_bytes(path: McxPath) -> Option<&'static [u8]> {
    if path.ptr.is_null() {
        return None;
    }
    unsafe { Some(core::slice::from_raw_parts(path.ptr, path.len)) }
}

unsafe fn disk_read(lba: u64, buf: *mut u8, len: usize) -> i32 {
    if STATE.disk_ops.is_null() {
        return ENOSYS;
    }
    ((*STATE.disk_ops).read_sector)(STATE.disk_id, lba, buf, len)
}

fn read_exact(offset: u64, out: &mut [u8]) -> i32 {
    let mut done = 0usize;
    while done < out.len() {
        let absolute = offset + done as u64;
        let lba = absolute / SECTOR_SIZE as u64;
        let sector_off = (absolute % SECTOR_SIZE as u64) as usize;
        let mut sector = [0u8; SECTOR_SIZE];
        let rc = unsafe { disk_read(lba, sector.as_mut_ptr(), SECTOR_SIZE) };
        if rc != 0 {
            return rc;
        }
        let take = core::cmp::min(SECTOR_SIZE - sector_off, out.len() - done);
        out[done..done + take].copy_from_slice(&sector[sector_off..sector_off + take]);
        done += take;
    }
    0
}

fn read_u16(offset: u64) -> Result<u16, i32> {
    let mut buf = [0u8; 2];
    let rc = read_exact(offset, &mut buf);
    if rc != 0 {
        return Err(rc);
    }
    Ok(u16::from_le_bytes(buf))
}

fn read_u32(offset: u64) -> Result<u32, i32> {
    let mut buf = [0u8; 4];
    let rc = read_exact(offset, &mut buf);
    if rc != 0 {
        return Err(rc);
    }
    Ok(u32::from_le_bytes(buf))
}

fn load_superblock() -> Result<Superblock, i32> {
    let magic = read_u16(1024 + 56)?;
    if magic != EXT2_MAGIC {
        return Err(EINVAL);
    }
    let log_block_size = read_u32(1024 + 24)?;
    let block_size = 1024u32.checked_shl(log_block_size).ok_or(EINVAL)?;
    if block_size as usize > MAX_BLOCK_SIZE {
        return Err(EINVAL);
    }
    Ok(Superblock {
        block_size,
        inode_size: read_u16(1024 + 88)?,
        inodes_per_group: read_u32(1024 + 40)?,
    })
}

fn load_group_desc(sb: Superblock, group: u32) -> Result<GroupDesc, i32> {
    let gdt_offset = if sb.block_size == 1024 {
        (sb.block_size as u64) * 2
    } else {
        sb.block_size as u64
    };
    let offset = gdt_offset + group as u64 * 32;
    Ok(GroupDesc {
        inode_table: read_u32(offset + 8)?,
    })
}

fn load_inode(ino: u32) -> Result<Inode, i32> {
    let sb = unsafe { STATE.sb };
    if ino < 1 {
        return Err(EINVAL);
    }
    let index = ino - 1;
    let group = index / sb.inodes_per_group;
    let index_in_group = index % sb.inodes_per_group;
    let gd = load_group_desc(sb, group)?;
    let inode_offset = gd.inode_table as u64 * sb.block_size as u64
        + index_in_group as u64 * sb.inode_size as u64;

    let mut blocks = [0u32; 15];
    let mut i = 0usize;
    while i < 15 {
        blocks[i] = read_u32(inode_offset + 40 + (i * 4) as u64)?;
        i += 1;
    }

    Ok(Inode {
        mode: read_u16(inode_offset)?,
        size: read_u32(inode_offset + 4)?,
        blocks,
    })
}

fn read_indirect_entry(block: u32, index: usize) -> Result<u32, i32> {
    let sb = unsafe { STATE.sb };
    read_u32(block as u64 * sb.block_size as u64 + (index * 4) as u64)
}

fn data_block_number(inode: Inode, block_index: usize) -> Result<u32, i32> {
    let sb = unsafe { STATE.sb };
    if block_index < 12 {
        return Ok(inode.blocks[block_index]);
    }
    let entries_per_block = (sb.block_size / 4) as usize;
    let single_index = block_index - 12;
    if single_index >= entries_per_block {
        return Err(ENOSYS);
    }
    let indirect = inode.blocks[12];
    if indirect == 0 {
        return Ok(0);
    }
    read_indirect_entry(indirect, single_index)
}

fn is_dir(mode: u16) -> bool {
    (mode & 0xf000) == S_IFDIR
}

fn is_file(mode: u16) -> bool {
    (mode & 0xf000) == S_IFREG
}

fn lookup_name_in_dir(dir_ino: u32, name: &[u8]) -> Result<u32, i32> {
    let sb = unsafe { STATE.sb };
    let dir = load_inode(dir_ino)?;
    if !is_dir(dir.mode) {
        return Err(ENOTDIR);
    }
    let blocks = (dir.size as usize).div_ceil(sb.block_size as usize);
    let mut block_index = 0usize;
    let mut block_buf = [0u8; MAX_BLOCK_SIZE];
    while block_index < blocks {
        let block = data_block_number(dir, block_index)?;
        if block == 0 {
            block_index += 1;
            continue;
        }
        let rc = read_exact(
            block as u64 * sb.block_size as u64,
            &mut block_buf[..sb.block_size as usize],
        );
        if rc != 0 {
            return Err(rc);
        }
        let mut off = 0usize;
        while off + 8 <= sb.block_size as usize {
            let inode = u32::from_le_bytes([
                block_buf[off],
                block_buf[off + 1],
                block_buf[off + 2],
                block_buf[off + 3],
            ]);
            let rec_len = u16::from_le_bytes([block_buf[off + 4], block_buf[off + 5]]) as usize;
            let name_len = block_buf[off + 6] as usize;
            if rec_len == 0 || off + rec_len > sb.block_size as usize {
                break;
            }
            if inode != 0 && off + 8 + name_len <= sb.block_size as usize {
                let entry_name = &block_buf[off + 8..off + 8 + name_len];
                if entry_name == name {
                    return Ok(inode);
                }
            }
            off += rec_len;
        }
        block_index += 1;
    }
    Err(ENOENT)
}

fn resolve_path(path: &[u8]) -> Result<(u32, Inode), i32> {
    if path.is_empty() || path[0] != b'/' {
        return Err(EINVAL);
    }
    if path == b"/" {
        let inode = load_inode(ROOT_INO)?;
        return Ok((ROOT_INO, inode));
    }
    let mut current = ROOT_INO;
    let mut start = 1usize;
    while start < path.len() {
        while start < path.len() && path[start] == b'/' {
            start += 1;
        }
        if start >= path.len() {
            break;
        }
        let mut end = start;
        while end < path.len() && path[end] != b'/' {
            end += 1;
        }
        let comp = &path[start..end];
        if comp == b"." {
            start = end;
            continue;
        }
        if comp == b".." {
            return Err(EINVAL);
        }
        current = lookup_name_in_dir(current, comp)?;
        start = end;
    }
    Ok((current, load_inode(current)?))
}

fn read_file_bytes(inode: Inode, offset: u64, out: &mut [u8]) -> Result<usize, i32> {
    let sb = unsafe { STATE.sb };
    if !is_file(inode.mode) {
        return Err(EISDIR);
    }
    if offset >= inode.size as u64 {
        return Ok(0);
    }
    let mut copied = 0usize;
    let mut file_off = offset as usize;
    let limit = core::cmp::min(out.len(), inode.size as usize - file_off);
    let mut sector = [0u8; SECTOR_SIZE];
    while copied < limit {
        let block_index = file_off / sb.block_size as usize;
        let within_block = file_off % sb.block_size as usize;
        let data_block = data_block_number(inode, block_index)?;
        if data_block == 0 {
            return Ok(copied);
        }
        let block_abs = data_block as u64 * sb.block_size as u64 + within_block as u64;
        let sector_lba = block_abs / SECTOR_SIZE as u64;
        let sector_off = (block_abs % SECTOR_SIZE as u64) as usize;
        let rc = unsafe { disk_read(sector_lba, sector.as_mut_ptr(), SECTOR_SIZE) };
        if rc != 0 {
            return Err(rc);
        }
        let take = core::cmp::min(SECTOR_SIZE - sector_off, limit - copied);
        out[copied..copied + take].copy_from_slice(&sector[sector_off..sector_off + take]);
        copied += take;
        file_off += take;
    }
    Ok(copied)
}

extern "C" fn mount_impl(device_id: u32) -> i32 {
    let Some(_) = (unsafe { STATE.disk_ops.as_ref() }) else {
        return ENOSYS;
    };
    unsafe {
        STATE.disk_id = device_id;
    }
    match load_superblock() {
        Ok(sb) => unsafe {
            STATE.sb = sb;
            STATE.mounted = true;
            READY.store(true, Ordering::Release);
            0
        },
        Err(rc) => {
            if rc == EINVAL {
                log_bytes(b"ext2.cext: mount invalid superblock");
            } else {
                log_bytes(b"ext2.cext: mount read failed");
            }
            rc
        }
    }
}

extern "C" fn set_disk_ops_impl(ops: *const McxDiskOps) -> i32 {
    if ops.is_null() {
        return EINVAL;
    }
    unsafe {
        STATE.disk_ops = ops;
    }
    0
}

extern "C" fn create_impl(_path: McxPath, _mode: u32) -> i32 {
    ENOSYS
}

extern "C" fn remove_impl(_path: McxPath, _is_dir: u32) -> i32 {
    ENOSYS
}

extern "C" fn rename_impl(_src: McxPath, _dst: McxPath) -> i32 {
    ENOSYS
}

extern "C" fn read_impl(path: McxPath, offset: u64, buf: McxBuffer, out_read: *mut usize) -> i32 {
    if buf.ptr.is_null() || out_read.is_null() {
        return EINVAL;
    }
    let Some(path) = path_bytes(path) else {
        return EINVAL;
    };
    let inode = match resolve_path(path) {
        Ok((_, inode)) => inode,
        Err(rc) => return rc,
    };
    let dst = unsafe { core::slice::from_raw_parts_mut(buf.ptr, buf.len) };
    match read_file_bytes(inode, offset, dst) {
        Ok(read) => unsafe {
            *out_read = read;
            0
        },
        Err(rc) => rc,
    }
}

extern "C" fn write_impl(
    _path: McxPath,
    _offset: u64,
    _buf: McxBuffer,
    _out_written: *mut usize,
) -> i32 {
    ENOSYS
}

extern "C" fn truncate_impl(_path: McxPath, _len: u64) -> i32 {
    ENOSYS
}

extern "C" fn stat_impl(
    path: McxPath,
    out_mode: *mut u16,
    out_size: *mut u64,
) -> i32 {
    if out_mode.is_null() || out_size.is_null() {
        return EINVAL;
    }
    let Some(path) = path_bytes(path) else {
        return EINVAL;
    };
    let inode = match resolve_path(path) {
        Ok((_, inode)) => inode,
        Err(rc) => return rc,
    };
    unsafe {
        *out_mode = inode.mode;
        *out_size = inode.size as u64;
    }
    0
}

extern "C" fn readdir_impl(path: McxPath, buf: McxBuffer, out_len: *mut usize) -> i32 {
    if buf.ptr.is_null() || out_len.is_null() {
        return EINVAL;
    }
    let Some(path) = path_bytes(path) else {
        return EINVAL;
    };
    let (_, inode) = match resolve_path(path) {
        Ok(v) => v,
        Err(rc) => return rc,
    };
    if !is_dir(inode.mode) {
        return ENOTDIR;
    }
    let sb = unsafe { STATE.sb };
    let dst = unsafe { core::slice::from_raw_parts_mut(buf.ptr, buf.len) };
    let mut written = 0usize;
    let mut block_buf = [0u8; MAX_BLOCK_SIZE];
    let blocks = (inode.size as usize).div_ceil(sb.block_size as usize);
    let mut block_index = 0usize;
    while block_index < blocks {
        let block = match data_block_number(inode, block_index) {
            Ok(v) => v,
            Err(rc) => return rc,
        };
        if block == 0 {
            block_index += 1;
            continue;
        }
        let rc = read_exact(
            block as u64 * sb.block_size as u64,
            &mut block_buf[..sb.block_size as usize],
        );
        if rc != 0 {
            return rc;
        }
        let mut off = 0usize;
        while off + 8 <= sb.block_size as usize {
            let inode_num = u32::from_le_bytes([
                block_buf[off],
                block_buf[off + 1],
                block_buf[off + 2],
                block_buf[off + 3],
            ]);
            let rec_len = u16::from_le_bytes([block_buf[off + 4], block_buf[off + 5]]) as usize;
            let name_len = block_buf[off + 6] as usize;
            if rec_len == 0 || off + rec_len > sb.block_size as usize {
                break;
            }
            if inode_num != 0 && off + 8 + name_len <= sb.block_size as usize {
                let name = &block_buf[off + 8..off + 8 + name_len];
                if name != b"." && name != b".." {
                    if written + name_len + 1 > dst.len() {
                        unsafe {
                            *out_len = written;
                        }
                        return 0;
                    }
                    dst[written..written + name_len].copy_from_slice(name);
                    written += name_len;
                    dst[written] = 0;
                    written += 1;
                }
            }
            off += rec_len;
        }
        block_index += 1;
    }
    unsafe {
        *out_len = written;
    }
    0
}

static OPS: McxFsOps = McxFsOps {
    mount: mount_impl,
    set_disk_ops: set_disk_ops_impl,
    create: create_impl,
    remove: remove_impl,
    rename: rename_impl,
    read: read_impl,
    write: write_impl,
    truncate: truncate_impl,
    stat: stat_impl,
    readdir: readdir_impl,
};

#[unsafe(no_mangle)]
pub unsafe extern "C" fn memcpy(dst: *mut u8, src: *const u8, len: usize) -> *mut u8 {
    let mut i = 0usize;
    while i < len {
        *dst.add(i) = *src.add(i);
        i += 1;
    }
    dst
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn memset(dst: *mut u8, byte: i32, len: usize) -> *mut u8 {
    let value = byte as u8;
    let mut i = 0usize;
    while i < len {
        *dst.add(i) = value;
        i += 1;
    }
    dst
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn memcmp(lhs: *const u8, rhs: *const u8, len: usize) -> i32 {
    let mut i = 0usize;
    while i < len {
        let a = *lhs.add(i);
        let b = *rhs.add(i);
        if a != b {
            return a as i32 - b as i32;
        }
        i += 1;
    }
    0
}

#[unsafe(export_name = "_RNvNtNtCsljbRsbwaaOA_4core5slice5index16slice_index_fail")]
pub extern "C" fn rust_slice_index_fail() -> ! {
    loop {
        core::hint::spin_loop();
    }
}

#[unsafe(export_name = "_RNvNtCsljbRsbwaaOA_4core9panicking18panic_bounds_check")]
pub extern "C" fn rust_panic_bounds_check() -> ! {
    loop {
        core::hint::spin_loop();
    }
}

#[unsafe(export_name = "_RNvNtNtCsljbRsbwaaOA_4core9panicking11panic_const23panic_const_div_by_zero")]
pub extern "C" fn rust_panic_div_zero() -> ! {
    loop {
        core::hint::spin_loop();
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn mochi_module_init(api: *const McxKernelApi) -> *const McxFsOps {
    if api.is_null() {
        return core::ptr::null();
    }
    unsafe {
        if (*api).abi != MCX_CEXT_ABI {
            return core::ptr::null();
        }
        KERNEL_API = api;
    }
    &OPS
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        core::hint::spin_loop();
    }
}
