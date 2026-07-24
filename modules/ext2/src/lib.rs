#![no_std]
#![allow(unsafe_op_in_unsafe_fn)]

//! Metadata and data writes are flushed synchronously, but this minimal ext2 writer has no
//! journal. A power loss during an operation can therefore leave the filesystem inconsistent.

use core::sync::atomic::{AtomicBool, Ordering};

use mochi_cext_abi::{
    EEXIST, EFBIG, EINVAL, EISDIR, ENOENT, ENOSPC, ENOSYS, ENOTDIR, EOVERFLOW, EROFS, MCX_CEXT_ABI,
    MCX_FS_MOUNT_READ_ONLY, MCX_LOG_INFO, McxBuffer, McxDiskOps, McxFsOps, McxKernelApi, McxPath,
};

const EXT2_MAGIC: u16 = 0xef53;
const GPT_HEADER_SIGNATURE: &[u8; 8] = b"EFI PART";
const ROOT_INO: u32 = 2;
const S_IFDIR: u16 = 0x4000;
const S_IFREG: u16 = 0x8000;
const MAX_BLOCK_SIZE: usize = 4096;
const MAX_INODE_SIZE: usize = 512;
const SECTOR_SIZE: usize = 512;
const MAX_READ_TRANSFER_BYTES: usize = 64 * 1024;
const EXT2_FT_REG_FILE: u8 = 1;
const EXT2_FT_DIR: u8 = 2;
const EXT2_INDEX_FL: u32 = 0x0000_1000;
const EXT2_FEATURE_COMPAT_HAS_JOURNAL: u32 = 0x0004;
const EXT2_FEATURE_INCOMPAT_FILETYPE: u32 = 0x0002;
const EXT2_FEATURE_RO_COMPAT_SPARSE_SUPER: u32 = 0x0001;
const EXT2_FEATURE_RO_COMPAT_LARGE_FILE: u32 = 0x0002;
const MAX_WRITABLE_BLOCKS: usize = 12 + MAX_BLOCK_SIZE / 4;

#[repr(C)]
#[derive(Clone, Copy)]
struct Superblock {
    blocks_count: u32,
    first_data_block: u32,
    block_size: u32,
    inode_size: u16,
    first_inode: u32,
    blocks_per_group: u32,
    inodes_per_group: u32,
    inodes_count: u32,
    feature_compat: u32,
    feature_incompat: u32,
    feature_ro_compat: u32,
}

#[repr(C)]
#[derive(Clone, Copy)]
struct GroupDesc {
    block_bitmap: u32,
    inode_bitmap: u32,
    inode_table: u32,
}

#[repr(C)]
#[derive(Clone, Copy)]
struct Inode {
    mode: u16,
    size: u32,
    flags: u32,
    blocks: [u32; 15],
}

struct State {
    disk_ops: *const McxDiskOps,
    mounted: bool,
    writable: bool,
    disk_id: u32,
    partition_lba_base: u64,
    sb: Superblock,
}

static READY: AtomicBool = AtomicBool::new(false);
static mut STATE: State = State {
    disk_ops: core::ptr::null(),
    mounted: false,
    disk_id: 0,
    partition_lba_base: 0,
    sb: Superblock {
        blocks_count: 0,
        first_data_block: 0,
        block_size: 0,
        inode_size: 0,
        first_inode: 0,
        blocks_per_group: 0,
        inodes_per_group: 0,
        inodes_count: 0,
        feature_compat: 0,
        feature_incompat: 0,
        feature_ro_compat: 0,
    },
    writable: false,
};
static mut KERNEL_API: *const McxKernelApi = core::ptr::null();

fn superblock() -> Superblock {
    unsafe { STATE.sb }
}

fn state_is_writable() -> bool {
    unsafe { STATE.writable }
}

fn log_bytes(bytes: &[u8]) {
    unsafe {
        let api = KERNEL_API;
        if !api.is_null() {
            ((*api).log)(MCX_LOG_INFO, bytes.as_ptr(), bytes.len());
        }
    }
}

fn log_str(text: &str) {
    log_bytes(text.as_bytes());
}

fn debug_trace_path(prefix: &str, path: &[u8]) {
    if path != b"/drivers/usb" && path != b"/drivers" {
        return;
    }
    log_str(prefix);
}

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

unsafe fn disk_write(lba: u64, buf: *const u8, len: usize) -> i32 {
    if STATE.disk_ops.is_null() {
        return ENOSYS;
    }
    ((*STATE.disk_ops).write_sector)(STATE.disk_id, lba, buf, len)
}

fn disk_flush() -> i32 {
    unsafe {
        if STATE.disk_ops.is_null() {
            return ENOSYS;
        }
        ((*STATE.disk_ops).flush)(STATE.disk_id)
    }
}

fn read_exact_raw(offset: u64, out: &mut [u8]) -> i32 {
    let mut done = 0usize;
    while done < out.len() {
        let absolute = offset + done as u64;
        let lba = absolute / SECTOR_SIZE as u64;
        let sector_off = (absolute % SECTOR_SIZE as u64) as usize;
        let remaining = out.len() - done;
        if sector_off == 0 && remaining >= SECTOR_SIZE {
            let aligned_len = core::cmp::min(
                remaining - (remaining % SECTOR_SIZE),
                MAX_READ_TRANSFER_BYTES,
            );
            let rc = unsafe { disk_read(lba, out.as_mut_ptr().add(done), aligned_len) };
            if rc != 0 {
                return rc;
            }
            done += aligned_len;
            continue;
        }
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

fn write_exact_raw(offset: u64, data: &[u8]) -> i32 {
    let mut done = 0usize;
    while done < data.len() {
        let absolute = offset + done as u64;
        let lba = absolute / SECTOR_SIZE as u64;
        let sector_off = (absolute % SECTOR_SIZE as u64) as usize;
        let mut sector = [0u8; SECTOR_SIZE];
        let rc = unsafe { disk_read(lba, sector.as_mut_ptr(), SECTOR_SIZE) };
        if rc != 0 {
            return rc;
        }
        let take = core::cmp::min(SECTOR_SIZE - sector_off, data.len() - done);
        sector[sector_off..sector_off + take].copy_from_slice(&data[done..done + take]);
        let rc = unsafe { disk_write(lba, sector.as_ptr(), SECTOR_SIZE) };
        if rc != 0 {
            return rc;
        }
        done += take;
    }
    0
}

fn read_exact(offset: u64, out: &mut [u8]) -> i32 {
    let base = unsafe { STATE.partition_lba_base } * SECTOR_SIZE as u64;
    read_exact_raw(base + offset, out)
}

fn write_exact(offset: u64, data: &[u8]) -> i32 {
    let base = unsafe { STATE.partition_lba_base } * SECTOR_SIZE as u64;
    write_exact_raw(base + offset, data)
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

fn write_u16(offset: u64, value: u16) -> i32 {
    write_exact(offset, &value.to_le_bytes())
}

fn write_u32(offset: u64, value: u32) -> i32 {
    write_exact(offset, &value.to_le_bytes())
}

fn decrement_u32_at(offset: u64) -> Result<(), i32> {
    let value = read_u32(offset)?;
    if value == 0 {
        return Err(ENOSPC);
    }
    let rc = write_u32(offset, value - 1);
    if rc != 0 {
        return Err(rc);
    }
    Ok(())
}

fn increment_u32_at(offset: u64) -> Result<(), i32> {
    let value = read_u32(offset)?;
    let next = value.checked_add(1).ok_or(EOVERFLOW)?;
    let rc = write_u32(offset, next);
    if rc != 0 {
        return Err(rc);
    }
    Ok(())
}

fn set_u16(buf: &mut [u8], offset: usize, value: u16) {
    buf[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
}

fn get_u16(buf: &[u8], offset: usize) -> u16 {
    u16::from_le_bytes([buf[offset], buf[offset + 1]])
}

fn set_u32(buf: &mut [u8], offset: usize, value: u32) {
    buf[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}

fn get_u32(buf: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes([
        buf[offset],
        buf[offset + 1],
        buf[offset + 2],
        buf[offset + 3],
    ])
}

fn get_u64(buf: &[u8], offset: usize) -> u64 {
    u64::from_le_bytes([
        buf[offset],
        buf[offset + 1],
        buf[offset + 2],
        buf[offset + 3],
        buf[offset + 4],
        buf[offset + 5],
        buf[offset + 6],
        buf[offset + 7],
    ])
}

fn round_up_4(value: usize) -> usize {
    (value + 3) & !3
}

fn group_desc_offset(sb: Superblock, group: u32) -> u64 {
    let gdt_offset = if sb.block_size == 1024 {
        (sb.block_size as u64) * 2
    } else {
        sb.block_size as u64
    };
    gdt_offset + group as u64 * 32
}

fn inode_offset(sb: Superblock, gd: GroupDesc, index_in_group: u32) -> u64 {
    gd.inode_table as u64 * sb.block_size as u64 + index_in_group as u64 * sb.inode_size as u64
}

fn load_superblock() -> Result<Superblock, i32> {
    let mut raw = [0u8; 1024];
    let rc = read_exact(1024, &mut raw);
    if rc != 0 {
        return Err(rc);
    }
    let magic = u16::from_le_bytes([raw[56], raw[57]]);
    if magic != EXT2_MAGIC {
        return Err(EINVAL);
    }
    let log_block_size = get_u32(&raw, 24);
    let block_size = 1024u32.checked_shl(log_block_size).ok_or(EINVAL)?;
    if block_size as usize > MAX_BLOCK_SIZE {
        return Err(EINVAL);
    }
    let inode_size = u16::from_le_bytes([raw[88], raw[89]]);
    let inode_size = if inode_size == 0 { 128 } else { inode_size };
    let sb = Superblock {
        blocks_count: get_u32(&raw, 4),
        first_data_block: get_u32(&raw, 20),
        block_size,
        inode_size,
        first_inode: get_u32(&raw, 84),
        blocks_per_group: get_u32(&raw, 32),
        inodes_per_group: get_u32(&raw, 40),
        inodes_count: get_u32(&raw, 0),
        feature_compat: get_u32(&raw, 92),
        feature_incompat: get_u32(&raw, 96),
        feature_ro_compat: get_u32(&raw, 100),
    };
    if sb.blocks_count <= sb.first_data_block
        || sb.block_size < 1024
        || !sb.block_size.is_power_of_two()
        || sb.inode_size < 128
        || sb.inode_size as usize > MAX_INODE_SIZE
        || sb.inode_size as u32 > sb.block_size
        || sb.blocks_per_group == 0
        || sb.blocks_per_group > sb.block_size * 8
        || sb.inodes_per_group == 0
        || sb.inodes_per_group > sb.block_size * 8
        || sb.inodes_count == 0
    {
        return Err(EINVAL);
    }
    Ok(sb)
}

fn validate_mount_features(sb: Superblock, writable: bool) -> Result<(), i32> {
    if (sb.feature_incompat & !EXT2_FEATURE_INCOMPAT_FILETYPE) != 0 {
        return Err(EINVAL);
    }
    if writable {
        let supported_ro = EXT2_FEATURE_RO_COMPAT_SPARSE_SUPER | EXT2_FEATURE_RO_COMPAT_LARGE_FILE;
        if (sb.feature_ro_compat & !supported_ro) != 0
            || (sb.feature_compat & EXT2_FEATURE_COMPAT_HAS_JOURNAL) != 0
        {
            return Err(EROFS);
        }
    }
    Ok(())
}

fn validate_write_access(writable: bool) -> Result<(), i32> {
    if writable { Ok(()) } else { Err(EROFS) }
}

fn try_mount_at_lba(base_lba: u64) -> Result<Superblock, i32> {
    unsafe {
        STATE.partition_lba_base = base_lba;
    }
    load_superblock()
}

fn find_ext2_partition_lba() -> Result<(u64, Superblock), i32> {
    match try_mount_at_lba(0) {
        Ok(sb) => return Ok((0, sb)),
        Err(rc) if rc != EINVAL => return Err(rc),
        Err(_) => {}
    }

    let mut header = [0u8; SECTOR_SIZE];
    let rc = read_exact_raw(SECTOR_SIZE as u64, &mut header);
    if rc != 0 {
        return Err(rc);
    }
    if &header[0..8] != GPT_HEADER_SIGNATURE {
        return Err(EINVAL);
    }

    let entries_lba = get_u64(&header, 72);
    let entry_count = get_u32(&header, 80);
    let entry_size = get_u32(&header, 84);
    if entries_lba == 0 || entry_count == 0 || entry_size < 128 || entry_size > 512 {
        return Err(EINVAL);
    }

    let max_entries = core::cmp::min(entry_count, 128);
    let mut entry = [0u8; 512];
    let mut index = 0u32;
    while index < max_entries {
        let offset = entries_lba
            .saturating_mul(SECTOR_SIZE as u64)
            .saturating_add(index as u64 * entry_size as u64);
        let rc = read_exact_raw(offset, &mut entry[..entry_size as usize]);
        if rc != 0 {
            return Err(rc);
        }
        let mut empty_type = true;
        let mut i = 0usize;
        while i < 16 {
            if entry[i] != 0 {
                empty_type = false;
                break;
            }
            i += 1;
        }
        if !empty_type {
            let first_lba = get_u64(&entry, 32);
            if first_lba != 0 {
                match try_mount_at_lba(first_lba) {
                    Ok(sb) => return Ok((first_lba, sb)),
                    Err(rc) if rc != EINVAL => return Err(rc),
                    Err(_) => {}
                }
            }
        }
        index += 1;
    }

    Err(EINVAL)
}

fn load_group_desc(sb: Superblock, group: u32) -> Result<GroupDesc, i32> {
    let offset = group_desc_offset(sb, group);
    let mut raw = [0u8; 12];
    let rc = read_exact(offset, &mut raw);
    if rc != 0 {
        return Err(rc);
    }
    Ok(GroupDesc {
        block_bitmap: get_u32(&raw, 0),
        inode_bitmap: get_u32(&raw, 4),
        inode_table: get_u32(&raw, 8),
    })
}

fn read_inode_raw(ino: u32, out: &mut [u8]) -> Result<(), i32> {
    let sb = superblock();
    if ino < 1 || sb.inode_size as usize > out.len() {
        return Err(EINVAL);
    }
    let index = ino - 1;
    let group = index / sb.inodes_per_group;
    let index_in_group = index % sb.inodes_per_group;
    let gd = load_group_desc(sb, group)?;
    let rc = read_exact(
        inode_offset(sb, gd, index_in_group),
        &mut out[..sb.inode_size as usize],
    );
    if rc != 0 {
        return Err(rc);
    }
    Ok(())
}

fn write_inode_raw(ino: u32, data: &[u8]) -> i32 {
    let sb = superblock();
    if ino < 1 || sb.inode_size as usize > data.len() {
        return EINVAL;
    }
    let index = ino - 1;
    let group = index / sb.inodes_per_group;
    let index_in_group = index % sb.inodes_per_group;
    let gd = match load_group_desc(sb, group) {
        Ok(v) => v,
        Err(rc) => return rc,
    };
    write_exact(
        inode_offset(sb, gd, index_in_group),
        &data[..sb.inode_size as usize],
    )
}

fn load_inode(ino: u32) -> Result<Inode, i32> {
    let mut raw = [0u8; MAX_INODE_SIZE];
    read_inode_raw(ino, &mut raw)?;
    Ok(load_inode_from_raw(&raw))
}

fn read_indirect_entry(block: u32, index: usize) -> Result<u32, i32> {
    let sb = superblock();
    read_u32(block as u64 * sb.block_size as u64 + (index * 4) as u64)
}

fn write_indirect_entry(block: u32, index: usize, value: u32) -> i32 {
    let sb = superblock();
    write_u32(
        block as u64 * sb.block_size as u64 + (index * 4) as u64,
        value,
    )
}

fn data_block_number(inode: Inode, block_index: usize) -> Result<u32, i32> {
    let sb = superblock();
    if block_index < 12 {
        return Ok(inode.blocks[block_index]);
    }
    let entries_per_block = (sb.block_size / 4) as usize;
    let single_index = block_index - 12;
    if single_index < entries_per_block {
        let indirect = inode.blocks[12];
        if indirect == 0 {
            return Ok(0);
        }
        return read_indirect_entry(indirect, single_index);
    }

    let double_index = single_index - entries_per_block;
    let double_span = entries_per_block
        .checked_mul(entries_per_block)
        .ok_or(ENOSYS)?;
    if double_index < double_span {
        let double_indirect = inode.blocks[13];
        if double_indirect == 0 {
            return Ok(0);
        }
        let l1_index = double_index / entries_per_block;
        let l2_index = double_index % entries_per_block;
        let indirect = read_indirect_entry(double_indirect, l1_index)?;
        if indirect == 0 {
            return Ok(0);
        }
        return read_indirect_entry(indirect, l2_index);
    }

    Err(ENOSYS)
}

struct SingleIndirectCache {
    block: u32,
    data: [u8; MAX_BLOCK_SIZE],
}

impl SingleIndirectCache {
    const fn new() -> Self {
        Self {
            block: 0,
            data: [0; MAX_BLOCK_SIZE],
        }
    }

    fn data_block_number(&mut self, inode: Inode, block_index: usize) -> Result<u32, i32> {
        if block_index < 12 {
            return Ok(inode.blocks[block_index]);
        }
        let sb = superblock();
        let entries_per_block = (sb.block_size / 4) as usize;
        let single_index = block_index - 12;
        if single_index >= entries_per_block {
            return data_block_number(inode, block_index);
        }
        let indirect = inode.blocks[12];
        if indirect == 0 {
            return Ok(0);
        }
        if self.block != indirect {
            let rc = read_block(indirect, &mut self.data);
            if rc != 0 {
                return Err(rc);
            }
            self.block = indirect;
        }
        Ok(get_u32(&self.data, single_index * 4))
    }
}

fn is_dir(mode: u16) -> bool {
    (mode & 0xf000) == S_IFDIR
}

fn is_file(mode: u16) -> bool {
    (mode & 0xf000) == S_IFREG
}

fn read_block(block: u32, data: &mut [u8]) -> i32 {
    let sb = superblock();
    read_exact(
        block as u64 * sb.block_size as u64,
        &mut data[..sb.block_size as usize],
    )
}

fn write_block(block: u32, data: &[u8]) -> i32 {
    let sb = superblock();
    write_exact(
        block as u64 * sb.block_size as u64,
        &data[..sb.block_size as usize],
    )
}

fn lookup_name_in_dir(dir_ino: u32, name: &[u8]) -> Result<u32, i32> {
    let sb = superblock();
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
        let rc = read_block(block, &mut block_buf);
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
    debug_trace_path("ext2: resolve_path\n", path);
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

fn split_parent(path: &[u8]) -> Result<(&[u8], &[u8]), i32> {
    if path.is_empty() || path[0] != b'/' || path == b"/" {
        return Err(EINVAL);
    }
    let mut end = path.len();
    while end > 1 && path[end - 1] == b'/' {
        end -= 1;
    }
    let trimmed = &path[..end];
    let mut slash = trimmed.len() - 1;
    while slash > 0 && trimmed[slash] != b'/' {
        slash -= 1;
    }
    let name = &trimmed[slash + 1..];
    if name.is_empty() {
        return Err(EINVAL);
    }
    let parent = if slash == 0 {
        b"/".as_slice()
    } else {
        &trimmed[..slash]
    };
    Ok((parent, name))
}

fn contains_slash(bytes: &[u8]) -> bool {
    let mut index = 0usize;
    while index < bytes.len() {
        if bytes[index] == b'/' {
            return true;
        }
        index += 1;
    }
    false
}

fn read_file_bytes(inode: Inode, offset: u64, out: &mut [u8]) -> Result<usize, i32> {
    let sb = superblock();
    if !is_file(inode.mode) {
        return Err(EISDIR);
    }
    if offset >= inode.size as u64 {
        return Ok(0);
    }
    let mut copied = 0usize;
    let mut file_off = offset as usize;
    let limit = core::cmp::min(out.len(), inode.size as usize - file_off);
    let mut block_buf = [0u8; MAX_BLOCK_SIZE];
    let mut indirect_cache = SingleIndirectCache::new();
    while copied < limit {
        let block_index = file_off / sb.block_size as usize;
        let within_block = file_off % sb.block_size as usize;
        let data_block = indirect_cache.data_block_number(inode, block_index)?;
        let take = core::cmp::min(sb.block_size as usize - within_block, limit - copied);
        if data_block == 0 {
            out[copied..copied + take].fill(0);
        } else if within_block == 0 && take == sb.block_size as usize {
            let max_run_blocks = core::cmp::min(
                (limit - copied) / sb.block_size as usize,
                MAX_READ_TRANSFER_BYTES / sb.block_size as usize,
            );
            let mut run_blocks = 1usize;
            while run_blocks < max_run_blocks {
                let next_block =
                    indirect_cache.data_block_number(inode, block_index + run_blocks)?;
                if next_block != data_block.saturating_add(run_blocks as u32) {
                    break;
                }
                run_blocks += 1;
            }
            let run_bytes = run_blocks * sb.block_size as usize;
            let rc = read_exact(
                data_block as u64 * sb.block_size as u64,
                &mut out[copied..copied + run_bytes],
            );
            if rc != 0 {
                return Err(rc);
            }
            copied += run_bytes;
            file_off += run_bytes;
            continue;
        } else {
            let rc = read_block(data_block, &mut block_buf);
            if rc != 0 {
                return Err(rc);
            }
            out[copied..copied + take]
                .copy_from_slice(&block_buf[within_block..within_block + take]);
        }
        copied += take;
        file_off += take;
    }
    Ok(copied)
}

fn decrement_u16_at(offset: u64) -> Result<(), i32> {
    let value = read_u16(offset)?;
    if value == 0 {
        return Err(ENOSPC);
    }
    let rc = write_u16(offset, value - 1);
    if rc != 0 {
        return Err(rc);
    }
    Ok(())
}

fn increment_u16_at(offset: u64) -> Result<(), i32> {
    let value = read_u16(offset)?;
    let next = value.checked_add(1).ok_or(EOVERFLOW)?;
    let rc = write_u16(offset, next);
    if rc != 0 {
        return Err(rc);
    }
    Ok(())
}

fn alloc_bitmap_bit(bitmap_block: u32, start_bit: u32, max_bits: u32) -> Result<u32, i32> {
    let mut bitmap = [0u8; MAX_BLOCK_SIZE];
    let rc = read_block(bitmap_block, &mut bitmap);
    if rc != 0 {
        return Err(rc);
    }
    let mut bit = start_bit;
    while bit < max_bits {
        let byte_idx = (bit / 8) as usize;
        let mask = 1u8 << (bit % 8);
        if (bitmap[byte_idx] & mask) == 0 {
            bitmap[byte_idx] |= mask;
            let rc = write_block(bitmap_block, &bitmap);
            if rc != 0 {
                return Err(rc);
            }
            return Ok(bit);
        }
        bit += 1;
    }
    Err(ENOSPC)
}

fn clear_bitmap_bit(bitmap_block: u32, bit: u32) -> Result<(), i32> {
    let mut bitmap = [0u8; MAX_BLOCK_SIZE];
    let rc = read_block(bitmap_block, &mut bitmap);
    if rc != 0 {
        return Err(rc);
    }
    let byte_idx = (bit / 8) as usize;
    let mask = 1u8 << (bit % 8);
    if (bitmap[byte_idx] & mask) == 0 {
        return Err(EINVAL);
    }
    bitmap[byte_idx] &= !mask;
    let rc = write_block(bitmap_block, &bitmap);
    if rc != 0 {
        return Err(rc);
    }
    Ok(())
}

fn allocate_inode_number() -> Result<u32, i32> {
    let sb = superblock();
    let groups = sb.inodes_count.div_ceil(sb.inodes_per_group);
    let mut group = 0u32;
    while group < groups {
        let gd = load_group_desc(sb, group)?;
        let start_bit = if group == 0 {
            sb.first_inode.saturating_sub(1)
        } else {
            0
        };
        let max_bits = core::cmp::min(
            sb.inodes_per_group,
            sb.inodes_count.saturating_sub(group * sb.inodes_per_group),
        );
        match alloc_bitmap_bit(gd.inode_bitmap, start_bit, max_bits) {
            Ok(bit) => {
                if let Err(rc) = decrement_u32_at(1024 + 16) {
                    let _ = clear_bitmap_bit(gd.inode_bitmap, bit);
                    return Err(rc);
                }
                if let Err(rc) = decrement_u16_at(group_desc_offset(sb, group) + 14) {
                    let _ = increment_u32_at(1024 + 16);
                    let _ = clear_bitmap_bit(gd.inode_bitmap, bit);
                    return Err(rc);
                }
                return Ok(group * sb.inodes_per_group + bit + 1);
            }
            Err(ENOSPC) => group += 1,
            Err(rc) => return Err(rc),
        }
    }
    Err(ENOSPC)
}

fn free_inode_number(ino: u32) -> Result<(), i32> {
    let sb = superblock();
    let index = ino - 1;
    let group = index / sb.inodes_per_group;
    let bit = index % sb.inodes_per_group;
    let gd = load_group_desc(sb, group)?;
    clear_bitmap_bit(gd.inode_bitmap, bit)?;
    increment_u32_at(1024 + 16)?;
    increment_u16_at(group_desc_offset(sb, group) + 14)?;
    Ok(())
}

fn increment_used_dirs(ino: u32) -> Result<(), i32> {
    let sb = superblock();
    let group = (ino - 1) / sb.inodes_per_group;
    increment_u16_at(group_desc_offset(sb, group) + 16)
}

fn decrement_used_dirs(ino: u32) -> Result<(), i32> {
    let sb = superblock();
    let group = (ino - 1) / sb.inodes_per_group;
    decrement_u16_at(group_desc_offset(sb, group) + 16)
}

fn allocate_block_number() -> Result<u32, i32> {
    let sb = superblock();
    let data_blocks = sb.blocks_count.saturating_sub(sb.first_data_block);
    let groups = data_blocks.div_ceil(sb.blocks_per_group);
    let mut group = 0u32;
    while group < groups {
        let gd = load_group_desc(sb, group)?;
        let group_base = sb
            .first_data_block
            .checked_add(group.saturating_mul(sb.blocks_per_group))
            .ok_or(EOVERFLOW)?;
        let max_bits = core::cmp::min(
            sb.blocks_per_group,
            sb.blocks_count.saturating_sub(group_base),
        );
        match alloc_bitmap_bit(gd.block_bitmap, 0, max_bits) {
            Ok(bit) => {
                if let Err(rc) = decrement_u32_at(1024 + 12) {
                    let _ = clear_bitmap_bit(gd.block_bitmap, bit);
                    return Err(rc);
                }
                if let Err(rc) = decrement_u16_at(group_desc_offset(sb, group) + 12) {
                    let _ = increment_u32_at(1024 + 12);
                    let _ = clear_bitmap_bit(gd.block_bitmap, bit);
                    return Err(rc);
                }
                return Ok(group_base + bit);
            }
            Err(ENOSPC) => group += 1,
            Err(rc) => return Err(rc),
        }
    }
    Err(ENOSPC)
}

fn free_block_number(block: u32) -> Result<(), i32> {
    let sb = superblock();
    if block < sb.first_data_block || block >= sb.blocks_count {
        return Err(EINVAL);
    }
    let relative = block - sb.first_data_block;
    let group = relative / sb.blocks_per_group;
    let bit = relative % sb.blocks_per_group;
    let gd = load_group_desc(sb, group)?;
    clear_bitmap_bit(gd.block_bitmap, bit)?;
    increment_u32_at(1024 + 12)?;
    increment_u16_at(group_desc_offset(sb, group) + 12)?;
    Ok(())
}

fn require_writable() -> Result<(), i32> {
    validate_write_access(state_is_writable())
}

fn now_seconds() -> u32 {
    unsafe {
        if KERNEL_API.is_null() {
            0
        } else {
            ((*KERNEL_API).now_seconds)()
        }
    }
}

fn update_change_times(inode_raw: &mut [u8]) {
    let now = now_seconds();
    set_u32(inode_raw, 12, now);
    set_u32(inode_raw, 16, now);
}

fn inode_blocks_512(inode_raw: &[u8]) -> u32 {
    get_u32(inode_raw, 28)
}

fn set_inode_blocks_512(inode_raw: &mut [u8], value: u32) {
    set_u32(inode_raw, 28, value);
}

fn set_inode_block_ptr(inode_raw: &mut [u8], block_index: usize, value: u32) {
    set_u32(inode_raw, 40 + block_index * 4, value);
}

fn set_data_block_number(inode_raw: &mut [u8], block_index: usize, value: u32) -> Result<(), i32> {
    let sb = superblock();
    if block_index < 12 {
        set_inode_block_ptr(inode_raw, block_index, value);
        return Ok(());
    }
    let single_index = block_index - 12;
    let entries_per_block = (sb.block_size / 4) as usize;
    if single_index >= entries_per_block {
        return Err(EFBIG);
    }
    let mut indirect = get_u32(inode_raw, 40 + 12 * 4);
    let created_indirect = indirect == 0;
    if indirect == 0 {
        indirect = allocate_block_number()?;
        let zero = [0u8; MAX_BLOCK_SIZE];
        let rc = write_block(indirect, &zero);
        if rc != 0 {
            let _ = free_block_number(indirect);
            return Err(rc);
        }
        set_inode_block_ptr(inode_raw, 12, indirect);
        set_inode_blocks_512(
            inode_raw,
            inode_blocks_512(inode_raw) + (sb.block_size / 512),
        );
    }
    let rc = write_indirect_entry(indirect, single_index, value);
    if rc != 0 {
        if created_indirect {
            set_inode_block_ptr(inode_raw, 12, 0);
            set_inode_blocks_512(
                inode_raw,
                inode_blocks_512(inode_raw).saturating_sub(sb.block_size / 512),
            );
            let _ = free_block_number(indirect);
        }
        return Err(rc);
    }
    Ok(())
}

fn ensure_data_block(inode_raw: &mut [u8], block_index: usize) -> Result<u32, i32> {
    if block_index >= max_writable_blocks() {
        return Err(EFBIG);
    }
    let inode = load_inode_from_raw(inode_raw);
    let existing = data_block_number(inode, block_index)?;
    if existing != 0 {
        return Ok(existing);
    }
    let block = allocate_block_number()?;
    let zero = [0u8; MAX_BLOCK_SIZE];
    let rc = write_block(block, &zero);
    if rc != 0 {
        let _ = free_block_number(block);
        return Err(rc);
    }
    if let Err(rc) = set_data_block_number(inode_raw, block_index, block) {
        let _ = free_block_number(block);
        return Err(rc);
    }
    let sb = superblock();
    set_inode_blocks_512(
        inode_raw,
        inode_blocks_512(inode_raw) + (sb.block_size / 512),
    );
    Ok(block)
}

fn max_writable_blocks() -> usize {
    let sb = superblock();
    12 + (sb.block_size / 4) as usize
}

fn max_writable_size() -> u64 {
    max_writable_blocks() as u64 * superblock().block_size as u64
}

fn load_inode_from_raw(raw: &[u8]) -> Inode {
    let mut blocks = [0u32; 15];
    let mut i = 0usize;
    while i < 15 {
        blocks[i] = get_u32(raw, 40 + i * 4);
        i += 1;
    }
    Inode {
        mode: u16::from_le_bytes([raw[0], raw[1]]),
        size: get_u32(raw, 4),
        flags: get_u32(raw, 32),
        blocks,
    }
}

fn add_dir_entry(
    parent_ino: u32,
    parent_inode: Inode,
    name: &[u8],
    child_ino: u32,
    file_type: u8,
    child_is_dir: bool,
) -> Result<(), i32> {
    let sb = superblock();
    if (parent_inode.flags & EXT2_INDEX_FL) != 0 {
        return Err(ENOSYS);
    }
    let needed_len = 8 + round_up_4(name.len());
    let mut block_buf = [0u8; MAX_BLOCK_SIZE];
    let blocks = (parent_inode.size as usize).div_ceil(sb.block_size as usize);
    let mut block_index = 0usize;
    while block_index < blocks {
        let block = data_block_number(parent_inode, block_index)?;
        if block == 0 {
            block_index += 1;
            continue;
        }
        let rc = read_block(block, &mut block_buf);
        if rc != 0 {
            return Err(rc);
        }
        let mut off = 0usize;
        while off + 8 <= sb.block_size as usize {
            let rec_len = u16::from_le_bytes([block_buf[off + 4], block_buf[off + 5]]) as usize;
            let name_len = block_buf[off + 6] as usize;
            if rec_len == 0 || off + rec_len > sb.block_size as usize {
                break;
            }
            let ideal = 8 + round_up_4(name_len);
            if rec_len >= ideal + needed_len {
                let remaining = rec_len - ideal;
                block_buf[off + 4..off + 6].copy_from_slice(&(ideal as u16).to_le_bytes());
                let new_off = off + ideal;
                block_buf[new_off..new_off + 4].copy_from_slice(&child_ino.to_le_bytes());
                block_buf[new_off + 4..new_off + 6]
                    .copy_from_slice(&(remaining as u16).to_le_bytes());
                block_buf[new_off + 6] = name.len() as u8;
                block_buf[new_off + 7] = file_type;
                block_buf[new_off + 8..new_off + 8 + name.len()].copy_from_slice(name);
                let rc = write_block(block, &block_buf);
                if rc != 0 {
                    return Err(rc);
                }
                let mut parent_raw = [0u8; MAX_INODE_SIZE];
                read_inode_raw(parent_ino, &mut parent_raw)?;
                if child_is_dir {
                    let links = get_u16(&parent_raw, 26).checked_add(1).ok_or(EOVERFLOW)?;
                    set_u16(&mut parent_raw, 26, links);
                }
                update_change_times(&mut parent_raw);
                let rc = write_inode_raw(parent_ino, &parent_raw);
                if rc != 0 {
                    return Err(rc);
                }
                return Ok(());
            }
            off += rec_len;
        }
        block_index += 1;
    }

    if blocks >= 12 {
        return Err(ENOSYS);
    }

    let new_block = allocate_block_number()?;
    let mut block_buf = [0u8; MAX_BLOCK_SIZE];
    block_buf[0..4].copy_from_slice(&child_ino.to_le_bytes());
    block_buf[4..6].copy_from_slice(&(sb.block_size as u16).to_le_bytes());
    block_buf[6] = name.len() as u8;
    block_buf[7] = file_type;
    block_buf[8..8 + name.len()].copy_from_slice(name);
    let rc = write_block(new_block, &block_buf);
    if rc != 0 {
        return Err(rc);
    }

    let mut parent_raw = [0u8; MAX_INODE_SIZE];
    read_inode_raw(parent_ino, &mut parent_raw)?;
    set_inode_block_ptr(&mut parent_raw, blocks, new_block);
    set_u32(&mut parent_raw, 4, parent_inode.size + sb.block_size);
    let blocks_512 = inode_blocks_512(&parent_raw) + (sb.block_size / 512);
    set_inode_blocks_512(&mut parent_raw, blocks_512);
    if child_is_dir {
        let links = get_u16(&parent_raw, 26).checked_add(1).ok_or(EOVERFLOW)?;
        set_u16(&mut parent_raw, 26, links);
    }
    update_change_times(&mut parent_raw);
    let rc = write_inode_raw(parent_ino, &parent_raw);
    if rc != 0 {
        return Err(rc);
    }
    Ok(())
}

fn init_directory_block(block: u32, self_ino: u32, parent_ino: u32) -> i32 {
    let sb = superblock();
    let mut block_buf = [0u8; MAX_BLOCK_SIZE];
    let dot_len = 8 + round_up_4(1);
    block_buf[0..4].copy_from_slice(&self_ino.to_le_bytes());
    block_buf[4..6].copy_from_slice(&(dot_len as u16).to_le_bytes());
    block_buf[6] = 1;
    block_buf[7] = EXT2_FT_DIR;
    block_buf[8] = b'.';
    let dotdot_off = dot_len;
    block_buf[dotdot_off..dotdot_off + 4].copy_from_slice(&parent_ino.to_le_bytes());
    block_buf[dotdot_off + 4..dotdot_off + 6]
        .copy_from_slice(&((sb.block_size as usize - dotdot_off) as u16).to_le_bytes());
    block_buf[dotdot_off + 6] = 2;
    block_buf[dotdot_off + 7] = EXT2_FT_DIR;
    block_buf[dotdot_off + 8] = b'.';
    block_buf[dotdot_off + 9] = b'.';
    write_block(block, &block_buf)
}

extern "C" fn mount_impl(device_id: u32, flags: u32) -> i32 {
    if (flags & !MCX_FS_MOUNT_READ_ONLY) != 0 {
        return EINVAL;
    }
    let Some(_) = (unsafe { STATE.disk_ops.as_ref() }) else {
        return ENOSYS;
    };
    unsafe {
        STATE.disk_id = device_id;
    }
    match find_ext2_partition_lba() {
        Ok((base_lba, sb)) => {
            let writable = (flags & MCX_FS_MOUNT_READ_ONLY) == 0;
            if let Err(rc) = validate_mount_features(sb, writable) {
                log_bytes(b"ext2.cext: unsupported filesystem features");
                return rc;
            }
            unsafe {
                STATE.partition_lba_base = base_lba;
                STATE.sb = sb;
                STATE.mounted = true;
                STATE.writable = writable;
                READY.store(true, Ordering::Release);
                0
            }
        }
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

extern "C" fn create_impl(path: McxPath, mode: u32) -> i32 {
    if let Err(rc) = require_writable() {
        return rc;
    }
    let Some(path) = path_bytes(path) else {
        return EINVAL;
    };
    match resolve_path(path) {
        Ok(_) => return EEXIST,
        Err(ENOENT) => {}
        Err(rc) => return rc,
    }
    let (parent_path, name) = match split_parent(path) {
        Ok(v) => v,
        Err(rc) => {
            return rc;
        }
    };
    if name.len() > u8::MAX as usize || contains_slash(name) {
        return EINVAL;
    }
    let (parent_ino, parent_inode) = match resolve_path(parent_path) {
        Ok(v) => v,
        Err(rc) => {
            return rc;
        }
    };
    if !is_dir(parent_inode.mode) {
        return ENOTDIR;
    }

    let ino = match allocate_inode_number() {
        Ok(v) => v,
        Err(rc) => {
            return rc;
        }
    };
    let mut inode_raw = [0u8; MAX_INODE_SIZE];
    let requested_type = (mode as u16) & 0xf000;
    let is_directory = requested_type == S_IFDIR;
    let file_type = if is_directory {
        EXT2_FT_DIR
    } else {
        EXT2_FT_REG_FILE
    };
    set_u16(
        &mut inode_raw,
        0,
        if is_directory {
            S_IFDIR | ((mode as u16) & 0o777)
        } else {
            S_IFREG | ((mode as u16) & 0o777)
        },
    );
    set_u16(&mut inode_raw, 26, if is_directory { 2 } else { 1 });
    let now = now_seconds();
    set_u32(&mut inode_raw, 8, now);
    set_u32(&mut inode_raw, 12, now);
    set_u32(&mut inode_raw, 16, now);
    if is_directory {
        let block = match allocate_block_number() {
            Ok(v) => v,
            Err(rc) => {
                let _ = free_inode_number(ino);
                return rc;
            }
        };
        let rc = init_directory_block(block, ino, parent_ino);
        if rc != 0 {
            let _ = free_block_number(block);
            let _ = free_inode_number(ino);
            return rc;
        }
        set_u32(&mut inode_raw, 4, superblock().block_size);
        set_u32(&mut inode_raw, 28, superblock().block_size / 512);
        set_inode_block_ptr(&mut inode_raw, 0, block);
    } else {
        set_u32(&mut inode_raw, 4, 0);
        set_u32(&mut inode_raw, 28, 0);
    }
    let rc = write_inode_raw(ino, &inode_raw);
    if rc != 0 {
        if is_directory {
            let block = get_u32(&inode_raw, 40);
            if block != 0 {
                let _ = free_block_number(block);
            }
        }
        let _ = free_inode_number(ino);
        return rc;
    }
    if is_directory {
        if let Err(err) = increment_used_dirs(ino) {
            let block = get_u32(&inode_raw, 40);
            if block != 0 {
                let _ = free_block_number(block);
            }
            let _ = free_inode_number(ino);
            return err;
        }
    }
    if let Err(err) = add_dir_entry(parent_ino, parent_inode, name, ino, file_type, is_directory) {
        if is_directory {
            let _ = decrement_used_dirs(ino);
        }
        if is_directory {
            let block = get_u32(&inode_raw, 40);
            if block != 0 {
                let _ = free_block_number(block);
            }
        }
        let _ = free_inode_number(ino);
        return err;
    }
    if resolve_path(path).is_err() {
        return ENOENT;
    }
    let rc = disk_flush();
    if rc != 0 {
        return rc;
    }
    0
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
    path: McxPath,
    offset: u64,
    buf: McxBuffer,
    out_written: *mut usize,
) -> i32 {
    if let Err(rc) = require_writable() {
        return rc;
    }
    if buf.ptr.is_null() || out_written.is_null() {
        return EINVAL;
    }
    let Some(path) = path_bytes(path) else {
        return EINVAL;
    };
    let (ino, inode) = match resolve_path(path) {
        Ok(v) => v,
        Err(rc) => return rc,
    };
    if !is_file(inode.mode) {
        return EISDIR;
    }
    let sb = superblock();
    let src = unsafe { core::slice::from_raw_parts(buf.ptr as *const u8, buf.len) };
    unsafe {
        *out_written = 0;
    }
    let end = match offset.checked_add(src.len() as u64) {
        Some(value) => value,
        None => return EFBIG,
    };
    if end > max_writable_size() || end > u32::MAX as u64 {
        return EFBIG;
    }
    let mut inode_raw = [0u8; MAX_INODE_SIZE];
    if let Err(rc) = read_inode_raw(ino, &mut inode_raw) {
        return rc;
    }

    let mut written = 0usize;
    while written < src.len() {
        let file_off = offset as usize + written;
        let block_index = file_off / sb.block_size as usize;
        let block_off = file_off % sb.block_size as usize;
        let chunk = core::cmp::min(sb.block_size as usize - block_off, src.len() - written);
        let inode_before = load_inode_from_raw(&inode_raw);
        let block_existed = match data_block_number(inode_before, block_index) {
            Ok(value) => value != 0,
            Err(rc) => return finish_write(out_written, written, rc),
        };
        let block = match ensure_data_block(&mut inode_raw, block_index) {
            Ok(v) => v,
            Err(rc) => return finish_write(out_written, written, rc),
        };
        if !block_existed {
            let rc = write_inode_raw(ino, &inode_raw);
            if rc != 0 {
                rollback_new_data_block(
                    &mut inode_raw,
                    block_index,
                    block,
                    inode_before.blocks[12],
                );
                return finish_write(out_written, written, rc);
            }
        }
        let mut block_buf = [0u8; MAX_BLOCK_SIZE];
        if block_existed {
            let rc = read_block(block, &mut block_buf);
            if rc != 0 {
                return finish_write(out_written, written, rc);
            }
        }
        block_buf[block_off..block_off + chunk].copy_from_slice(&src[written..written + chunk]);
        let rc = write_block(block, &block_buf);
        if rc != 0 {
            if !block_existed {
                rollback_new_data_block(
                    &mut inode_raw,
                    block_index,
                    block,
                    inode_before.blocks[12],
                );
                let _ = write_inode_raw(ino, &inode_raw);
            }
            return finish_write(out_written, written, rc);
        }
        let committed = written + chunk;
        let new_size = core::cmp::max(get_u32(&inode_raw, 4) as u64, offset + committed as u64);
        set_u32(&mut inode_raw, 4, new_size as u32);
        update_change_times(&mut inode_raw);
        let rc = write_inode_raw(ino, &inode_raw);
        if rc != 0 {
            return finish_write(out_written, written, rc);
        }
        written = committed;
    }
    let rc = disk_flush();
    if rc != 0 {
        return rc;
    }
    unsafe {
        *out_written = written;
    }
    0
}

fn finish_write(out_written: *mut usize, written: usize, rc: i32) -> i32 {
    unsafe {
        *out_written = written;
    }
    if written == 0 {
        rc
    } else {
        let _ = disk_flush();
        0
    }
}

fn rollback_new_data_block(
    inode_raw: &mut [u8],
    block_index: usize,
    block: u32,
    old_indirect: u32,
) {
    let sb = superblock();
    if block_index < 12 {
        set_inode_block_ptr(inode_raw, block_index, 0);
    } else {
        let indirect = get_u32(inode_raw, 40 + 12 * 4);
        let _ = write_indirect_entry(indirect, block_index - 12, 0);
        if old_indirect == 0 {
            set_inode_block_ptr(inode_raw, 12, 0);
            let _ = free_block_number(indirect);
            set_inode_blocks_512(
                inode_raw,
                inode_blocks_512(inode_raw).saturating_sub(sb.block_size / 512),
            );
        }
    }
    let _ = free_block_number(block);
    set_inode_blocks_512(
        inode_raw,
        inode_blocks_512(inode_raw).saturating_sub(sb.block_size / 512),
    );
}

extern "C" fn truncate_impl(path: McxPath, len: u64) -> i32 {
    if let Err(rc) = require_writable() {
        return rc;
    }
    let Some(path) = path_bytes(path) else {
        return EINVAL;
    };
    let (ino, inode) = match resolve_path(path) {
        Ok(v) => v,
        Err(rc) => return rc,
    };
    if !is_file(inode.mode) {
        return EISDIR;
    }
    if len > max_writable_size() || len > u32::MAX as u64 {
        return EFBIG;
    }
    if len == inode.size as u64 {
        return 0;
    }
    let mut inode_raw = [0u8; MAX_INODE_SIZE];
    if let Err(rc) = read_inode_raw(ino, &mut inode_raw) {
        return rc;
    }
    let old_len = inode.size as u64;
    if old_len > max_writable_size() {
        return EFBIG;
    }
    if len > old_len {
        set_u32(&mut inode_raw, 4, len as u32);
        update_change_times(&mut inode_raw);
        let rc = write_inode_raw(ino, &inode_raw);
        return if rc == 0 { disk_flush() } else { rc };
    }

    let sb = superblock();
    let block_size = sb.block_size as u64;
    let new_blocks = len.div_ceil(block_size) as usize;
    let old_blocks = old_len.div_ceil(block_size) as usize;

    if len != 0 && (len % block_size) != 0 {
        let retained_index = (len / block_size) as usize;
        let retained = match data_block_number(inode, retained_index) {
            Ok(value) => value,
            Err(rc) => return rc,
        };
        if retained != 0 {
            let mut block_buf = [0u8; MAX_BLOCK_SIZE];
            let rc = read_block(retained, &mut block_buf);
            if rc != 0 {
                return rc;
            }
            block_buf[len as usize % sb.block_size as usize..sb.block_size as usize].fill(0);
            let rc = write_block(retained, &block_buf);
            if rc != 0 {
                return rc;
            }
        }
    }

    let mut detached = [0u32; MAX_WRITABLE_BLOCKS + 1];
    let mut detached_count = 0usize;
    let direct_end = core::cmp::min(old_blocks, 12);
    let mut index = core::cmp::min(new_blocks, 12);
    while index < direct_end {
        let block = get_u32(&inode_raw, 40 + index * 4);
        if block != 0 {
            detached[detached_count] = block;
            detached_count += 1;
            set_inode_block_ptr(&mut inode_raw, index, 0);
        }
        index += 1;
    }

    let indirect = get_u32(&inode_raw, 40 + 12 * 4);
    if indirect != 0 {
        let mut indirect_buf = [0u8; MAX_BLOCK_SIZE];
        let rc = read_block(indirect, &mut indirect_buf);
        if rc != 0 {
            return rc;
        }
        let entries = (sb.block_size / 4) as usize;
        let first = new_blocks.saturating_sub(12).min(entries);
        let end = old_blocks.saturating_sub(12).min(entries);
        let mut entry = first;
        while entry < end {
            let block = get_u32(&indirect_buf, entry * 4);
            if block != 0 {
                detached[detached_count] = block;
                detached_count += 1;
                set_u32(&mut indirect_buf, entry * 4, 0);
            }
            entry += 1;
        }
        if new_blocks <= 12 {
            set_inode_block_ptr(&mut inode_raw, 12, 0);
            detached[detached_count] = indirect;
            detached_count += 1;
        } else {
            let rc = write_block(indirect, &indirect_buf);
            if rc != 0 {
                return rc;
            }
        }
    }

    let sectors_per_block = sb.block_size / 512;
    let removed_sectors = (detached_count as u32).saturating_mul(sectors_per_block);
    let remaining_sectors = inode_blocks_512(&inode_raw).saturating_sub(removed_sectors);
    set_inode_blocks_512(&mut inode_raw, remaining_sectors);
    set_u32(&mut inode_raw, 4, len as u32);
    update_change_times(&mut inode_raw);
    let rc = write_inode_raw(ino, &inode_raw);
    if rc != 0 {
        return rc;
    }
    let mut free_index = 0usize;
    while free_index < detached_count {
        if let Err(rc) = free_block_number(detached[free_index]) {
            return rc;
        }
        free_index += 1;
    }
    disk_flush()
}

extern "C" fn sync_impl() -> i32 {
    disk_flush()
}

extern "C" fn stat_impl(path: McxPath, out_mode: *mut u16, out_size: *mut u64) -> i32 {
    if out_mode.is_null() || out_size.is_null() {
        return EINVAL;
    }
    let Some(path) = path_bytes(path) else {
        return EINVAL;
    };
    debug_trace_path("ext2: stat\n", path);
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
    debug_trace_path("ext2: readdir\n", path);
    let (_, inode) = match resolve_path(path) {
        Ok(v) => v,
        Err(rc) => return rc,
    };
    if !is_dir(inode.mode) {
        return ENOTDIR;
    }
    let sb = superblock();
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
        let rc = read_block(block, &mut block_buf);
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
    sync: sync_impl,
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
pub extern "C" fn slice_index_fail() -> ! {
    loop {
        core::hint::spin_loop();
    }
}

#[unsafe(export_name = "_RNvNtCsljbRsbwaaOA_4core9panicking18panic_bounds_check")]
pub extern "C" fn panic_bounds_check() -> ! {
    loop {
        core::hint::spin_loop();
    }
}

#[unsafe(export_name = "_RNvNtNtCsljbRsbwaaOA_4core9panicking11panic_const23panic_const_div_by_zero")]
pub extern "C" fn panic_const_div_by_zero() -> ! {
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

#[cfg(not(test))]
#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        core::hint::spin_loop();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn superblock_with_features(
        feature_compat: u32,
        feature_incompat: u32,
        feature_ro_compat: u32,
    ) -> Superblock {
        Superblock {
            blocks_count: 64,
            first_data_block: 1,
            block_size: 1024,
            inode_size: 128,
            first_inode: 11,
            blocks_per_group: 64,
            inodes_per_group: 32,
            inodes_count: 32,
            feature_compat,
            feature_incompat,
            feature_ro_compat,
        }
    }

    #[test]
    fn writable_mount_rejects_unsupported_features() {
        let journal = superblock_with_features(EXT2_FEATURE_COMPAT_HAS_JOURNAL, 0, 0);
        assert_eq!(validate_mount_features(journal, true), Err(EROFS));
        assert_eq!(validate_mount_features(journal, false), Ok(()));

        let unknown_ro = superblock_with_features(0, 0, 0x8000_0000);
        assert_eq!(validate_mount_features(unknown_ro, true), Err(EROFS));
        assert_eq!(validate_mount_features(unknown_ro, false), Ok(()));

        let unknown_incompat = superblock_with_features(0, 0x8000_0000, 0);
        assert_eq!(validate_mount_features(unknown_incompat, true), Err(EINVAL));
        assert_eq!(
            validate_mount_features(unknown_incompat, false),
            Err(EINVAL)
        );
    }

    #[test]
    fn read_only_mount_rejects_mutation() {
        assert_eq!(validate_write_access(false), Err(EROFS));
        assert_eq!(validate_write_access(true), Ok(()));
    }
}
