#![no_std]
#![allow(unsafe_op_in_unsafe_fn)]

//! Metadata and data writes are flushed synchronously, but this minimal ext2 writer has no
//! journal. A power loss during an operation can therefore leave the filesystem inconsistent.

use core::sync::atomic::{AtomicBool, Ordering};

use mochi_cext_abi::{
    EEXIST, EFBIG, EINVAL, EISDIR, ENOENT, ENOSPC, ENOSYS, ENOTDIR, EOVERFLOW, EROFS, MCX_CEXT_ABI,
    MCX_FS_MOUNT_READ_ONLY, MCX_LOG_BOOT, MCX_LOG_INFO, McxBuffer, McxDiskOps, McxFsOps,
    McxKernelApi, McxPath,
};

const EXT2_MAGIC: u16 = 0xef53;
const GPT_HEADER_SIGNATURE: &[u8; 8] = b"EFI PART";
// GPT stores the first three UUID fields little-endian.
const SYSTEM_PARTITION_TYPE: [u8; 16] = [
    0x68, 0x63, 0x6f, 0x6d, 0x4f, 0x69, 0x00, 0x53,
    0x80, 0x00, 0x6d, 0x50, 0x61, 0x72, 0x74, 0x01,
];
const DATA_PARTITION_TYPE: [u8; 16] = [
    0x68, 0x63, 0x6f, 0x6d, 0x4f, 0x69, 0x00, 0x53,
    0x80, 0x00, 0x6d, 0x50, 0x61, 0x72, 0x74, 0x02,
];
const ROOT_INO: u32 = 2;
const S_IFDIR: u16 = 0x4000;
const S_IFREG: u16 = 0x8000;
const MAX_BLOCK_SIZE: usize = 4096;
const MAX_INODE_SIZE: usize = 512;
const SECTOR_SIZE: usize = 512;
const MAX_READ_TRANSFER_BYTES: usize = 256 * 1024;
const EXT2_FT_REG_FILE: u8 = 1;
const EXT2_FT_DIR: u8 = 2;
const EXT2_INDEX_FL: u32 = 0x0000_1000;
const EXT2_FEATURE_COMPAT_HAS_JOURNAL: u32 = 0x0004;
const EXT2_FEATURE_INCOMPAT_FILETYPE: u32 = 0x0002;
const EXT2_FEATURE_RO_COMPAT_SPARSE_SUPER: u32 = 0x0001;
const EXT2_FEATURE_RO_COMPAT_LARGE_FILE: u32 = 0x0002;
const MAX_WRITABLE_BLOCKS: usize = 12 + MAX_BLOCK_SIZE / 4;
const ENOTEMPTY: i32 = -39;

#[repr(C)]
#[derive(Clone, Copy)]
struct Superblock {
    blocks_count: u32,
    first_data_block: u32,
    block_size: u32,
    last_write_time: u32,
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
    uid: u32,
    gid: u32,
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
    partition_lba_count: u64,
    sb: Superblock,
    system_volume: Option<Volume>,
    data_volume: Option<Volume>,
}

#[derive(Clone, Copy)]
struct Volume {
    base: u64,
    count: u64,
    sb: Superblock,
    writable: bool,
}

static READY: AtomicBool = AtomicBool::new(false);
static OPERATION_LOCK: AtomicBool = AtomicBool::new(false);
static mut STATE: State = State {
    disk_ops: core::ptr::null(),
    mounted: false,
    disk_id: 0,
    partition_lba_base: 0,
    partition_lba_count: 0,
    sb: Superblock {
        blocks_count: 0,
        first_data_block: 0,
        block_size: 0,
        last_write_time: 0,
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
    system_volume: None,
    data_volume: None,
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

fn log_boot_bytes(bytes: &[u8]) {
    unsafe {
        let api = KERNEL_API;
        if !api.is_null() {
            ((*api).log)(MCX_LOG_BOOT, bytes.as_ptr(), bytes.len());
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
        let remaining = data.len() - done;
        if sector_off == 0 && remaining >= SECTOR_SIZE {
            let aligned_len = core::cmp::min(
                remaining - (remaining % SECTOR_SIZE),
                MAX_READ_TRANSFER_BYTES,
            );
            let rc = unsafe { disk_write(lba, data.as_ptr().add(done), aligned_len) };
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

fn partition_offset(base_lba: u64, lba_count: u64, offset: u64, len: usize) -> Result<u64, i32> {
    let base = base_lba.checked_mul(SECTOR_SIZE as u64).ok_or(EOVERFLOW)?;
    let end = offset.checked_add(len as u64).ok_or(EOVERFLOW)?;
    if lba_count != 0 {
        let capacity = lba_count.checked_mul(SECTOR_SIZE as u64).ok_or(EOVERFLOW)?;
        if end > capacity {
            return Err(EINVAL);
        }
    }
    base.checked_add(end).ok_or(EOVERFLOW)?;
    base.checked_add(offset).ok_or(EOVERFLOW)
}

fn read_exact(offset: u64, out: &mut [u8]) -> i32 {
    let (base, count) = unsafe { (STATE.partition_lba_base, STATE.partition_lba_count) };
    let absolute = match partition_offset(base, count, offset, out.len()) {
        Ok(value) => value,
        Err(rc) => return rc,
    };
    read_exact_raw(absolute, out)
}

fn write_exact(offset: u64, data: &[u8]) -> i32 {
    let (base, count) = unsafe { (STATE.partition_lba_base, STATE.partition_lba_count) };
    let absolute = match partition_offset(base, count, offset, data.len()) {
        Ok(value) => value,
        Err(rc) => return rc,
    };
    write_exact_raw(absolute, data)
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

fn decrement_u32_by(offset: u64, amount: u32) -> Result<(), i32> {
    let value = read_u32(offset)?;
    let Some(next) = value.checked_sub(amount) else {
        return Err(ENOSPC);
    };
    let rc = write_u32(offset, next);
    if rc != 0 {
        return Err(rc);
    }
    Ok(())
}

fn increment_u32_by(offset: u64, amount: u32) -> Result<(), i32> {
    let value = read_u32(offset)?;
    let Some(next) = value.checked_add(amount) else {
        return Err(EOVERFLOW);
    };
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
        last_write_time: get_u32(&raw, 48),
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

fn try_mount_at_lba(base_lba: u64, lba_count: u64) -> Result<Superblock, i32> {
    unsafe {
        STATE.partition_lba_base = base_lba;
        STATE.partition_lba_count = lba_count;
    }
    let sb = load_superblock()?;
    if lba_count != 0 {
        let filesystem_bytes = (sb.blocks_count as u64)
            .checked_mul(sb.block_size as u64)
            .ok_or(EINVAL)?;
        let partition_bytes = lba_count.checked_mul(SECTOR_SIZE as u64).ok_or(EINVAL)?;
        if filesystem_bytes > partition_bytes {
            return Err(EINVAL);
        }
    }
    Ok(sb)
}

fn is_system_slot_entry(entry: &[u8], slot: u32) -> bool {
    let name = match slot {
        1 => b"mochiOS System A".as_slice(),
        2 => b"mochiOS System B".as_slice(),
        _ => return false,
    };
    if entry.len() < 128 || entry[..16] != SYSTEM_PARTITION_TYPE {
        return false;
    }
    for (index, byte) in name.iter().enumerate() {
        if entry[56 + index * 2] != *byte || entry[57 + index * 2] != 0 {
            return false;
        }
    }
    entry[56 + name.len() * 2] == 0 && entry[57 + name.len() * 2] == 0
}

fn is_data_entry(entry: &[u8]) -> bool {
    if entry.len() < 128 || entry[..16] != DATA_PARTITION_TYPE {
        return false;
    }
    let name = b"mochiOS Data";
    for (index, byte) in name.iter().enumerate() {
        if entry[56 + index * 2] != *byte || entry[57 + index * 2] != 0 {
            return false;
        }
    }
    entry[56 + name.len() * 2] == 0 && entry[57 + name.len() * 2] == 0
}

fn find_ext2_partition_lba() -> Result<(Volume, Option<Volume>), i32> {
    let slot = unsafe {
        KERNEL_API.as_ref().map(|api| (api.boot_system_slot)())
    }.ok_or(ENOSYS)?;
    if slot > 2 {
        return Err(EINVAL);
    }
    if slot == 0 {
        match try_mount_at_lba(0, 0) {
            Ok(sb) => return Ok((Volume { base: 0, count: 0, sb, writable: true }, None)),
            Err(rc) if rc != EINVAL => return Err(rc),
            Err(_) => {}
        }
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
    let first_usable_lba = get_u64(&header, 40);
    let last_usable_lba = get_u64(&header, 48);
    if first_usable_lba == 0 || last_usable_lba < first_usable_lba {
        return Err(EINVAL);
    }
    let mut selected_partition = None;
    let mut data_partition = None;
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
            let last_lba = get_u64(&entry, 40);
            if first_lba < first_usable_lba || last_lba > last_usable_lba || last_lba < first_lba {
                return Err(EINVAL);
            }
            let lba_count = last_lba
                .checked_sub(first_lba)
                .and_then(|span| span.checked_add(1))
                .ok_or(EINVAL)?;
            if first_lba != 0 {
                if slot == 0 {
                    match try_mount_at_lba(first_lba, lba_count) {
                        Ok(sb) => return Ok((Volume { base: first_lba, count: lba_count, sb, writable: true }, None)),
                        Err(rc) if rc != EINVAL => return Err(rc),
                        Err(_) => {}
                    }
                } else if is_system_slot_entry(&entry[..entry_size as usize], slot) {
                    if selected_partition.replace((first_lba, lba_count)).is_some() {
                        return Err(EINVAL);
                    }
                } else if is_data_entry(&entry[..entry_size as usize]) {
                    if data_partition.replace((first_lba, lba_count)).is_some() {
                        return Err(EINVAL);
                    }
                }
            }
        }
        index += 1;
    }

    if let (Some((lba, count)), Some((data_lba, data_count))) =
        (selected_partition, data_partition)
    {
        let system_end = lba.checked_add(count).ok_or(EINVAL)?;
        let data_end = data_lba.checked_add(data_count).ok_or(EINVAL)?;
        if lba < data_end && data_lba < system_end {
            return Err(EINVAL);
        }
        let sb = try_mount_at_lba(lba, count)?;
        let data_sb = try_mount_at_lba(data_lba, data_count)?;
        return Ok((
            Volume { base: lba, count, sb, writable: true },
            Some(Volume { base: data_lba, count: data_count, sb: data_sb, writable: true }),
        ));
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

struct IndirectCache {
    block: u32,
    data: [u8; MAX_BLOCK_SIZE],
    double_root: u32,
    double_l1_index: usize,
    double_leaf: u32,
}

impl IndirectCache {
    const fn new() -> Self {
        Self {
            block: 0,
            data: [0; MAX_BLOCK_SIZE],
            double_root: 0,
            double_l1_index: usize::MAX,
            double_leaf: 0,
        }
    }

    fn data_block_number(&mut self, inode: Inode, block_index: usize) -> Result<u32, i32> {
        if block_index < 12 {
            return Ok(inode.blocks[block_index]);
        }
        let sb = superblock();
        let entries_per_block = (sb.block_size / 4) as usize;
        let single_index = block_index - 12;
        if single_index < entries_per_block {
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
            return Ok(get_u32(&self.data, single_index * 4));
        }

        let double_index = single_index - entries_per_block;
        let double_span = entries_per_block
            .checked_mul(entries_per_block)
            .ok_or(ENOSYS)?;
        if double_index >= double_span {
            return Err(ENOSYS);
        }
        let double_root = inode.blocks[13];
        if double_root == 0 {
            return Ok(0);
        }
        let l1_index = double_index / entries_per_block;
        let l2_index = double_index % entries_per_block;
        let double_leaf = if self.double_root == double_root && self.double_l1_index == l1_index {
            self.double_leaf
        } else {
            let leaf = read_indirect_entry(double_root, l1_index)?;
            self.double_root = double_root;
            self.double_l1_index = l1_index;
            self.double_leaf = leaf;
            leaf
        };
        if double_leaf == 0 {
            return Ok(0);
        }
        if self.block != double_leaf {
            let rc = read_block(double_leaf, &mut self.data);
            if rc != 0 {
                return Err(rc);
            }
            self.block = double_leaf;
        }
        Ok(get_u32(&self.data, l2_index * 4))
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
    let mut indirect_cache = IndirectCache::new();
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

fn decrement_u16_by(offset: u64, amount: u16) -> Result<(), i32> {
    let value = read_u16(offset)?;
    let Some(next) = value.checked_sub(amount) else {
        return Err(ENOSPC);
    };
    let rc = write_u16(offset, next);
    if rc != 0 {
        return Err(rc);
    }
    Ok(())
}

fn increment_u16_by(offset: u64, amount: u16) -> Result<(), i32> {
    let value = read_u16(offset)?;
    let Some(next) = value.checked_add(amount) else {
        return Err(EOVERFLOW);
    };
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

fn allocate_contiguous_blocks(requested: usize) -> Result<(u32, usize), i32> {
    if requested == 0 || requested > u16::MAX as usize {
        return Err(EINVAL);
    }
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
        let mut bitmap = [0u8; MAX_BLOCK_SIZE];
        let rc = read_block(gd.block_bitmap, &mut bitmap);
        if rc != 0 {
            return Err(rc);
        }
        let mut bit = 0u32;
        while bit < max_bits {
            while bit < max_bits && (bitmap[(bit / 8) as usize] & (1u8 << (bit % 8))) != 0 {
                bit += 1;
            }
            if bit >= max_bits {
                break;
            }
            let start = bit;
            let mut count = 0usize;
            while bit < max_bits
                && count < requested
                && (bitmap[(bit / 8) as usize] & (1u8 << (bit % 8))) == 0
            {
                bit += 1;
                count += 1;
            }
            if count == 0 {
                continue;
            }
            let mut allocated = 0usize;
            while allocated < count {
                let current = start + allocated as u32;
                bitmap[(current / 8) as usize] |= 1u8 << (current % 8);
                allocated += 1;
            }
            let rc = write_block(gd.block_bitmap, &bitmap);
            if rc != 0 {
                return Err(rc);
            }
            if let Err(rc) = decrement_u32_by(1024 + 12, count as u32) {
                let _ = free_contiguous_blocks_bitmap_only(gd.block_bitmap, start, count);
                return Err(rc);
            }
            if let Err(rc) = decrement_u16_by(group_desc_offset(sb, group) + 12, count as u16) {
                let _ = increment_u32_by(1024 + 12, count as u32);
                let _ = free_contiguous_blocks_bitmap_only(gd.block_bitmap, start, count);
                return Err(rc);
            }
            return Ok((group_base + start, count));
        }
        group += 1;
    }
    Err(ENOSPC)
}

fn free_contiguous_blocks_bitmap_only(
    bitmap_block: u32,
    start_bit: u32,
    count: usize,
) -> Result<(), i32> {
    let mut bitmap = [0u8; MAX_BLOCK_SIZE];
    let rc = read_block(bitmap_block, &mut bitmap);
    if rc != 0 {
        return Err(rc);
    }
    let mut index = 0usize;
    while index < count {
        let bit = start_bit + index as u32;
        bitmap[(bit / 8) as usize] &= !(1u8 << (bit % 8));
        index += 1;
    }
    let rc = write_block(bitmap_block, &bitmap);
    if rc == 0 { Ok(()) } else { Err(rc) }
}

fn free_contiguous_blocks(start_block: u32, count: usize) -> Result<(), i32> {
    let sb = superblock();
    let relative = start_block.checked_sub(sb.first_data_block).ok_or(EINVAL)?;
    let group = relative / sb.blocks_per_group;
    let start_bit = relative % sb.blocks_per_group;
    if start_bit as usize + count > sb.blocks_per_group as usize {
        return Err(EINVAL);
    }
    let gd = load_group_desc(sb, group)?;
    free_contiguous_blocks_bitmap_only(gd.block_bitmap, start_bit, count)?;
    increment_u32_by(1024 + 12, count as u32)?;
    increment_u16_by(group_desc_offset(sb, group) + 12, count as u16)
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
    let now = unsafe {
        if KERNEL_API.is_null() {
            0
        } else {
            ((*KERNEL_API).now_seconds)()
        }
    };
    // A guest without a synchronized clock may report 1970 even for a filesystem
    // created later. Earlier deletion times look like corrupt orphan links to fsck.
    now.max(superblock().last_write_time)
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
    if single_index < entries_per_block {
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
        return Ok(());
    }

    let double_index = single_index - entries_per_block;
    let double_span = entries_per_block
        .checked_mul(entries_per_block)
        .ok_or(EFBIG)?;
    if double_index >= double_span {
        return Err(EFBIG);
    }
    let l1_index = double_index / entries_per_block;
    let l2_index = double_index % entries_per_block;
    let mut double_indirect = get_u32(inode_raw, 40 + 13 * 4);
    let created_double = double_indirect == 0;
    if double_indirect == 0 {
        double_indirect = allocate_block_number()?;
        let zero = [0u8; MAX_BLOCK_SIZE];
        let rc = write_block(double_indirect, &zero);
        if rc != 0 {
            let _ = free_block_number(double_indirect);
            return Err(rc);
        }
        set_inode_block_ptr(inode_raw, 13, double_indirect);
        set_inode_blocks_512(
            inode_raw,
            inode_blocks_512(inode_raw) + (sb.block_size / 512),
        );
    }
    let mut indirect = match read_indirect_entry(double_indirect, l1_index) {
        Ok(value) => value,
        Err(rc) => {
            if created_double {
                set_inode_block_ptr(inode_raw, 13, 0);
                set_inode_blocks_512(
                    inode_raw,
                    inode_blocks_512(inode_raw).saturating_sub(sb.block_size / 512),
                );
                let _ = free_block_number(double_indirect);
            }
            return Err(rc);
        }
    };
    let created_indirect = indirect == 0;
    if indirect == 0 {
        indirect = match allocate_block_number() {
            Ok(block) => block,
            Err(rc) => {
                if created_double {
                    set_inode_block_ptr(inode_raw, 13, 0);
                    set_inode_blocks_512(
                        inode_raw,
                        inode_blocks_512(inode_raw).saturating_sub(sb.block_size / 512),
                    );
                    let _ = free_block_number(double_indirect);
                }
                return Err(rc);
            }
        };
        let zero = [0u8; MAX_BLOCK_SIZE];
        let rc = write_block(indirect, &zero);
        if rc != 0 {
            let _ = free_block_number(indirect);
            if created_double {
                set_inode_block_ptr(inode_raw, 13, 0);
                set_inode_blocks_512(
                    inode_raw,
                    inode_blocks_512(inode_raw).saturating_sub(sb.block_size / 512),
                );
                let _ = free_block_number(double_indirect);
            }
            return Err(rc);
        }
        let rc = write_indirect_entry(double_indirect, l1_index, indirect);
        if rc != 0 {
            let _ = free_block_number(indirect);
            if created_double {
                set_inode_block_ptr(inode_raw, 13, 0);
                set_inode_blocks_512(
                    inode_raw,
                    inode_blocks_512(inode_raw).saturating_sub(sb.block_size / 512),
                );
                let _ = free_block_number(double_indirect);
            }
            return Err(rc);
        }
        set_inode_blocks_512(
            inode_raw,
            inode_blocks_512(inode_raw) + (sb.block_size / 512),
        );
    }
    let rc = write_indirect_entry(indirect, l2_index, value);
    if rc != 0 {
        if created_indirect {
            set_inode_blocks_512(
                inode_raw,
                inode_blocks_512(inode_raw).saturating_sub(sb.block_size / 512),
            );
            let _ = free_block_number(indirect);
            let _ = write_indirect_entry(double_indirect, l1_index, 0);
        }
        if created_double {
            set_inode_block_ptr(inode_raw, 13, 0);
            set_inode_blocks_512(
                inode_raw,
                inode_blocks_512(inode_raw).saturating_sub(sb.block_size / 512),
            );
            let _ = free_block_number(double_indirect);
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
    let entries = (sb.block_size / 4) as usize;
    12 + entries + entries * entries
}

fn max_writable_size() -> u64 {
    max_writable_blocks() as u64 * superblock().block_size as u64
}

fn append_full_blocks(
    ino: u32,
    inode_raw: &mut [u8],
    offset: u64,
    src: &[u8],
) -> Result<usize, i32> {
    let sb = superblock();
    let block_size = sb.block_size as usize;
    if offset != get_u32(inode_raw, 4) as u64
        || (offset as usize & (block_size - 1)) != 0
        || src.len() < block_size
    {
        return Ok(0);
    }
    let requested = core::cmp::min(src.len() / block_size, MAX_READ_TRANSFER_BYTES / block_size);
    let block_index = offset as usize / block_size;
    if block_index.checked_add(requested).ok_or(EFBIG)? > max_writable_blocks() {
        return Err(EFBIG);
    }
    let (first_block, count) = allocate_contiguous_blocks(requested)?;
    let mut inode_before = [0u8; MAX_INODE_SIZE];
    inode_before.copy_from_slice(&inode_raw[..MAX_INODE_SIZE]);
    let data_blocks_512 = (count as u32)
        .checked_mul(sb.block_size / 512)
        .ok_or(EOVERFLOW)?;
    let mut mapped = 0usize;
    while mapped < count {
        if let Err(rc) =
            set_data_block_number(inode_raw, block_index + mapped, first_block + mapped as u32)
        {
            let _ = rollback_contiguous_append(
                ino,
                inode_raw,
                &inode_before,
                block_index,
                mapped,
                first_block,
                count,
            );
            return Err(rc);
        }
        mapped += 1;
    }
    set_inode_blocks_512(
        inode_raw,
        inode_blocks_512(inode_raw)
            .checked_add(data_blocks_512)
            .ok_or(EOVERFLOW)?,
    );
    let rc = write_inode_raw(ino, inode_raw);
    if rc != 0 {
        let _ = rollback_contiguous_append(
            ino,
            inode_raw,
            &inode_before,
            block_index,
            count,
            first_block,
            count,
        );
        return Err(rc);
    }
    let bytes = count * block_size;
    let rc = write_exact(first_block as u64 * sb.block_size as u64, &src[..bytes]);
    if rc != 0 {
        let _ = rollback_contiguous_append(
            ino,
            inode_raw,
            &inode_before,
            block_index,
            count,
            first_block,
            count,
        );
        return Err(rc);
    }
    let new_size = offset.checked_add(bytes as u64).ok_or(EFBIG)?;
    set_u32(inode_raw, 4, new_size as u32);
    update_change_times(inode_raw);
    let rc = write_inode_raw(ino, inode_raw);
    if rc != 0 {
        let _ = rollback_contiguous_append(
            ino,
            inode_raw,
            &inode_before,
            block_index,
            count,
            first_block,
            count,
        );
        return Err(rc);
    }
    Ok(bytes)
}

fn rollback_contiguous_append(
    ino: u32,
    inode_raw: &mut [u8],
    inode_before: &[u8; MAX_INODE_SIZE],
    block_index: usize,
    mapped: usize,
    first_block: u32,
    allocated: usize,
) -> Result<(), i32> {
    let mut index = 0usize;
    while index < mapped {
        set_data_block_number(inode_raw, block_index + index, 0)?;
        index += 1;
    }
    inode_raw[..MAX_INODE_SIZE].copy_from_slice(inode_before);
    let rc = write_inode_raw(ino, inode_raw);
    if rc != 0 {
        return Err(rc);
    }
    free_contiguous_blocks(first_block, allocated)
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
        uid: get_u16(raw, 2) as u32 | ((get_u16(raw, 120) as u32) << 16),
        gid: get_u16(raw, 24) as u32 | ((get_u16(raw, 122) as u32) << 16),
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
            let entry_ino = get_u32(&block_buf, off);
            if entry_ino == 0 && rec_len >= needed_len {
                block_buf[off..off + rec_len].fill(0);
                set_u32(&mut block_buf, off, child_ino);
                set_u16(&mut block_buf, off + 4, rec_len as u16);
                block_buf[off + 6] = name.len() as u8;
                block_buf[off + 7] = file_type;
                block_buf[off + 8..off + 8 + name.len()].copy_from_slice(name);
                let rc = write_block(block, &block_buf);
                if rc != 0 {
                    return Err(rc);
                }
                update_parent_after_entry_change(parent_ino, child_is_dir, true)?;
                return Ok(());
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

fn update_parent_after_entry_change(
    parent_ino: u32,
    child_is_dir: bool,
    added: bool,
) -> Result<(), i32> {
    let mut parent_raw = [0u8; MAX_INODE_SIZE];
    read_inode_raw(parent_ino, &mut parent_raw)?;
    if child_is_dir {
        let links = get_u16(&parent_raw, 26);
        let links = if added {
            links.checked_add(1).ok_or(EOVERFLOW)?
        } else {
            links.checked_sub(1).ok_or(EINVAL)?
        };
        set_u16(&mut parent_raw, 26, links);
    }
    update_change_times(&mut parent_raw);
    let rc = write_inode_raw(parent_ino, &parent_raw);
    if rc == 0 { Ok(()) } else { Err(rc) }
}

fn remove_dir_entry(
    parent_ino: u32,
    parent_inode: Inode,
    name: &[u8],
    child_ino: u32,
    child_is_dir: bool,
) -> Result<(), i32> {
    let sb = superblock();
    if (parent_inode.flags & EXT2_INDEX_FL) != 0 {
        return Err(ENOSYS);
    }
    let blocks = (parent_inode.size as usize).div_ceil(sb.block_size as usize);
    let mut block_buf = [0u8; MAX_BLOCK_SIZE];
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
        let mut previous = None;
        while off + 8 <= sb.block_size as usize {
            let entry_ino = get_u32(&block_buf, off);
            let rec_len = get_u16(&block_buf, off + 4) as usize;
            let name_len = block_buf[off + 6] as usize;
            if rec_len == 0 || off + rec_len > sb.block_size as usize {
                break;
            }
            if entry_ino == child_ino
                && off + 8 + name_len <= sb.block_size as usize
                && &block_buf[off + 8..off + 8 + name_len] == name
            {
                if let Some(previous_off) = previous {
                    let previous_len = get_u16(&block_buf, previous_off + 4) as usize;
                    let merged = previous_len.checked_add(rec_len).ok_or(EOVERFLOW)?;
                    set_u16(&mut block_buf, previous_off + 4, merged as u16);
                } else {
                    set_u32(&mut block_buf, off, 0);
                }
                let rc = write_block(block, &block_buf);
                if rc != 0 {
                    return Err(rc);
                }
                update_parent_after_entry_change(parent_ino, child_is_dir, false)?;
                return Ok(());
            }
            if entry_ino != 0 {
                previous = Some(off);
            }
            off += rec_len;
        }
        block_index += 1;
    }
    Err(ENOENT)
}

fn rename_dir_entry(
    parent_ino: u32,
    parent_inode: Inode,
    old_name: &[u8],
    new_name: &[u8],
    child_ino: u32,
    file_type: u8,
) -> Result<(), i32> {
    let sb = superblock();
    let blocks = (parent_inode.size as usize).div_ceil(sb.block_size as usize);
    let mut block_buf = [0u8; MAX_BLOCK_SIZE];
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
            let entry_ino = get_u32(&block_buf, off);
            let rec_len = get_u16(&block_buf, off + 4) as usize;
            let name_len = block_buf[off + 6] as usize;
            if rec_len == 0 || off + rec_len > sb.block_size as usize {
                break;
            }
            if entry_ino == child_ino
                && off + 8 + name_len <= sb.block_size as usize
                && &block_buf[off + 8..off + 8 + name_len] == old_name
                && new_name.len() <= rec_len.saturating_sub(8)
            {
                block_buf[off + 6] = new_name.len() as u8;
                block_buf[off + 7] = file_type;
                block_buf[off + 8..off + rec_len].fill(0);
                block_buf[off + 8..off + 8 + new_name.len()].copy_from_slice(new_name);
                let rc = write_block(block, &block_buf);
                if rc != 0 {
                    return Err(rc);
                }
                update_parent_after_entry_change(parent_ino, false, true)?;
                return Ok(());
            }
            off += rec_len;
        }
        block_index += 1;
    }

    add_dir_entry(
        parent_ino,
        parent_inode,
        new_name,
        child_ino,
        file_type,
        false,
    )?;
    remove_dir_entry(
        parent_ino,
        load_inode(parent_ino)?,
        old_name,
        child_ino,
        false,
    )
}

fn directory_is_empty(inode: Inode) -> Result<bool, i32> {
    let sb = superblock();
    let blocks = (inode.size as usize).div_ceil(sb.block_size as usize);
    let mut block_buf = [0u8; MAX_BLOCK_SIZE];
    let mut block_index = 0usize;
    while block_index < blocks {
        let block = data_block_number(inode, block_index)?;
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
            let entry_ino = get_u32(&block_buf, off);
            let rec_len = get_u16(&block_buf, off + 4) as usize;
            let name_len = block_buf[off + 6] as usize;
            if rec_len == 0 || off + rec_len > sb.block_size as usize {
                break;
            }
            if entry_ino != 0 && off + 8 + name_len <= sb.block_size as usize {
                let name = &block_buf[off + 8..off + 8 + name_len];
                if name != b"." && name != b".." {
                    return Ok(false);
                }
            }
            off += rec_len;
        }
        block_index += 1;
    }
    Ok(true)
}

fn free_inode_blocks(inode: Inode) -> Result<(), i32> {
    if inode.blocks[13] != 0 || inode.blocks[14] != 0 {
        return Err(ENOSYS);
    }
    let sb = superblock();
    let mut index = 0usize;
    while index < 12 {
        if inode.blocks[index] != 0 {
            free_block_number(inode.blocks[index])?;
        }
        index += 1;
    }
    if inode.blocks[12] != 0 {
        let mut indirect = [0u8; MAX_BLOCK_SIZE];
        let rc = read_block(inode.blocks[12], &mut indirect);
        if rc != 0 {
            return Err(rc);
        }
        let mut entry = 0usize;
        while entry < (sb.block_size / 4) as usize {
            let block = get_u32(&indirect, entry * 4);
            if block != 0 {
                free_block_number(block)?;
            }
            entry += 1;
        }
        free_block_number(inode.blocks[12])?;
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
        Ok((mut system, mut data)) => {
            let writable = (flags & MCX_FS_MOUNT_READ_ONLY) == 0;
            let ab_layout = data.is_some();
            if let Err(rc) = validate_mount_features(system.sb, writable && !ab_layout) {
                log_bytes(b"ext2.cext: unsupported filesystem features");
                return rc;
            }
            if let Some(volume) = data {
                if let Err(rc) = validate_mount_features(volume.sb, writable) {
                    log_bytes(b"ext2.cext: unsupported data filesystem features");
                    return rc;
                }
            }
            system.writable = writable && !ab_layout;
            if let Some(ref mut volume) = data { volume.writable = writable; }
            unsafe {
                STATE.partition_lba_base = system.base;
                STATE.partition_lba_count = system.count;
                STATE.sb = system.sb;
                STATE.mounted = true;
                STATE.writable = system.writable;
                STATE.system_volume = Some(system);
                STATE.data_volume = data;
                READY.store(true, Ordering::Release);
            }
            let slot = unsafe { KERNEL_API.as_ref().map(|api| (api.boot_system_slot)()) };
            match slot {
                Some(1) => log_boot_bytes(b"ext2.cext: mounted system A"),
                Some(2) => log_boot_bytes(b"ext2.cext: mounted system B"),
                _ => {}
            }
            if data.is_some() { log_boot_bytes(b"ext2.cext: mounted data"); }
            0
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

extern "C" fn create_raw(path: McxPath, mode: u32, uid: u32, gid: u32) -> i32 {
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
    if uid != u32::MAX {
        set_u16(&mut inode_raw, 2, uid as u16);
        set_u16(&mut inode_raw, 120, (uid >> 16) as u16);
    }
    if gid != u32::MAX {
        set_u16(&mut inode_raw, 24, gid as u16);
        set_u16(&mut inode_raw, 122, (gid >> 16) as u16);
    }
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

extern "C" fn remove_raw(path: McxPath, remove_directory: u32) -> i32 {
    if let Err(rc) = require_writable() {
        return rc;
    }
    let Some(path) = path_bytes(path) else {
        return EINVAL;
    };
    if path == b"/" || remove_directory > 1 {
        return EINVAL;
    }
    let (ino, inode) = match resolve_path(path) {
        Ok(value) => value,
        Err(rc) => return rc,
    };
    let target_is_dir = is_dir(inode.mode);
    if !target_is_dir && !is_file(inode.mode) {
        return ENOSYS;
    }
    if target_is_dir != (remove_directory != 0) {
        return if target_is_dir { EISDIR } else { ENOTDIR };
    }
    if inode.blocks[13] != 0 || inode.blocks[14] != 0 {
        return ENOSYS;
    }
    if target_is_dir {
        match directory_is_empty(inode) {
            Ok(true) => {}
            Ok(false) => return ENOTEMPTY,
            Err(rc) => return rc,
        }
    }
    let (parent_path, name) = match split_parent(path) {
        Ok(value) => value,
        Err(rc) => return rc,
    };
    let (parent_ino, parent_inode) = match resolve_path(parent_path) {
        Ok(value) => value,
        Err(rc) => return rc,
    };
    if let Err(rc) = remove_dir_entry(parent_ino, parent_inode, name, ino, target_is_dir) {
        return rc;
    }
    if let Err(rc) = free_inode_blocks(inode) {
        return rc;
    }
    let mut inode_raw = [0u8; MAX_INODE_SIZE];
    if let Err(rc) = read_inode_raw(ino, &mut inode_raw) {
        return rc;
    }
    inode_raw.fill(0);
    set_u32(&mut inode_raw, 20, now_seconds());
    let rc = write_inode_raw(ino, &inode_raw);
    if rc != 0 {
        return rc;
    }
    if target_is_dir {
        if let Err(rc) = decrement_used_dirs(ino) {
            return rc;
        }
    }
    if let Err(rc) = free_inode_number(ino) {
        return rc;
    }
    disk_flush()
}

extern "C" fn rename_raw(src: McxPath, dst: McxPath) -> i32 {
    if let Err(rc) = require_writable() {
        return rc;
    }
    let (Some(src), Some(dst)) = (path_bytes(src), path_bytes(dst)) else {
        return EINVAL;
    };
    if src == b"/" || dst == b"/" || src == dst {
        return if src == dst { 0 } else { EINVAL };
    }
    match resolve_path(dst) {
        Ok(_) => return EEXIST,
        Err(ENOENT) => {}
        Err(rc) => return rc,
    }
    let (src_parent_path, src_name) = match split_parent(src) {
        Ok(value) => value,
        Err(rc) => return rc,
    };
    let (dst_parent_path, dst_name) = match split_parent(dst) {
        Ok(value) => value,
        Err(rc) => return rc,
    };
    if src_parent_path != dst_parent_path
        || dst_name.len() > u8::MAX as usize
        || contains_slash(dst_name)
    {
        return EINVAL;
    }
    let (child_ino, child_inode) = match resolve_path(src) {
        Ok(value) => value,
        Err(rc) => return rc,
    };
    if !is_dir(child_inode.mode) && !is_file(child_inode.mode) {
        return ENOSYS;
    }
    let (parent_ino, parent_inode) = match resolve_path(src_parent_path) {
        Ok(value) => value,
        Err(rc) => return rc,
    };
    let file_type = if is_dir(child_inode.mode) {
        EXT2_FT_DIR
    } else {
        EXT2_FT_REG_FILE
    };
    if let Err(rc) = rename_dir_entry(
        parent_ino,
        parent_inode,
        src_name,
        dst_name,
        child_ino,
        file_type,
    ) {
        return rc;
    }
    let mut child_raw = [0u8; MAX_INODE_SIZE];
    if let Err(rc) = read_inode_raw(child_ino, &mut child_raw) {
        return rc;
    }
    update_change_times(&mut child_raw);
    let rc = write_inode_raw(child_ino, &child_raw);
    if rc != 0 {
        return rc;
    }
    disk_flush()
}

extern "C" fn read_raw(path: McxPath, offset: u64, buf: McxBuffer, out_read: *mut usize) -> i32 {
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

extern "C" fn write_raw(
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
        match append_full_blocks(
            ino,
            &mut inode_raw,
            offset + written as u64,
            &src[written..],
        ) {
            Ok(0) => {}
            Ok(bytes) => {
                written += bytes;
                continue;
            }
            Err(rc) => return finish_write(out_written, written, rc),
        }
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
                let rollback_rc =
                    rollback_new_data_block(ino, &mut inode_raw, block_index, block, inode_before);
                return finish_write(
                    out_written,
                    written,
                    if rollback_rc == 0 { rc } else { rollback_rc },
                );
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
                let rollback_rc =
                    rollback_new_data_block(ino, &mut inode_raw, block_index, block, inode_before);
                return finish_write(
                    out_written,
                    written,
                    if rollback_rc == 0 { rc } else { rollback_rc },
                );
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
    if written == 0 { rc } else { disk_flush() }
}

fn rollback_new_data_block(
    ino: u32,
    inode_raw: &mut [u8],
    block_index: usize,
    block: u32,
    old_inode: Inode,
) -> i32 {
    let sb = superblock();
    if block_index < 12 {
        set_inode_block_ptr(inode_raw, block_index, 0);
        set_inode_blocks_512(
            inode_raw,
            inode_blocks_512(inode_raw).saturating_sub(sb.block_size / 512),
        );
        let rc = write_inode_raw(ino, inode_raw);
        if rc != 0 {
            return rc;
        }
    } else if block_index < 12 + (sb.block_size / 4) as usize {
        let indirect = get_u32(inode_raw, 40 + 12 * 4);
        if old_inode.blocks[12] == 0 {
            set_inode_block_ptr(inode_raw, 12, 0);
            set_inode_blocks_512(
                inode_raw,
                inode_blocks_512(inode_raw).saturating_sub(2 * (sb.block_size / 512)),
            );
            let rc = write_inode_raw(ino, inode_raw);
            if rc != 0 {
                return rc;
            }
            if let Err(rc) = free_block_number(block) {
                return rc;
            }
            return match free_block_number(indirect) {
                Ok(()) => 0,
                Err(rc) => rc,
            };
        }
        let rc = write_indirect_entry(indirect, block_index - 12, 0);
        if rc != 0 {
            return rc;
        }
        set_inode_blocks_512(
            inode_raw,
            inode_blocks_512(inode_raw).saturating_sub(sb.block_size / 512),
        );
        let rc = write_inode_raw(ino, inode_raw);
        if rc != 0 {
            return rc;
        }
    } else {
        let entries = (sb.block_size / 4) as usize;
        let double_index = block_index - 12 - entries;
        let l1_index = double_index / entries;
        let l2_index = double_index % entries;
        let double_indirect = get_u32(inode_raw, 40 + 13 * 4);
        let old_double = old_inode.blocks[13];
        let old_indirect = if old_double == 0 {
            0
        } else {
            match read_indirect_entry(old_double, l1_index) {
                Ok(value) => value,
                Err(rc) => return rc,
            }
        };
        let indirect = match read_indirect_entry(double_indirect, l1_index) {
            Ok(value) => value,
            Err(rc) => return rc,
        };
        if old_double == 0 {
            set_inode_block_ptr(inode_raw, 13, 0);
            set_inode_blocks_512(
                inode_raw,
                inode_blocks_512(inode_raw).saturating_sub(3 * (sb.block_size / 512)),
            );
            let rc = write_inode_raw(ino, inode_raw);
            if rc != 0 {
                return rc;
            }
            if let Err(rc) = free_block_number(block) {
                return rc;
            }
            if let Err(rc) = free_block_number(indirect) {
                return rc;
            }
            return match free_block_number(double_indirect) {
                Ok(()) => 0,
                Err(rc) => rc,
            };
        }
        if old_indirect == 0 {
            let rc = write_indirect_entry(double_indirect, l1_index, 0);
            if rc != 0 {
                return rc;
            }
            set_inode_blocks_512(
                inode_raw,
                inode_blocks_512(inode_raw).saturating_sub(2 * (sb.block_size / 512)),
            );
            let rc = write_inode_raw(ino, inode_raw);
            if rc != 0 {
                return rc;
            }
            if let Err(rc) = free_block_number(block) {
                return rc;
            }
            return match free_block_number(indirect) {
                Ok(()) => 0,
                Err(rc) => rc,
            };
        }
        let rc = write_indirect_entry(indirect, l2_index, 0);
        if rc != 0 {
            return rc;
        }
        set_inode_blocks_512(
            inode_raw,
            inode_blocks_512(inode_raw).saturating_sub(sb.block_size / 512),
        );
        let rc = write_inode_raw(ino, inode_raw);
        if rc != 0 {
            return rc;
        }
    }
    match free_block_number(block) {
        Ok(()) => 0,
        Err(rc) => rc,
    }
}

extern "C" fn truncate_raw(path: McxPath, len: u64) -> i32 {
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

    let double_indirect = get_u32(&inode_raw, 40 + 13 * 4);
    let mut double_removed_data = 0usize;
    let mut double_metadata = [0u32; MAX_BLOCK_SIZE / 4 + 1];
    let mut double_metadata_count = 0usize;
    if double_indirect != 0 {
        let entries = (sb.block_size / 4) as usize;
        let double_base = 12 + entries;
        let old_double_blocks = old_blocks.saturating_sub(double_base);
        let new_double_blocks = new_blocks.saturating_sub(double_base);
        let mut double_buf = [0u8; MAX_BLOCK_SIZE];
        let rc = read_block(double_indirect, &mut double_buf);
        if rc != 0 {
            return rc;
        }
        let l1_end = old_double_blocks.div_ceil(entries).min(entries);
        let mut l1_index = new_double_blocks / entries;
        while l1_index < l1_end {
            let indirect = get_u32(&double_buf, l1_index * 4);
            if indirect == 0 {
                l1_index += 1;
                continue;
            }
            let mut indirect_buf = [0u8; MAX_BLOCK_SIZE];
            let rc = read_block(indirect, &mut indirect_buf);
            if rc != 0 {
                return rc;
            }
            let first = if l1_index == new_double_blocks / entries {
                new_double_blocks % entries
            } else {
                0
            };
            let end = old_double_blocks
                .saturating_sub(l1_index * entries)
                .min(entries);
            let mut removed = [0u32; MAX_BLOCK_SIZE / 4];
            let mut removed_count = 0usize;
            let mut entry = first;
            while entry < end {
                let block = get_u32(&indirect_buf, entry * 4);
                if block != 0 {
                    removed[removed_count] = block;
                    removed_count += 1;
                    set_u32(&mut indirect_buf, entry * 4, 0);
                }
                entry += 1;
            }
            let empty = indirect_buf[..sb.block_size as usize]
                .iter()
                .all(|byte| *byte == 0);
            if empty {
                set_u32(&mut double_buf, l1_index * 4, 0);
                double_metadata[double_metadata_count] = indirect;
                double_metadata_count += 1;
            } else {
                let rc = write_block(indirect, &indirect_buf);
                if rc != 0 {
                    return rc;
                }
            }
            let mut removed_index = 0usize;
            while removed_index < removed_count {
                if let Err(rc) = free_block_number(removed[removed_index]) {
                    return rc;
                }
                removed_index += 1;
            }
            double_removed_data += removed_count;
            l1_index += 1;
        }
        let double_empty = double_buf[..sb.block_size as usize]
            .iter()
            .all(|byte| *byte == 0);
        if double_empty {
            set_inode_block_ptr(&mut inode_raw, 13, 0);
            double_metadata[double_metadata_count] = double_indirect;
            double_metadata_count += 1;
        } else {
            let rc = write_block(double_indirect, &double_buf);
            if rc != 0 {
                return rc;
            }
        }
    }

    let sectors_per_block = sb.block_size / 512;
    let removed_blocks = detached_count
        .saturating_add(double_removed_data)
        .saturating_add(double_metadata_count);
    let removed_sectors = (removed_blocks as u32).saturating_mul(sectors_per_block);
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
    let mut metadata_index = 0usize;
    while metadata_index < double_metadata_count {
        if let Err(rc) = free_block_number(double_metadata[metadata_index]) {
            return rc;
        }
        metadata_index += 1;
    }
    disk_flush()
}

extern "C" fn sync_raw() -> i32 {
    disk_flush()
}

extern "C" fn stat_raw(
    path: McxPath,
    out_mode: *mut u16,
    out_size: *mut u64,
    out_uid: *mut u32,
    out_gid: *mut u32,
) -> i32 {
    if out_mode.is_null() || out_size.is_null() || out_uid.is_null() || out_gid.is_null() {
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
        *out_uid = inode.uid;
        *out_gid = inode.gid;
    }
    0
}

extern "C" fn chmod_raw(path: McxPath, mode: u32) -> i32 {
    if let Err(rc) = require_writable() {
        return rc;
    }
    let Some(path) = path_bytes(path) else {
        return EINVAL;
    };
    let (ino, _) = match resolve_path(path) {
        Ok(value) => value,
        Err(rc) => return rc,
    };
    let mut inode_raw = [0u8; MAX_INODE_SIZE];
    if let Err(rc) = read_inode_raw(ino, &mut inode_raw) {
        return rc;
    }
    let file_type = get_u16(&inode_raw, 0) & 0xf000;
    set_u16(&mut inode_raw, 0, file_type | ((mode as u16) & 0o7777));
    update_change_times(&mut inode_raw);
    write_inode_raw(ino, &inode_raw)
}

extern "C" fn chown_raw(path: McxPath, uid: u32, gid: u32) -> i32 {
    if let Err(rc) = require_writable() {
        return rc;
    }
    let Some(path) = path_bytes(path) else {
        return EINVAL;
    };
    let (ino, _) = match resolve_path(path) {
        Ok(value) => value,
        Err(rc) => return rc,
    };
    let mut inode_raw = [0u8; MAX_INODE_SIZE];
    if let Err(rc) = read_inode_raw(ino, &mut inode_raw) {
        return rc;
    }
    if uid != u32::MAX {
        set_u16(&mut inode_raw, 2, uid as u16);
        set_u16(&mut inode_raw, 120, (uid >> 16) as u16);
    }
    if gid != u32::MAX {
        set_u16(&mut inode_raw, 24, gid as u16);
        set_u16(&mut inode_raw, 122, (gid >> 16) as u16);
    }
    update_change_times(&mut inode_raw);
    write_inode_raw(ino, &inode_raw)
}

extern "C" fn readdir_raw(path: McxPath, buf: McxBuffer, out_len: *mut usize) -> i32 {
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

fn path_in_directory(path: &[u8], directory: &[u8]) -> bool {
    path == directory || (path.starts_with(directory) && path.get(directory.len()) == Some(&b'/'))
}

fn is_data_path(path: &[u8]) -> bool {
    [b"/home".as_slice(), b"/var", b"/tmp", b"/system/users", b"/system/logs"]
        .iter()
        .any(|directory| path_in_directory(path, directory))
}

struct OperationGuard;

impl OperationGuard {
    fn acquire() -> Self {
        while OPERATION_LOCK.compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed).is_err() {
            core::hint::spin_loop();
        }
        Self
    }
}

impl Drop for OperationGuard {
    fn drop(&mut self) {
        OPERATION_LOCK.store(false, Ordering::Release);
    }
}

fn with_path_volume(path: McxPath, operation: impl FnOnce() -> i32) -> i32 {
    let data_path = path_bytes(path).is_some_and(is_data_path);
    let _guard = OperationGuard::acquire();
    unsafe {
        let volume = if data_path { STATE.data_volume.or(STATE.system_volume) } else { STATE.system_volume };
        if let Some(volume) = volume {
            STATE.partition_lba_base = volume.base;
            STATE.partition_lba_count = volume.count;
            STATE.sb = volume.sb;
            STATE.writable = volume.writable;
        }
    }
    operation()
}

extern "C" fn create_impl(path: McxPath, mode: u32, uid: u32, gid: u32) -> i32 {
    with_path_volume(path, || create_raw(path, mode, uid, gid))
}
extern "C" fn remove_impl(path: McxPath, remove_directory: u32) -> i32 {
    with_path_volume(path, || remove_raw(path, remove_directory))
}
extern "C" fn rename_impl(src: McxPath, dst: McxPath) -> i32 {
    let Some(src_path) = path_bytes(src) else { return EINVAL; };
    let Some(dst_path) = path_bytes(dst) else { return EINVAL; };
    with_path_volume(src, || {
        let data_volume = unsafe { STATE.data_volume };
        if data_volume.is_some() && is_data_path(src_path) != is_data_path(dst_path) {
            return -18; // EXDEV: rename must never move an inode across partitions.
        }
        rename_raw(src, dst)
    })
}
extern "C" fn read_impl(path: McxPath, offset: u64, buf: McxBuffer, out_read: *mut usize) -> i32 {
    with_path_volume(path, || read_raw(path, offset, buf, out_read))
}
extern "C" fn write_impl(path: McxPath, offset: u64, buf: McxBuffer, out_written: *mut usize) -> i32 {
    with_path_volume(path, || write_raw(path, offset, buf, out_written))
}
extern "C" fn truncate_impl(path: McxPath, len: u64) -> i32 {
    with_path_volume(path, || truncate_raw(path, len))
}
extern "C" fn stat_impl(path: McxPath, mode: *mut u16, size: *mut u64, uid: *mut u32, gid: *mut u32) -> i32 {
    with_path_volume(path, || stat_raw(path, mode, size, uid, gid))
}
extern "C" fn chmod_impl(path: McxPath, mode: u32) -> i32 {
    with_path_volume(path, || chmod_raw(path, mode))
}
extern "C" fn chown_impl(path: McxPath, uid: u32, gid: u32) -> i32 {
    with_path_volume(path, || chown_raw(path, uid, gid))
}
extern "C" fn readdir_impl(path: McxPath, buf: McxBuffer, out_len: *mut usize) -> i32 {
    with_path_volume(path, || readdir_raw(path, buf, out_len))
}
extern "C" fn sync_impl() -> i32 {
    let _guard = OperationGuard::acquire();
    sync_raw()
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
    chmod: chmod_impl,
    chown: chown_impl,
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
pub extern "C" fn mnu_module_init(api: *const McxKernelApi) -> *const McxFsOps {
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

    #[test]
    fn gpt_system_slots_are_matched_by_type_and_name() {
        let mut entry = [0u8; 128];
        entry[..16].copy_from_slice(&SYSTEM_PARTITION_TYPE);
        for (index, byte) in b"mochiOS System A".iter().enumerate() {
            entry[56 + index * 2] = *byte;
        }
        assert!(is_system_slot_entry(&entry, 1));
        assert!(!is_system_slot_entry(&entry, 2));
        entry[56 + 15 * 2] = b'B';
        assert!(is_system_slot_entry(&entry, 2));
        entry[0] ^= 1;
        assert!(!is_system_slot_entry(&entry, 2));
    }

    #[test]
    fn gpt_data_partition_is_matched_by_type_and_name() {
        let mut entry = [0u8; 128];
        entry[..16].copy_from_slice(&DATA_PARTITION_TYPE);
        for (index, byte) in b"mochiOS Data".iter().enumerate() {
            entry[56 + index * 2] = *byte;
        }
        assert!(is_data_entry(&entry));
        entry[56] = b'X';
        assert!(!is_data_entry(&entry));
    }

    #[test]
    fn mutable_paths_are_routed_only_at_directory_boundaries() {
        for path in [b"/home".as_slice(), b"/home/root", b"/var/config", b"/tmp/file", b"/system/users/users.db", b"/system/logs/audit.log"] {
            assert!(is_data_path(path));
        }
        for path in [b"/".as_slice(), b"/homebrew", b"/variety", b"/system/users-old", b"/system/services/update.service"] {
            assert!(!is_data_path(path));
        }
    }

    fn superblock_with_features(
        feature_compat: u32,
        feature_incompat: u32,
        feature_ro_compat: u32,
    ) -> Superblock {
        Superblock {
            blocks_count: 64,
            first_data_block: 1,
            block_size: 1024,
            last_write_time: 0,
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

    #[test]
    fn partition_io_cannot_cross_into_the_next_gpt_partition() {
        let base_lba = 2048;
        let lba_count = 8;
        assert_eq!(partition_offset(base_lba, lba_count, 0, 4096), Ok(1_048_576));
        assert_eq!(partition_offset(base_lba, lba_count, 4095, 1), Ok(1_052_671));
        assert_eq!(partition_offset(base_lba, lba_count, 4095, 2), Err(EINVAL));
        assert_eq!(partition_offset(base_lba, lba_count, 4096, 1), Err(EINVAL));
        assert_eq!(partition_offset(base_lba, lba_count, u64::MAX, 1), Err(EOVERFLOW));
        assert_eq!(partition_offset(u64::MAX, 0, 0, 1), Err(EOVERFLOW));
    }

    #[test]
    fn inode_owner_uses_low_and_high_uid_gid_fields() {
        let mut raw = [0u8; MAX_INODE_SIZE];
        set_u16(&mut raw, 0, S_IFREG | 0o640);
        set_u16(&mut raw, 2, 0x5678);
        set_u16(&mut raw, 24, 0xdef0);
        set_u16(&mut raw, 120, 0x1234);
        set_u16(&mut raw, 122, 0x9abc);
        let inode = load_inode_from_raw(&raw);
        assert_eq!(inode.uid, 0x1234_5678);
        assert_eq!(inode.gid, 0x9abc_def0);
        assert_eq!(inode.mode, S_IFREG | 0o640);
    }
}
