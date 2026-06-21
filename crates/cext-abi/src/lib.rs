#![no_std]

#[repr(C)]
#[derive(Clone, Copy)]
pub struct McxDmaRegion {
    pub virt: *mut u8,
    pub phys: u64,
    pub len: usize,
}

#[repr(C)]
pub struct McxKernelApi {
    pub abi: u16,
    pub struct_size: u16,
    pub alloc_dma:
        extern "C" fn(size: usize, align: usize, out_region: *mut McxDmaRegion) -> i32,
    pub log: extern "C" fn(level: u32, ptr: *const u8, len: usize),
    pub register_irq: extern "C" fn(irq: u8, handler: extern "C" fn(u8)) -> i32,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct McxBuffer {
    pub ptr: *mut u8,
    pub len: usize,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct McxPath {
    pub ptr: *const u8,
    pub len: usize,
}

#[repr(C)]
pub struct McxDiskOps {
    pub probe: extern "C" fn() -> i32,
    pub read_sector: extern "C" fn(disk_id: u32, lba: u64, buf: *mut u8, buf_len: usize) -> i32,
    pub write_sector: extern "C" fn(disk_id: u32, lba: u64, buf: *const u8, buf_len: usize) -> i32,
    pub flush: extern "C" fn(disk_id: u32) -> i32,
}

#[repr(C)]
pub struct McxFsOps {
    pub mount: extern "C" fn(device_id: u32) -> i32,
    pub set_disk_ops: extern "C" fn(ops: *const McxDiskOps) -> i32,
    pub create: extern "C" fn(path: McxPath, mode: u32) -> i32,
    pub remove: extern "C" fn(path: McxPath, is_dir: u32) -> i32,
    pub rename: extern "C" fn(src: McxPath, dst: McxPath) -> i32,
    pub read:
        extern "C" fn(path: McxPath, offset: u64, buf: McxBuffer, out_read: *mut usize) -> i32,
    pub write:
        extern "C" fn(path: McxPath, offset: u64, buf: McxBuffer, out_written: *mut usize) -> i32,
    pub truncate: extern "C" fn(path: McxPath, len: u64) -> i32,
    pub stat: extern "C" fn(path: McxPath, out_mode: *mut u16, out_size: *mut u64) -> i32,
    pub readdir: extern "C" fn(path: McxPath, buf: McxBuffer, out_len: *mut usize) -> i32,
}

pub const MCX_CEXT_ABI: u16 = 1;
pub const MCX_LOG_ERROR: u32 = 0;
pub const MCX_LOG_WARN: u32 = 1;
pub const MCX_LOG_INFO: u32 = 2;
pub const MCX_LOG_DEBUG: u32 = 3;

pub const ENOENT: i32 = -2;
pub const EIO: i32 = -5;
pub const ENXIO: i32 = -6;
pub const EFAULT: i32 = -14;
pub const EEXIST: i32 = -17;
pub const EINVAL: i32 = -22;
pub const EISDIR: i32 = -21;
pub const ENOTDIR: i32 = -20;
pub const ENOSPC: i32 = -28;
pub const ENOSYS: i32 = -38;
