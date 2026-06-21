#![no_std]
#![allow(unsafe_op_in_unsafe_fn)]

use core::arch::asm;
use core::mem::size_of;
use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::{AtomicBool, Ordering, fence};

use mochi_cext_abi::{
    EINVAL, EIO, ENOSYS, ENXIO, MCX_CEXT_ABI, MCX_LOG_INFO, McxDiskOps, McxDmaRegion,
    McxKernelApi,
};

const PCI_CONFIG_ADDR: u16 = 0x0cf8;
const PCI_CONFIG_DATA: u16 = 0x0cfc;
const PCI_VENDOR_VIRTIO: u16 = 0x1af4;
const PCI_DEVICE_VIRTIO_BLK_LEGACY: u16 = 0x1001;
const PCI_COMMAND_OFFSET: u8 = 0x04;
const PCI_BAR0_OFFSET: u8 = 0x10;
const PCI_COMMAND_IO: u16 = 1 << 0;
const PCI_COMMAND_BUS_MASTER: u16 = 1 << 2;

const VIRTIO_PCI_GUEST_FEATURES: u16 = 0x04;
const VIRTIO_PCI_QUEUE_PFN: u16 = 0x08;
const VIRTIO_PCI_QUEUE_NUM: u16 = 0x0c;
const VIRTIO_PCI_QUEUE_SEL: u16 = 0x0e;
const VIRTIO_PCI_QUEUE_NOTIFY: u16 = 0x10;
const VIRTIO_PCI_STATUS: u16 = 0x12;
const VIRTIO_PCI_ISR: u16 = 0x13;
const VIRTIO_PCI_DEVICE_SPECIFIC: u16 = 0x14;
const VIRTIO_PCI_GUEST_PAGE_SIZE: u16 = 0x28;

const VIRTIO_STATUS_ACKNOWLEDGE: u8 = 1;
const VIRTIO_STATUS_DRIVER: u8 = 2;
const VIRTIO_STATUS_DRIVER_OK: u8 = 4;
const VIRTIO_STATUS_FAILED: u8 = 128;

const VIRTIO_BLK_T_IN: u32 = 0;
const VIRTQ_DESC_F_NEXT: u16 = 1;
const VIRTQ_DESC_F_WRITE: u16 = 2;
const SECTOR_SIZE: usize = 512;
const QUEUE_ALIGN: usize = 4096;
const DISK_ID: u32 = 0;

#[repr(C, align(16))]
struct VirtqDesc {
    addr: u64,
    len: u32,
    flags: u16,
    next: u16,
}

#[repr(C, align(2))]
struct VirtqAvailHeader {
    flags: u16,
    idx: u16,
}

#[repr(C, align(4))]
struct VirtqUsedElem {
    id: u32,
    len: u32,
}

#[repr(C, align(4))]
struct VirtqUsedHeader {
    flags: u16,
    idx: u16,
}

#[repr(C)]
struct VirtioBlkReq {
    typ: u32,
    reserved: u32,
    sector: u64,
}

struct DriverState {
    api: *const McxKernelApi,
    io_base: u16,
    capacity_sectors: u64,
    dma: McxDmaRegion,
    queue_size: u16,
}

static READY: AtomicBool = AtomicBool::new(false);
static mut STATE: DriverState = DriverState {
    api: core::ptr::null(),
    io_base: 0,
    capacity_sectors: 0,
    dma: McxDmaRegion {
        virt: core::ptr::null_mut(),
        phys: 0,
        len: 0,
    },
    queue_size: 0,
};

#[inline(always)]
unsafe fn outl(port: u16, value: u32) {
    asm!("out dx, eax", in("dx") port, in("eax") value, options(nostack, preserves_flags));
}

#[inline(always)]
unsafe fn outw(port: u16, value: u16) {
    asm!("out dx, ax", in("dx") port, in("ax") value, options(nostack, preserves_flags));
}

#[inline(always)]
unsafe fn outb(port: u16, value: u8) {
    asm!("out dx, al", in("dx") port, in("al") value, options(nostack, preserves_flags));
}

#[inline(always)]
unsafe fn inl(port: u16) -> u32 {
    let value: u32;
    asm!("in eax, dx", in("dx") port, out("eax") value, options(nostack, preserves_flags));
    value
}

#[inline(always)]
unsafe fn inw(port: u16) -> u16 {
    let value: u16;
    asm!("in ax, dx", in("dx") port, out("ax") value, options(nostack, preserves_flags));
    value
}

#[inline(always)]
unsafe fn inb(port: u16) -> u8 {
    let value: u8;
    asm!("in al, dx", in("dx") port, out("al") value, options(nostack, preserves_flags));
    value
}

fn log_bytes(level: u32, bytes: &[u8]) {
    unsafe {
        let api = STATE.api;
        if !api.is_null() {
            ((*api).log)(level, bytes.as_ptr(), bytes.len());
        }
    }
}

fn log_u64(prefix: &[u8], value: u64) {
    let mut buf = [0u8; 96];
    let mut len = 0usize;
    while len < prefix.len() && len < buf.len() {
        buf[len] = prefix[len];
        len += 1;
    }
    if len < buf.len() {
        buf[len] = b'0';
        len += 1;
        if len < buf.len() {
            buf[len] = b'x';
            len += 1;
        }
    }
    let mut started = false;
    let mut shift = 60i32;
    while shift >= 0 && len < buf.len() {
        let digit = ((value >> shift) & 0xf) as u8;
        if digit != 0 || started || shift == 0 {
            started = true;
            buf[len] = if digit < 10 {
                b'0' + digit
            } else {
                b'a' + (digit - 10)
            };
            len += 1;
        }
        shift -= 4;
    }
    log_bytes(MCX_LOG_INFO, &buf[..len]);
}

fn log_u16(prefix: &[u8], value: u16) {
    log_u64(prefix, value as u64);
}

fn log_u8(prefix: &[u8], value: u8) {
    log_u64(prefix, value as u64);
}

fn ring_dma_bytes(queue_size: u16) -> usize {
    let desc = size_of::<VirtqDesc>() * queue_size as usize;
    let avail = 6 + (queue_size as usize * 2);
    let used = 6 + (queue_size as usize * size_of::<VirtqUsedElem>());
    let used_off = align_up(desc + avail, QUEUE_ALIGN);
    let req_off = align_up(used_off + used, 16);
    align_up(req_off + size_of::<VirtioBlkReq>() + SECTOR_SIZE + 1, 4096)
}

unsafe fn pci_config_read_u32(bus: u8, device: u8, func: u8, offset: u8) -> u32 {
    let address = 0x8000_0000u32
        | ((bus as u32) << 16)
        | ((device as u32) << 11)
        | ((func as u32) << 8)
        | ((offset as u32) & 0xfc);
    outl(PCI_CONFIG_ADDR, address);
    inl(PCI_CONFIG_DATA)
}

unsafe fn pci_config_write_u16(bus: u8, device: u8, func: u8, offset: u8, value: u16) {
    let aligned = offset & !0x3;
    let shift = ((offset & 0x2) as u32) * 8;
    let mut current = pci_config_read_u32(bus, device, func, aligned);
    current &= !(0xffffu32 << shift);
    current |= (value as u32) << shift;
    let address = 0x8000_0000u32
        | ((bus as u32) << 16)
        | ((device as u32) << 11)
        | ((func as u32) << 8)
        | ((aligned as u32) & 0xfc);
    outl(PCI_CONFIG_ADDR, address);
    outl(PCI_CONFIG_DATA, current);
}

unsafe fn find_virtio_blk_legacy() -> Option<(u8, u8, u8, u16)> {
    let mut bus = 0u16;
    while bus <= 255 {
        let mut device = 0u8;
        while device < 32 {
            let vendor_device = pci_config_read_u32(bus as u8, device, 0, 0x00);
            let vendor = (vendor_device & 0xffff) as u16;
            let dev = (vendor_device >> 16) as u16;
            if vendor == PCI_VENDOR_VIRTIO && dev == PCI_DEVICE_VIRTIO_BLK_LEGACY {
                let bar0 = pci_config_read_u32(bus as u8, device, 0, PCI_BAR0_OFFSET);
                if (bar0 & 1) != 0 {
                    let io_base = (bar0 & !0x3) as u16;
                    return Some((bus as u8, device, 0, io_base));
                }
            }
            device += 1;
        }
        bus += 1;
    }
    None
}

unsafe fn setup_device(api: *const McxKernelApi) -> i32 {
    let Some((bus, device, func, io_base)) = find_virtio_blk_legacy() else {
        return ENXIO;
    };

    let command = (pci_config_read_u32(bus, device, func, PCI_COMMAND_OFFSET) & 0xffff) as u16;
    pci_config_write_u16(
        bus,
        device,
        func,
        PCI_COMMAND_OFFSET,
        command | PCI_COMMAND_IO | PCI_COMMAND_BUS_MASTER,
    );

    outb(io_base + VIRTIO_PCI_STATUS, 0);
    outb(
        io_base + VIRTIO_PCI_STATUS,
        VIRTIO_STATUS_ACKNOWLEDGE | VIRTIO_STATUS_DRIVER,
    );
    outl(io_base + VIRTIO_PCI_GUEST_PAGE_SIZE, 4096);
    outl(io_base + VIRTIO_PCI_GUEST_FEATURES, 0);
    outw(io_base + VIRTIO_PCI_QUEUE_SEL, 0);
    let queue_num = inw(io_base + VIRTIO_PCI_QUEUE_NUM);
    if queue_num < 3 {
        outb(io_base + VIRTIO_PCI_STATUS, VIRTIO_STATUS_FAILED);
        return EIO;
    }
    let dma_bytes = ring_dma_bytes(queue_num);
    let mut dma = McxDmaRegion {
        virt: core::ptr::null_mut(),
        phys: 0,
        len: 0,
    };
    let rc = ((*api).alloc_dma)(dma_bytes, 4096, &mut dma);
    if rc != 0 || dma.virt.is_null() || dma.phys == 0 || dma.len < dma_bytes {
        outb(io_base + VIRTIO_PCI_STATUS, VIRTIO_STATUS_FAILED);
        return EIO;
    }

    STATE.api = api;
    STATE.io_base = io_base;
    STATE.dma = dma;
    STATE.queue_size = queue_num;
    outl(io_base + VIRTIO_PCI_QUEUE_PFN, (dma.phys >> 12) as u32);
    outb(
        io_base + VIRTIO_PCI_STATUS,
        VIRTIO_STATUS_ACKNOWLEDGE | VIRTIO_STATUS_DRIVER | VIRTIO_STATUS_DRIVER_OK,
    );

    let low = inl(io_base + VIRTIO_PCI_DEVICE_SPECIFIC) as u64;
    let high = inl(io_base + VIRTIO_PCI_DEVICE_SPECIFIC + 4) as u64;
    STATE.capacity_sectors = low | (high << 32);
    log_bytes(MCX_LOG_INFO, b"disk.cext: virtio-blk ready");
    READY.store(true, Ordering::Release);
    0
}

#[inline(always)]
unsafe fn desc_ptr() -> *mut VirtqDesc {
    STATE.dma.virt as *mut VirtqDesc
}

#[inline(always)]
fn desc_bytes() -> usize {
    size_of::<VirtqDesc>() * unsafe { STATE.queue_size as usize }
}

#[inline(always)]
fn avail_bytes() -> usize {
    6 + (unsafe { STATE.queue_size as usize } * 2)
}

#[inline(always)]
fn used_bytes() -> usize {
    6 + (unsafe { STATE.queue_size as usize } * size_of::<VirtqUsedElem>())
}

#[inline(always)]
fn align_up(value: usize, align: usize) -> usize {
    (value + (align - 1)) & !(align - 1)
}

#[inline(always)]
unsafe fn avail_ptr() -> *mut VirtqAvailHeader {
    STATE.dma.virt.add(desc_bytes()) as *mut VirtqAvailHeader
}

#[inline(always)]
unsafe fn avail_ring_ptr() -> *mut u16 {
    STATE.dma.virt.add(desc_bytes() + 4) as *mut u16
}

#[inline(always)]
unsafe fn used_ptr() -> *mut VirtqUsedHeader {
    let off = align_up(desc_bytes() + avail_bytes(), QUEUE_ALIGN);
    STATE.dma.virt.add(off) as *mut VirtqUsedHeader
}

#[inline(always)]
unsafe fn used_ring_ptr() -> *mut VirtqUsedElem {
    let off = align_up(desc_bytes() + avail_bytes(), QUEUE_ALIGN) + 4;
    STATE.dma.virt.add(off) as *mut VirtqUsedElem
}

#[inline(always)]
unsafe fn req_ptr() -> *mut VirtioBlkReq {
    let off = align_up(
        align_up(desc_bytes() + avail_bytes(), QUEUE_ALIGN) + used_bytes(),
        16,
    );
    STATE.dma.virt.add(off) as *mut VirtioBlkReq
}

#[inline(always)]
unsafe fn data_ptr() -> *mut u8 {
    (req_ptr() as *mut u8).add(size_of::<VirtioBlkReq>())
}

#[inline(always)]
unsafe fn status_ptr() -> *mut u8 {
    data_ptr().add(SECTOR_SIZE)
}

#[inline(always)]
unsafe fn req_phys() -> u64 {
    let off = (req_ptr() as usize).saturating_sub(STATE.dma.virt as usize);
    STATE.dma.phys + off as u64
}

#[inline(always)]
unsafe fn data_phys() -> u64 {
    let off = (data_ptr() as usize).saturating_sub(STATE.dma.virt as usize);
    STATE.dma.phys + off as u64
}

#[inline(always)]
unsafe fn status_phys() -> u64 {
    let off = (status_ptr() as usize).saturating_sub(STATE.dma.virt as usize);
    STATE.dma.phys + off as u64
}

unsafe fn submit_read(sector: u64, dst: *mut u8) -> i32 {
    let io_base = STATE.io_base;
    let desc = desc_ptr();
    let avail = avail_ptr();
    let used = used_ptr();
    let req = req_ptr();
    let data = data_ptr();
    let status = status_ptr();
    let used_before = read_volatile(core::ptr::addr_of!((*used).idx));
    let avail_idx = read_volatile(core::ptr::addr_of!((*avail).idx));
    let queue_size = STATE.queue_size;

    (*req).typ = VIRTIO_BLK_T_IN;
    (*req).reserved = 0;
    (*req).sector = sector;
    write_volatile(status, 0xff);

    write_volatile(desc.add(0), VirtqDesc {
        addr: req_phys(),
        len: size_of::<VirtioBlkReq>() as u32,
        flags: VIRTQ_DESC_F_NEXT,
        next: 1,
    });
    write_volatile(desc.add(1), VirtqDesc {
        addr: data_phys(),
        len: SECTOR_SIZE as u32,
        flags: VIRTQ_DESC_F_WRITE | VIRTQ_DESC_F_NEXT,
        next: 2,
    });
    write_volatile(desc.add(2), VirtqDesc {
        addr: status_phys(),
        len: 1,
        flags: VIRTQ_DESC_F_WRITE,
        next: 0,
    });

    let slot = (avail_idx % queue_size) as usize;
    write_volatile(avail_ring_ptr().add(slot), 0);
    fence(Ordering::SeqCst);
    write_volatile(core::ptr::addr_of_mut!((*avail).idx), avail_idx.wrapping_add(1));
    fence(Ordering::SeqCst);
    outw(io_base + VIRTIO_PCI_QUEUE_NOTIFY, 0);

    let mut spins = 0u32;
    while read_volatile(core::ptr::addr_of!((*used).idx)) == used_before {
        core::hint::spin_loop();
        spins = spins.wrapping_add(1);
        if spins == 100_000_000 {
            log_u64(b"disk.cext: timeout sector=", sector);
            log_u16(
                b"disk.cext: used.idx=",
                read_volatile(core::ptr::addr_of!((*used).idx)),
            );
            log_u8(
                b"disk.cext: isr=",
                inb(io_base + VIRTIO_PCI_ISR),
            );
            log_u8(
                b"disk.cext: status_reg=",
                inb(io_base + VIRTIO_PCI_STATUS),
            );
            return EIO;
        }
    }
    fence(Ordering::SeqCst);
    let _ = inb(io_base + VIRTIO_PCI_ISR);
    let used_slot = (used_before % queue_size) as usize;
    let used_elem = read_volatile(used_ring_ptr().add(used_slot));
    if used_elem.id != 0 {
        log_u64(b"disk.cext: used.id=", used_elem.id as u64);
        return EIO;
    }
    if read_volatile(status) != 0 {
        log_u64(b"disk.cext: status error sector=", sector);
        return EIO;
    }
    let mut i = 0usize;
    while i < SECTOR_SIZE {
        *dst.add(i) = *data.add(i);
        i += 1;
    }
    0
}

extern "C" fn probe_impl() -> i32 {
    if READY.load(Ordering::Acquire) {
        1
    } else {
        ENXIO
    }
}

extern "C" fn read_sector_impl(disk_id: u32, lba: u64, buf: *mut u8, buf_len: usize) -> i32 {
    if !READY.load(Ordering::Acquire) {
        return ENOSYS;
    }
    if disk_id != DISK_ID || buf.is_null() || buf_len == 0 || (buf_len % SECTOR_SIZE) != 0 {
        return EINVAL;
    }
    unsafe {
        let sectors = buf_len / SECTOR_SIZE;
        let capacity = STATE.capacity_sectors;
        if lba.checked_add(sectors as u64).is_none()
            || lba + sectors as u64 > capacity
        {
            return EIO;
        }
        let mut i = 0usize;
        while i < sectors {
            let rc = submit_read(lba + i as u64, buf.add(i * SECTOR_SIZE));
            if rc != 0 {
                return rc;
            }
            i += 1;
        }
    }
    0
}

extern "C" fn write_sector_impl(
    _disk_id: u32,
    _lba: u64,
    _buf: *const u8,
    _buf_len: usize,
) -> i32 {
    ENOSYS
}

static OPS: McxDiskOps = McxDiskOps {
    probe: probe_impl,
    read_sector: read_sector_impl,
    write_sector: write_sector_impl,
};

#[unsafe(no_mangle)]
pub extern "C" fn mochi_module_init(api: *const McxKernelApi) -> *const McxDiskOps {
    if api.is_null() {
        return core::ptr::null();
    }
    unsafe {
        if (*api).abi != MCX_CEXT_ABI {
            return core::ptr::null();
        }
        if setup_device(api) != 0 {
            return core::ptr::null();
        }
    }
    &OPS
}

#[unsafe(export_name = "_RNvNtNtCsljbRsbwaaOA_4core9panicking11panic_const23panic_const_rem_by_zero")]
pub extern "C" fn panic_const_rem_by_zero() -> ! {
    loop {
        core::hint::spin_loop();
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        core::hint::spin_loop();
    }
}
