// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![recursion_limit = "2048"]

//! Apple AOP driver
//!
//! Copyright (C) The Asahi Linux Contributors

use core::sync::atomic::{AtomicU32, Ordering};
use core::{arch::asm, mem, ptr, slice};

use kernel::{
    bindings, c_str,
    device::{self, RawDevice},
    dma,
    driver::DeviceRemoval,
    error::from_err_ptr,
    init::Zeroable,
    io_mem::IoMem,
    module_platform_driver, new_condvar, new_mutex, of, platform,
    prelude::*,
    soc::apple::rtkit,
    sync::{Arc, CondVar, Mutex},
    types::{ARef, ForeignOwnable},
};

const AOP_MMIO_SIZE: usize = 0x1e0000;
const ASC_MMIO_SIZE: usize = 0x4000;
const BOOTARGS_OFFSET: usize = 0x22c;
const BOOTARGS_SIZE: usize = 0x230;
const CPU_CONTROL: usize = 0x44;
const CPU_RUN: u32 = 0x1 << 4;
const AFK_ENDPOINT_START: u8 = 0x20;
const AFK_ENDPOINT_COUNT: u8 = 0xc;
const AFK_OPC_GET_BUF: u64 = 0x89;
const AFK_OPC_INIT: u64 = 0x80;
const AFK_OPC_INIT_RX: u64 = 0x8b;
const AFK_OPC_INIT_TX: u64 = 0x8a;
const AFK_OPC_INIT_UNK: u64 = 0x8c;
const AFK_OPC_SEND: u64 = 0xa2;
const AFK_OPC_START_ACK: u64 = 0x86;
const AFK_OPC_SHUTDOWN_ACK: u64 = 0xc1;
const AFK_OPC_RECV: u64 = 0x85;
const AFK_MSG_GET_BUF_ACK: u64 = 0xa1 << 48;
const AFK_MSG_INIT: u64 = AFK_OPC_INIT << 48;
const AFK_MSG_INIT_ACK: u64 = 0xa0 << 48;
const AFK_MSG_START: u64 = 0xa3 << 48;
const AFK_MSG_SHUTDOWN: u64 = 0xc0 << 48;
const AFK_RB_BLOCK_STEP: usize = 0x40;
const EPIC_TYPE_NOTIFY: u32 = 0;
const EPIC_CATEGORY_REPORT: u8 = 0x00;
const EPIC_CATEGORY_NOTIFY: u8 = 0x10;
const EPIC_CATEGORY_REPLY: u8 = 0x20;
const EPIC_SUBTYPE_STD_SERVICE: u16 = 0xc0;
const EPIC_SUBTYPE_FAKEHID_REPORT: u16 = 0xc4;
const EPIC_SUBTYPE_RETCODE: u16 = 0x84;
const EPIC_SUBTYPE_RETCODE_PAYLOAD: u16 = 0xa0;
const EPIC_SUBTYPE_WRAPPED_CALL: u16 = 0x20;
const EPIC_SUBTYPE_SET_ALS_PROPERTY: u16 = 0x4;
const QE_MAGIC1: u32 = from_fourcc(b" POI");
const QE_MAGIC2: u32 = from_fourcc(b" POA");
const CALLTYPE_AUDIO_ATTACH_DEVICE: u32 = 0xc3000002;
const CALLTYPE_AUDIO_SET_PROP: u32 = 0xc3000005;
const PDM_NUM_COEFFS: usize = 120;
const AUDIO_DEV_PDM0: u32 = from_fourcc(b"pdm0");
const AUDIO_DEV_LPAI: u32 = from_fourcc(b"lpai");
const AUDIO_DEV_HPAI: u32 = from_fourcc(b"hpai");

const fn from_fourcc(b: &[u8]) -> u32 {
    b[3] as u32 | (b[2] as u32) << 8 | (b[1] as u32) << 16 | (b[0] as u32) << 24
}

fn align_up(v: usize, a: usize) -> usize {
    (v + a - 1) & !(a - 1)
}

fn f32_to_u32(f: u32) -> u32 {
    if f & 0x80000000 != 0 {
        return 0;
    }
    let exp = ((f & 0x7f800000) >> 23) as i32 - 127;
    if exp < 0 {
        return 0;
    }
    if exp == 128 && f & 0x7fffff != 0 {
        return 0;
    }
    let mant = f & 0x7fffff | 0x800000;
    if exp <= 23 {
        return mant >> (23 - exp);
    }
    if exp >= 32 {
        return u32::MAX;
    }
    mant << (exp - 23)
}

#[inline(always)]
fn mem_sync() {
    unsafe {
        asm!("dsb sy");
    }
}

#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
struct QEHeader {
    magic: u32,
    size: u32,
    channel: u32,
    ty: u32,
}

#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
struct EPICHeader {
    version: u8,
    seq: u16,
    _pad0: u8,
    _unk0: u32,
    timestamp: u64,
    // Subheader
    length: u32,
    sub_version: u8,
    category: u8,
    subtype: u16,
    tag: u16,
    _unk1: u16,
    _pad1: u64,
    inline_len: u32,
}

#[repr(C, packed)]
struct EPICServiceAnnounce {
    name: [u8; 20],
    _unk0: u32,
    retcode: u32,
    _unk1: u32,
    channel: u32,
    _unk2: u32,
    _unk3: u32,
}

#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
struct AudioAttachDevice {
    _zero0: u32,
    unk0: u32,
    calltype: u32,
    _zero1: u64,
    _zero2: u64,
    _pad0: u32,
    len: u64,
    dev_id: u32,
    _pad1: u32,
}

impl AudioAttachDevice {
    fn new(dev_id: u32) -> AudioAttachDevice {
        AudioAttachDevice {
            unk0: 0xFFFFFFFF,
            calltype: CALLTYPE_AUDIO_ATTACH_DEVICE,
            dev_id,
            len: 0x2c,
            ..AudioAttachDevice::default()
        }
    }
}

#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
struct LpaiChannelConfig {
    unk1: u32,
    unk2: u32,
    unk3: u32,
    unk4: u32,
}

#[repr(C, packed)]
#[derive(Debug, Copy, Clone)]
struct PDMConfig {
    bytes_per_sample: u32,
    clock_source: u32,
    pdm_frequency: u32,
    pdmc_frequency: u32,
    slow_clock_speed: u32,
    fast_clock_speed: u32,
    channel_polarity_select: u32,
    channel_phase_select: u32,
    unk1: u32,
    unk2: u16,
    ratio1: u8,
    ratio2: u8,
    ratio3: u8,
    _pad0: u8,
    filter_lengths: u32,
    coeff_bulk: u32,
    coeffs: [u8; PDM_NUM_COEFFS * mem::size_of::<u32>()],
    unk3: u32,
    mic_turn_on_time_ms: u32,
    _zero0: u64,
    _zero1: u64,
    unk4: u32,
    mic_settle_time_ms: u32,
    _zero2: [u8; 69], // ?????
}

unsafe impl Zeroable for PDMConfig {}

#[repr(C, packed)]
#[derive(Debug, Copy, Clone)]
struct DecimatorConfig {
    latency: u32,
    ratio1: u8,
    ratio2: u8,
    ratio3: u8,
    _pad0: u8,
    filter_lengths: u32,
    coeff_bulk: u32,
    coeffs: [u8; PDM_NUM_COEFFS * mem::size_of::<u32>()],
}

unsafe impl Zeroable for DecimatorConfig {}

#[repr(C, packed)]
#[derive(Clone, Copy, Default, Debug)]
struct PowerSetting {
    dev_id: u32,
    cookie: u32,
    _unk0: u32,
    _zero0: u64,
    target_pstate: u32,
    unk1: u32,
    _zero1: [u8; 20],
}

impl PowerSetting {
    fn new(dev_id: u32, cookie: u32, target_pstate: u32, unk1: u32) -> PowerSetting {
        PowerSetting {
            dev_id,
            cookie,
            target_pstate,
            unk1,
            ..PowerSetting::default()
        }
    }
}

#[repr(C, packed)]
#[derive(Clone, Copy, Default, Debug)]
struct AudioSetDeviceProp<T> {
    _zero0: u32,
    unk0: u32,
    calltype: u32,
    _zero1: u64,
    _zero2: u64,
    _pad0: u32,
    len: u64,
    dev_id: u32,
    modifier: u32,
    len2: u32,
    data: T,
}

impl<T: Default> AudioSetDeviceProp<T> {
    fn new(dev_id: u32, modifier: u32, data: T) -> AudioSetDeviceProp<T> {
        AudioSetDeviceProp {
            unk0: 0xFFFFFFFF,
            calltype: CALLTYPE_AUDIO_SET_PROP,
            dev_id,
            modifier,
            len: mem::size_of::<T>() as u64 + 0x30,
            len2: mem::size_of::<T>() as u32,
            data,
            ..AudioSetDeviceProp::default()
        }
    }
}

unsafe impl<T> Zeroable for AudioSetDeviceProp<T> {}

impl<T: Zeroable> AudioSetDeviceProp<T> {
    fn try_init<E>(
        dev_id: u32,
        modifier: u32,
        data: impl Init<T, E>,
    ) -> impl Init<AudioSetDeviceProp<T>, Error>
    where
        Error: From<E>,
    {
        try_init!(
            AudioSetDeviceProp {
                unk0: 0xFFFFFFFF,
                calltype: CALLTYPE_AUDIO_SET_PROP,
                dev_id,
                modifier,
                len: mem::size_of::<T>() as u64 + 0x30,
                len2: mem::size_of::<T>() as u32,
                data <- data,
                ..Zeroable::zeroed()
            }
        )
    }
}

#[pin_data]
struct FutureValue<T> {
    #[pin]
    val: Mutex<Option<T>>,
    #[pin]
    completion: CondVar,
}

impl<T: Clone> FutureValue<T> {
    fn pin_init() -> impl PinInit<FutureValue<T>> {
        pin_init!(
            FutureValue {
                val <- new_mutex!(None),
                completion <- new_condvar!()
            }
        )
    }
    fn complete(&self, val: T) {
        *self.val.lock() = Some(val);
        self.completion.notify_all();
    }
    fn wait(&self) -> T {
        let mut ret_guard = self.val.lock();
        while ret_guard.is_none() {
            self.completion.wait(&mut ret_guard);
        }
        ret_guard.as_ref().unwrap().clone()
    }
    fn try_get(&self) -> Option<T> {
        self.val.lock().as_ref().cloned()
    }
    fn reset(&self) {
        *self.val.lock() = None;
    }
}

struct AFKRingBuffer {
    offset: usize,
    block_size: usize,
    buf_size: usize,
}

struct AFKEndpoint {
    index: u8,
    iomem: Option<dma::CoherentAllocation<u8, dma::CoherentAllocator>>,
    txbuf: Option<AFKRingBuffer>,
    rxbuf: Option<AFKRingBuffer>,
    seq: u16,
    calls: [Option<Arc<FutureValue<u32>>>; 8],
}

unsafe impl Send for AFKEndpoint {}

impl AFKEndpoint {
    fn new(index: u8) -> AFKEndpoint {
        AFKEndpoint {
            index,
            iomem: None,
            txbuf: None,
            rxbuf: None,
            seq: 0,
            calls: [const { None }; 8],
        }
    }

    fn start(&self, rtkit: &mut rtkit::RtKit<AopData>) -> Result<()> {
        rtkit.send_message(self.index, AFK_MSG_INIT)
    }

    fn stop(&self, rtkit: &mut rtkit::RtKit<AopData>) -> Result<()> {
        rtkit.send_message(self.index, AFK_MSG_SHUTDOWN)
    }

    fn recv_message(
        &mut self,
        client: &AopData,
        rtkit: &mut rtkit::RtKit<AopData>,
        msg: u64,
    ) -> Result<()> {
        let opc = msg >> 48;
        match opc {
            AFK_OPC_INIT => {
                rtkit.send_message(self.index, AFK_MSG_INIT_ACK)?;
            }
            AFK_OPC_GET_BUF => {
                self.recv_get_buf(client.dev.clone(), rtkit, msg)?;
            }
            AFK_OPC_INIT_UNK => {} // no-op
            AFK_OPC_START_ACK => {}
            AFK_OPC_INIT_RX => {
                if self.rxbuf.is_some() {
                    dev_err!(
                        client.dev,
                        "Got InitRX message with existing rxbuf at endpoint {}",
                        self.index
                    );
                    return Err(EIO);
                }
                self.rxbuf = Some(self.parse_ring_buf(&client.dev, msg)?);
                if self.txbuf.is_some() {
                    rtkit.send_message(self.index, AFK_MSG_START)?;
                }
            }
            AFK_OPC_INIT_TX => {
                if self.txbuf.is_some() {
                    dev_err!(
                        client.dev,
                        "Got InitTX message with existing txbuf at endpoint {}",
                        self.index
                    );
                    return Err(EIO);
                }
                self.txbuf = Some(self.parse_ring_buf(&client.dev, msg)?);
                if self.rxbuf.is_some() {
                    rtkit.send_message(self.index, AFK_MSG_START)?;
                }
            }
            AFK_OPC_RECV => {
                self.recv_rb(client)?;
            }
            AFK_OPC_SHUTDOWN_ACK => {
                client.shutdown_complete();
            }
            _ => dev_err!(
                client.dev,
                "AFK endpoint {} got unknown message {}",
                self.index,
                msg
            ),
        }
        Ok(())
    }

    fn parse_ring_buf(&self, dev: &ARef<device::Device>, msg: u64) -> Result<AFKRingBuffer> {
        let msg = msg as usize;
        let size = ((msg >> 16) & 0xFFFF) * AFK_RB_BLOCK_STEP;
        let offset = ((msg >> 32) & 0xFFFF) * AFK_RB_BLOCK_STEP;
        let buf_size = self.iomem_read32(dev, offset)? as usize;
        let block_size = (size - buf_size) / 3;
        Ok(AFKRingBuffer {
            offset,
            block_size,
            buf_size,
        })
    }
    fn iomem_write32(&mut self, dev: &ARef<device::Device>, off: usize, data: u32) -> Result<()> {
        let iomem = self.iomem.as_mut().unwrap();
        if off + mem::size_of::<u32>() > iomem.count() {
            dev_err!(dev, "Out of bounds iomem write");
            return Err(EIO);
        }
        unsafe {
            let ptr = iomem.first_ptr_mut().offset(off as isize) as *mut u32;
            *ptr = data;
        }
        Ok(())
    }
    fn iomem_read32(&self, dev: &ARef<device::Device>, off: usize) -> Result<u32> {
        let iomem = self.iomem.as_ref().unwrap();
        if off + mem::size_of::<u32>() > iomem.count() {
            dev_err!(dev, "Out of bounds iomem read");
            return Err(EIO);
        }
        // SAFETY: all bit patterns are valid u32s
        unsafe {
            let ptr = iomem.first_ptr().offset(off as isize) as *const u32;
            Ok(*ptr)
        }
    }
    fn memcpy_from_iomem(
        &self,
        dev: &ARef<device::Device>,
        off: usize,
        target: &mut [u8],
    ) -> Result<()> {
        let iomem = self.iomem.as_ref().unwrap();
        if off + target.len() > iomem.count() {
            dev_err!(dev, "Out of bounds iomem read");
            return Err(EIO);
        }
        // SAFETY: We checked that it is in bounds above
        unsafe {
            let ptr = iomem.first_ptr().offset(off as isize);
            let src = slice::from_raw_parts(ptr, target.len());
            target.copy_from_slice(src);
        }
        Ok(())
    }

    fn memcpy_to_iomem(&self, dev: &ARef<device::Device>, off: usize, src: &[u8]) -> Result<()> {
        let iomem = self.iomem.as_ref().unwrap();
        if off + src.len() > iomem.count() {
            dev_err!(dev, "Out of bounds iomem write");
            return Err(EIO);
        }
        // SAFETY: We checked that it is in bounds above
        unsafe {
            let ptr = iomem.first_ptr_mut().offset(off as isize);
            let target = slice::from_raw_parts_mut(ptr, src.len());
            target.copy_from_slice(src);
        }
        Ok(())
    }

    fn recv_get_buf(
        &mut self,
        dev: ARef<device::Device>,
        rtkit: &mut rtkit::RtKit<AopData>,
        msg: u64,
    ) -> Result<()> {
        let size = ((msg & 0xFFFF0000) >> 16) as usize * AFK_RB_BLOCK_STEP;
        if self.iomem.is_some() {
            dev_err!(
                dev,
                "Got GetBuf message with existing buffer on endpoint {}",
                self.index
            );
            return Err(EIO);
        }
        let iomem = dma::try_alloc_coherent(dev, size, false)?;
        rtkit.send_message(self.index, AFK_MSG_GET_BUF_ACK | iomem.dma_handle)?;
        self.iomem = Some(iomem);
        Ok(())
    }

    fn recv_rb(&mut self, client: &AopData) -> Result<()> {
        let (buf_offset, block_size, buf_size) = match self.rxbuf.as_ref() {
            Some(b) => (b.offset, b.block_size, b.buf_size),
            None => {
                dev_err!(
                    client.dev,
                    "Got Recv message with no rxbuf at endpoint {}",
                    self.index
                );
                return Err(EIO);
            }
        };
        let mut rptr = self.iomem_read32(&client.dev, buf_offset + block_size)? as usize;
        let mut wptr = self.iomem_read32(&client.dev, buf_offset + block_size * 2)?;
        mem_sync();
        let base = buf_offset + block_size * 3;
        let mut msg_buf = Vec::new();
        const QEH_SIZE: usize = mem::size_of::<QEHeader>();
        while wptr as usize != rptr {
            let mut qeh_bytes = [0; QEH_SIZE];
            self.memcpy_from_iomem(&client.dev, base + rptr, &mut qeh_bytes)?;
            let mut qeh = unsafe { &*(qeh_bytes.as_ptr() as *const QEHeader) };
            if qeh.magic != QE_MAGIC1 && qeh.magic != QE_MAGIC2 {
                let magic = qeh.magic;
                dev_err!(
                    client.dev,
                    "Invalid magic on ep {}, got {:x}",
                    self.index,
                    magic
                );
                return Err(EIO);
            }
            if qeh.size as usize > (buf_size - rptr - QEH_SIZE) {
                rptr = 0;
                self.memcpy_from_iomem(&client.dev, base + rptr, &mut qeh_bytes)?;
                qeh = unsafe { &*(qeh_bytes.as_ptr() as *const QEHeader) };

                if qeh.magic != QE_MAGIC1 && qeh.magic != QE_MAGIC2 {
                    let magic = qeh.magic;
                    dev_err!(
                        client.dev,
                        "Invalid magic on ep {}, got {:x}",
                        self.index,
                        magic
                    );
                    return Err(EIO);
                }
            }
            msg_buf.resize(qeh.size as usize, 0, GFP_KERNEL)?;
            self.memcpy_from_iomem(&client.dev, base + rptr + QEH_SIZE, &mut msg_buf)?;
            let (hdr_bytes, msg) = msg_buf.split_at(mem::size_of::<EPICHeader>());
            let header = unsafe { &*(hdr_bytes.as_ptr() as *const EPICHeader) };
            self.handle_ipc(client, qeh, header, msg)?;
            rptr = align_up(rptr + QEH_SIZE + qeh.size as usize, block_size) % buf_size;
            mem_sync();
            self.iomem_write32(&client.dev, buf_offset + block_size, rptr as u32)?;
            wptr = self.iomem_read32(&client.dev, buf_offset + block_size * 2)?;
            mem_sync();
        }
        Ok(())
    }
    fn handle_ipc(
        &mut self,
        client: &AopData,
        qhdr: &QEHeader,
        ehdr: &EPICHeader,
        data: &[u8],
    ) -> Result<()> {
        let subtype = ehdr.subtype;
        if ehdr.category == EPIC_CATEGORY_REPORT {
            if subtype == EPIC_SUBTYPE_STD_SERVICE {
                let announce = unsafe { &*(data.as_ptr() as *const EPICServiceAnnounce) };
                let chan = announce.channel;
                let name_len = announce
                    .name
                    .iter()
                    .position(|x| *x == 0)
                    .unwrap_or(announce.name.len());
                client.register_service(self, chan, &announce.name[..name_len]);
                return Ok(());
            } else if subtype == EPIC_SUBTYPE_FAKEHID_REPORT {
                return client.process_fakehid_report(self, qhdr.channel, data);
            } else {
                dev_err!(
                    client.dev,
                    "Unexpected EPIC report subtype {:x} on endpoint {}",
                    subtype,
                    self.index
                );
                return Err(EIO);
            }
        } else if ehdr.category == EPIC_CATEGORY_REPLY {
            if subtype == EPIC_SUBTYPE_RETCODE_PAYLOAD || subtype == EPIC_SUBTYPE_RETCODE {
                if data.len() < mem::size_of::<u32>() {
                    dev_err!(
                        client.dev,
                        "Retcode data too short on endpoint {}",
                        self.index
                    );
                    return Err(EIO);
                }
                let retcode = u32::from_ne_bytes(data[..4].try_into().unwrap());
                let tag = ehdr.tag as usize;
                if tag == 0 || tag - 1 > self.calls.len() || self.calls[tag - 1].is_none() {
                    dev_err!(
                        client.dev,
                        "Got a retcode with invalid tag {:?} on endpoint {}",
                        tag,
                        self.index
                    );
                    return Err(EIO);
                }
                self.calls[tag - 1].take().unwrap().complete(retcode);
                return Ok(());
            } else {
                dev_err!(
                    client.dev,
                    "Unexpected EPIC reply subtype {:x} on endpoint {}",
                    subtype,
                    self.index
                );
                return Err(EIO);
            }
        }
        dev_err!(
            client.dev,
            "Unexpected EPIC category {:x} on endpoint {}",
            ehdr.category,
            self.index
        );
        Err(EIO)
    }
    fn send_rb(
        &mut self,
        client: &AopData,
        rtkit: &mut rtkit::RtKit<AopData>,
        channel: u32,
        ty: u32,
        header: &[u8],
        data: &[u8],
    ) -> Result<()> {
        let (buf_offset, block_size, buf_size) = match self.txbuf.as_ref() {
            Some(b) => (b.offset, b.block_size, b.buf_size),
            None => {
                dev_err!(
                    client.dev,
                    "Attempting to send message with no txbuf at endpoint {}",
                    self.index
                );
                return Err(EIO);
            }
        };
        let base = buf_offset + block_size * 3;
        mem_sync();
        let rptr = self.iomem_read32(&client.dev, buf_offset + block_size)? as usize;
        let mut wptr = self.iomem_read32(&client.dev, buf_offset + block_size * 2)? as usize;
        const QEH_SIZE: usize = mem::size_of::<QEHeader>();
        if wptr < rptr && wptr + QEH_SIZE >= rptr {
            dev_err!(client.dev, "Tx buffer full at endpoint {}", self.index);
            return Err(EIO);
        }
        let payload_len = header.len() + data.len();
        let qeh = QEHeader {
            magic: QE_MAGIC1,
            size: payload_len as u32,
            channel,
            ty,
        };
        let qeh_bytes = unsafe {
            slice::from_raw_parts(
                &qeh as *const QEHeader as *const u8,
                mem::size_of::<QEHeader>(),
            )
        };
        self.memcpy_to_iomem(&client.dev, base + wptr, qeh_bytes)?;
        if payload_len > buf_size - wptr - QEH_SIZE {
            wptr = 0;
            self.memcpy_to_iomem(&client.dev, base + wptr, qeh_bytes)?;
        }
        self.memcpy_to_iomem(&client.dev, base + wptr + QEH_SIZE, header)?;
        self.memcpy_to_iomem(&client.dev, base + wptr + QEH_SIZE + header.len(), data)?;
        wptr = align_up(wptr + QEH_SIZE + payload_len, block_size) % buf_size;
        self.iomem_write32(&client.dev, buf_offset + block_size * 2, wptr as u32)?;
        let msg = wptr as u64 | (AFK_OPC_SEND << 48);
        rtkit.send_message(self.index, msg)
    }
    fn epic_notify(
        &mut self,
        client: &AopData,
        rtkit: &mut rtkit::RtKit<AopData>,
        channel: u32,
        subtype: u16,
        data: &[u8],
    ) -> Result<Arc<FutureValue<u32>>> {
        let mut tag = 0;
        for i in 0..self.calls.len() {
            if self.calls[i].is_none() {
                tag = i + 1;
                break;
            }
        }
        if tag == 0 {
            dev_err!(
                client.dev,
                "Too many inflight calls on endpoint {}",
                self.index
            );
            return Err(EIO);
        }
        let call = Arc::pin_init(FutureValue::pin_init(), GFP_KERNEL)?;
        let hdr = EPICHeader {
            version: 2,
            seq: self.seq,
            length: data.len() as u32,
            sub_version: 2,
            category: EPIC_CATEGORY_NOTIFY,
            subtype,
            tag: tag as u16,
            ..EPICHeader::default()
        };
        self.send_rb(
            client,
            rtkit,
            channel,
            EPIC_TYPE_NOTIFY,
            unsafe {
                slice::from_raw_parts(
                    &hdr as *const EPICHeader as *const u8,
                    mem::size_of::<EPICHeader>(),
                )
            },
            data,
        )?;
        self.seq = self.seq.wrapping_add(1);
        self.calls[tag - 1] = Some(call.clone());
        Ok(call)
    }
}

#[derive(Clone, Copy)]
struct EPICService {
    channel: u32,
    endpoint: u8,
}

#[pin_data]
struct AopData {
    dev: ARef<device::Device>,
    aop_mmio: IoMem<AOP_MMIO_SIZE>,
    asc_mmio: IoMem<ASC_MMIO_SIZE>,
    #[pin]
    rtkit: Mutex<Option<rtkit::RtKit<AopData>>>,
    #[pin]
    endpoints: [Mutex<AFKEndpoint>; AFK_ENDPOINT_COUNT as usize],
    #[pin]
    audio_service: FutureValue<EPICService>,
    #[pin]
    las_service: Mutex<Option<EPICService>>,
    #[pin]
    als_service: FutureValue<EPICService>,
    pstate_cookie: AtomicU32,
    #[pin]
    ep_shutdown: FutureValue<()>,
    sbs_cfg: AtomicU32,
    lid_angle: AtomicU32,
    lux_value: AtomicU32,
}

impl AopData {
    fn new(pdev: &mut platform::Device) -> Result<Arc<AopData>> {
        let aop_mmio = unsafe { pdev.ioremap_resource(0)? };
        let asc_mmio = unsafe { pdev.ioremap_resource(1)? };
        Arc::pin_init(
            pin_init!(
                AopData {
                    dev: device:: Device::from_dev(pdev),
                    aop_mmio,
                    asc_mmio,
                    rtkit <- new_mutex!(None),
                    endpoints <- init::pin_init_array_from_fn(|i| {
                        new_mutex!(AFKEndpoint::new(AFK_ENDPOINT_START + i as u8))
                    }),
                    audio_service <- FutureValue::pin_init(),
                    las_service <- new_mutex!(None),
                    als_service <- FutureValue::pin_init(),
                    pstate_cookie: AtomicU32::new(1),
                    ep_shutdown <- FutureValue::pin_init(),
                    sbs_cfg: AtomicU32::new(0),
                    lid_angle: AtomicU32::new(0),
                    lux_value: AtomicU32::new(0),
                }
            ),
            GFP_KERNEL,
        )
    }
    fn start(&self) -> Result<()> {
        {
            let mut guard = self.rtkit.lock();
            let rtk = guard.as_mut().unwrap();
            rtk.wake()?;
        }
        for ep in 0..AFK_ENDPOINT_COUNT {
            let rtk_ep_num = AFK_ENDPOINT_START + ep;
            let mut guard = self.rtkit.lock();
            let rtk = guard.as_mut().unwrap();
            if !rtk.has_endpoint(rtk_ep_num) {
                dev_warn!(self.dev, "Endpoint {} does not exist, skipping", rtk_ep_num);
                continue;
            }
            rtk.start_endpoint(rtk_ep_num)?;
            let ep_guard = self.endpoints[ep as usize].lock();
            ep_guard.start(rtk)?;
        }
        let audio_svc = self.audio_service.wait();
        for dev in [AUDIO_DEV_PDM0, AUDIO_DEV_HPAI, AUDIO_DEV_LPAI] {
            self.audio_attach_device(&audio_svc, dev)?;
        }
        self.set_lpai_channel_cfg(&audio_svc)?;
        self.set_pdm_config(&audio_svc)?;
        self.set_decimator_config(&audio_svc)?;
        let als_svc = self.als_service.wait();
        self.enable_als(&als_svc)?;
        Ok(())
    }
    fn set_pdm_config(&self, svc: &EPICService) -> Result<()> {
        let of = self.dev.of_node().ok_or(EIO)?;
        let of = &of;

        let pdm_cfg = try_init!(PDMConfig {
            bytes_per_sample: of.get_property(c_str!("apple,bytes-per-sample"))?,
            clock_source: of.get_property(c_str!("apple,clock-source"))?,
            pdm_frequency: of.get_property(c_str!("apple,pdm-frequency"))?,
            pdmc_frequency: of.get_property(c_str!("apple,pdmc-frequency"))?,
            slow_clock_speed: of.get_property(c_str!("apple,slow-clock-speed"))?,
            fast_clock_speed: of.get_property(c_str!("apple,fast-clock-speed"))?,
            channel_polarity_select: of.get_property(c_str!("apple,channel-polarity-select"))?,
            channel_phase_select: of.get_property(c_str!("apple,channel-phase-select"))?,
            unk1: 0xf7600,
            unk2: 0,
            filter_lengths: of.get_property(c_str!("apple,filter-lengths"))?,
            coeff_bulk: PDM_NUM_COEFFS as u32,
            unk3: 1,
            mic_turn_on_time_ms: of.get_property(c_str!("apple,mic-turn-on-time-ms"))?,
            unk4: 1,
            mic_settle_time_ms: of.get_property(c_str!("apple,mic-settle-time-ms"))?,
            ..Zeroable::zeroed()
        })
        .chain(|ret| {
            let prop = of.find_property(c_str!("apple,decm-ratios"))
                .ok_or(EIO)?;
            let ratios = prop.value();
            ret.ratio1 = ratios[0];
            ret.ratio2 = ratios[1];
            ret.ratio3 = ratios[2];
            let n_coeffs = (ratios[0] + ratios[1] + ratios[2] + 3) as usize * 16;
            of.find_property(c_str!("apple,coefficients"))
                .ok_or(EIO)?
                .copy_to_slice(&mut ret.coeffs[..n_coeffs])
        });
        let set_prop = AudioSetDeviceProp::<PDMConfig>::try_init(AUDIO_DEV_PDM0, 200, pdm_cfg);
        let msg = Box::try_init(set_prop, GFP_KERNEL)?;
        let ret = self.epic_wrapped_call(svc, msg.as_ref())?;
        if ret != 0 {
            dev_err!(self.dev, "Unable to set pdm config, return code {}", ret);
            return Err(EIO);
        } else {
            Ok(())
        }
    }
    fn set_decimator_config(&self, svc: &EPICService) -> Result<()> {
        let of = self.dev.of_node().ok_or(EIO)?;
        let of = &of;
        let pdm_cfg = try_init!(DecimatorConfig {
            latency: of.get_property(c_str!("apple,decm-latency"))?,
            filter_lengths: of.get_property(c_str!("apple,filter-lengths"))?,
            coeff_bulk: 120,
            ..Zeroable::zeroed()
        })
        .chain(|ret| {
            let prop = of.find_property(c_str!("apple,decm-ratios"))
                .ok_or(EIO)?;
            let ratios = prop.value();
            ret.ratio1 = ratios[0];
            ret.ratio2 = ratios[1];
            ret.ratio3 = ratios[2];
            let n_coeffs = (ratios[0] + ratios[1] + ratios[2] + 3) as usize * 16;
            of.find_property(c_str!("apple,coefficients"))
                .ok_or(EIO)?
                .copy_to_slice(&mut ret.coeffs[..n_coeffs])
        });
        let set_prop =
            AudioSetDeviceProp::<DecimatorConfig>::try_init(AUDIO_DEV_PDM0, 210, pdm_cfg);
        let msg = Box::try_init(set_prop, GFP_KERNEL)?;
        let ret = self.epic_wrapped_call(svc, msg.as_ref())?;
        if ret != 0 {
            dev_err!(
                self.dev,
                "Unable to set decimator config, return code {}",
                ret
            );
            return Err(EIO);
        } else {
            Ok(())
        }
    }
    fn set_lpai_channel_cfg(&self, svc: &EPICService) -> Result<()> {
        let cfg = LpaiChannelConfig {
            unk1: 7,
            unk2: 7,
            unk3: 1,
            unk4: 7,
        };
        let msg = AudioSetDeviceProp::new(AUDIO_DEV_LPAI, 301, cfg);
        let ret = self.epic_wrapped_call(svc, &msg)?;
        if ret != 0 {
            dev_err!(
                self.dev,
                "Unable to set lpai channel config, return code {}",
                ret
            );
            return Err(EIO);
        } else {
            Ok(())
        }
    }
    fn audio_attach_device(&self, svc: &EPICService, dev_id: u32) -> Result<()> {
        let msg = AudioAttachDevice::new(dev_id);
        let ret = self.epic_wrapped_call(svc, &msg)?;
        if ret != 0 {
            dev_err!(
                self.dev,
                "Unable to attach device {:?}, return code {}",
                dev_id,
                ret
            );
            return Err(EIO);
        } else {
            Ok(())
        }
    }
    fn enable_als(&self, svc: &EPICService) -> Result<()> {
        let of = self.dev.of_node().ok_or(EIO)?;
        if let Some(prop) = of.find_property(c_str!("apple,als-calibration")) {
            self.set_als_property(svc, 0xb, prop.value())?;
            self.set_als_property(svc, 0, &200000u32.to_le_bytes())?;
        } else {
            dev_warn!(self.dev, "ALS Calibration not found, will not enable it");
        }
        Ok(())
    }
    fn set_als_property(&self, svc: &EPICService, tag: u32, data: &[u8]) -> Result<u32> {
        let mut buf = Vec::new();
        buf.resize(data.len() + 8, 0, GFP_KERNEL)?;
        buf[8..].copy_from_slice(data);
        buf[4..8].copy_from_slice(&tag.to_le_bytes());
        self.epic_call(svc, EPIC_SUBTYPE_SET_ALS_PROPERTY, &buf)
    }
    fn epic_wrapped_call<T>(&self, svc: &EPICService, data: &T) -> Result<u32> {
        let msg_bytes =
            unsafe { slice::from_raw_parts(data as *const T as *const u8, mem::size_of::<T>()) };
        self.epic_call(svc, EPIC_SUBTYPE_WRAPPED_CALL, msg_bytes)
    }
    fn epic_call(&self, svc: &EPICService, subtype: u16, msg_bytes: &[u8]) -> Result<u32> {
        let ep_idx = svc.endpoint - AFK_ENDPOINT_START;
        let call = {
            let mut rtk_guard = self.rtkit.lock();
            let rtk = rtk_guard.as_mut().unwrap();
            let mut ep_guard = self.endpoints[ep_idx as usize].lock();
            ep_guard.epic_notify(self, rtk, svc.channel, subtype, msg_bytes)?
        };
        Ok(call.wait())
    }
    fn register_service(&self, ep: &mut AFKEndpoint, channel: u32, name: &[u8]) {
        let svc = EPICService {
            channel,
            endpoint: ep.index,
        };
        match name {
            b"aop-audio" => self.audio_service.complete(svc),
            b"las" => *self.las_service.lock() = Some(svc),
            b"als" => self.als_service.complete(svc),
            _ => {}
        }
    }
    fn set_audio_power(&self, pstate: u32, unk1: u32) -> Result<()> {
        let svc = self.audio_service.wait();
        let set_pstate = PowerSetting::new(
            AUDIO_DEV_HPAI,
            self.pstate_cookie.fetch_add(1, Ordering::Relaxed),
            pstate,
            unk1,
        );
        let msg = AudioSetDeviceProp::new(AUDIO_DEV_HPAI, 202, set_pstate);
        let ret = self.epic_wrapped_call(&svc, &msg)?;
        if ret != 0 {
            dev_err!(
                self.dev,
                "Unable to set power state {:?}, return code {}",
                pstate,
                ret
            );
            return Err(EIO);
        } else {
            Ok(())
        }
    }

    fn process_fakehid_report(&self, ep: &AFKEndpoint, ch: u32, data: &[u8]) -> Result<()> {
        if let Some(las_service) = *self.las_service.lock() {
            if ep.index == las_service.endpoint && ch == las_service.channel {
                self.lid_angle.store(data[1] as u32, Ordering::Relaxed);
                return Ok(());
            }
        }
        if let Some(als_service) = self.als_service.try_get() {
            if ep.index == als_service.endpoint && ch == als_service.channel {
                let raw = u32::from_le_bytes(data[40..44].try_into().unwrap());
                self.lux_value.store(f32_to_u32(raw), Ordering::Relaxed);
                return Ok(());
            }
        }
        dev_err!(self.dev, "Unexpected report from endpoint {}", ep.index);
        Err(EIO)
    }

    fn shutdown_complete(&self) {
        self.ep_shutdown.complete(());
    }

    fn stop(&self) -> Result<()> {
        for ep in 0..AFK_ENDPOINT_COUNT {
            dev_info!(self.dev, "Shutting down channel {}", ep);
            {
                let mut guard = self.rtkit.lock();
                let rtk = guard.as_mut().unwrap();
                let ep_guard = self.endpoints[ep as usize].lock();
                ep_guard.stop(rtk)?;
            }
            self.ep_shutdown.wait();
            self.ep_shutdown.reset();
        }
        Ok(())
    }

    fn aop_read32(&self, off: usize) -> u32 {
        self.aop_mmio.readl_relaxed(off)
    }

    fn patch_bootargs(&self, patches: &[(u32, u32)]) -> Result<()> {
        let offset = self.aop_read32(BOOTARGS_OFFSET) as usize;
        let size = self.aop_read32(BOOTARGS_SIZE) as usize;
        let mut arg_bytes = Vec::with_capacity(size, GFP_KERNEL)?;
        for _ in 0..size {
            arg_bytes.push(0, GFP_KERNEL).unwrap();
        }
        self.aop_mmio.try_memcpy_fromio(&mut arg_bytes, offset)?;
        let mut idx = 0;
        while idx < size {
            let key = u32::from_le_bytes(arg_bytes[idx..idx + 4].try_into().unwrap());
            let size = u32::from_le_bytes(arg_bytes[idx + 4..idx + 8].try_into().unwrap()) as usize;
            idx += 8;
            for (k, v) in patches.iter() {
                if *k != key {
                    continue;
                }
                arg_bytes[idx..idx + size].copy_from_slice(&(*v as u64).to_le_bytes()[..size]);
                break;
            }
            idx += size;
        }
        self.aop_mmio.try_memcpy_toio(offset, &arg_bytes)
    }

    fn start_cpu(&self) {
        let val = self.asc_mmio.readl_relaxed(CPU_CONTROL);
        self.asc_mmio.writel_relaxed(val | CPU_RUN, CPU_CONTROL);
    }
}

struct NoBuffer;
impl rtkit::Buffer for NoBuffer {
    fn iova(&self) -> Result<usize> {
        unreachable!()
    }
    fn buf(&mut self) -> Result<&mut [u8]> {
        unreachable!()
    }
}

#[vtable]
impl rtkit::Operations for AopData {
    type Data = Arc<AopData>;
    type Buffer = NoBuffer;

    fn recv_message(data: <Self::Data as ForeignOwnable>::Borrowed<'_>, ep: u8, msg: u64) {
        let mut rtk = data.rtkit.lock();
        let mut ep_guard = data.endpoints[(ep - AFK_ENDPOINT_START) as usize].lock();
        let ret = ep_guard.recv_message(&data, rtk.as_mut().unwrap(), msg);
        if let Err(e) = ret {
            dev_err!(data.dev, "Failed to handle rtkit message, error: {:?}", e);
        }
    }

    fn crashed(data: <Self::Data as ForeignOwnable>::Borrowed<'_>) {
        dev_err!(data.dev, "AOP firmware crashed");
    }
}

impl DeviceRemoval for AopData {
    fn device_remove(&self) {
        self.stop();
        *self.rtkit.lock() = None;
    }
}

struct AopDriver;

struct AlsaRegistration {
    card: *mut bindings::snd_card,
    pcm: *mut bindings::snd_pcm,
}

fn copy_str(target: &mut [i8], source: &[u8]) {
    for i in 0..source.len() {
        target[i] = source[i] as _;
    }
}

unsafe fn dmaengine_slave_config(
    chan: *mut bindings::dma_chan,
    config: *mut bindings::dma_slave_config,
) -> i32 {
    unsafe {
        match (*(*chan).device).device_config {
            Some(dc) => dc(chan, config),
            None => ENOSYS.to_errno(),
        }
    }
}

unsafe extern "C" fn aop_hw_params(
    substream: *mut bindings::snd_pcm_substream,
    params: *mut bindings::snd_pcm_hw_params,
) -> i32 {
    let chan = unsafe { bindings::snd_dmaengine_pcm_get_chan(substream) };
    let mut slave_config = bindings::dma_slave_config::default();
    let ret =
        unsafe { bindings::snd_hwparams_to_dma_slave_config(substream, params, &mut slave_config) };
    if ret < 0 {
        return ret;
    }
    slave_config.src_port_window_size = 4;
    unsafe { dmaengine_slave_config(chan, &mut slave_config) }
}

unsafe extern "C" fn aop_pcm_open(substream: *mut bindings::snd_pcm_substream) -> i32 {
    let data = unsafe { Arc::<AopData>::borrow((*substream).private_data) };
    if let Err(e) = data.set_audio_power(from_fourcc(b"pw1 "), 0) {
        dev_err!(data.dev, "Unable to enter 'pw1 ' state");
        return e.to_errno();
    }

    let dma_chan = match unsafe {
        from_err_ptr(bindings::of_dma_request_slave_channel(
            data.dev.of_node().unwrap().node() as *const _ as *mut _,
            c_str!("dma").as_ptr() as _,
        ))
    } {
        Ok(c) => c,
        Err(e) => {
            dev_err!(data.dev, "Unable to get dma channel");
            return e.to_errno();
        }
    };
    if data.sbs_cfg.fetch_add(1, Ordering::Relaxed) == 0 {
        let ret = unsafe {
            bindings::snd_pcm_set_managed_buffer(
                substream,
                bindings::SNDRV_DMA_TYPE_DEV_IRAM as i32,
                (*(*dma_chan).device).dev,
                0,
                0,
            )
        };
        if ret < 0 {
            dev_err!(data.dev, "Unable to allocate dma buffers");
            return ret;
        }
    }

    let mut hwparams = bindings::snd_pcm_hardware {
        info: bindings::SNDRV_PCM_INFO_MMAP
            | bindings::SNDRV_PCM_INFO_MMAP_VALID
            | bindings::SNDRV_PCM_INFO_INTERLEAVED,
        formats: bindings::BINDINGS_SNDRV_PCM_FMTBIT_FLOAT_LE,
        subformats: 0,
        rates: bindings::SNDRV_PCM_RATE_48000,
        rate_min: 48000,
        rate_max: 48000,
        channels_min: 3,
        channels_max: 3,
        periods_min: 2,
        buffer_bytes_max: usize::MAX,
        period_bytes_max: 0x4000,
        periods_max: u32::MAX,
        period_bytes_min: 256,
        fifo_size: 16,
    };
    let ret = unsafe {
        let mut dai_data = bindings::snd_dmaengine_dai_dma_data::default();
        bindings::snd_dmaengine_pcm_refine_runtime_hwparams(
            substream,
            &mut dai_data,
            &mut hwparams,
            dma_chan,
        )
    };
    if ret != 0 {
        dev_err!(data.dev, "Unable to refine hwparams");
        return ret;
    }
    if let Err(e) = data.set_audio_power(from_fourcc(b"pwrd"), 1) {
        dev_err!(data.dev, "Unable to power mic on");
        return e.to_errno();
    }
    unsafe {
        (*(*substream).runtime).hw = hwparams;
        bindings::snd_dmaengine_pcm_open(substream, dma_chan)
    }
}

unsafe extern "C" fn aop_pcm_prepare(_: *mut bindings::snd_pcm_substream) -> i32 {
    0
}

unsafe extern "C" fn aop_pcm_close(substream: *mut bindings::snd_pcm_substream) -> i32 {
    let data = unsafe { Arc::<AopData>::borrow((*substream).private_data) };
    if let Err(e) = data.set_audio_power(from_fourcc(b"pw1 "), 1) {
        dev_err!(data.dev, "Unable to power mic off");
        return e.to_errno();
    }
    let ret = unsafe { bindings::snd_dmaengine_pcm_close_release_chan(substream) };
    if ret != 0 {
        dev_err!(data.dev, "Unable to close channel");
        return ret;
    }
    if let Err(e) = data.set_audio_power(from_fourcc(b"idle"), 0) {
        dev_err!(data.dev, "Unable to enter 'idle' power state");
        return e.to_errno();
    }
    0
}

unsafe extern "C" fn aop_pcm_free_private(pcm: *mut bindings::snd_pcm) {
    unsafe {
        Arc::<AopData>::from_foreign((*pcm).private_data);
    }
}

impl AlsaRegistration {
    const VTABLE: bindings::snd_pcm_ops = bindings::snd_pcm_ops {
        open: Some(aop_pcm_open),
        close: Some(aop_pcm_close),
        prepare: Some(aop_pcm_prepare),
        trigger: Some(bindings::snd_dmaengine_pcm_trigger),
        pointer: Some(bindings::snd_dmaengine_pcm_pointer),
        ioctl: None,
        hw_params: Some(aop_hw_params),
        hw_free: None,
        sync_stop: None,
        get_time_info: None,
        fill_silence: None,
        copy: None,
        page: None,
        mmap: None,
        ack: None,
    };
    fn new(data: Arc<AopData>) -> Result<Self> {
        let mut this = AlsaRegistration {
            card: ptr::null_mut(),
            pcm: ptr::null_mut(),
        };
        let ret = unsafe {
            bindings::snd_card_new(
                data.dev.raw_device(),
                -1,
                ptr::null(),
                THIS_MODULE.as_ptr(),
                0,
                &mut this.card,
            )
        };
        if ret < 0 {
            dev_err!(data.dev, "Unable to allocate sound card");
            return Err(Error::from_errno(ret));
        }
        unsafe {
            let name = c_str!("aop_audio");
            copy_str((*this.card).driver.as_mut(), name.as_ref());
        }
        unsafe {
            let name = c_str!("aop_audio");
            copy_str((*this.card).shortname.as_mut(), name.as_ref());
        }
        unsafe {
            let name = c_str!("AOP Audio");
            copy_str((*this.card).longname.as_mut(), name.as_ref());
        }

        let pcm_name = c_str!("AOP Mic Input");
        let ret = unsafe {
            bindings::snd_pcm_new(this.card, pcm_name.as_ptr() as _, 0, 0, 1, &mut this.pcm)
        };
        if ret < 0 {
            dev_err!(data.dev, "Unable to allocate PCM device");
            return Err(Error::from_errno(ret));
        }

        unsafe {
            bindings::snd_pcm_set_ops(
                this.pcm,
                bindings::SNDRV_PCM_STREAM_CAPTURE as i32,
                &Self::VTABLE,
            );
        }

        unsafe {
            (*this.pcm).private_data = data.clone().into_foreign() as _;
            (*this.pcm).private_free = Some(aop_pcm_free_private);
            (*this.pcm).info_flags = 0;
            let name = c_str!("aop_audio");
            copy_str((*this.pcm).name.as_mut(), name.as_ref());
        }

        let ret = unsafe { bindings::snd_card_register(this.card) };
        if ret < 0 {
            dev_err!(data.dev, "Unable to register sound card");
            return Err(Error::from_errno(ret));
        }
        Ok(this)
    }
}

impl Drop for AlsaRegistration {
    fn drop(&mut self) {
        if self.card != ptr::null_mut() {
            unsafe {
                bindings::snd_card_free(self.card);
            }
        }
    }
}

unsafe impl Send for AlsaRegistration {}
unsafe impl Sync for AlsaRegistration {}

unsafe extern "C" fn aop_read_raw(
    dev: *mut bindings::iio_dev,
    chan: *const bindings::iio_chan_spec,
    val: *mut i32,
    _: *mut i32,
    mask: i64,
) -> i32 {
    let data = unsafe { Arc::<AopData>::borrow((*dev).priv_) };
    let ty = unsafe { (*chan).type_ };
    if mask != bindings::BINDINGS_IIO_CHAN_INFO_PROCESSED as i64
        && mask != bindings::BINDINGS_IIO_CHAN_INFO_RAW as i64
    {
        return EINVAL.to_errno();
    }
    let value = if ty == bindings::BINDINGS_IIO_LIGHT {
        data.lux_value.load(Ordering::Relaxed)
    } else if ty == bindings::BINDINGS_IIO_ANGL {
        data.lid_angle.load(Ordering::Relaxed)
    } else {
        return EINVAL.to_errno();
    };
    unsafe {
        *val = value as i32;
    }
    bindings::IIO_VAL_INT as i32
}

struct IIORegistration {
    dev: *mut bindings::iio_dev,
    spec: [bindings::iio_chan_spec; 2],
    vtable: bindings::iio_info,
    registered: bool,
}

impl IIORegistration {
    fn new(data: Arc<AopData>) -> Result<Box<Self>> {
        let mut this = Box::new(IIORegistration {
            dev: ptr::null_mut(),
            spec: [
                bindings::iio_chan_spec {
                    type_: bindings::BINDINGS_IIO_LIGHT,
                    __bindgen_anon_1: bindings::iio_chan_spec__bindgen_ty_1 {
                        scan_type: bindings::iio_scan_type {
                            sign: b'u' as _,
                            realbits: 32,
                            storagebits: 32,
                            ..Default::default()
                        },
                    },
                    info_mask_separate: 1 << bindings::BINDINGS_IIO_CHAN_INFO_PROCESSED,
                    ..Default::default()
                },
                bindings::iio_chan_spec {
                    type_: bindings::BINDINGS_IIO_ANGL,
                    __bindgen_anon_1: bindings::iio_chan_spec__bindgen_ty_1 {
                        scan_type: bindings::iio_scan_type {
                            sign: b'u' as _,
                            realbits: 32,
                            storagebits: 32,
                            ..Default::default()
                        },
                    },
                    info_mask_separate: 1 << bindings::BINDINGS_IIO_CHAN_INFO_RAW,
                    ..Default::default()
                },
            ],
            vtable: bindings::iio_info {
                read_raw: Some(aop_read_raw),
                ..Default::default()
            },
            registered: false,
        }, GFP_KERNEL)?;
        this.dev = unsafe { bindings::iio_device_alloc(data.dev.raw_device(), 0) };
        unsafe {
            (*this.dev).priv_ = data.clone().into_foreign() as _;
            (*this.dev).name = c_str!("aop-sensors").as_ptr() as _;
            (*this.dev).channels = this.spec.as_ptr();
            (*this.dev).num_channels = this.spec.len() as i32;
            (*this.dev).info = &this.vtable;
        }
        let ret = unsafe { bindings::__iio_device_register(this.dev, THIS_MODULE.as_ptr()) };
        if ret < 0 {
            dev_err!(data.dev, "Unable to register sound card");
            return Err(Error::from_errno(ret));
        }
        this.registered = true;
        Ok(this)
    }
}

impl Drop for IIORegistration {
    fn drop(&mut self) {
        if self.dev != ptr::null_mut() {
            unsafe {
                if self.registered {
                    bindings::iio_device_unregister(self.dev);
                }
                Arc::<AopData>::from_foreign((*self.dev).priv_);
                bindings::iio_device_free(self.dev);
            }
        }
    }
}

unsafe impl Send for IIORegistration {}
unsafe impl Sync for IIORegistration {}

struct AopRegistration {
    _alsa: AlsaRegistration,
    _iio: Box<IIORegistration>,
    data: Arc<AopData>,
}

impl AopRegistration {
    fn new(data: Arc<AopData>) -> Result<Self> {
        Ok(AopRegistration {
            _alsa: AlsaRegistration::new(data.clone())?,
            _iio: IIORegistration::new(data.clone())?,
            data,
        })
    }
}

impl DeviceRemoval for AopRegistration {
    fn device_remove(&self) {
        self.data.device_remove();
    }
}

kernel::define_of_id_table! {AOP_ID_TABLE, (), [
    (of::DeviceId::Compatible(b"apple,aop"), None)
]}

impl platform::Driver for AopDriver {
    type Data = Box<AopRegistration>;

    kernel::driver_of_id_table!(AOP_ID_TABLE);

    fn probe(pdev: &mut platform::Device, _info: Option<&()>) -> Result<Self::Data> {
        let dev = device::Device::from_dev(pdev);
        let data = AopData::new(pdev)?;
        let of = dev.of_node().ok_or(EIO)?;
        let alig = of.get_property(c_str!("apple,aop-alignment"))?;
        let aopt = of.get_property(c_str!("apple,aop-target"))?;
        data.patch_bootargs(&[
            (from_fourcc(b"EC0p"), 0x20000),
            (from_fourcc(b"nCal"), 0x0),
            (from_fourcc(b"alig"), alig),
            (from_fourcc(b"AOPt"), aopt),
        ])?;
        let rtkit = rtkit::RtKit::<AopData>::new(&dev, None, 0, data.clone())?;
        *data.rtkit.lock() = Some(rtkit);
        data.start_cpu();
        data.start()?;
        let reg = AopRegistration::new(data)?;
        Ok(Box::new(reg, GFP_KERNEL)?)
    }
}

kernel::module_of_id_table!(MOD_TABLE, AOP_ID_TABLE);

module_platform_driver! {
    type: AopDriver,
    name: "apple_aop",
    license: "Dual MIT/GPL",
}
