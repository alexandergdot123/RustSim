use half::f16;
use std::cell::UnsafeCell;
use std::marker::PhantomData;
use std::mem::MaybeUninit;
use std::ptr::{self, NonNull};
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::mpsc::Sender;

use crate::{CORES_IN_X_STACK, CoreLog};

pub const FP_PIPE_STAGES: usize = 4;
pub const DIV_LATENCY: usize = 32;
pub const DRAM_LATENCY_FAR: u64 = 300;
pub const DRAM_LATENCY_CLOSE: u64 = 150;
pub const REGS_PER_CONTEXT: usize = 16;
pub const SRAM_SIZE: usize = 48 * 1024;
pub const NUM_NOC_PIPES: usize = 32;
pub const HIGH_CAPACITY_NOC_PIPE_CNT: usize = 32;
pub const HIGH_CAPACITY_PIPE_SLOTS: usize = 4;
pub const OUTPUT_NOC_FIFO_CAPACITY: usize = 1;
pub const PRIORITIZE_X: bool = false;
pub const CORES_IN_X: u16 = 16;
pub const CORES_IN_Y: u16 = 16;
pub const DRAM_STACK_SIZE: usize = 1 << 29;
pub const DRAM_STACK_SIZE_LOG2: usize = 29;
pub const NOC_FIFO_SIZE: usize = 1;
pub const NOC_FIFO_LATENCY: usize = 1;
pub const DEBUG: bool = false;
pub const NOC_UTIL_EPOCH_LEN: usize = 250;
pub const CTX_CNT: usize = 16;
struct Inner<T> {
    buffer: Box<[UnsafeCell<MaybeUninit<T>>]>,
    capacity: usize,
    head: AtomicUsize, // producer writes
    tail: AtomicUsize, // consumer writes
}

unsafe impl<T: Send> Send for Inner<T> {}
unsafe impl<T: Send> Sync for Inner<T> {}

impl<T> Inner<T> {
    fn with_capacity(capacity: usize) -> Self {
        assert!(capacity > 0, "capacity must be non-zero");
        let mut v = Vec::with_capacity(capacity);
        for _ in 0..capacity {
            v.push(UnsafeCell::new(MaybeUninit::uninit()));
        }
        Self {
            buffer: v.into_boxed_slice(),
            capacity,
            head: AtomicUsize::new(0),
            tail: AtomicUsize::new(0),
        }
    }

    #[inline]
    fn capacity(&self) -> usize {
        self.capacity
    }

    #[inline]
    fn len(&self) -> usize {
        let h = self.head.load(Ordering::Acquire);
        let t = self.tail.load(Ordering::Acquire);
        h.wrapping_sub(t)
    }

    #[inline]
    fn is_empty(&self) -> bool {
        self.len() == 0
    }

    #[inline]
    fn is_full(&self) -> bool {
        self.len() == self.capacity
    }

    // Canonical SPSC ops used by both modes:

    #[inline]
    fn push(&self, value: T) -> Result<(), T> {
        let head = self.head.load(Ordering::Relaxed);
        let tail = self.tail.load(Ordering::Acquire);

        if head.wrapping_sub(tail) == self.capacity {
            return Err(value);
        }

        let idx = head % self.capacity;
        unsafe {
            (*self.buffer[idx].get()).write(value);
        }

        self.head.store(head.wrapping_add(1), Ordering::Release);
        Ok(())
    }

    #[inline]
    fn pop(&self) -> Option<T> {
        let tail = self.tail.load(Ordering::Relaxed);
        let head = self.head.load(Ordering::Acquire);

        if tail == head {
            return None;
        }

        let idx = tail % self.capacity;
        let value = unsafe { (*self.buffer[idx].get()).assume_init_read() };

        self.tail.store(tail.wrapping_add(1), Ordering::Release);
        Some(value)
    }

    #[inline]
    fn peek_ptr(&self) -> Option<NonNull<T>> {
        let tail = self.tail.load(Ordering::Relaxed);
        let head = self.head.load(Ordering::Acquire);

        if tail == head {
            return None;
        }

        let idx = tail % self.capacity;
        let p = unsafe { (*self.buffer[idx].get()).as_ptr() as *mut T };
        Some(NonNull::new(p).unwrap())
    }
}

impl<T> Drop for Inner<T> {
    fn drop(&mut self) {
        // drain remaining elements
        let mut t = self.tail.load(Ordering::Relaxed);
        let h = self.head.load(Ordering::Relaxed);

        while t != h {
            let idx = t % self.capacity;
            unsafe {
                ptr::drop_in_place((*self.buffer[idx].get()).as_mut_ptr());
            }
            t = t.wrapping_add(1);
        }
    }
}

// ===================== Peek guards =====================

pub struct PeekGuard<'a, T> {
    ptr: NonNull<T>,
    _borrow: PhantomData<&'a mut ()>, // ties to a mutable borrow of the owner handle
}

impl<'a, T> std::ops::Deref for PeekGuard<'a, T> {
    type Target = T;
    fn deref(&self) -> &T {
        unsafe { self.ptr.as_ref() }
    }
}

// ===================== Normal (single-thread) queue =====================

pub struct SpscQueue<T> {
    inner: Inner<T>,
}
impl<T: Clone> Clone for SpscQueue<T> {
    fn clone(&self) -> Self {
        let cap = self.inner.capacity();

        let mut new = SpscQueue::new(cap);

        // Snapshot current contents
        let len = self.inner.len();
        let tail = self.inner.tail.load(Ordering::Relaxed);

        for i in 0..len {
            let idx = (tail + i) % cap;
            let val = unsafe { (*self.inner.buffer[idx].get()).assume_init_ref().clone() };
            let _ = new.push(val);
        }

        new
    }
}
impl<T> SpscQueue<T> {
    pub fn new(capacity: usize) -> Self {
        Self {
            inner: Inner::with_capacity(capacity),
        }
    }

    #[inline]
    pub fn push(&mut self, value: T) -> Result<(), T> {
        // &mut self guarantees “normal use” (no concurrent producer/consumer)
        self.inner.push(value)
    }

    #[inline]
    pub fn pop(&mut self) -> Option<T> {
        self.inner.pop()
    }

    #[inline]
    pub fn peek<'a>(&'a mut self) -> Option<PeekGuard<'a, T>> {
        let ptr = self.inner.peek_ptr()?;
        Some(PeekGuard {
            ptr,
            _borrow: PhantomData,
        })
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.inner.len()
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.inner.is_empty()
    }

    #[inline]
    pub fn is_full(&self) -> bool {
        self.inner.is_full()
    }

    #[inline]
    pub fn capacity(&self) -> usize {
        self.inner.capacity()
    }

    /// Consume the queue and split into producer/consumer handles for two threads.
    pub fn split(self) -> (Feeder<T>, Eater<T>) {
        let arc = Arc::new(self.inner);
        (
            Feeder {
                q: arc.clone(),
            },
            Eater {
                q: arc,
            },
        )
    }
}

// ===================== Split endpoints =====================

pub struct Feeder<T> {
    q: Arc<Inner<T>>,
}

pub struct Eater<T> {
    q: Arc<Inner<T>>,
}

impl<T> Feeder<T> {
    #[inline]
    pub fn push(&mut self, value: T) -> Result<(), T> {
        self.q.push(value)
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.q.len()
    }

    #[inline]
    pub fn is_full(&self) -> bool {
        self.q.is_full()
    }

    #[inline]
    pub fn capacity(&self) -> usize {
        self.q.capacity()
    }
}

impl<T> Eater<T> {
    #[inline]
    pub fn pop(&mut self) -> Option<T> {
        self.q.pop()
    }

    #[inline]
    pub fn peek<'a>(&'a mut self) -> Option<PeekGuard<'a, T>> {
        let ptr = self.q.peek_ptr()?;
        Some(PeekGuard {
            ptr,
            _borrow: PhantomData,
        })
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.q.len()
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.q.is_empty()
    }

    #[inline]
    pub fn capacity(&self) -> usize {
        self.q.capacity()
    }
}

struct PipelineStage {
    cycle_to_read: u64,
    calculated_val: u32,
    register_index: usize,
}

pub struct Flit {
    cycle_to_read: u64,
    value_to_send: u32,
    mailbox_id: u16,
    destination_x: u16,
    destination_y: u16,
    origin_x: u16,
    origin_y: u16
}

pub struct BidirectionalNoc {
    eater: Eater<Flit>,
    feeder: Feeder<Flit>,
    latency: usize,
}
impl BidirectionalNoc {
    pub fn new(eater: Eater<Flit>, feeder: Feeder<Flit>, latency: usize) -> Self {
        Self { eater, feeder, latency }
    }
}


#[derive(PartialEq, Eq, Copy, Clone, Debug)]
pub enum Operation {
    // Integer ALU
    Add,
    Sub,
    And,
    Or,
    Xor,
    Sll,
    Srl,
    Sra,
    Mul,
    Div,
    Mod, 

    FPAdd,
    FPMul,
    FPSub,
    FPMac,
    FPEQ,
    FPLT,
    FPSetAccumulator,
    FPStoreAccumulator,
    FPMinMax,

    // Memory
    StoreByte,
    StoreHalf,
    StoreWord,
    LoadByteSigned,
    LoadByteUnsigned,
    LoadHalfSigned,
    LoadHalfUnsigned,
    LoadWord,

    // Control
    BranchEq,
    BranchNe,
    BranchLTE,
    BranchGT,
    Jump,

    BlockVal,
    NonBlockVal,
    Yield,
    GetThreadOwnership,
    SetCtx,
    RelinquishOwnership,
    ModifyInterruptEnable,
    SetMemoryBits,

    ReadSignedByteDram,
    ReadUnsignedByteDram,
    ReadSignedHalfDram,
    ReadUnsignedHalfDram,
    ReadWordDram,
    StoreWordDram,
    StoreHalfDram,
    StoreByteDram,
    AtomicAddDram,
    SendFlit,
}
#[derive(Copy, Clone)]
enum FpType {
    Fp32,
    Fp16,
    Fp8,
}
#[derive(Copy, Clone)]
struct DecodedInstruction {
    raw_instruction: u32,
    operation: Operation,
    fp_type: FpType,
    sr1: usize,
    sr2: usize,
    dr: usize,
    branch_hint: bool,
    imm_0: u32,
    imm_1: u32,
    is_imm: bool,
    pc: u16,
}
impl DecodedInstruction {
    pub fn dump(&self, context_in_progress: usize, reg_file: &Vec<u32>) {
        println!("DecodedInstruction {{");
        println!("  raw_instruction: 0x{:08X}", self.raw_instruction);
        println!("  operation: {:?}", self.operation);
        println!("  fp_type: unimplemented for now");
        println!("  sr1: {}", self.sr1);
        println!("  sr1_val: 0x{:08X} ({})", reg_file[context_in_progress * REGS_PER_CONTEXT + self.sr1], reg_file[context_in_progress * REGS_PER_CONTEXT + self.sr1]);
        println!("  sr2: {}", self.sr2);
        println!("  sr2_val: 0x{:08X} ({})", reg_file[context_in_progress * REGS_PER_CONTEXT + self.sr2], reg_file[context_in_progress * REGS_PER_CONTEXT + self.sr2]);
        println!("  dr: {}", self.dr);
        println!("  branch_hint: {}", self.branch_hint);
        println!("  imm_0: 0x{:08X} ({})", self.imm_0, self.imm_0);
        println!("  imm_1: 0x{:08X} ({})", self.imm_1, self.imm_1);
        println!("  is_imm: {}", self.is_imm);
        println!("  pc: 0x{:04X} ({})", self.pc, self.pc);
        println!("  context: {}", context_in_progress);
        println!("}}");
    }
}
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Fp8E4M3 {
    bits: u8,
}
impl Fp8E4M3 {
    pub fn from_bits(bits: u8) -> Self {
        Self { bits }
    }

    pub fn to_bits(self) -> u8 {
        self.bits
    }

    pub fn to_f32(self) -> f32 {
        let sign = if self.bits & 0x80 != 0 { -1.0 } else { 1.0 };
        let exp = (self.bits >> 3) & 0x0F;
        let mant = self.bits & 0x07;

        if exp == 0 {
            // subnormal
            sign * (mant as f32) * 2f32.powi(-6)
        } else if exp == 0x0F {
            // inf / NaN
            if mant == 0 {
                sign * f32::INFINITY
            } else {
                f32::NAN
            }
        } else {
            let e = (exp as i32) - 7;
            let m = 1.0 + (mant as f32) / 8.0;
            sign * m * 2f32.powi(e)
        }
    }
}
impl Fp8E4M3 {
    pub fn from_f32(x: f32) -> Self {
        if x.is_nan() {
            return Self { bits: 0x7F };
        }
        if x.is_infinite() {
            return Self {
                bits: if x.is_sign_negative() { 0xF8 } else { 0x78 },
            };
        }
        if x == 0.0 {
            return Self { bits: 0 };
        }

        let sign = if x.is_sign_negative() { 0x80 } else { 0 };
        let abs = x.abs();

        let exp = abs.log2().floor() as i32;
        let mant = abs / 2f32.powi(exp) - 1.0;

        let biased_exp = exp + 7;

        if biased_exp <= 0 {
            // subnormal
            let m = (abs * 2f32.powi(6)).round() as u8;
            return Self {
                bits: sign | (m & 0x07),
            };
        }

        if biased_exp >= 0x0F {
            // overflow → inf
            return Self { bits: sign | 0x78 };
        }

        let mant_bits = (mant * 8.0).round() as u8 & 0x07;

        Self {
            bits: sign | ((biased_exp as u8) << 3) | mant_bits,
        }
    }
}
#[inline]
pub fn fp8_add(a: u8, b: u8) -> u8 {
    let fa = Fp8E4M3::from_bits(a).to_f32();
    let fb = Fp8E4M3::from_bits(b).to_f32();
    Fp8E4M3::from_f32(fa + fb).to_bits()
}
#[inline]
pub fn fp8_sub(a: u8, b: u8) -> u8 {
    let fa = Fp8E4M3::from_bits(a).to_f32();
    let fb = Fp8E4M3::from_bits(b).to_f32();
    Fp8E4M3::from_f32(fa - fb).to_bits()
}
#[inline]
pub fn fp8_mul(a: u8, b: u8) -> u8 {
    let fa = Fp8E4M3::from_bits(a).to_f32();
    let fb = Fp8E4M3::from_bits(b).to_f32();
    Fp8E4M3::from_f32(fa * fb).to_bits()
}

fn decode_instruction(instruction_word: u32, pc: u16) -> DecodedInstruction {
    // Dummy implementation for illustration purposes
    let op_type = match instruction_word & 0x7F {
        0 => Operation::Add,
        1 => Operation::Sub,
        2 => Operation::And,
        3 => Operation::Or,
        4 => Operation::Xor,
        5 => Operation::Sll,
        6 => Operation::Srl,
        7 => Operation::Sra,
        8 => Operation::Mul,
        9 => Operation::Div,
        10 => Operation::FPAdd,
        11 => Operation::FPMul,
        12 => Operation::FPSub,
        13 => Operation::FPMac,
        14 => Operation::FPEQ,
        15 => Operation::FPLT,
        16 => Operation::FPSetAccumulator,
        17 => Operation::FPStoreAccumulator,
        18 => Operation::FPMinMax,
        19 => Operation::BranchEq,
        20 => Operation::BranchNe,
        21 => Operation::BranchLTE,
        22 => Operation::BranchGT,
        23 => Operation::Jump,
        24 => Operation::StoreByte,
        25 => Operation::StoreHalf,
        26 => Operation::StoreWord,
        27 => Operation::LoadByteSigned,
        28 => Operation::LoadByteUnsigned,
        29 => Operation::LoadHalfSigned,
        30 => Operation::LoadHalfUnsigned,
        31 => Operation::LoadWord,
        32 => Operation::BlockVal,
        33 => Operation::NonBlockVal,
        34 => Operation::Yield,
        35 => Operation::GetThreadOwnership,
        36 => Operation::SetCtx,
        37 => Operation::RelinquishOwnership,
        38 => Operation::ModifyInterruptEnable,
        39 => Operation::SetMemoryBits,
        40 => Operation::ReadSignedByteDram,
        41 => Operation::ReadUnsignedByteDram,
        42 => Operation::ReadSignedHalfDram,
        43 => Operation::ReadUnsignedHalfDram,
        44 => Operation::ReadWordDram,
        45 => Operation::StoreWordDram,
        46 => Operation::StoreHalfDram,
        47 => Operation::StoreByteDram,
        48 => Operation::AtomicAddDram,
        49 => Operation::SendFlit,
        50 => Operation::Mod,
        _ => panic!("UNKNOWN OPERATION"),
    };
    let fp_type = match (instruction_word >> 20) & 0x3 {
        0 => FpType::Fp32,
        1 => FpType::Fp16,
        2 => FpType::Fp8,
        _ => FpType::Fp32,
    };

    let dr_index = (instruction_word >> 7) as usize & 0xF;
    let sr1_index = (instruction_word >> 11) as usize & 0xF;
    let sr2_index = if instruction_word & 0x7F >= 19 && instruction_word & 0x7F < 23 {
        dr_index
    }
    else {
        ((instruction_word >> 16) & 0xF) as usize
    };
    let imm_0 = (instruction_word >> 16) & 0xFFFF;
    let imm_1 = (instruction_word >> 15) & 0x1;
    DecodedInstruction {
        raw_instruction: instruction_word,
        operation: op_type,
        fp_type: fp_type,
        sr1: sr1_index,
        sr2: sr2_index,
        dr: dr_index,
        branch_hint: imm_1 == 1,
        imm_0: imm_0,
        imm_1: imm_1,
        is_imm: imm_1 == 1,
        pc,
    }
}

fn num_source_registers(instr: &DecodedInstruction) -> u8 {
    use Operation::*;

    match instr.operation {
        Yield
        | GetThreadOwnership
        | RelinquishOwnership
        | NonBlockVal
        | ModifyInterruptEnable
        | SetCtx => 0,
        Jump | FPSetAccumulator | SetMemoryBits => 1,
        StoreByte | StoreHalf | StoreWord => {
            if instr.is_imm {
                1
            } else {
                2
            }
        }
        LoadByteSigned | LoadByteUnsigned | LoadHalfSigned | LoadHalfUnsigned | LoadWord => {
            if instr.is_imm { 0 } else { 1 }
        }

        Add | Sub | And | Or | Xor | Sll | Srl | Sra => {
            if instr.is_imm {
                1
            } else {
                2
            }
        }
        Mul | Div | Mod => 2,
        FPAdd | FPMul | FPSub | FPEQ | FPLT | FPMinMax => 2,
        FPMac => 2,
        FPStoreAccumulator => 0,
        BranchEq | BranchNe | BranchLTE | BranchGT => 2,
        BlockVal => 0,

        ReadSignedByteDram | ReadUnsignedByteDram | ReadSignedHalfDram | ReadUnsignedHalfDram
        | ReadWordDram => 1,
        StoreWordDram | StoreHalfDram | StoreByteDram | AtomicAddDram | SendFlit => 2,
    }
}

#[derive(Clone)]
pub struct LongDramOp {
    pub(crate) register_index: usize,
    pub(crate) calculated_val: u32,
    pub(crate) core_id: u32,
}

#[derive(Clone)]
pub struct LongDramRequest {
    pub(crate)register_index: usize,
    pub(crate)address: usize,
    pub(crate)op: Operation,
    pub(crate)value_to_write: u32,
    pub(crate)core_id: u32,
    pub(crate)origin_stack: usize,
}
pub struct Core {
    core_id: u32,
    top_bits_dram_stack: usize,

    x_dim: u16,
    y_dim: u16,
    fetch_pc: Option<u16>,
    fp_pipe: SpscQueue<PipelineStage>,
    dram_long_response: Option<Eater<LongDramOp>>, //input from other dram stack
    dram_long_request: Option<Vec<Sender<LongDramRequest>>>, //output to other dram stacks
    fp_accumulation: Vec<u32>,
    div_seq: SpscQueue<PipelineStage>,
    context_cnt: usize,
    dram_long_queue: SpscQueue<PipelineStage>,
    dram_short_queue: SpscQueue<PipelineStage>,
    context_in_progress: usize,
    register_file: Vec<u32>,
    register_ready: Vec<bool>,
    up_noc: Option<BidirectionalNoc>,    //upwards core
    down_noc: Option<BidirectionalNoc>,  //downwards core
    left_noc: Option<BidirectionalNoc>,  //leftwards core
    right_noc: Option<BidirectionalNoc>, //rightwards core
    up_noc_util: Vec<usize>,
    up_noc_congestion: Vec<usize>,
    down_noc_util: Vec<usize>,
    down_noc_congestion: Vec<usize>,
    left_noc_util: Vec<usize>,
    left_noc_congestion: Vec<usize>,
    right_noc_util: Vec<usize>,
    right_noc_congestion: Vec<usize>,
    mailbox_congestion: Vec<usize>,
    core_busy: Vec<usize>,
    dram_bytes_read_close: usize,
    dram_bytes_read_far: usize,
    dram_bytes_wrote_close: usize,
    dram_bytes_wrote_far: usize,
    flits_sent: usize,
    flits_received: usize,
    flit_sent_manhattan_distance_traversed: usize,
    flit_received_manhattan_distance_traversed: usize,
    ctx_ownership: i8,
    runnable: Vec<bool>,
    sram: [u8; SRAM_SIZE],
    memory_bits: [u32; REGS_PER_CONTEXT],

    pc: [u16; REGS_PER_CONTEXT],

    output_noc_send: SpscQueue<Flit>,
    noc_recv_fifo: Vec<SpscQueue<u32>>,
    interrupt_enable: Vec<bool>,

    fetched_value: Option<u32>,

    decoded_instruction: Option<DecodedInstruction>,
    cycle: u64,
}
impl Core {
    pub fn new(
        id: u32,
        x_dim: u16,
        y_dim: u16,
        dram_top_bits: usize,
    ) -> Self {
        let init_time_mem: [u32; 12] = [0x00008002,   //PC = 0x00000000
                0x00000027,   //PC = 0x00000004
                0x000000AC,   //PC = 0x00000008
                0x00048100,   //PC = 0x0000000C
                0x002C8180,   //PC = 0x00000010
                0x0000122C,   //PC = 0x00000014
                0x0003201A,   //PC = 0x00000018
                0x00049100,   //PC = 0x0000001C
                0x00049980,   //PC = 0x00000020
                0x00018000,   //PC = 0x00000024
                0x00148816,   //PC = 0x00000028
                0xDEADBEEF];  //PC = 0x0000002C
        let mut sram_array = [0u8; SRAM_SIZE];
        for i in 0..init_time_mem.len() {
            let word = init_time_mem[i];
            let base_address = (i * 4) as u16;
            sram_array[base_address as usize] = (word & 0xFF) as u8;
            sram_array[base_address as usize + 1] = ((word & 0xFF00) >> 8) as u8;
            sram_array[base_address as usize + 2] = ((word & 0xFF0000) >> 16) as u8;
            sram_array[base_address as usize + 3] = ((word & 0xFF000000) >> 24) as u8;
        };
        let mut regfile = vec![0u32; CTX_CNT * REGS_PER_CONTEXT];
        for i in 0..CTX_CNT {
            regfile[i * REGS_PER_CONTEXT + REGS_PER_CONTEXT - 1] = (id * REGS_PER_CONTEXT as u32 + i as u32) as u32;
        }
        Self {
            core_id: id,
            x_dim,
            y_dim,
            fetch_pc: None,
            fp_pipe: SpscQueue::<PipelineStage>::new(FP_PIPE_STAGES),
            dram_long_queue: SpscQueue::<PipelineStage>::new(DRAM_LATENCY_FAR as usize),
            dram_short_queue: SpscQueue::<PipelineStage>::new(DRAM_LATENCY_CLOSE as usize),
            context_cnt: 1,
            ctx_ownership: 0,
            context_in_progress: 0,
            fp_accumulation: vec![(0.0f32).to_bits();CTX_CNT],
            register_file: regfile,
            register_ready: vec![true; CTX_CNT * REGS_PER_CONTEXT],
            memory_bits: [0u32; CTX_CNT],
            up_noc: None,
            down_noc: None,
            left_noc: None,
            right_noc: None,  
            runnable: vec![false; CTX_CNT],
            sram: sram_array,
            pc: [0u16; CTX_CNT],

            noc_recv_fifo: {
                let mut noc_input_pipes = vec![];
                for _ in 0..HIGH_CAPACITY_NOC_PIPE_CNT {
                    noc_input_pipes.push(SpscQueue::<u32>::new(HIGH_CAPACITY_PIPE_SLOTS));
                }
                for _ in HIGH_CAPACITY_NOC_PIPE_CNT..NUM_NOC_PIPES {
                    noc_input_pipes.push(SpscQueue::<u32>::new(1));
                }

                noc_input_pipes
            },
            interrupt_enable: vec![false; NUM_NOC_PIPES],
            div_seq: SpscQueue::<PipelineStage>::new(1),
            fetched_value: None,
            decoded_instruction: None,
            cycle: 0,
            output_noc_send: SpscQueue::<Flit>::new(OUTPUT_NOC_FIFO_CAPACITY),
            top_bits_dram_stack: dram_top_bits,
            dram_long_request: None,
            dram_long_response: None,
            dram_bytes_read_close: 0,
            dram_bytes_read_far: 0,
            dram_bytes_wrote_close: 0,
            dram_bytes_wrote_far: 0,
            flits_sent: 0,
            flit_sent_manhattan_distance_traversed: 0,
            flits_received: 0,
            flit_received_manhattan_distance_traversed: 0,
            up_noc_util: vec![],
            up_noc_congestion: vec![],
            down_noc_util: vec![],
            down_noc_congestion: vec![],
            left_noc_util: vec![],
            left_noc_congestion: vec![],
            right_noc_util: vec![],
            right_noc_congestion: vec![],
            mailbox_congestion: vec![],
            core_busy: vec![],
        }
        
    }
    fn dump_instruction(&self, inst: &DecodedInstruction){
        inst.dump(self.context_in_progress, &self.register_file);
    }
    pub fn get_log(&self) -> CoreLog {
        CoreLog {
            up_noc_util: self.up_noc_util.clone(),
            up_noc_congestion: self.up_noc_congestion.clone(),
            down_noc_util: self.down_noc_util.clone(),
            down_noc_congestion: self.down_noc_congestion.clone(),
            left_noc_util: self.left_noc_util.clone(),
            left_noc_congestion: self.left_noc_congestion.clone(),
            right_noc_util: self.right_noc_util.clone(),
            right_noc_congestion: self.right_noc_congestion.clone(),
            mailbox_congestion: self.mailbox_congestion.clone(),
            core_busy: self.core_busy.clone(),

            dram_bytes_read_close: self.dram_bytes_read_close,
            dram_bytes_read_far: self.dram_bytes_read_far,
            dram_bytes_wrote_close: self.dram_bytes_wrote_close,
            dram_bytes_wrote_far: self.dram_bytes_wrote_far,

            flits_sent: self.flits_sent,
            flits_received: self.flits_received,
            flit_sent_manhattan_distance_traversed:
                self.flit_sent_manhattan_distance_traversed,
            flit_received_manhattan_distance_traversed:
                self.flit_received_manhattan_distance_traversed,
        }
    }
    pub fn give_far_dram(&mut self, dram_vec: Vec<Sender<LongDramRequest>>) {
        self.dram_long_request = Some(dram_vec);
    }
    pub fn give_far_dram_response(&mut self, dram_vec: Eater<LongDramOp>) {
        self.dram_long_response = Some(dram_vec);
    }
    pub fn give_up_noc(&mut self, up_noc: BidirectionalNoc) {
        self.up_noc = Some(up_noc);
    }
    pub fn give_down_noc(&mut self, down_noc: BidirectionalNoc) {
        self.down_noc = Some(down_noc);
    }
    pub fn give_left_noc(&mut self, left_noc: BidirectionalNoc) {
        self.left_noc = Some(left_noc);
    }
    pub fn give_right_noc(&mut self, right_noc: BidirectionalNoc) {
        self.right_noc = Some(right_noc);
    }
    pub fn get_stack(&self)->usize{
        self.top_bits_dram_stack
    }
    pub fn get_core_id(&self) -> u32 {
        self.core_id
    }
    pub fn get_x_index(&self) -> usize {
        self.x_dim as usize
    }
    pub fn get_y_index(&self) -> usize {
        self.y_dim as usize
    }
    pub fn get_local_read(&self) -> usize {
        self.dram_bytes_read_close
    }
    pub fn get_local_write(&self) -> usize {
        self.dram_bytes_wrote_close
    }
    pub fn get_foreign_read(&self) -> usize {
        self.dram_bytes_read_far
    }
    pub fn get_foreign_write(&self) -> usize {
        self.dram_bytes_wrote_far
    }
    pub fn write_to_sram(&mut self, mut base_address: u16, data: &Vec<u32>, words: usize) {
        assert!(base_address as usize + words * 4 <= SRAM_SIZE, "SRAM WRITE OUT OF BOUNDS!");
        assert!(data.len() >= words, "SRAM WRITE SIZE MISMATCH");
        for word_index in 0..words {
            self.write_sram_word(data[word_index], &base_address);
            base_address += 4;
        }
    }
    fn write_sram_byte(&mut self, byte: u8, address: &u16) {
        self.sram[*address as usize] = byte;
    }
    fn write_sram_half(&mut self, half: u16, address: &u16) {
        assert!(*address & 0x1 == 0, "HALF NOT ALIGNED!");
        self.sram[*address as usize] = (half & 0xFF) as u8;
        self.sram[*address as usize + 1] = ((half & 0xFF00) >> 8) as u8;
    }
    fn write_sram_word(&mut self, word: u32, address: &u16) {
        assert!(*address & 0x3 == 0, "WORD NOT ALIGNED!");
        self.sram[*address as usize] = (word & 0xFF) as u8;
        self.sram[*address as usize + 1] = ((word & 0xFF00) >> 8) as u8;
        self.sram[*address as usize + 2] = ((word & 0xFF0000) >> 16) as u8;
        self.sram[*address as usize + 3] = ((word & 0xFF000000) >> 24) as u8;
    }
    fn read_sram_byte_unsigned(&self, address: &u16) -> u32 {
        self.sram[*address as usize] as u32
    }
    fn read_sram_byte_signed(&self, address: &u16) -> u32 {
        ((self.sram[*address as usize] as i8) as i32) as u32
    }
    fn read_sram_half_unsigned(&self, address: &u16) -> u32 {
        assert!(*address & 0x1 == 0, "HALF NOT ALIGNED!");
        let low_half = self.sram[*address as usize] as u32;
        let high_half = (self.sram[*address as usize + 1] as u32) << 8;
        low_half | high_half
    }
    fn read_sram_half_signed(&self, address: &u16) -> u32 {
        assert!(*address & 0x1 == 0, "HALF NOT ALIGNED!");
        let lo = self.sram[*address as usize];
        let hi = self.sram[*address as usize + 1];

        let val = u16::from_le_bytes([lo, hi]) as i16;
        val as i32 as u32
    }
    fn read_sram_word(&self, address: &u16) -> u32 {
        assert!(address & 0x3 == 0, "WORD NOT ALIGNED!");
        let val = u32::from_le_bytes([
            self.sram[*address as usize],
            self.sram[*address as usize + 1],
            self.sram[*address as usize + 2],
            self.sram[*address as usize + 3],
        ]);
        val
    }
    fn cycle_noc(&mut self) {
        if self.output_noc_send.peek().is_some() {
            let flit_to_send = self.output_noc_send.peek().unwrap();
            let go_x = if PRIORITIZE_X {
                flit_to_send.destination_x != self.x_dim
            } else {
                flit_to_send.destination_y == self.y_dim
            };
            if go_x {
                if flit_to_send.destination_x > self.x_dim {
                    while self.right_noc_util.len() <= self.cycle as usize/NOC_UTIL_EPOCH_LEN {
                        self.right_noc_util.push(0);
                    }
                    self.right_noc_util[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                    if let Some(right_noc) = &mut self.right_noc {
                        if !right_noc.feeder.is_full() {
                            let mut flit_to_transmit = self.output_noc_send.pop().unwrap();
                            flit_to_transmit.cycle_to_read = self.cycle + right_noc.latency as u64;
                            let _ = right_noc.feeder.push(flit_to_transmit);
                        }
                    }
                    else{
                        while self.cycle as usize/NOC_UTIL_EPOCH_LEN >= self.right_noc_congestion.len(){
                            self.right_noc_congestion.push(0);
                        }
                        self.right_noc_congestion[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                    }
                } else {
                    if let Some(left_noc) = &mut self.left_noc {
                        while self.left_noc_util.len() <= self.cycle as usize/NOC_UTIL_EPOCH_LEN {
                            self.left_noc_util.push(0);
                        }
                        self.left_noc_util[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                        if !left_noc.feeder.is_full() {
                            let mut flit_to_transmit = self.output_noc_send.pop().unwrap();
                            flit_to_transmit.cycle_to_read = self.cycle + left_noc.latency as u64;
                            let _ = left_noc.feeder.push(flit_to_transmit);
                        }
                        else{
                            while self.cycle as usize/NOC_UTIL_EPOCH_LEN >= self.left_noc_congestion.len(){
                                self.left_noc_congestion.push(0);
                            }
                            self.left_noc_congestion[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                        }
                    }
                }
            } else {
                if flit_to_send.destination_y > self.y_dim {
                    if let Some(down_noc) = &mut self.down_noc {
                        while self.down_noc_util.len() <= self.cycle as usize/NOC_UTIL_EPOCH_LEN {
                            self.down_noc_util.push(0);
                        }
                        self.down_noc_util[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                        if !down_noc.feeder.is_full() {
                            let mut flit_to_transmit = self.output_noc_send.pop().unwrap();
                            flit_to_transmit.cycle_to_read = self.cycle + down_noc.latency as u64;
                            let _ = down_noc.feeder.push(flit_to_transmit);
                        }
                        else{
                            while self.cycle as usize/NOC_UTIL_EPOCH_LEN >= self.down_noc_congestion.len(){
                                self.down_noc_congestion.push(0);
                            }
                            self.down_noc_congestion[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                        }
                    }
                } else {
                    if let Some(up_noc) = &mut self.up_noc {
                        while self.up_noc_util.len() <= self.cycle as usize/NOC_UTIL_EPOCH_LEN {
                            self.up_noc_util.push(0);
                        }
                        self.up_noc_util[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                        if !up_noc.feeder.is_full() {
                            let mut flit_to_transmit = self.output_noc_send.pop().unwrap();
                            flit_to_transmit.cycle_to_read = self.cycle + up_noc.latency as u64;
                            let _ = up_noc.feeder.push(flit_to_transmit);
                        }
                        else{
                            while self.cycle as usize/NOC_UTIL_EPOCH_LEN >= self.up_noc_congestion.len(){
                                self.up_noc_congestion.push(0);
                            }
                            self.up_noc_congestion[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                        }
                    }
                }
            }
        }

        if let Some(left_noc) = self.left_noc.as_mut() {
            if !left_noc.eater.is_empty() {
                if let Some(flit_received) = left_noc.eater.peek() {
                    if flit_received.cycle_to_read <= self.cycle {
                        let go_x = if PRIORITIZE_X {
                            flit_received.destination_x != self.x_dim
                        } else {
                            flit_received.destination_y == self.y_dim
                        };
                        if flit_received.destination_x == self.x_dim
                            && flit_received.destination_y == self.y_dim
                        {
                            if !self.noc_recv_fifo[flit_received.mailbox_id as usize].is_full() {
                                let flit_to_process = left_noc.eater.pop().unwrap();
                                let _ = self.noc_recv_fifo[flit_to_process.mailbox_id as usize]
                                    .push(flit_to_process.value_to_send);
                                self.flits_received += 1;
                                self.flit_received_manhattan_distance_traversed += (flit_to_process.origin_x as i64 - flit_to_process.destination_x as i64).abs() as usize 
                                    + (flit_to_process.origin_y as i64 - flit_to_process.destination_y as i64).abs() as usize;
                                // println!("Core {} received flit at cycle {}", self.core_id, self.cycle);
                            }
                            else{
                                while self.cycle as usize/NOC_UTIL_EPOCH_LEN >= self.mailbox_congestion.len(){
                                    self.mailbox_congestion.push(0);
                                }
                                self.mailbox_congestion[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                            }
                        } else if go_x {
                            if flit_received.destination_x > self.x_dim {
                                if let Some(right_noc) = &mut self.right_noc {
                                    while self.right_noc_util.len() <= self.cycle as usize/NOC_UTIL_EPOCH_LEN {
                                        self.right_noc_util.push(0);
                                    }
                                    self.right_noc_util[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                                    if !right_noc.feeder.is_full() {
                                        let mut flit_to_transmit = left_noc.eater.pop().unwrap();
                                        flit_to_transmit.cycle_to_read =
                                            self.cycle + right_noc.latency as u64;
                                        let _ = right_noc.feeder.push(flit_to_transmit);
                                    }
                                    else{
                                        while self.right_noc_congestion.len() <= self.cycle as usize/NOC_UTIL_EPOCH_LEN {
                                            self.right_noc_congestion.push(0);
                                        }
                                        self.right_noc_congestion[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                                    }
                                }
                            } else {
                                if let Some(left_noc) = &mut self.left_noc {
                                    while self.left_noc_util.len() <= self.cycle as usize/NOC_UTIL_EPOCH_LEN {
                                        self.left_noc_util.push(0);
                                    }
                                    self.left_noc_util[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                                    if !left_noc.feeder.is_full() {
                                        let mut flit_to_transmit = left_noc.eater.pop().unwrap();
                                        flit_to_transmit.cycle_to_read =
                                            self.cycle + left_noc.latency as u64;
                                        let _ = left_noc.feeder.push(flit_to_transmit);
                                    }
                                    else{
                                        while self.left_noc_congestion.len() <= self.cycle as usize/NOC_UTIL_EPOCH_LEN {
                                            self.left_noc_congestion.push(0);
                                        }
                                        self.left_noc_congestion[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                                    }
                                }
                            }
                        } else {
                            if flit_received.destination_y > self.y_dim {
                                if let Some(down_noc) = &mut self.down_noc {
                                    while self.down_noc_util.len() <= self.cycle as usize/NOC_UTIL_EPOCH_LEN {
                                        self.down_noc_util.push(0);
                                    }
                                    self.down_noc_util[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                                    if !down_noc.feeder.is_full() {
                                        let mut flit_to_transmit = left_noc.eater.pop().unwrap();
                                        flit_to_transmit.cycle_to_read =
                                            self.cycle + down_noc.latency as u64;
                                        let _ = down_noc.feeder.push(flit_to_transmit);
                                    }
                                    else{
                                        while self.down_noc_congestion.len() <= self.cycle as usize/NOC_UTIL_EPOCH_LEN {
                                            self.down_noc_congestion.push(0);
                                        }
                                        self.down_noc_congestion[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                                    }
                                }
                            } else {
                                if let Some(up_noc) = &mut self.up_noc {
                                    while self.up_noc_util.len() <= self.cycle as usize/NOC_UTIL_EPOCH_LEN {
                                        self.up_noc_util.push(0);
                                    }
                                    self.up_noc_util[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                                    if !up_noc.feeder.is_full() {
                                        let mut flit_to_transmit = left_noc.eater.pop().unwrap();
                                        flit_to_transmit.cycle_to_read =
                                            self.cycle + up_noc.latency as u64;
                                        let _ = up_noc.feeder.push(flit_to_transmit);
                                    }
                                    else{
                                        while self.up_noc_congestion.len() <= self.cycle as usize/NOC_UTIL_EPOCH_LEN {
                                            self.up_noc_congestion.push(0);
                                        }
                                        self.up_noc_congestion[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        if let Some(right_noc) = self.right_noc.as_mut() {
            if !right_noc.eater.is_empty() {
                if let Some(flit_received) = right_noc.eater.peek() {
                    if flit_received.cycle_to_read <= self.cycle {
                        let go_x = if PRIORITIZE_X {
                            flit_received.destination_x != self.x_dim
                        } else {
                            flit_received.destination_y == self.y_dim
                        };
                        if flit_received.destination_x == self.x_dim
                            && flit_received.destination_y == self.y_dim {

                            if !self.noc_recv_fifo[flit_received.mailbox_id as usize].is_full() {
                                let flit_to_process = right_noc.eater.pop().unwrap();
                                let _ = self.noc_recv_fifo[flit_to_process.mailbox_id as usize]
                                    .push(flit_to_process.value_to_send);
                                self.flits_received += 1;
                                self.flit_received_manhattan_distance_traversed += (flit_to_process.origin_x as i64 - flit_to_process.destination_x as i64).abs() as usize 
                                    + (flit_to_process.origin_y as i64 - flit_to_process.destination_y as i64).abs() as usize;
                            }
                            else{
                                while self.cycle as usize/NOC_UTIL_EPOCH_LEN >= self.mailbox_congestion.len(){
                                    self.mailbox_congestion.push(0);
                                }
                                self.mailbox_congestion[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                            }
                        } else if go_x {
                            if flit_received.destination_x > self.x_dim {
                                if let Some(right_noc) = &mut self.right_noc {
                                    while self.right_noc_util.len() <= self.cycle as usize/NOC_UTIL_EPOCH_LEN {
                                        self.right_noc_util.push(0);
                                    }
                                    self.right_noc_util[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                                    if !right_noc.feeder.is_full() {
                                        let mut flit_to_transmit = right_noc.eater.pop().unwrap();
                                        flit_to_transmit.cycle_to_read =
                                            self.cycle + right_noc.latency as u64;
                                        let _ = right_noc.feeder.push(flit_to_transmit);
                                    }
                                    else{
                                        while self.right_noc_congestion.len() <= self.cycle as usize/NOC_UTIL_EPOCH_LEN {
                                            self.right_noc_congestion.push(0);
                                        }
                                        self.right_noc_congestion[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                                    }
                                }
                            } else {
                                if let Some(left_noc) = &mut self.left_noc {
                                    while self.left_noc_util.len() <= self.cycle as usize/NOC_UTIL_EPOCH_LEN {
                                        self.left_noc_util.push(0);
                                    }
                                    self.left_noc_util[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                                    if !left_noc.feeder.is_full() {
                                        let mut flit_to_transmit = right_noc.eater.pop().unwrap();
                                        flit_to_transmit.cycle_to_read =
                                            self.cycle + left_noc.latency as u64;
                                        let _ = left_noc.feeder.push(flit_to_transmit);
                                    }
                                    else{
                                        while self.left_noc_congestion.len() <= self.cycle as usize/NOC_UTIL_EPOCH_LEN {
                                            self.left_noc_congestion.push(0);
                                        }
                                        self.left_noc_congestion[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                                    }
                                }
                            }
                        } else {
                            if flit_received.destination_y > self.y_dim {
                                if let Some(down_noc) = &mut self.down_noc {
                                    while self.down_noc_util.len() <= self.cycle as usize/NOC_UTIL_EPOCH_LEN {
                                        self.down_noc_util.push(0);
                                    }
                                    self.down_noc_util[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                                    if !down_noc.feeder.is_full() {
                                        let mut flit_to_transmit = right_noc.eater.pop().unwrap();
                                        flit_to_transmit.cycle_to_read =
                                            self.cycle + down_noc.latency as u64;
                                        let _ = down_noc.feeder.push(flit_to_transmit);
                                    }
                                    else{
                                        while self.down_noc_congestion.len() <= self.cycle as usize/NOC_UTIL_EPOCH_LEN {
                                            self.down_noc_congestion.push(0);
                                        }
                                        self.down_noc_congestion[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                                    }
                                }
                            } else {
                                if let Some(up_noc) = &mut self.up_noc {
                                    while self.up_noc_util.len() <= self.cycle as usize/NOC_UTIL_EPOCH_LEN {
                                        self.up_noc_util.push(0);
                                    }
                                    self.up_noc_util[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                                    if !up_noc.feeder.is_full() {
                                        let mut flit_to_transmit = right_noc.eater.pop().unwrap();
                                        flit_to_transmit.cycle_to_read =
                                            self.cycle + up_noc.latency as u64;
                                        let _ = up_noc.feeder.push(flit_to_transmit);
                                    }
                                    else{
                                        while self.up_noc_congestion.len() <= self.cycle as usize/NOC_UTIL_EPOCH_LEN {
                                            self.up_noc_congestion.push(0);
                                        }
                                        self.up_noc_congestion[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        if let Some(up_noc) = self.up_noc.as_mut() {
            if !up_noc.eater.is_empty() {
                if let Some(flit_received) = up_noc.eater.peek() {
                    if flit_received.cycle_to_read <= self.cycle {
                        let go_x = if PRIORITIZE_X {
                            flit_received.destination_x != self.x_dim
                        } else {
                            flit_received.destination_y == self.y_dim
                        };
                        if flit_received.destination_x == self.x_dim
                            && flit_received.destination_y == self.y_dim
                        {
                            if !self.noc_recv_fifo[flit_received.mailbox_id as usize].is_full() {
                                let flit_to_process = up_noc.eater.pop().unwrap();
                                let _ = self.noc_recv_fifo[flit_to_process.mailbox_id as usize]
                                    .push(flit_to_process.value_to_send);
                                self.flits_received += 1;
                                self.flit_received_manhattan_distance_traversed += (flit_to_process.origin_x as i64 - flit_to_process.destination_x as i64).abs() as usize 
                                    + (flit_to_process.origin_y as i64 - flit_to_process.destination_y as i64).abs() as usize;
                            }
                            else{
                                while self.cycle as usize/NOC_UTIL_EPOCH_LEN >= self.mailbox_congestion.len(){
                                    self.mailbox_congestion.push(0);
                                }
                                self.mailbox_congestion[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                            }
                        } else if go_x {
                            if flit_received.destination_x > self.x_dim {
                                if let Some(right_noc) = &mut self.right_noc {
                                    while self.right_noc_util.len() <= self.cycle as usize/NOC_UTIL_EPOCH_LEN {
                                        self.right_noc_util.push(0);
                                    }
                                    self.right_noc_util[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                                    if !right_noc.feeder.is_full() {
                                        let mut flit_to_transmit = up_noc.eater.pop().unwrap();
                                        flit_to_transmit.cycle_to_read =
                                            self.cycle + right_noc.latency as u64;
                                        let _ = right_noc.feeder.push(flit_to_transmit);
                                    }
                                    else{
                                        while self.right_noc_congestion.len() <= self.cycle as usize/NOC_UTIL_EPOCH_LEN {
                                            self.right_noc_congestion.push(0);
                                        }
                                        self.right_noc_congestion[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                                    }
                                }
                            } else {
                                if let Some(left_noc) = &mut self.left_noc {
                                    while self.left_noc_util.len() <= self.cycle as usize/NOC_UTIL_EPOCH_LEN {
                                        self.left_noc_util.push(0);
                                    }
                                    self.left_noc_util[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                                    if !left_noc.feeder.is_full() {
                                        let mut flit_to_transmit = up_noc.eater.pop().unwrap();
                                        flit_to_transmit.cycle_to_read =
                                            self.cycle + left_noc.latency as u64;
                                        let _ = left_noc.feeder.push(flit_to_transmit);
                                    }
                                    else{
                                        while self.left_noc_congestion.len() <= self.cycle as usize/NOC_UTIL_EPOCH_LEN {
                                            self.left_noc_congestion.push(0);
                                        }
                                        self.left_noc_congestion[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                                    }
                                }
                            }
                        } else {
                            if flit_received.destination_y > self.y_dim {
                                if let Some(down_noc) = &mut self.down_noc {
                                    while self.down_noc_util.len() <= self.cycle as usize/NOC_UTIL_EPOCH_LEN {
                                        self.down_noc_util.push(0);
                                    }
                                    self.down_noc_util[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                                    if !down_noc.feeder.is_full() {
                                        let mut flit_to_transmit = up_noc.eater.pop().unwrap();
                                        flit_to_transmit.cycle_to_read =
                                            self.cycle + down_noc.latency as u64;
                                        let _ = down_noc.feeder.push(flit_to_transmit);
                                    }
                                    else{
                                        while self.down_noc_congestion.len() <= self.cycle as usize/NOC_UTIL_EPOCH_LEN {
                                            self.down_noc_congestion.push(0);
                                        }
                                        self.down_noc_congestion[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                                    }
                                }
                            } else {
                                if let Some(up_noc) = &mut self.up_noc {
                                    while self.up_noc_util.len() <= self.cycle as usize/NOC_UTIL_EPOCH_LEN {
                                        self.up_noc_util.push(0);
                                    }
                                    self.up_noc_util[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                                    if !up_noc.feeder.is_full() {
                                        let mut flit_to_transmit = up_noc.eater.pop().unwrap();
                                        flit_to_transmit.cycle_to_read =
                                            self.cycle + up_noc.latency as u64;
                                        let _ = up_noc.feeder.push(flit_to_transmit);
                                    }
                                    else{
                                        while self.up_noc_congestion.len() <= self.cycle as usize/NOC_UTIL_EPOCH_LEN {
                                            self.up_noc_congestion.push(0);
                                        }
                                        self.up_noc_congestion[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        if let Some(down_noc) = self.down_noc.as_mut() {
            if !down_noc.eater.is_empty() {
                if let Some(flit_received) = down_noc.eater.peek() {
                    if flit_received.cycle_to_read <= self.cycle {
                        let go_x = if PRIORITIZE_X {
                            flit_received.destination_x != self.x_dim
                        } else {
                            flit_received.destination_y == self.y_dim
                        };
                        if flit_received.destination_x == self.x_dim
                            && flit_received.destination_y == self.y_dim
                        {
                            if !self.noc_recv_fifo[flit_received.mailbox_id as usize].is_full() {
                                let flit_to_process = down_noc.eater.pop().unwrap();
                                let _ = self.noc_recv_fifo[flit_to_process.mailbox_id as usize]
                                    .push(flit_to_process.value_to_send);
                                self.flits_received += 1;
                                self.flit_received_manhattan_distance_traversed += (flit_to_process.origin_x as i64 - flit_to_process.destination_x as i64).abs() as usize 
                                    + (flit_to_process.origin_y as i64 - flit_to_process.destination_y as i64).abs() as usize;
                            }
                            else{
                                while self.cycle as usize/NOC_UTIL_EPOCH_LEN >= self.mailbox_congestion.len(){
                                    self.mailbox_congestion.push(0);
                                }
                                self.mailbox_congestion[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                            }
                        } else if go_x {
                            if flit_received.destination_x > self.x_dim {
                                if let Some(right_noc) = &mut self.right_noc {
                                    while self.right_noc_util.len() <= self.cycle as usize/NOC_UTIL_EPOCH_LEN {
                                        self.right_noc_util.push(0);
                                    }
                                    self.right_noc_util[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                                    if !right_noc.feeder.is_full() {
                                        let mut flit_to_transmit = down_noc.eater.pop().unwrap();
                                        flit_to_transmit.cycle_to_read =
                                            self.cycle + right_noc.latency as u64;
                                        let _ = right_noc.feeder.push(flit_to_transmit);
                                    }
                                    else{
                                        while self.right_noc_congestion.len() <= self.cycle as usize/NOC_UTIL_EPOCH_LEN {
                                            self.right_noc_congestion.push(0);
                                        }
                                        self.right_noc_congestion[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                                    }
                                }
                            } else {
                                if let Some(left_noc) = &mut self.left_noc {
                                    while self.left_noc_util.len() <= self.cycle as usize/NOC_UTIL_EPOCH_LEN {
                                        self.left_noc_util.push(0);
                                    }
                                    self.left_noc_util[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                                    if !left_noc.feeder.is_full() {
                                        let mut flit_to_transmit = down_noc.eater.pop().unwrap();
                                        flit_to_transmit.cycle_to_read =
                                            self.cycle + left_noc.latency as u64;
                                        let _ = left_noc.feeder.push(flit_to_transmit);
                                    }
                                    else{
                                        while self.left_noc_congestion.len() <= self.cycle as usize/NOC_UTIL_EPOCH_LEN {
                                            self.left_noc_congestion.push(0);
                                        }
                                        self.left_noc_congestion[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                                    }
                                }
                            }
                        } else {
                            if flit_received.destination_y > self.y_dim {
                                if let Some(down_noc) = &mut self.down_noc {
                                    while self.down_noc_util.len() <= self.cycle as usize/NOC_UTIL_EPOCH_LEN {
                                        self.down_noc_util.push(0);
                                    }
                                    self.down_noc_util[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                                    if !down_noc.feeder.is_full() {
                                        let mut flit_to_transmit = down_noc.eater.pop().unwrap();
                                        flit_to_transmit.cycle_to_read =
                                            self.cycle + down_noc.latency as u64;
                                        let _ = down_noc.feeder.push(flit_to_transmit);
                                    }
                                    else{
                                        while self.down_noc_congestion.len() <= self.cycle as usize/NOC_UTIL_EPOCH_LEN {
                                            self.down_noc_congestion.push(0);
                                        }
                                        self.down_noc_congestion[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                                    }
                                }
                            } else {
                                if let Some(up_noc) = &mut self.up_noc {
                                    while self.up_noc_util.len() <= self.cycle as usize/NOC_UTIL_EPOCH_LEN {
                                        self.up_noc_util.push(0);
                                    }
                                    self.up_noc_util[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                                    if !up_noc.feeder.is_full() {
                                        let mut flit_to_transmit = down_noc.eater.pop().unwrap();
                                        flit_to_transmit.cycle_to_read =
                                            self.cycle + up_noc.latency as u64;
                                        let _ = up_noc.feeder.push(flit_to_transmit);
                                    }
                                    else{
                                        while self.up_noc_congestion.len() <= self.cycle as usize/NOC_UTIL_EPOCH_LEN {
                                            self.up_noc_congestion.push(0);
                                        }
                                        self.up_noc_congestion[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    pub fn tick(&mut self, dram: &mut Vec<u32>) {
        while self.fp_pipe.peek().is_some()
            && self.fp_pipe.peek().unwrap().cycle_to_read <= self.cycle
        {
            let fp_stage = self.fp_pipe.pop().unwrap();
            self.register_file[fp_stage.register_index] =
                fp_stage.calculated_val;
            self.register_ready[fp_stage.register_index] =
                true;
        }

        while self.div_seq.peek().is_some()
            && self.div_seq.peek().unwrap().cycle_to_read <= self.cycle
        {
            let div_stage = self.div_seq.pop().unwrap();
            self.register_file[div_stage.register_index] =
                div_stage.calculated_val;
            self.register_ready[div_stage.register_index] =
                true;
        }

        while self.dram_short_queue.peek().is_some()
            && self.dram_short_queue.peek().unwrap().cycle_to_read <= self.cycle
        {
            let dram_stage = self.dram_short_queue.pop().unwrap();
            self.register_file[dram_stage.register_index] =
                dram_stage.calculated_val;
            self.register_ready[dram_stage.register_index] =
                true;
        }

        loop {
            let (req_cycle, req_reg) = {
                let q = match self.dram_long_queue.peek() {
                    Some(g) => g,
                    None => break,
                };
                (q.cycle_to_read, q.register_index)
            };

            if req_cycle > self.cycle {
                break;
            }

            let resp = match self.dram_long_response.as_mut() {
                Some(r) => r,
                None => break,
            };

            if resp.is_empty() {
                break;
            }

            let (resp_reg, resp_val) = {
                let g = resp.peek().unwrap(); // safe since !is_empty()
                (g.register_index, g.calculated_val)
            };

            assert!(resp_reg == req_reg, "MISMATCHED REGISTER INDEX BETWEEN FAR DRAM QUEUES");

            self.dram_long_queue.pop();
            resp.pop();

            let idx = resp_reg;
            self.register_file[idx] = resp_val;
            self.register_ready[idx] = true;
        }


        self.cycle_noc();

        let mut flush = false;
        let mut long_latency_op = false;
        let mut switch_ctx = false;
        if let Some(instruction_to_execute) = self.decoded_instruction {
            let number_of_instructions = num_source_registers(&instruction_to_execute);
            match number_of_instructions {
                0 => {}
                1 => {
                    switch_ctx = !self.register_ready
                        [instruction_to_execute.sr1 + self.context_in_progress * REGS_PER_CONTEXT];
                }
                2 => {
                    let sr1_ready = self.register_ready
                        [instruction_to_execute.sr1 + self.context_in_progress * REGS_PER_CONTEXT];
                    let sr2_ready = self.register_ready
                        [instruction_to_execute.sr2 + self.context_in_progress * REGS_PER_CONTEXT];
                    switch_ctx = !(sr1_ready && sr2_ready);
                }
                _ => panic!("INVALID NUMBER OF SOURCE REGISTERS"),
            }

            if let Operation::Div = instruction_to_execute.operation {
                if !self.div_seq.is_empty() {
                    // println!("DIV BUSY RN BBY");
                    switch_ctx = true;
                }
            }
            assert!(
                instruction_to_execute.pc == self.pc[self.context_in_progress],
                "PC MISMATCH DURING EXECUTION"
            );
            if DEBUG {
                println!(
                    "Core {} executing instruction x{:08X} ({:?}) at PC {:08X} in context {} at cycle {}, dr: {}, sr1: {}, is_imm: {}, sr2: {}, imm_val: {}",
                    self.core_id,
                    instruction_to_execute.raw_instruction,
                    instruction_to_execute.operation,
                    instruction_to_execute.pc,
                    self.context_in_progress,
                    self.cycle,
                    instruction_to_execute.dr,
                    instruction_to_execute.sr1,
                    instruction_to_execute.imm_1,
                    instruction_to_execute.sr2,
                    instruction_to_execute.imm_0
                );
            }
            if !switch_ctx {
                match instruction_to_execute.operation {
                    Operation::Add => {
                        let sr2_val = if instruction_to_execute.is_imm {
                            instruction_to_execute.imm_0 as i16 as i32 as u32
                        } else {
                            self.register_file
                                [instruction_to_execute.sr2 + self.context_in_progress * REGS_PER_CONTEXT]
                        };
                        self.register_file
                            [instruction_to_execute.dr + self.context_in_progress * REGS_PER_CONTEXT] =
                            ((self.register_file
                                [instruction_to_execute.sr1 + self.context_in_progress * REGS_PER_CONTEXT]
                                as i32).wrapping_add(sr2_val as i32)) as u32;
                        self.pc[self.context_in_progress] += 4;
                    }
                    Operation::Sub => {
                        let sr2_val = if instruction_to_execute.is_imm {
                            instruction_to_execute.imm_0 as i16 as i32 as u32
                        } else {
                            self.register_file
                                [instruction_to_execute.sr2 + self.context_in_progress * REGS_PER_CONTEXT]
                        };
                        self.register_file
                            [instruction_to_execute.dr + self.context_in_progress * REGS_PER_CONTEXT] =
                            ((self.register_file
                                [instruction_to_execute.sr1 + self.context_in_progress * REGS_PER_CONTEXT]
                                as i32)
                                .wrapping_sub(sr2_val as i32)) as u32;

                        self.pc[self.context_in_progress] += 4;
                    }
                    Operation::And => {
                        let sr2_val = if instruction_to_execute.is_imm {
                            instruction_to_execute.imm_0 as i16 as i32 as u32
                        } else {
                            self.register_file
                                [instruction_to_execute.sr2 + self.context_in_progress * REGS_PER_CONTEXT]
                        };
                        self.register_file
                            [instruction_to_execute.dr + self.context_in_progress * REGS_PER_CONTEXT] = self
                            .register_file
                            [instruction_to_execute.sr1 + self.context_in_progress * REGS_PER_CONTEXT]
                            & sr2_val;
                        self.pc[self.context_in_progress] += 4;
                    }
                    Operation::Or => {
                        let sr2_val = if instruction_to_execute.is_imm {
                            instruction_to_execute.imm_0 as i16 as i32 as u32
                        } else {
                            self.register_file
                                [instruction_to_execute.sr2 + self.context_in_progress * REGS_PER_CONTEXT]
                        };
                        self.register_file
                            [instruction_to_execute.dr + self.context_in_progress * REGS_PER_CONTEXT] = self
                            .register_file
                            [instruction_to_execute.sr1 + self.context_in_progress * REGS_PER_CONTEXT]
                            | sr2_val;
                        self.pc[self.context_in_progress] += 4;
                    }
                    Operation::Xor => {
                        let sr2_val = if instruction_to_execute.is_imm {
                            instruction_to_execute.imm_0 as i16 as i32 as u32
                        } else {
                            self.register_file
                                [instruction_to_execute.sr2 + self.context_in_progress * REGS_PER_CONTEXT]
                        };
                        self.register_file
                            [instruction_to_execute.dr + self.context_in_progress * REGS_PER_CONTEXT] = self
                            .register_file
                            [instruction_to_execute.sr1 + self.context_in_progress * REGS_PER_CONTEXT]
                            ^ sr2_val;
                        self.pc[self.context_in_progress] += 4;
                    }
                    Operation::Sll => {
                        let sr2_val = if instruction_to_execute.is_imm {
                            instruction_to_execute.imm_0
                        } else {
                            self.register_file
                                [instruction_to_execute.sr2 + self.context_in_progress * REGS_PER_CONTEXT]
                        };
                        self.register_file
                            [instruction_to_execute.dr + self.context_in_progress * REGS_PER_CONTEXT] = self
                            .register_file
                            [instruction_to_execute.sr1 + self.context_in_progress * REGS_PER_CONTEXT]
                            << sr2_val;
                        self.pc[self.context_in_progress] += 4;
                    }
                    Operation::Srl => {
                        let sr2_val = if instruction_to_execute.is_imm {
                            instruction_to_execute.imm_0
                        } else {
                            self.register_file
                                [instruction_to_execute.sr2 + self.context_in_progress * REGS_PER_CONTEXT]
                        };
                        self.register_file
                            [instruction_to_execute.dr + self.context_in_progress * REGS_PER_CONTEXT] = self
                            .register_file
                            [instruction_to_execute.sr1 + self.context_in_progress * REGS_PER_CONTEXT]
                            >> sr2_val;
                        self.pc[self.context_in_progress] += 4;
                    }
                    Operation::Sra => {
                        let sr2_val = if instruction_to_execute.is_imm {
                            instruction_to_execute.imm_0
                        } else {
                            self.register_file
                                [instruction_to_execute.sr2 + self.context_in_progress * REGS_PER_CONTEXT]
                        };
                        self.register_file
                            [instruction_to_execute.dr + self.context_in_progress * REGS_PER_CONTEXT] =
                            ((self.register_file
                                [instruction_to_execute.sr1 + self.context_in_progress * REGS_PER_CONTEXT]
                                as i32)
                                >> sr2_val) as u32;
                        self.pc[self.context_in_progress] += 4;
                    }
                    Operation::Mul => {
                        long_latency_op = true;
                        let sr2_val = if instruction_to_execute.is_imm {
                            instruction_to_execute.imm_0 as i16 as i32 as u32
                        } else {
                            self.register_file
                                [instruction_to_execute.sr2 + self.context_in_progress * REGS_PER_CONTEXT]
                        };
                        let mul_result = self.register_file
                            [instruction_to_execute.sr1 + self.context_in_progress * REGS_PER_CONTEXT].wrapping_mul(sr2_val);
                        assert!(!self.fp_pipe.is_full(), "FP PIPE IS FULL");
                        let mul_pipe_stage: PipelineStage = PipelineStage {
                            cycle_to_read: self.cycle + FP_PIPE_STAGES as u64,
                            calculated_val: mul_result,
                            register_index: instruction_to_execute.dr + self.context_in_progress * REGS_PER_CONTEXT,
                        };
                        let _ = self.fp_pipe.push(mul_pipe_stage);
                        self.pc[self.context_in_progress] += 4;
                        if self.core_id == 0 && instruction_to_execute.imm_1 == 0 {
                            println!("MUL RESULT: {}, context: {}", mul_result as i32, self.context_in_progress);
                        }
                    }
                    Operation::Div => {
                        long_latency_op = true;
                        let sr2_val = if instruction_to_execute.is_imm {
                            instruction_to_execute.imm_0 as i16 as i32 as u32
                        } else {
                            self.register_file
                                [instruction_to_execute.sr2 + self.context_in_progress * REGS_PER_CONTEXT]
                        };
                        let div_result = self.register_file
                            [instruction_to_execute.sr1 + self.context_in_progress * REGS_PER_CONTEXT]
                            / sr2_val;
                        let div_pipe_stage: PipelineStage = PipelineStage {
                            cycle_to_read: self.cycle + DIV_LATENCY as u64,
                            calculated_val: div_result,
                            register_index: instruction_to_execute.dr + self.context_in_progress * REGS_PER_CONTEXT,
                        };
                        if self.div_seq.is_full() {
                            switch_ctx = true;
                        } else {
                            let _ = self.div_seq.push(div_pipe_stage);
                            self.pc[self.context_in_progress] += 4;
                        }
                    }
                    Operation::Mod => {
                        long_latency_op = true;
                        let sr2_val = if instruction_to_execute.is_imm {
                            instruction_to_execute.imm_0 as i16 as i32 as u32
                        } else {
                            self.register_file
                                [instruction_to_execute.sr2 + self.context_in_progress * REGS_PER_CONTEXT]
                        };
                        let div_result = self.register_file
                            [instruction_to_execute.sr1 + self.context_in_progress * REGS_PER_CONTEXT]
                            % sr2_val;
                        let div_pipe_stage: PipelineStage = PipelineStage {
                            cycle_to_read: self.cycle + DIV_LATENCY as u64,
                            calculated_val: div_result,
                            register_index: instruction_to_execute.dr + self.context_in_progress * REGS_PER_CONTEXT,
                        };
                        if self.div_seq.is_full() {
                            switch_ctx = true;
                        } else {
                            let _ = self.div_seq.push(div_pipe_stage);
                            self.pc[self.context_in_progress] += 4;
                        }
                    }
                    Operation::FPAdd => {
                        long_latency_op = true;

                        let idx1 = instruction_to_execute.sr1 + self.context_in_progress * REGS_PER_CONTEXT;
                        let idx2 = instruction_to_execute.sr2 + self.context_in_progress * REGS_PER_CONTEXT;

                        let fpadd_result = match instruction_to_execute.fp_type {
                            FpType::Fp32 => {
                                let a = f32::from_bits(self.register_file[idx1]);
                                let b = f32::from_bits(self.register_file[idx2]);
                                (a + b).to_bits()
                            }
                            FpType::Fp16 => {
                                let a = f16::from_bits(self.register_file[idx1] as u16);
                                let b = f16::from_bits(self.register_file[idx2] as u16);
                                (a + b).to_bits() as u32
                            }
                            FpType::Fp8 => {
                                let a = self.register_file[idx1] as u8;
                                let b = self.register_file[idx2] as u8;
                                fp8_add(a, b) as u32
                            }
                        };

                        assert!(!self.fp_pipe.is_full(), "FP PIPE IS FULL");

                        let fpadd_pipe_stage: PipelineStage = PipelineStage {
                            cycle_to_read: self.cycle + FP_PIPE_STAGES as u64,
                            calculated_val: fpadd_result,
                            register_index: instruction_to_execute.dr + self.context_in_progress * REGS_PER_CONTEXT,
                        };

                        let _ = self.fp_pipe.push(fpadd_pipe_stage);
                        self.pc[self.context_in_progress] += 4;
                    }
                    Operation::FPMul => {
                        long_latency_op = true;

                        let idx1 = instruction_to_execute.sr1 + self.context_in_progress * REGS_PER_CONTEXT;
                        let idx2 = instruction_to_execute.sr2 + self.context_in_progress * REGS_PER_CONTEXT;

                        let fpmul_result = match instruction_to_execute.fp_type {
                            FpType::Fp32 => {
                                let a = f32::from_bits(self.register_file[idx1]);
                                let b = f32::from_bits(self.register_file[idx2]);
                                (a * b).to_bits()
                            }
                            FpType::Fp16 => {
                                let a = f16::from_bits(self.register_file[idx1] as u16);
                                let b = f16::from_bits(self.register_file[idx2] as u16);
                                (a * b).to_bits() as u32
                            }
                            FpType::Fp8 => {
                                let a = self.register_file[idx1] as u8;
                                let b = self.register_file[idx2] as u8;
                                fp8_mul(a, b) as u32
                            }
                        };

                        assert!(!self.fp_pipe.is_full(), "FP PIPE IS FULL");

                        let fpmul_pipe_stage: PipelineStage = PipelineStage {
                            cycle_to_read: self.cycle + FP_PIPE_STAGES as u64,
                            calculated_val: fpmul_result,
                            register_index: instruction_to_execute.dr + self.context_in_progress * REGS_PER_CONTEXT,
                        };

                        let _ = self.fp_pipe.push(fpmul_pipe_stage);
                        self.pc[self.context_in_progress] += 4;
                    }
                    Operation::FPSub => {
                        long_latency_op = true;

                        let idx1 = instruction_to_execute.sr1 + self.context_in_progress * REGS_PER_CONTEXT;
                        let idx2 = instruction_to_execute.sr2 + self.context_in_progress * REGS_PER_CONTEXT;

                        let fpsub_result = match instruction_to_execute.fp_type {
                            FpType::Fp32 => {
                                let a = f32::from_bits(self.register_file[idx1]);
                                let b = f32::from_bits(self.register_file[idx2]);
                                (a - b).to_bits()
                            }
                            FpType::Fp16 => {
                                let a = f16::from_bits(self.register_file[idx1] as u16);
                                let b = f16::from_bits(self.register_file[idx2] as u16);
                                (a - b).to_bits() as u32
                            }
                            FpType::Fp8 => {
                                let a = self.register_file[idx1] as u8;
                                let b = self.register_file[idx2] as u8;
                                fp8_sub(a, b) as u32
                            }
                        };

                        assert!(!self.fp_pipe.is_full(), "FP PIPE IS FULL");

                        let fpsub_pipe_stage: PipelineStage = PipelineStage {
                            cycle_to_read: self.cycle + FP_PIPE_STAGES as u64,
                            calculated_val: fpsub_result,
                            register_index: instruction_to_execute.dr + self.context_in_progress * REGS_PER_CONTEXT,
                        };

                        let _ = self.fp_pipe.push(fpsub_pipe_stage);
                        self.pc[self.context_in_progress] += 4;
                    }
                    Operation::FPMac => {
                        let idx1 = instruction_to_execute.sr1 + self.context_in_progress * REGS_PER_CONTEXT;
                        let idx2 = instruction_to_execute.sr2 + self.context_in_progress * REGS_PER_CONTEXT;

                        let fpmac_result = match instruction_to_execute.fp_type {
                            FpType::Fp32 => {
                                let a = f32::from_bits(self.register_file[idx1]);
                                let b = f32::from_bits(self.register_file[idx2]);
                                (a * b + f32::from_bits(self.fp_accumulation[self.context_in_progress])).to_bits()
                            }
                            FpType::Fp16 => {
                                let a = f32::from(f16::from_bits(self.register_file[idx1] as u16));
                                let b = f32::from(f16::from_bits(self.register_file[idx2] as u16));
                                let acc = f32::from_bits(self.fp_accumulation[self.context_in_progress]);
                                (a * b + acc).to_bits()

                            }
                            FpType::Fp8 => {
                                let a = Fp8E4M3::from_bits(self.register_file[idx1] as u8).to_f32();
                                let b = Fp8E4M3::from_bits(self.register_file[idx2] as u8).to_f32();
                                let acc = f32::from_bits(self.fp_accumulation[self.context_in_progress]);

                                (a * b + acc).to_bits()
                            }
                        };

                        self.fp_accumulation[self.context_in_progress] = fpmac_result;
                        self.pc[self.context_in_progress] += 4;
                    }
                    Operation::FPEQ => {
                        long_latency_op = true;

                        let idx1 = instruction_to_execute.sr1 + self.context_in_progress * REGS_PER_CONTEXT;
                        let idx2 = instruction_to_execute.sr2 + self.context_in_progress * REGS_PER_CONTEXT;

                        let fpeq_result = match instruction_to_execute.fp_type {
                            FpType::Fp32 => {
                                let a = f32::from_bits(self.register_file[idx1]);
                                let b = f32::from_bits(self.register_file[idx2]);

                                if a.is_nan() || b.is_nan() {
                                    0
                                } else if a == b {
                                    1
                                } else {
                                    0
                                }
                            }

                            FpType::Fp16 => {
                                let a = f32::from(f16::from_bits(self.register_file[idx1] as u16));
                                let b = f32::from(f16::from_bits(self.register_file[idx2] as u16));

                                if a.is_nan() || b.is_nan() {
                                    0
                                } else if a == b {
                                    1
                                } else {
                                    0
                                }
                            }

                            FpType::Fp8 => {
                                let a = Fp8E4M3::from_bits(self.register_file[idx1] as u8).to_f32();
                                let b = Fp8E4M3::from_bits(self.register_file[idx2] as u8).to_f32();

                                if a.is_nan() || b.is_nan() {
                                    0
                                } else if a == b {
                                    1
                                } else {
                                    0
                                }
                            }
                        };

                        assert!(!self.fp_pipe.is_full(), "FP PIPE IS FULL");

                        let _ = self.fp_pipe.push(PipelineStage {
                            cycle_to_read: self.cycle + FP_PIPE_STAGES as u64,
                            calculated_val: fpeq_result,
                            register_index: instruction_to_execute.dr + self.context_in_progress * REGS_PER_CONTEXT,
                        });
                        self.pc[self.context_in_progress] += 4;
                    }
                    Operation::FPLT => {
                        long_latency_op = true;

                        let idx1 = instruction_to_execute.sr1 + self.context_in_progress * REGS_PER_CONTEXT;
                        let idx2 = instruction_to_execute.sr2 + self.context_in_progress * REGS_PER_CONTEXT;

                        let fplt_result = match instruction_to_execute.fp_type {
                            FpType::Fp32 => {
                                let a = f32::from_bits(self.register_file[idx1]);
                                let b = f32::from_bits(self.register_file[idx2]);

                                if a.is_nan() || b.is_nan() {
                                    0
                                } else if a < b {
                                    1
                                } else {
                                    0
                                }
                            }

                            FpType::Fp16 => {
                                let a = f32::from(f16::from_bits(self.register_file[idx1] as u16));
                                let b = f32::from(f16::from_bits(self.register_file[idx2] as u16));

                                if a.is_nan() || b.is_nan() {
                                    0
                                } else if a < b {
                                    1
                                } else {
                                    0
                                }
                            }

                            FpType::Fp8 => {
                                let a = Fp8E4M3::from_bits(self.register_file[idx1] as u8).to_f32();
                                let b = Fp8E4M3::from_bits(self.register_file[idx2] as u8).to_f32();

                                if a.is_nan() || b.is_nan() {
                                    0
                                } else if a < b {
                                    1
                                } else {
                                    0
                                }
                            }
                        };

                        assert!(!self.fp_pipe.is_full(), "FP PIPE IS FULL");

                        let _ = self.fp_pipe.push(PipelineStage {
                            cycle_to_read: self.cycle + FP_PIPE_STAGES as u64,
                            calculated_val: fplt_result,
                            register_index: instruction_to_execute.dr + self.context_in_progress * REGS_PER_CONTEXT,
                        });
                        self.pc[self.context_in_progress] += 4;
                    }
                    Operation::FPSetAccumulator => {
                        let register = self.register_file
                            [instruction_to_execute.sr1 + self.context_in_progress * REGS_PER_CONTEXT];
                        let value_to_set = match instruction_to_execute.fp_type {
                            FpType::Fp32 => register,
                            FpType::Fp16 => f32::from(f16::from_bits(register as u16)).to_bits(),
                            FpType::Fp8 => Fp8E4M3::from_bits(register as u8).to_f32().to_bits(),
                        };
                        self.fp_accumulation[self.context_in_progress] = value_to_set;
                        self.pc[self.context_in_progress] += 4;
                    }
                    Operation::FPStoreAccumulator => {
                        let accumulated_value = match instruction_to_execute.fp_type{
                            FpType::Fp32 => self.fp_accumulation[self.context_in_progress],
                            FpType::Fp16 => f16::from_f32(f32::from_bits(self.fp_accumulation[self.context_in_progress])).to_bits() as u32,
                            FpType::Fp8 => Fp8E4M3::from_f32(f32::from_bits(self.fp_accumulation[self.context_in_progress])).to_bits() as u32,
                        };
                        long_latency_op = true;
                        let _ = self.fp_pipe.push(PipelineStage {
                            cycle_to_read: self.cycle + FP_PIPE_STAGES as u64,
                            calculated_val: accumulated_value,
                            register_index: instruction_to_execute.dr + self.context_in_progress * REGS_PER_CONTEXT,
                        });
                        self.pc[self.context_in_progress] += 4;
                    }
                    Operation::FPMinMax => {
                        let idx1 = instruction_to_execute.sr1 + self.context_in_progress * REGS_PER_CONTEXT;
                        let idx2 = instruction_to_execute.sr2 + self.context_in_progress * REGS_PER_CONTEXT;

                        let fpminmax_result = match instruction_to_execute.fp_type {
                            FpType::Fp32 => {
                                let a = f32::from_bits(self.register_file[idx1]);
                                let b = f32::from_bits(self.register_file[idx2]);

                                (match (a < b) == (instruction_to_execute.imm_1 == 0) {
                                    true => a,
                                    false => b,
                                })
                                .to_bits()
                            }
                            FpType::Fp16 => {
                                let a = f32::from(f16::from_bits(self.register_file[idx1] as u16));
                                let b = f32::from(f16::from_bits(self.register_file[idx2] as u16));
                                (match (a < b) == (instruction_to_execute.imm_1 == 0) {
                                    true => a,
                                    false => b,
                                })
                                .to_bits()
                            }
                            FpType::Fp8 => {
                                let a = Fp8E4M3::from_bits(self.register_file[idx1] as u8).to_f32();
                                let b = Fp8E4M3::from_bits(self.register_file[idx2] as u8).to_f32();
                                (match (a < b) == (instruction_to_execute.imm_1 == 0) {
                                    true => a,
                                    false => b,
                                })
                                .to_bits()
                            }
                        };
                        assert!(!self.fp_pipe.is_full(), "FP PIPE IS FULL");
                        long_latency_op = true;

                        let fpminmax_pipe_stage: PipelineStage = PipelineStage {
                            cycle_to_read: self.cycle + FP_PIPE_STAGES as u64,
                            calculated_val: fpminmax_result,
                            register_index: instruction_to_execute.dr + self.context_in_progress * REGS_PER_CONTEXT,
                        };

                        let _ = self.fp_pipe.push(fpminmax_pipe_stage);
                        self.pc[self.context_in_progress] += 4;
                    }
                    Operation::StoreByte => {
                        if instruction_to_execute.is_imm {
                            let byte = self.register_file
                                [instruction_to_execute.sr1 + self.context_in_progress * REGS_PER_CONTEXT]
                                as u8;
                            self.write_sram_byte(byte, &(instruction_to_execute.imm_0 as u16));
                        } else {
                            let byte = self.register_file
                                [instruction_to_execute.sr1 + self.context_in_progress * REGS_PER_CONTEXT]
                                as u8;
                            let address = self.register_file
                                [instruction_to_execute.sr2 + self.context_in_progress * REGS_PER_CONTEXT]
                                as u16;
                            self.write_sram_byte(byte, &address);
                        }
                        self.pc[self.context_in_progress] += 4;
                    }
                    Operation::StoreHalf => {
                        if instruction_to_execute.is_imm {
                            let half = self.register_file
                                [instruction_to_execute.sr1 + self.context_in_progress * REGS_PER_CONTEXT]
                                as u16;
                            self.write_sram_half(half, &(instruction_to_execute.imm_0 as u16));
                        } else {
                            let half = self.register_file
                                [instruction_to_execute.sr1 + self.context_in_progress * REGS_PER_CONTEXT]
                                as u16;
                            let address = self.register_file
                                [instruction_to_execute.sr2 + self.context_in_progress * REGS_PER_CONTEXT]
                                as u16;
                            self.write_sram_half(half, &address);
                        }
                        self.pc[self.context_in_progress] += 4;

                    }
                    Operation::StoreWord => {
                        if instruction_to_execute.is_imm {
                            let word = self.register_file
                                [instruction_to_execute.sr1 + self.context_in_progress * REGS_PER_CONTEXT]
                                as u32;
                            self.write_sram_word(word, &(instruction_to_execute.imm_0 as u16));
                        } else {
                            let word = self.register_file
                                [instruction_to_execute.sr1 + self.context_in_progress * REGS_PER_CONTEXT]
                                as u32;
                            if DEBUG {
                                println!("WORD TO STORE CORE {}: 0x{:08X}/{}, cycle: {}", self.core_id, word, word as i32, self.cycle);
                            }
                            let address = self.register_file
                                [instruction_to_execute.sr2 + self.context_in_progress * REGS_PER_CONTEXT]
                                as u16;
                            self.write_sram_word(word, &address);
                        }
                        self.pc[self.context_in_progress] += 4;
                    }
                    Operation::LoadByteSigned
                    | Operation::LoadByteUnsigned
                    | Operation::LoadHalfSigned
                    | Operation::LoadHalfUnsigned
                    | Operation::LoadWord => {
                        let address = if instruction_to_execute.imm_1 != 0 {
                            instruction_to_execute.imm_0 as u16
                        } else {
                            self.register_file[self.context_in_progress * REGS_PER_CONTEXT + instruction_to_execute.sr2] as u16
                        };
                        let value = match instruction_to_execute.operation {
                            Operation::LoadByteSigned =>
                                self.read_sram_byte_signed(&address),

                            Operation::LoadByteUnsigned =>
                                self.read_sram_byte_unsigned(&address),

                            Operation::LoadHalfSigned =>
                                self.read_sram_half_signed(&address),

                            Operation::LoadHalfUnsigned =>
                                self.read_sram_half_unsigned(&address),

                            Operation::LoadWord =>
                                self.read_sram_word(&address),

                            _ => unreachable!(),
                        };
                        self.register_file[self.context_in_progress * REGS_PER_CONTEXT + instruction_to_execute.dr] = value;

                        self.pc[self.context_in_progress] += 4;
                    }

                    Operation::BranchEq => {
                        if self.register_file
                            [self.context_in_progress * REGS_PER_CONTEXT + instruction_to_execute.sr1]
                            == self.register_file
                                [self.context_in_progress * REGS_PER_CONTEXT + instruction_to_execute.sr2]
                        {
                            self.pc[self.context_in_progress] = instruction_to_execute.imm_0 as u16;
                            flush = !instruction_to_execute.branch_hint;
                        } else {
                            self.pc[self.context_in_progress] += 4;
                            flush = instruction_to_execute.branch_hint;
                        }
                    }
                    Operation::BranchNe => {
                        if self.register_file
                            [self.context_in_progress * REGS_PER_CONTEXT + instruction_to_execute.sr1]
                            != self.register_file
                                [self.context_in_progress * REGS_PER_CONTEXT + instruction_to_execute.sr2]
                        {
                            self.pc[self.context_in_progress] = instruction_to_execute.imm_0 as u16;
                            flush = !instruction_to_execute.branch_hint;
                        } else {
                            self.pc[self.context_in_progress] += 4;
                            flush = instruction_to_execute.branch_hint;
                        }
                    }
                    Operation::BranchLTE => {
                        let is_lt = match instruction_to_execute.imm_1 {
                            0 => {
                                self.register_file[self.context_in_progress * REGS_PER_CONTEXT
                                    + instruction_to_execute.sr1]
                                    <= self.register_file[self.context_in_progress * REGS_PER_CONTEXT
                                        + instruction_to_execute.sr2]
                            }
                            _ => {
                                self.register_file[self.context_in_progress * REGS_PER_CONTEXT
                                    + instruction_to_execute.sr1]
                                    as i32
                                    <= self.register_file[self.context_in_progress * REGS_PER_CONTEXT
                                        + instruction_to_execute.sr2]
                                        as i32
                            }
                        };
                        if is_lt {
                            self.pc[self.context_in_progress] = instruction_to_execute.imm_0 as u16;
                            flush = !instruction_to_execute.branch_hint;
                        } else {
                            self.pc[self.context_in_progress] += 4;
                            flush = instruction_to_execute.branch_hint;
                        }
                    }
                    Operation::BranchGT => {
                        let is_gt = match instruction_to_execute.imm_1 {
                            0 => {
                                self.register_file[self.context_in_progress * REGS_PER_CONTEXT
                                    + instruction_to_execute.sr1]
                                    > self.register_file[self.context_in_progress * REGS_PER_CONTEXT
                                        + instruction_to_execute.sr2]
                            }
                            _ => {
                                self.register_file[self.context_in_progress * REGS_PER_CONTEXT
                                    + instruction_to_execute.sr1]
                                    as i32
                                    > self.register_file[self.context_in_progress * REGS_PER_CONTEXT
                                        + instruction_to_execute.sr2]
                                        as i32
                            }
                        };
                        if is_gt {
                            self.pc[self.context_in_progress] = instruction_to_execute.imm_0 as u16;
                            flush = !instruction_to_execute.branch_hint;
                        } else {
                            self.pc[self.context_in_progress] += 4;
                            flush = instruction_to_execute.branch_hint;
                        }
                    }
                    Operation::Jump => {
                        let temp = self.pc[self.context_in_progress] as u32 + 4;
                        self.pc[self.context_in_progress] = if instruction_to_execute.is_imm {
                            instruction_to_execute.imm_0 as u16
                        } else {
                            flush = true;
                            self.register_file
                                [self.context_in_progress * REGS_PER_CONTEXT + instruction_to_execute.sr2]
                                as u16
                        };
                        self.register_file
                            [self.context_in_progress * REGS_PER_CONTEXT + instruction_to_execute.dr] = temp;
                    }
                    Operation::BlockVal => {
                        let fifo_idx = if instruction_to_execute.imm_1 != 0 {
                            instruction_to_execute.imm_0 as usize
                        } else {
                            self.register_file[
                                self.context_in_progress * REGS_PER_CONTEXT + instruction_to_execute.sr2
                            ] as usize
                        };

                        if let Some(val) = self.noc_recv_fifo[fifo_idx].pop() {
                            self.register_file[self.context_in_progress * REGS_PER_CONTEXT + instruction_to_execute.dr] = val;
                            self.pc[self.context_in_progress] += 4;
                        } else {
                            switch_ctx = true;
                        }
                    }
                    Operation::NonBlockVal => {
                        let fifo_idx = if instruction_to_execute.imm_1 != 0 {
                            instruction_to_execute.imm_0 as usize
                        } else {
                            self.register_file[
                                self.context_in_progress * REGS_PER_CONTEXT + instruction_to_execute.sr2
                            ] as usize
                        };

                        let fill = self.noc_recv_fifo[fifo_idx].len();

                        self.register_file[
                            self.context_in_progress * REGS_PER_CONTEXT + instruction_to_execute.dr
                        ] = fill as u32;

                        self.pc[self.context_in_progress] += 4;
                    }

                    Operation::Yield => {
                        switch_ctx = true;
                        if self.ctx_ownership >= 0 {
                            self.runnable[self.context_in_progress] = false;
                            assert!(
                                self.ctx_ownership as usize != self.context_in_progress,
                                "Dumb programmer"
                            );
                        } else {
                            let mut interrupt_pending = false;
                            let mut index = 0;
                            for fifo in &self.noc_recv_fifo {
                                if !fifo.is_empty() && self.interrupt_enable[index] {
                                    interrupt_pending = true;
                                    break;
                                }
                                index += 1;
                            }
                            if interrupt_pending {
                                self.register_file[self.context_in_progress * REGS_PER_CONTEXT
                                    + instruction_to_execute.dr] =
                                    self.pc[self.context_in_progress] as u32;
                                self.pc[self.context_in_progress] =
                                    SRAM_SIZE as u16 - NUM_NOC_PIPES as u16 * 4 + index as u16 * 4;
                            } else {
                                self.pc[self.context_in_progress] += 4;
                            }
                        }
                    }
                    Operation::GetThreadOwnership => {
                        self.runnable[self.context_in_progress] = false;
                        let mut any_runnable = false;
                        if self.ctx_ownership < 0 {
                            self.ctx_ownership = self.context_in_progress as i8;
                        } 
                        assert!(
                            self.ctx_ownership == self.context_in_progress as i8,
                            "ANOTHER Dumb programmer!"
                        );
                        for context in &self.runnable {
                            if *context {
                                any_runnable = true;
                            }
                        }
                        if !any_runnable {
                            self.pc[self.context_in_progress] += 4;
                        } else {
                            switch_ctx = true;
                        }
                    }
                    Operation::SetCtx => {
                        let mut cur_index = self.context_in_progress;
                        assert!(
                            instruction_to_execute.imm_0 <= REGS_PER_CONTEXT as u32,
                            "STUPID PROGRAMMER"
                        );
                        assert!(
                            instruction_to_execute.imm_0 > 0,
                            "STUPID PROGRAMMER"
                        );
                        for _ in 0..instruction_to_execute.imm_0 {
                            self.runnable[cur_index] = true;
                            cur_index = (cur_index + 1) % REGS_PER_CONTEXT;
                        }
                        while cur_index != self.context_in_progress {
                            self.runnable[cur_index] = false;
                            cur_index = (cur_index + 1) % REGS_PER_CONTEXT;
                        }
                        self.pc[self.context_in_progress] += 4;
                    }
                    Operation::RelinquishOwnership => {
                        assert!(
                            self.ctx_ownership == self.context_in_progress as i8,
                            "STUPID PROGRAMMER"
                        );
                        self.ctx_ownership = -1;
                        switch_ctx = true;
                        self.pc[self.context_in_progress] += 4;
                        for context in 0..REGS_PER_CONTEXT {
                            self.pc[context] = self.pc[self.context_in_progress];
                        }
                    }
                    Operation::ModifyInterruptEnable => {
                        assert!(
                            instruction_to_execute.imm_0 < NUM_NOC_PIPES as u32,
                            "IMM IS LESS THAN NUMBER OF NOC PIPES DUMMY"
                        );
                        self.interrupt_enable[instruction_to_execute.imm_0 as usize] =
                            instruction_to_execute.imm_1 != 0;
                        self.pc[self.context_in_progress] += 4;
                    }
                    Operation::SetMemoryBits => {
                        self.memory_bits[self.context_in_progress] = self.register_file
                            [self.context_in_progress * REGS_PER_CONTEXT + instruction_to_execute.sr1];
                        self.pc[self.context_in_progress] += 4;
                    }
                    Operation::ReadSignedByteDram => {
                        long_latency_op = true;
                        let dram_address = self.register_file
                            [self.context_in_progress * REGS_PER_CONTEXT + instruction_to_execute.sr1]
                            as usize
                            | (self.memory_bits[self.context_in_progress] as usize) << DRAM_STACK_SIZE_LOG2;
                        if self.top_bits_dram_stack != dram_address / DRAM_STACK_SIZE {
                            self.dram_bytes_read_far += 1;
                            let long_dram_request = LongDramRequest {
                                register_index: instruction_to_execute.dr + self.context_in_progress * REGS_PER_CONTEXT,
                                address: dram_address,
                                op: instruction_to_execute.operation,
                                value_to_write: 0,
                                core_id: self.core_id,
                                origin_stack: self.top_bits_dram_stack,
                            };
                            if let Some(dram_request) = &self.dram_long_request {
                                let could_send = dram_request[dram_address / DRAM_STACK_SIZE]
                                    .send(long_dram_request);
                                assert!(
                                    could_send.is_ok(),
                                    "Failed to send Request between OS Threads"
                                );
                            }
                            let internal_long_dram_op = PipelineStage {
                                cycle_to_read: self.cycle + DRAM_LATENCY_FAR,
                                calculated_val: 0,
                                register_index: instruction_to_execute.dr + self.context_in_progress * REGS_PER_CONTEXT,
                            };
                            let _ = self.dram_long_queue.push(internal_long_dram_op);
                        } else {
                            self.dram_bytes_read_close += 1;
                            let value = dram[dram_address / 4];
                            let byte_offset = dram_address % 4;
                            let load_dispatch = PipelineStage {
                                cycle_to_read: self.cycle + DRAM_LATENCY_CLOSE,
                                calculated_val: ((value >> (byte_offset * 8)) & 0xFF) as i8 as i32
                                    as u32,
                                register_index: instruction_to_execute.dr + self.context_in_progress * REGS_PER_CONTEXT,
                            };
                            let _ = self.dram_short_queue.push(load_dispatch);
                        }

                        self.pc[self.context_in_progress] += 4;
                    }
                    Operation::ReadUnsignedByteDram => {
                        long_latency_op = true;
                        let dram_address = self.register_file
                            [self.context_in_progress * REGS_PER_CONTEXT + instruction_to_execute.sr1]
                            as usize
                            | (self.memory_bits[self.context_in_progress] as usize) << DRAM_STACK_SIZE_LOG2;
                        if self.top_bits_dram_stack != dram_address / DRAM_STACK_SIZE {
                            self.dram_bytes_read_far += 1;
                            let long_dram_request = LongDramRequest {
                                register_index: instruction_to_execute.dr + self.context_in_progress * REGS_PER_CONTEXT,
                                address: dram_address,
                                op: instruction_to_execute.operation,
                                value_to_write: 0,
                                core_id: self.core_id,
                                origin_stack: self.top_bits_dram_stack,
                            };
                            if let Some(dram_request) = &self.dram_long_request {
                                let could_send = dram_request[dram_address / DRAM_STACK_SIZE]
                                    .send(long_dram_request);
                                assert!(
                                    could_send.is_ok(),
                                    "Failed to send Request between OS Threads"
                                );
                            }
                            let internal_long_dram_op = PipelineStage {
                                cycle_to_read: self.cycle + DRAM_LATENCY_FAR,
                                calculated_val: 0,
                                register_index: instruction_to_execute.dr + self.context_in_progress * REGS_PER_CONTEXT,
                            };
                            let _ = self.dram_long_queue.push(internal_long_dram_op);
                        } else {
                            self.dram_bytes_read_close += 1;
                            let value = dram[dram_address / 4];
                            let byte_offset = dram_address % 4;
                            let load_dispatch = PipelineStage {
                                cycle_to_read: self.cycle + DRAM_LATENCY_CLOSE,
                                calculated_val: ((value >> (byte_offset * 8)) & 0xFF) as u8 as u32,
                                register_index: instruction_to_execute.dr + self.context_in_progress * REGS_PER_CONTEXT,
                            };
                            let _ = self.dram_short_queue.push(load_dispatch);
                        }
                        self.pc[self.context_in_progress] += 4;
                    }
                    Operation::ReadSignedHalfDram => {
                        long_latency_op = true;
                        let dram_address = self.register_file
                            [self.context_in_progress * REGS_PER_CONTEXT + instruction_to_execute.sr1]
                            as usize
                            | (self.memory_bits[self.context_in_progress] as usize) << DRAM_STACK_SIZE_LOG2;
                        assert!(
                            dram_address & 0x1 == 0,
                            "DRAM Half LOADS CAN'T BE UNALIGNED"
                        );
                        if self.top_bits_dram_stack != dram_address / DRAM_STACK_SIZE {
                            self.dram_bytes_read_far += 2;
                            let long_dram_request = LongDramRequest {
                                register_index: instruction_to_execute.dr + self.context_in_progress * REGS_PER_CONTEXT,
                                address: dram_address,
                                op: instruction_to_execute.operation,
                                value_to_write: 0,
                                core_id: self.core_id,
                                origin_stack: self.top_bits_dram_stack,
                            };
                            if let Some(dram_request) = &self.dram_long_request {
                                let could_send = dram_request[dram_address / DRAM_STACK_SIZE]
                                    .send(long_dram_request);
                                assert!(
                                    could_send.is_ok(),
                                    "Failed to send Request between OS Threads"
                                );
                            }
                            let internal_long_dram_op = PipelineStage {
                                cycle_to_read: self.cycle + DRAM_LATENCY_FAR,
                                calculated_val: 0,
                                register_index: instruction_to_execute.dr + self.context_in_progress * REGS_PER_CONTEXT,
                            };
                            let _ = self.dram_long_queue.push(internal_long_dram_op);
                        } else {
                            self.dram_bytes_read_close += 2;
                            let value = dram[dram_address / 4];
                            let byte_offset = dram_address & 0x2; //will be either 0 or 2
                            let load_dispatch = PipelineStage {
                                cycle_to_read: self.cycle + DRAM_LATENCY_CLOSE,
                                calculated_val: ((value >> (byte_offset * 8)) & 0xFFFF) as i16
                                    as i32 as u32,
                                register_index: instruction_to_execute.dr + self.context_in_progress * REGS_PER_CONTEXT,
                            };
                            let _ = self.dram_short_queue.push(load_dispatch);
                        }
                        self.pc[self.context_in_progress] += 4;
                    }
                    Operation::ReadUnsignedHalfDram => {
                        long_latency_op = true;
                        let dram_address = self.register_file
                            [self.context_in_progress * REGS_PER_CONTEXT + instruction_to_execute.sr1]
                            as usize
                            | (self.memory_bits[self.context_in_progress] as usize) << DRAM_STACK_SIZE_LOG2;
                        assert!(
                            dram_address & 0x1 == 0,
                            "DRAM Half LOADS CAN'T BE UNALIGNED"
                        );
                        if self.top_bits_dram_stack != dram_address / DRAM_STACK_SIZE {
                            self.dram_bytes_read_far += 2;
                            let long_dram_request = LongDramRequest {
                                register_index: instruction_to_execute.dr + self.context_in_progress * REGS_PER_CONTEXT,
                                address: dram_address,
                                op: instruction_to_execute.operation,
                                value_to_write: 0,
                                core_id: self.core_id,
                                origin_stack: self.top_bits_dram_stack,
                            };
                            if let Some(dram_request) = &self.dram_long_request {
                                let could_send = dram_request[dram_address / DRAM_STACK_SIZE]
                                    .send(long_dram_request);
                                assert!(
                                    could_send.is_ok(),
                                    "Failed to send Request between OS Threads"
                                );
                            }
                            let internal_long_dram_op = PipelineStage {
                                cycle_to_read: self.cycle + DRAM_LATENCY_FAR,
                                calculated_val: 0,
                                register_index: instruction_to_execute.dr + self.context_in_progress * REGS_PER_CONTEXT,
                            };
                            let _ = self.dram_long_queue.push(internal_long_dram_op);
                        } else {
                            self.dram_bytes_read_close += 2;
                            let value = dram[dram_address / 4];

                            let byte_offset = dram_address & 0x2; //will be either 0 or 2
                            let load_dispatch = PipelineStage {
                                cycle_to_read: self.cycle + DRAM_LATENCY_CLOSE,
                                calculated_val: ((value >> (byte_offset * 8)) & 0xFFFF) as u16
                                    as u32,
                                register_index: instruction_to_execute.dr + self.context_in_progress * REGS_PER_CONTEXT,
                            };
                            let _ = self.dram_short_queue.push(load_dispatch);
                        }
                        self.pc[self.context_in_progress] += 4;
                    }
                    Operation::ReadWordDram => {
                        long_latency_op = true;
                        let dram_address = self.register_file
                            [self.context_in_progress * REGS_PER_CONTEXT + instruction_to_execute.sr1]
                            as usize
                            | (self.memory_bits[self.context_in_progress] as usize) << DRAM_STACK_SIZE_LOG2;
                        assert!(
                            dram_address & 0x3 == 0,
                            "DRAM Word LOADS CAN'T BE UNALIGNED"
                        );
                        // println!("DRAM ADDRESS FOR CORE {}: {:08X}", self.core_id, dram_address);
                        if self.top_bits_dram_stack != dram_address / DRAM_STACK_SIZE {
                            self.dram_bytes_read_far += 4;
                            let long_dram_request = LongDramRequest {
                                register_index: instruction_to_execute.dr + self.context_in_progress * REGS_PER_CONTEXT,
                                address: dram_address,
                                op: instruction_to_execute.operation,
                                value_to_write: 0,
                                core_id: self.core_id,
                                origin_stack: self.top_bits_dram_stack,
                            };
                            if let Some(dram_request) = &self.dram_long_request {
                                let could_send = dram_request[dram_address / DRAM_STACK_SIZE]
                                    .send(long_dram_request);
                                assert!(
                                    could_send.is_ok(),
                                    "Failed to send Request between OS Threads"
                                );
                            }
                            let internal_long_dram_op = PipelineStage {
                                cycle_to_read: self.cycle + DRAM_LATENCY_FAR,
                                calculated_val: 0,
                                register_index: instruction_to_execute.dr + self.context_in_progress * REGS_PER_CONTEXT,
                            };
                            let _ = self.dram_long_queue.push(internal_long_dram_op);
                        } else {
                            self.dram_bytes_read_close += 4;
                            let value = dram[dram_address / 4];
                            let load_dispatch = PipelineStage {
                                cycle_to_read: self.cycle + DRAM_LATENCY_CLOSE,
                                calculated_val: value,
                                register_index: instruction_to_execute.dr + self.context_in_progress * REGS_PER_CONTEXT,
                            };
                            let _ = self.dram_short_queue.push(load_dispatch);
                        }
                        self.pc[self.context_in_progress] += 4;
                    }
                    Operation::StoreWordDram => {
                        let dram_address = self.register_file
                            [self.context_in_progress * REGS_PER_CONTEXT + instruction_to_execute.sr2]
                            as usize
                            | (self.memory_bits[self.context_in_progress] as usize) << DRAM_STACK_SIZE_LOG2;
                        assert!(
                            dram_address & 0x3 == 0,
                            "DRAM Word LOADS CAN'T BE UNALIGNED"
                        );
                        let value_to_store = self.register_file[self.context_in_progress * REGS_PER_CONTEXT + instruction_to_execute.sr1];
                        if self.top_bits_dram_stack != dram_address / DRAM_STACK_SIZE {
                            self.dram_bytes_wrote_far += 4;
                            let long_dram_request = LongDramRequest {
                                register_index: 0,
                                address: dram_address,
                                op: instruction_to_execute.operation,
                                value_to_write: value_to_store,
                                core_id: self.core_id,
                                origin_stack: self.top_bits_dram_stack,
                            };
                            if let Some(dram_request) = &self.dram_long_request {
                                let could_send = dram_request[dram_address / DRAM_STACK_SIZE]
                                    .send(long_dram_request);
                                assert!(
                                    could_send.is_ok(),
                                    "Failed to send Request between OS Threads"
                                );
                            }
                        } else {
                            self.dram_bytes_wrote_close += 4;
                            dram[dram_address / 4] = value_to_store;
                        }
                        self.pc[self.context_in_progress] += 4;

                    }
                    Operation::StoreHalfDram => {
                        let dram_address = self.register_file
                            [self.context_in_progress * REGS_PER_CONTEXT + instruction_to_execute.sr2]
                            as usize
                            | (self.memory_bits[self.context_in_progress] as usize) << DRAM_STACK_SIZE_LOG2;
                        let value_to_store = self.register_file
                            [self.context_in_progress * REGS_PER_CONTEXT + instruction_to_execute.sr1]
                            & 0xFFFF;
                        assert!(
                            dram_address & 0x1 == 0,
                            "DRAM Half LOADS CAN'T BE UNALIGNED"
                        );
                        if self.top_bits_dram_stack != dram_address / DRAM_STACK_SIZE {
                            self.dram_bytes_wrote_far += 2;
                            let long_dram_request = LongDramRequest {
                                register_index: 0,
                                address: dram_address,
                                op: instruction_to_execute.operation,
                                value_to_write: value_to_store,
                                core_id: self.core_id,
                                origin_stack: self.top_bits_dram_stack,
                            };

                            if let Some(dram_request) = &self.dram_long_request {
                                let could_send = dram_request[dram_address / DRAM_STACK_SIZE]
                                    .send(long_dram_request);
                                assert!(
                                    could_send.is_ok(),
                                    "Failed to send Request between OS Threads"
                                );
                            }
                        } else {
                            self.dram_bytes_wrote_close += 2;
                            let old_value = dram[dram_address / 4];
                            let byte_offset = dram_address & 0x2; //will be either 0 or 2
                            let mut value_to_store = (value_to_store as u32) << (byte_offset * 8);
                            value_to_store =
                                (old_value & !(0xFFFF << (byte_offset * 8))) | value_to_store;
                            dram[dram_address / 4] = value_to_store;
                        }
                        self.pc[self.context_in_progress] += 4;
                    }
                    Operation::StoreByteDram => {
                        let dram_address = self.register_file
                            [self.context_in_progress * REGS_PER_CONTEXT + instruction_to_execute.sr2]
                            as usize
                            | (self.memory_bits[self.context_in_progress] as usize) << DRAM_STACK_SIZE_LOG2;
                        let value_to_store = self.register_file
                            [self.context_in_progress * REGS_PER_CONTEXT + instruction_to_execute.sr1]
                            & 0xFF;
                        if self.top_bits_dram_stack != dram_address / DRAM_STACK_SIZE {
                            self.dram_bytes_wrote_far += 1;
                            let long_dram_request = LongDramRequest {
                                register_index: 0,
                                address: dram_address,
                                op: instruction_to_execute.operation,
                                value_to_write: value_to_store,
                                core_id: self.core_id,
                                origin_stack: self.top_bits_dram_stack,
                            };

                            if let Some(dram_request) = &self.dram_long_request {
                                let could_send = dram_request[dram_address / DRAM_STACK_SIZE]
                                    .send(long_dram_request);
                                assert!(
                                    could_send.is_ok(),
                                    "Failed to send Request between OS Threads"
                                );
                            }
                        } else {
                            self.dram_bytes_wrote_close += 1;
                            let old_value = dram[dram_address / 4];
                            let byte_offset = dram_address & 0x3; //will be between 0 and 3
                            let mut value_to_store = (value_to_store as u32) << (byte_offset * 8);
                            value_to_store =
                                (old_value & !(0xFF << (byte_offset * 8))) | value_to_store;
                            dram[dram_address / 4] = value_to_store;
                        }
                        self.pc[self.context_in_progress] += 4;
                    }
                    Operation::AtomicAddDram => {
                        let dram_address = self.register_file
                            [self.context_in_progress * REGS_PER_CONTEXT + instruction_to_execute.sr2]
                            as usize
                            | (self.memory_bits[self.context_in_progress] as usize) << DRAM_STACK_SIZE_LOG2;
                        let value_to_store = self.register_file
                            [self.context_in_progress * REGS_PER_CONTEXT + instruction_to_execute.sr1];
                        assert!(
                            dram_address & 0x3 == 0,
                            "DRAM Word LOADS CAN'T BE UNALIGNED"
                        );
                        if self.top_bits_dram_stack != dram_address / DRAM_STACK_SIZE {
                            self.dram_bytes_wrote_far += 4;
                            self.dram_bytes_read_far += 4;
                            let long_dram_request = LongDramRequest {
                                register_index: instruction_to_execute.dr + self.context_in_progress * REGS_PER_CONTEXT,
                                address: dram_address,
                                op: instruction_to_execute.operation,
                                value_to_write: value_to_store,
                                core_id: self.core_id,
                                origin_stack: self.top_bits_dram_stack,
                            };
                            if let Some(dram_request) = &self.dram_long_request {
                                let could_send = dram_request[dram_address / DRAM_STACK_SIZE]
                                    .send(long_dram_request);
                                assert!(
                                    could_send.is_ok(),
                                    "Failed to send Request between OS Threads"
                                );
                            }
                            let internal_long_dram_op = PipelineStage {
                                cycle_to_read: self.cycle + DRAM_LATENCY_FAR,
                                calculated_val: 0,
                                register_index: instruction_to_execute.dr + self.context_in_progress * REGS_PER_CONTEXT,
                            };
                            let _ = self.dram_long_queue.push(internal_long_dram_op);
                        } else {
                            self.dram_bytes_wrote_close += 4;
                            self.dram_bytes_read_close += 4;
                            let old_value = dram[dram_address / 4];
                            dram[dram_address / 4] = old_value + value_to_store;
                            let load_dispatch = PipelineStage {
                                cycle_to_read: self.cycle + DRAM_LATENCY_CLOSE,
                                calculated_val: old_value,
                                register_index: instruction_to_execute.dr + self.context_in_progress * REGS_PER_CONTEXT,
                            };
                            let _ = self.dram_short_queue.push(load_dispatch);
                        }
                        self.pc[self.context_in_progress] += 4;
                    }
                    Operation::SendFlit => {
                        let mailbox_index = if instruction_to_execute.imm_1 != 0 {
                            instruction_to_execute.imm_0
                        }
                        else {
                            self.register_file[self.context_in_progress * REGS_PER_CONTEXT + instruction_to_execute.sr2]
                        };
                        let value_to_send = self.register_file
                            [self.context_in_progress * REGS_PER_CONTEXT + instruction_to_execute.sr1];
                        let destination_full = self.register_file
                            [self.context_in_progress * REGS_PER_CONTEXT + instruction_to_execute.dr];

                        let outgoing_flit = Flit {
                            destination_x: (destination_full % (CORES_IN_X * CORES_IN_X_STACK)as u32) as u16,
                            destination_y: (destination_full / (CORES_IN_X * CORES_IN_X_STACK) as u32) as u16,
                            mailbox_id: mailbox_index as u16,
                            value_to_send,
                            cycle_to_read: self.cycle + 1,
                            origin_x: self.x_dim,
                            origin_y: self.y_dim
                        };
                        self.flits_sent += 1;
                        self.flit_sent_manhattan_distance_traversed += (outgoing_flit.origin_x as i64 - outgoing_flit.destination_x as i64).abs() as usize 
                            + (outgoing_flit.origin_y as i64 - outgoing_flit.destination_y as i64).abs() as usize;
                        if self.output_noc_send.is_full() {
                            switch_ctx = true;
                        } else {
                            let _ = self.output_noc_send.push(outgoing_flit);
                            self.pc[self.context_in_progress] += 4;
                        }
                    }
                }
            }
            else{
                while self.core_busy.len() <= self.cycle as usize/NOC_UTIL_EPOCH_LEN {
                    self.core_busy.push(0);
                }
                self.core_busy[self.cycle as usize/NOC_UTIL_EPOCH_LEN] += 1
            }
            self.register_file[REGS_PER_CONTEXT * self.context_in_progress + REGS_PER_CONTEXT - 1] =
                (CTX_CNT * self.core_id as usize + self.context_in_progress) as u32;
            if long_latency_op && instruction_to_execute.dr != REGS_PER_CONTEXT - 1 {
                self.register_ready
                    [REGS_PER_CONTEXT * self.context_in_progress + instruction_to_execute.dr] = false;
            }
            if switch_ctx {
                let mut next_ctx = None;
                let mut search_idx = (self.context_in_progress + 1) % CTX_CNT;
                while search_idx != self.context_in_progress {
                    if self.runnable[search_idx] {
                        next_ctx = Some(search_idx);
                        break;
                    }
                    search_idx = (search_idx + 1) % CTX_CNT;
                }
                self.context_in_progress = match next_ctx {
                    Some(idx) => idx,
                    None => self.ctx_ownership.try_into().expect("There was no owner"),
                };
                flush = true;
            }
        }
        let next_fetched_value: Option<u32>;
        let next_fetch_pc: Option<u16>;
        let next_decoded: Option<DecodedInstruction>;

        if !flush {
            match (self.fetched_value, self.fetch_pc) {
                (Some(word), Some(pc)) => {
                    next_decoded = Some(decode_instruction(word, pc));
                }
                (None, None) => {
                    next_decoded = None;
                }
                _ => {
                    panic!("FETCH/PC PAIRING BROKEN: fetched_value and fetch_pc must be both Some or both None");
                }
            }
        } else {
            next_decoded = None;
        }

        let next_req_pc: u16 = if flush {
            self.pc[self.context_in_progress]
        } else if let Some(instr) = &next_decoded {
            let is_control =
                matches!(
                    instr.operation,
                    Operation::BranchNe
                        | Operation::BranchEq
                        | Operation::BranchGT
                        | Operation::BranchLTE
                );

            if (is_control && instr.branch_hint) || (instr.operation == Operation::Jump && instr.is_imm) {
                instr.imm_0 as u16
            } else {
                (instr.pc as u16).wrapping_add(4)
            }
        } else {
    
            self.pc[self.context_in_progress]
        };

        next_fetched_value = Some(self.read_sram_word(&next_req_pc));
        next_fetch_pc = Some(next_req_pc);

        self.fetched_value = next_fetched_value;
        self.fetch_pc = next_fetch_pc;

        self.decoded_instruction = next_decoded;


        self.cycle += 1;
    }
}

