use std::collections::HashMap;
use std::env;
use std::fs;
use std::io::{self, Write};
use std::path::Path;


#[derive(Copy, Clone, Debug)]
enum FpType {
    Fp32 = 0,
    Fp16 = 1,
    Fp8 = 2,
}

/* ===================== ISA TABLE ===================== */

fn opcode_table() -> HashMap<&'static str, u32> {
    HashMap::from([
        ("add", 0), ("sub", 1), ("and", 2), ("or", 3), ("xor", 4),
        ("sll", 5), ("srl", 6), ("sra", 7), ("mul", 8), ("div", 9), ("mod", 50),

        ("fpadd", 10), ("fpmul", 11), ("fpsub", 12), ("fpmac", 13),
        ("fpeq", 14), ("fplt", 15),
        ("fpsetacc", 16), ("fpstoreacc", 17), ("fpminmax", 18),

        ("beq", 19), ("bne", 20), ("blte", 21), ("bgt", 22),
        ("jmp", 23),

        ("sb", 24), ("sh", 25), ("sw", 26),
        ("lb", 27), ("lbu", 28), ("lh", 29), ("lhu", 30), ("lw", 31),

        ("block", 32), ("nonblock", 33),
        ("yield", 34),
        ("getowner", 35),
        ("setctx", 36),
        ("relinquish", 37),
        ("intena", 38),
        ("setmembits", 39),

        ("lb_d", 40), ("lbu_d", 41), ("lh_d", 42), ("lhu_d", 43),
        ("lw_d", 44), ("sw_d", 45), ("sh_d", 46), ("sb_d", 47),
        ("atomadd_d", 48),

        ("sendflit", 49),
    ])
}

/* ===================== HELPERS ===================== */

fn is_alu(op: u32) -> bool { (0..=9).contains(&op)  || op == 49 || op == 50   }
fn is_fp(op: u32) -> bool { (10..=18).contains(&op) }
fn is_branch(op: u32) -> bool { (19..=22).contains(&op) }

fn parse_reg(s: &str) -> u32 {
    if !s.starts_with('r') {
        panic!("Expected register, got '{s}'");
    }
    let n: u32 = s[1..].parse().expect("Bad register number");
    if n > 15 {
        panic!("Register out of range r0–r15");
    }
    n
}

fn parse_fp_suffix(op: &str) -> (String, FpType) {
    if let Some((base, suf)) = op.split_once('.') {
        let fp = match suf {
            "32" => FpType::Fp32,
            "16" => FpType::Fp16,
            "8"  => FpType::Fp8,
            "data" => FpType::Fp32, // .data treated as FP32 for simplicity
            _ => panic!("Bad FP suffix '.{suf}'"),
        };
        (base.to_lowercase(), fp)
    } else {
        (op.to_lowercase(), FpType::Fp32)
    }
}

fn parse_bool(s: &str) -> bool {
    matches!(s.to_lowercase().as_str(), "1" | "true" | "yes" | "on")
}

fn parse_imm_or_label(tok: &str, labels: &HashMap<String, u16>) -> u16 {
    if let Some(hex) = tok.strip_prefix("0x") {
        return u16::from_str_radix(hex, 16).expect("Bad hex immediate");
    }
    if let Ok(v) = tok.parse::<u32>() {
        if v > 0xFFFF {
            panic!("Immediate out of range");
        }
        return v as u16;
    }
    if let Some(addr) = labels.get(tok) {
        return *addr;
    }
    panic!("Unknown immediate or label '{tok}'");
}

/* ===================== PASS 1: LABELS ===================== */

fn collect_labels(src: &str) -> (u16, HashMap<String, u16>) {
    let mut origin: Option<u16> = None;
    let mut pc: u16 = 0;
    let mut labels = HashMap::new();

    for (lineno, raw) in src.lines().enumerate() {
        let line = raw.split('#').next().unwrap().trim();
        if line.is_empty() { continue; }

        if line.starts_with(".org") {
            let parts: Vec<_> = line.split_whitespace().collect();
            if parts.len() != 2 {
                panic!("Bad .org on line {}", lineno + 1);
            }
            let o = parse_imm_or_label(parts[1], &labels);
            if o & 0x3 != 0 {
                panic!(".org must be 4-byte aligned");
            }
            origin = Some(o);
            pc = o;
            continue;
        }

        if line.ends_with(':') {
            let label = line[..line.len() - 1].to_string();
            if labels.contains_key(&label) {
                panic!("Duplicate label '{label}'");
            }
            labels.insert(label, pc);
            continue;
        }

        if origin.is_none() {
            panic!("Program must start with .org");
        }
        pc = pc.wrapping_add(4);
    }

    (origin.expect("Missing .org"), labels)
}

/* ===================== PASS 2: ENCODE ===================== */

fn assemble_instruction(
    line: &str,
    labels: &HashMap<String, u16>,
) -> u32 {
    let line = line.trim();
    if line.is_empty() {
        return 0;
    }

    let mut instr = 0u32;
    let mut parts = line.split_whitespace();
    let op_raw = parts.next().unwrap();

    let rest = parts.collect::<Vec<_>>().join(" ");
    let args: Vec<_> = rest
        .split(',')
        .map(|s| s.trim())
        .filter(|s| !s.is_empty())
        .collect();

    // Handle .data (case-insensitive), emit a raw u32 word.
    // Syntax:
    //   .data 123
    //   .data 0xDEADBEEF
    if op_raw.eq_ignore_ascii_case(".data") {
        let v = rest.trim();
        if v.is_empty() {
            panic!(".data requires a value");
        }
        if let Some(hex) = v.strip_prefix("0x").or_else(|| v.strip_prefix("0X")) {
            return u32::from_str_radix(hex.trim(), 16).expect("Bad .data hex value");
        } else {
            return v.parse::<u32>().expect("Bad .data value");
        }
    }

    let (op_name_base, fp_type) = parse_fp_suffix(op_raw);
    let op_name = op_name_base.to_lowercase();

    let opcode = *opcode_table()
        .get(op_name.as_str())
        .unwrap_or_else(|| panic!("Unknown opcode '{}'", op_name));

    instr |= opcode & 0x7F;
    if is_fp(opcode) {
        instr |= (fp_type as u32) << 20;
    }

    // Field setters (mask to 4 bits for regs, but assume parse_reg already bounds)
    let set_dr = |i: &mut u32, r: u32| *i |= (r & 0xF) << 7;
    let set_sr1 = |i: &mut u32, r: u32| *i |= (r & 0xF) << 11;
    let set_sr2 = |i: &mut u32, r: u32| *i |= (r & 0xF) << 16;
    let set_imm = |i: &mut u32, v: u16| *i |= (v as u32) << 16;
    let set_imm1 = |i: &mut u32, v: u32| *i |= v << 15;

    /* ---------- SPECIAL FORMS (per your spec) ---------- */

    match opcode {
        32 | 33 => { // block / nonblock: block rD, mailbox
            if args.len() != 2 { panic!("{} expects: {} rD, MAILBOX", op_name, op_name); }
            set_dr(&mut instr, parse_reg(args[0]));
            if args[1].starts_with('r'){
                set_sr2(&mut instr, parse_reg(args[1]));
                set_imm1(&mut instr, 0);
            } else {
                set_imm(&mut instr, parse_imm_or_label(args[1], labels));
                set_imm1(&mut instr, 1);
            }
            return instr;
        }
        34 => { // yield: yield rD
            if args.len() != 1 { panic!("yield expects: yield rD"); }
            set_dr(&mut instr, parse_reg(args[0]));
            set_imm1(&mut instr, 0);
            return instr;
        }
        35 | 37 => { // getowner / relinquish: no operands
            if !args.is_empty() { panic!("{} takes no operands", op_name); }
            return instr;
        }
        36 => { // setctx: setctx IMM
            if args.len() != 1 { panic!("setctx expects: setctx IMM"); }
            set_imm(&mut instr, parse_imm_or_label(args[0], labels));
            set_imm1(&mut instr, 1);
            return instr;
        }
        38 => { // intena: intena MAILBOX, bool
            if args.len() != 2 { panic!("intena expects: intena MAILBOX, true|false"); }
            set_imm(&mut instr, parse_imm_or_label(args[0], labels));
            if parse_bool(args[1]) { set_imm1(&mut instr, 1); } else {set_imm1(&mut instr, 0);}
            return instr;
        }
        39 => { // setmembits: setmembits rS1
            if args.len() != 1 { panic!("setmembits expects: setmembits rS1"); }
            set_sr1(&mut instr, parse_reg(args[0]));
            return instr;
        }
        23 => { // JMP
            if args.len() != 23 { panic!("jmp expects: dr rS1|IMM16"); }
            set_dr(&mut instr, parse_reg(args[0]));

            if args[1].starts_with('r'){
                set_sr2(&mut instr, parse_reg(args[1]));
                set_imm1(&mut instr, 0);
            } else {
                set_imm(&mut instr, parse_imm_or_label(args[1], labels));
                set_imm1(&mut instr, 1);
            }
            return instr;
        }
        _ => {}
    }

    /* ---------- MEMORY OPS ---------- */

    // SRAM loads: lbs/lbu/lhs/lhu/lw  (27..=31)
    // DRAM loads: ldsb_d/ldub_d/ldsh_d/lduh_d/ldw_d (40..=44)
    let is_load_s = (27..=31).contains(&opcode);
    let is_load_d = (40..=44).contains(&opcode);
    // SRAM stores: sb/sh/sw (24..=26)
    // DRAM stores: stb_d/sth_d/stw_d (47..=45)  (note: 45..=47 are stores)
    let is_store_s = (24..=26).contains(&opcode);
    let is_store_d = (45..=47).contains(&opcode);
    // Atomic add DRAM: 48
    let is_atomicadd = opcode == 48;

    if is_load_s {
        if args.len() != 2 {
            panic!("{} expects: {} rD, rBASE OR IMM16|label", op_name, op_name);
        }
        let rd = parse_reg(args[0]);
        let (is_imm, base) = if args[1].starts_with('r') {
            (false, parse_reg(args[1]) as u16)
        } else {
            if args[1].starts_with("0x") {
                (true, parse_imm_or_label(args[1], labels))
            } else {
                (true, args[1].parse::<u16>().expect("Bad immediate"))
            }
        };

        set_dr(&mut instr, rd);
        if is_imm{
            set_imm(&mut instr, base);
            set_imm1(&mut instr, 1);
        }
        else {
            set_sr2(&mut instr, base as u32);
            set_imm1(&mut instr, 0);
        }
        return instr;
    }

    if is_load_d {
        if args.len() != 2 {
            panic!("{} expects: {} rD, rBASE", op_name, op_name);
        }
        let rd = parse_reg(args[0]);
        let base = parse_reg(args[1]);

        set_dr(&mut instr, rd);
        set_sr1(&mut instr, base);
        set_imm1(&mut instr, 0);
        return instr;
    }

    if is_store_s {
        if args.len() != 2 {
            panic!("{} expects: {} rD, rBASE OR IMM16|label", op_name, op_name);
        }
        let rs = parse_reg(args[0]);
        let (is_imm, base) = if args[1].starts_with('r') {
            (false, parse_reg(args[1]) as u16)
        } else {
            if args[1].starts_with("0x") {
                (true, parse_imm_or_label(args[1], labels))
            } else {
                (true, args[1].parse::<u16>().expect("Bad immediate"))
            }
        };
        set_sr1(&mut instr, rs);

        if is_imm{
            set_imm(&mut instr, base);
            set_imm1(&mut instr, 1);
        }
        else {
            set_sr2(&mut instr, base as u32);
            set_imm1(&mut instr, 0);
        }
        return instr;
    }

    if is_store_d  {
        // format: OP rSRC, rBASE, IMM16|label
        // encoding: sr1=value, sr2=base, imm0=offset, imm1=1
        if args.len() != 2 {
            panic!("{} expects: {} rSRC, rBASE, IMM16|label", op_name, op_name);
        }
        let src = parse_reg(args[0]);
        let base = parse_reg(args[1]);

        set_sr1(&mut instr, src);
        set_sr2(&mut instr, base);
        set_imm1(&mut instr, 0);
        return instr;
    }

    if is_atomicadd {
        // format: atomadd_d rSRC, rBASE, IMM16|label
        // encoding: sr1=value, sr2=base, imm0=offset, imm1=1
        if args.len() != 3 {
            panic!("atomadd_d expects: atomadd_d rdest, rsrc, rbase");
        }
        let dest = parse_reg(args[0]);
        let src = parse_reg(args[1]);
        let base = parse_reg(args[2]);

        set_dr(&mut instr, dest);
        set_sr1(&mut instr, src);
        set_sr2(&mut instr, base);
        set_imm1(&mut instr, 0);
        return instr;
    }

    /* ---------- BRANCH ---------- */

    if is_branch(opcode) {
        // format: OP rS1, rS2, ABS_ADDR|label
        // special rule: sr2 stored in dr slot
        if args.len() != 4 {
            panic!("{} expects: {} rS1, rS2, ABS_ADDR|label", op_name, op_name);
        }
        set_sr1(&mut instr, parse_reg(args[0]));
        set_dr(&mut instr, parse_reg(args[1])); // sr2 in dr slot
        set_imm(&mut instr, parse_imm_or_label(args[2], labels));
        set_imm1(&mut instr, parse_bool(args[3]) as u32);
        return instr;
    }

    /* ---------- ALU ---------- */

    if is_alu(opcode) {
        // format: OP rD, rS1, (rS2|IMM16|label)
        if args.len() != 3 {
            panic!("{} expects: {} rD, rS1, rS2|IMM16|label", op_name, op_name);
        }
        set_dr(&mut instr, parse_reg(args[0]));
        set_sr1(&mut instr, parse_reg(args[1]));
        if args[2].starts_with('r') {
            set_sr2(&mut instr, parse_reg(args[2]));
            set_imm1(&mut instr, 0);
        } else {
            set_imm(&mut instr, parse_imm_or_label(args[2], labels));
            set_imm1(&mut instr, 1);
        }
        return instr;
    }

    /* ---------- FP ---------- */

    if is_fp(opcode) {
        // NOTE: FP ops never take immediates in your ISA.
        // Any operand that isn't a register is an error.

        let reg = |s: &str| -> u32 {
            if !s.starts_with('r') {
                panic!("FP op {} expects register operands (got '{}')", op_name, s);
            }
            parse_reg(s)
        };

        match opcode {
            // fpadd/fpmul/fpsub/fpminmax: rD, rS1, rS2
            10 | 11 | 12 | 14 | 15 | 18 => {
                if args.len() != 3 {
                    panic!("{} expects: {}.(32|16|8) rD, rS1, rS2", op_name, op_name);
                }
                set_dr(&mut instr, reg(args[0]));
                set_sr1(&mut instr, reg(args[1]));
                set_sr2(&mut instr, reg(args[2]));
                set_imm1(&mut instr, 0);
                return instr;
            }

            // fpmac: rS1, rS2  (accumulator is implicit)
            13 => {
                if args.len() != 2 {
                    panic!("{} expects: {}.(32|16|8) rS1, rS2", op_name, op_name);
                }
                set_sr1(&mut instr, reg(args[0]));
                set_sr2(&mut instr, reg(args[1]));
                set_imm1(&mut instr, 0);
                return instr;
            }

            // fpsetacc: rS1   (load accumulator from a register)
            16 => {
                if args.len() != 1 {
                    panic!("{} expects: {}.(32|16|8) rS1", op_name, op_name);
                }
                set_sr1(&mut instr, reg(args[0]));
                set_imm1(&mut instr, 0);
                return instr;
            }

            // fpstoreacc: rD  (store accumulator to dest register)
            17 => {
                if args.len() != 1 {
                    panic!("{} expects: {}.(32|16|8) rD", op_name, op_name);
                }
                set_dr(&mut instr, reg(args[0]));
                set_imm1(&mut instr, 0);
                return instr;
            }

            _ => {
                panic!("Unhandled FP opcode {} ({})", opcode, op_name);
            }
        }
    }


    println!("BAD LINE: {}", line);
    panic!("Unhandled opcode encoding");
}


/* ===================== DRIVER ===================== */

pub fn assemble_program(src: &str) -> (u16, Vec<u32>, Vec<String>) {
    let (origin, labels) = collect_labels(src);
    let mut words = Vec::new();
    let mut lines = vec![];
    for raw in src.lines() {
        let line = raw.split('#').next().unwrap().trim();
        if line.is_empty() || line.starts_with(".org") || line.ends_with(':') {
            continue;
        }
        words.push(assemble_instruction(line, &labels));
        lines.push(raw.to_string());
    }

    (origin, words, lines)
}

/* ===================== DEMO ===================== */

// fn main() {
//     let src = r#"
//         .org 0x0100
//         Add r1, r1, 1
//         start:
//             add r1, r1, 1
//             blte r1, r2, start

//         done:
//             yield r6
//         .data 0xDEADBEEF
//         yield r7
//     "#;
  
//     let (origin, words) = assemble_program(src);
//     println!("origin = 0x{:04X}", origin);

//     let mut pc = origin;
//     for w in words {
//         println!("0x{:04X}: {:08X}", pc, w);
//         pc += 4;
//     }
// }

fn main() -> io::Result<()> {
    let args: Vec<String> = env::args().collect();

    if args.len() < 2 || args.len() > 3 {
        eprintln!("Usage:");
        eprintln!("  {} <input.asm> [output.bin]", args[0]);
        std::process::exit(1);
    }

    let input_path = &args[1];
    let output_path = args.get(2);

    // ---- Read source file ----
    let src = fs::read_to_string(input_path)
        .unwrap_or_else(|e| panic!("Failed to read '{}': {}", input_path, e));

    // ---- Assemble ----
    let (origin, words, lines) = assemble_program(&src);

    println!("Origin = 0x{:04X}", origin);
    println!("Bytes = {}", words.len() * 4);
    println!();

    // ---- Output binary to stdout ----
    println!("===================== BINARY =====================");
    let mut pc = origin;
    let mut i = 0;
    for w in &words {
        println!(
            "{:032b}  #PC = 0x{:08X},     line: {}",
            w,
            pc,
            lines[i]
        );
        pc = pc.wrapping_add(4);
        i+=1;
    }

    println!();

    // ---- Output hex to stdout ----
    println!("============ HEX ============");
    pc = origin;
    i=0;
    for w in &words {
        println!("0x{:08X},   //PC = 0x{:08X},     line: {}", w, pc, lines[i]);
        pc = pc.wrapping_add(4);
        i+=1;
    }

    // ---- Optional binary file output ----
    if let Some(out_path) = output_path {
        let path = Path::new(out_path);
        let mut file = fs::File::create(path)
            .unwrap_or_else(|e| panic!("Failed to create '{}': {}", out_path, e));

        // Write words as little-endian u32
        for w in &words {
            file.write_all(&w.to_le_bytes())?;
        }

        println!();
        println!(
            "Wrote {} words ({} bytes) to '{}'",
            words.len(),
            words.len() * 4,
            out_path
        );
    }

    Ok(())
}
