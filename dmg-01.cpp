#include <iostream>
#include <cstdint>
#include <bitset>
#include <bit>
#include <array>
#include <optional>
#include "SafeInt.hpp"

using u8 = uint8_t;
using i8 = int8_t;
using u16 = uint16_t;

namespace consts {
    const u8 zero_flag_pos = 7;
    const u8 sub_flag_pos = 6;
    const u8 half_carry_flag_pos = 5;
    const u8 carry_flag_pos = 4;
}

struct FlagsRegister {
    bool zero, subtract, half_carry, carry;

    u8 get() const;
    void set(u8 byte);
};

u8 FlagsRegister::get() const {
    return (
        (zero ? 1 : 0) << consts::zero_flag_pos |
        (subtract ? 1 : 0) << consts::sub_flag_pos |
        (half_carry ? 1 : 0) << consts::half_carry_flag_pos |
        (carry ? 1 : 0) << consts::carry_flag_pos
    );
}

void FlagsRegister::set(u8 byte) {
    zero = ((byte >> consts::zero_flag_pos) & 0) != 0;
    subtract = ((byte >> consts::sub_flag_pos) & 0) != 0;
    half_carry = ((byte >> consts::half_carry_flag_pos) & 0) != 0;
    carry = ((byte >> consts::carry_flag_pos) & 0) != 0;
}

struct Register {
    u8 first, second;

    bool is_flag;
    FlagsRegister flags;

    Register(u8 first, u8 second) : first(first), second(second), is_flag(false) {}
    Register(u8 first, FlagsRegister f) : first(first), is_flag(true), flags(f) {}

    u16 get() const;
    void set(u16 value);
};

u16 Register::get() const {
    if (is_flag) {
        return (static_cast<u16>(first) << 8 | static_cast<u16>(flags.get()));
    }
    else {
        return (static_cast<u16>(first) << 8 | static_cast<u16>(second));
    }
}

void Register::set(u16 value) {
    first = static_cast<u8>(((value & 0xff00) >> 8));
    
    if (is_flag) {
        flags.set(value & 0xff);
    }
    else {
        second = static_cast<u8>(value & 0xff);
    }
}

struct Registers {
    Register AF;
    Register BC;
    Register DE;
    Register HL;

    Registers(u8 a, u8 b, u8 c, u8 d, u8 e, FlagsRegister f, u8 h, u8 l)
        : AF(a, f), BC(b, c), DE(d, e), HL(h, l) {}
};

enum struct InstructionEnum : u8 {
    ADD, ADDHL, ADDC, SUB, SBC, AND, OR, XOR, CP, INC, DEC, CCF, SCF, RRA, RLA, RRCA, RLCA, CPL, BIT,
    RESET, SET, SRL, RR, RL, RRC, RLC, SRA, SLA, SWAP,
    JP, JR, JP_HL,
};

enum struct ArithmeticTarget : u8 {
    A, B, C, D, E, H, L,
    AF, BC, DE, HL
};

enum struct RegisterBit : u8 {
    _0, _1, _2, _3, _4, _5, _6, _7
};

enum struct JumpTest : u8 {
    NotZero, Zero, NotCarry, Carry, Always
};

struct Instruction {
    InstructionEnum inst_enum;
    ArithmeticTarget target;
    RegisterBit bit;
    JumpTest test;

    Instruction(InstructionEnum inst_enum) : inst_enum(inst_enum) {}
    Instruction(InstructionEnum inst_enum, ArithmeticTarget target) : inst_enum(inst_enum), target(target) {}
    Instruction(InstructionEnum inst_enum, ArithmeticTarget target, RegisterBit bit) : inst_enum(inst_enum), target(target), bit(bit) {}
    Instruction(InstructionEnum inst_enum, JumpTest test) : inst_enum(inst_enum), test(test) {}

    static std::optional<Instruction> from_byte(u8 byte, bool prefixed);
    static std::optional<Instruction> from_byte_prefixed(u8 byte);
    static std::optional<Instruction> from_byte_not_prefixed(u8 byte);
};

struct ADD : Instruction { ADD(ArithmeticTarget target) : Instruction(InstructionEnum::ADD, target) {} };
struct ADDHL : Instruction { ADDHL(ArithmeticTarget target) : Instruction(InstructionEnum::ADDHL, target) {} };
struct ADC : Instruction { ADC(ArithmeticTarget target) : Instruction(InstructionEnum::ADDC, target) {} };
struct SUB : Instruction { SUB(ArithmeticTarget target) : Instruction(InstructionEnum::SUB, target) {} };
struct SBC : Instruction { SBC(ArithmeticTarget target) : Instruction(InstructionEnum::SBC, target) {} };
struct AND : Instruction { AND(ArithmeticTarget target) : Instruction(InstructionEnum::AND, target) {} };
struct OR : Instruction { OR(ArithmeticTarget target) : Instruction(InstructionEnum::OR, target) {} };
struct XOR : Instruction { XOR(ArithmeticTarget target) : Instruction(InstructionEnum::XOR, target) {} };
struct CP : Instruction { CP(ArithmeticTarget target) : Instruction(InstructionEnum::CP, target) {} };
struct INC : Instruction { INC(ArithmeticTarget target) : Instruction(InstructionEnum::INC, target) {} };
struct DEC : Instruction { DEC(ArithmeticTarget target) : Instruction(InstructionEnum::DEC, target) {} };
struct CCF : Instruction { CCF() : Instruction(InstructionEnum::CCF) {} };
struct SCF : Instruction { SCF() : Instruction(InstructionEnum::SCF) {} };
struct RRA : Instruction { RRA() : Instruction(InstructionEnum::RRA) {} };
struct RLA : Instruction { RLA() : Instruction(InstructionEnum::RLA) {} };
struct RRCA : Instruction { RRCA() : Instruction(InstructionEnum::RRCA) {} };
struct RLCA : Instruction { RLCA() : Instruction(InstructionEnum::RLCA) {} };
struct CPL : Instruction { CPL() : Instruction(InstructionEnum::CPL) {} };
struct BIT : Instruction { BIT(RegisterBit bit, ArithmeticTarget target) : Instruction(InstructionEnum::BIT, target, bit) {} };
struct RESET : Instruction { RESET(RegisterBit bit, ArithmeticTarget target) : Instruction(InstructionEnum::RESET, target, bit) {} };
struct SET : Instruction { SET(RegisterBit bit, ArithmeticTarget target) : Instruction(InstructionEnum::SET, target, bit) {} };
struct SRL : Instruction { SRL(ArithmeticTarget target) : Instruction(InstructionEnum::SRL, target) {} };
struct RR : Instruction { RR(ArithmeticTarget target) : Instruction(InstructionEnum::RR, target) {} };
struct RL : Instruction { RL(ArithmeticTarget target) : Instruction(InstructionEnum::RL, target) {} };
struct RRC : Instruction { RRC(ArithmeticTarget target) : Instruction(InstructionEnum::RRC, target) {} };
struct RLC : Instruction { RLC(ArithmeticTarget target) : Instruction(InstructionEnum::RLC, target) {} };
struct SRA : Instruction { SRA(ArithmeticTarget target) : Instruction(InstructionEnum::SRA, target) {} };
struct SLA : Instruction { SLA(ArithmeticTarget target) : Instruction(InstructionEnum::SLA, target) {} };
struct SWAP : Instruction { SWAP(ArithmeticTarget target) : Instruction(InstructionEnum::SWAP, target) {} };

struct JP : Instruction { JP(JumpTest test) : Instruction(InstructionEnum::JP, test) {} };
struct JR : Instruction { JR(JumpTest test) : Instruction(InstructionEnum::JR, test) {} };
struct JP_HL : Instruction { JP_HL() : Instruction(InstructionEnum::JP_HL) {} };

struct MemoryBus {
    std::array<u8, 0xFFFF> memory;

    u8 read_byte(u8 address) {
        return memory[address];
    }
};

struct CPU {
    Registers registers;
    u16 pc;
    MemoryBus bus;

    u16 execute(Instruction instruction);
    void step();

    u8 add(u8 value);
    u16 addhl(u16 value);
    u8 adc(u8 value);
    u8 sub(u8 value);
    u8 sbc(u8 value);
    u8 bitwise_and(u8 value);
    u8 bitwise_or(u8 value);
    u8 bitwise_xor(u8 value);
    void cp(u8 value);
    u8 inc(u8 value);
    u8 dec(u8 value);
    void ccf();
    void scf();
    void rra();
    void rla();
    void rrca();
    void rlca();
    void cpl();
    void bit(u8 reg, u8 bit);
    u8 reset(u8 reg, u8 bit);
    u8 set(u8 reg, u8 bit);
    u8 srl(u8 value);
    u8 rr(u8 value);
    u8 rl(u8 value);
    u8 rrc(u8 value);
    u8 rlc(u8 value);
    u8 sra(u8 value);
    u8 sla(u8 value);
    u8 swap(u8 reg);

    u16 jump(bool should_jump);
    u16 jr(bool should_jump);
    u16 jp_hl();
};

u8 CPU::add(u8 value) {
    u8 new_value = 0;
    bool did_overflow = !SafeAdd(registers.AF.first, value, new_value);
    registers.AF.flags.zero = new_value == 0;
    registers.AF.flags.subtract = false;
    registers.AF.flags.carry = did_overflow;
    registers.AF.flags.half_carry = (registers.AF.first & 0xf) + (value & 0xf) > 0xf;
    return new_value;
}

u16 CPU::addhl(u16 value) {
    u16 new_value = 0;
    bool did_overflow = !SafeAdd(registers.HL.get(), value, new_value);
    registers.AF.flags.zero = new_value == 0;
    registers.AF.flags.subtract = false;
    registers.AF.flags.carry = did_overflow;
    registers.AF.flags.half_carry = (registers.HL.get() & 0xf) + (value & 0xf) > 0xf;
    return new_value;
}

u8 CPU::adc(u8 value) {
    u8 new_value = 0;
    u8 val = value + (registers.AF.flags.carry ? 1 : 0);
    bool did_overflow = !SafeAdd(registers.AF.first, val, new_value);
    registers.AF.flags.zero = new_value == 0;
    registers.AF.flags.subtract = false;
    registers.AF.flags.carry = did_overflow;
    //TODO(Andrea): check for correctness, in particular value over val
    registers.AF.flags.half_carry = (registers.AF.first & 0xf) + (val & 0xf) > 0xf;
    return new_value;
}

u8 CPU::sub(u8 value) {
    u8 new_value = 0;
    bool did_overflow = !SafeSubtract(registers.AF.first, value, new_value);
    registers.AF.flags.zero = new_value == 0;
    registers.AF.flags.subtract = true;
    registers.AF.flags.carry = did_overflow;
    registers.AF.flags.half_carry = (registers.AF.first & 0xf) + (value & 0xf) > 0xf;
    return new_value;
}

u8 CPU::sbc(u8 value) {
    u8 new_value = 0;
    u8 val = value - (registers.AF.flags.carry ? 1 : 0);
    bool did_overflow = !SafeSubtract(registers.AF.first, val, new_value);
    registers.AF.flags.zero = new_value == 0;
    registers.AF.flags.subtract = true;
    registers.AF.flags.carry = did_overflow;
    registers.AF.flags.half_carry = (registers.AF.first & 0xf) + (val & 0xf) > 0xf;
    return new_value;
}

u8 CPU::bitwise_and(u8 value) {
    u8 new_value = value & registers.AF.first;
    registers.AF.flags.zero = new_value == 0;
    registers.AF.flags.subtract = false;
    registers.AF.flags.carry = false;
    registers.AF.flags.half_carry = (registers.AF.first & 0xf) + (value & 0xf) > 0xf;
    return new_value;
}

u8 CPU::bitwise_or(u8 value) {
    u8 new_value = value | registers.AF.first;
    registers.AF.flags.zero = new_value == 0;
    registers.AF.flags.subtract = false;
    registers.AF.flags.carry = false;
    registers.AF.flags.half_carry = (registers.AF.first & 0xf) + (value & 0xf) > 0xf;
    return new_value;
}

u8 CPU::bitwise_xor(u8 value) {
    u8 new_value = value ^ registers.AF.first;
    registers.AF.flags.zero = new_value == 0;
    registers.AF.flags.subtract = false;
    registers.AF.flags.carry = false;
    registers.AF.flags.half_carry = (registers.AF.first & 0xf) + (value & 0xf) > 0xf;
    return new_value;
}

void CPU::cp(u8 value) {
    u8 new_value = 0;
    bool did_overflow = !SafeSubtract(registers.AF.first, value, new_value);
    registers.AF.flags.zero = new_value == 0;
    registers.AF.flags.subtract = true;
    registers.AF.flags.carry = did_overflow;
    registers.AF.flags.half_carry = (registers.AF.first & 0xf) + (value & 0xf) > 0xf;
}

u8 CPU::inc(u8 value) {
    u8 new_value = 0;
    bool did_overflow = !SafeAdd(value, 1, new_value);
    registers.AF.flags.zero = new_value == 0;
    registers.AF.flags.subtract = false;
    registers.AF.flags.carry = did_overflow;
    registers.AF.flags.half_carry = (registers.AF.first & 0xf) + (value & 0xf) > 0xf;
    return new_value;
}

u8 CPU::dec(u8 value) {
    u8 new_value = 0;
    bool did_overflow = !SafeSubtract(value, 1, new_value);
    registers.AF.flags.zero = new_value == 0;
    registers.AF.flags.subtract = false;
    registers.AF.flags.carry = did_overflow;
    registers.AF.flags.half_carry = (registers.AF.first & 0xf) + (value & 0xf) > 0xf;
    return new_value;
}

void CPU::ccf() {
    registers.AF.flags.carry = registers.AF.flags.carry ? false : true;
}

void CPU::scf() {
    registers.AF.flags.subtract = false;
    registers.AF.flags.carry = true;
}

void CPU::rra() {
    auto reg_a = registers.AF.first;
    auto reg_a_bitset = std::bitset<8>(reg_a);
    auto original_carry = registers.AF.flags.carry;
    auto original_least_significant = reg_a_bitset[7];

    auto rotated_reg_a = std::bitset<8>(std::rotr(reg_a, 1));
    //replace most-significant bit with carry flag's previous value
    rotated_reg_a[0] = original_carry;
    auto value = static_cast<u8>(rotated_reg_a.to_ulong());
    registers.AF.first = value;

    //registers.AF.flags.zero = value == 0;
    registers.AF.flags.subtract = false;
    registers.AF.flags.carry = original_least_significant;
    registers.AF.flags.half_carry = 0;
}

void CPU::rla() {
    auto reg_a = registers.AF.first;
    auto reg_a_bitset = std::bitset<8>(reg_a);
    auto original_carry = registers.AF.flags.carry;
    auto original_most_significant = reg_a_bitset[0];

    auto rotated_reg_a = std::bitset<8>(std::rotl(reg_a, 1));
    //replace least-significant bit with carry flag's previous value
    rotated_reg_a[7] = original_carry;
    auto value = static_cast<u8>(rotated_reg_a.to_ulong());
    registers.AF.first = value;

    //registers.AF.flags.zero = value == 0;
    registers.AF.flags.subtract = false;
    registers.AF.flags.carry = original_most_significant;
    registers.AF.flags.half_carry = 0;
}

void CPU::rrca() {
    auto reg_a = registers.AF.first;
    auto reg_a_bitset = std::bitset<8>(reg_a);
    auto original_least_significant = reg_a_bitset[7];

    auto rotated_reg_a = std::bitset<8>(std::rotr(reg_a, 1));
    //replace most-significant bit with least-significant's previous value
    rotated_reg_a[0] = original_least_significant;
    auto value = static_cast<u8>(rotated_reg_a.to_ulong());
    registers.AF.first = value;

    //registers.AF.flags.zero = value == 0;
    registers.AF.flags.subtract = false;
    registers.AF.flags.carry = original_least_significant;
    registers.AF.flags.half_carry = 0;
}

void CPU::rlca() {
    //TODO(Andrea): check for correctness, copy-paste from RRCA
    auto reg_a = registers.AF.first;
    auto reg_a_bitset = std::bitset<8>(reg_a);
    auto original_least_significant = reg_a_bitset[7];

    auto rotated_reg_a = std::bitset<8>(std::rotl(reg_a, 1));
    //replace most-significant bit with least-significant's previous value
    rotated_reg_a[0] = original_least_significant;
    auto value = static_cast<u8>(rotated_reg_a.to_ulong());
    registers.AF.first = value;

    //registers.AF.flags.zero = value == 0;
    registers.AF.flags.subtract = false;
    registers.AF.flags.carry = original_least_significant;
    registers.AF.flags.half_carry = 0;
}

void CPU::cpl() {
    auto reg_mirror = std::bitset<8>(registers.AF.first);
    for (int i = 0; i < 8; i++) {
        reg_mirror[i].flip();
    }
    registers.AF.first = static_cast<u8>(reg_mirror.to_ulong());

    registers.AF.flags.subtract = false;
}

void CPU::bit(u8 reg, u8 bit) {
    auto reg_mirror = std::bitset<8>(reg);
    bool res = reg_mirror[static_cast<size_t>(7) - bit];

    //registers.AF.flags.subtract = false;
    registers.AF.flags.zero = !res;
    registers.AF.flags.half_carry = true;
}

u8 CPU::reset(u8 reg, u8 bit) {
    auto reg_mirror = std::bitset<8>(reg);
    reg_mirror[bit] = false;
    u8 res = static_cast<u8>(reg_mirror.to_ulong());

    registers.AF.flags.zero = res == 0;
    registers.AF.flags.subtract = false;

    return res;
}

u8 CPU::set(u8 reg, u8 bit) {
    auto reg_mirror = std::bitset<8>(reg);
    reg_mirror[bit] = true;
    u8 res = static_cast<u8>(reg_mirror.to_ulong());

    registers.AF.flags.subtract = false;

    return res;
}

u8 CPU::srl(u8 value) {
    auto reg_mirror = std::bitset<8>(value);
    bool least_significant_bit = reg_mirror[7];
    u8 res = value >> 1;

    registers.AF.flags.zero = res == 0;
    registers.AF.flags.subtract = false;
    registers.AF.flags.carry = least_significant_bit;
    registers.AF.flags.half_carry = false;

    return res;
}

u8 CPU::rr(u8 value) {
    auto reg_bitset = std::bitset<8>(value);
    auto original_carry = registers.AF.flags.carry;
    auto original_least_significant = reg_bitset[7];

    auto rotated_reg = std::bitset<8>(std::rotr(value, 1));
    //replace most-significant bit with carry flag's previous value
    rotated_reg[0] = original_carry;
    auto res = static_cast<u8>(rotated_reg.to_ulong());

    registers.AF.flags.zero = res == 0;
    registers.AF.flags.subtract = res < 0;
    registers.AF.flags.carry = original_least_significant;
    registers.AF.flags.half_carry = 0;

    return res;
}

u8 CPU::rl(u8 value) {
    auto reg_bitset = std::bitset<8>(value);
    auto original_carry = registers.AF.flags.carry;
    auto original_least_significant = reg_bitset[7];

    auto rotated_reg = std::bitset<8>(std::rotl(value, 1));
    //replace most-significant bit with carry flag's previous value
    rotated_reg[0] = original_carry;
    auto res = static_cast<u8>(rotated_reg.to_ulong());

    registers.AF.flags.zero = res == 0;
    registers.AF.flags.subtract = res < 0;
    registers.AF.flags.carry = original_least_significant;
    registers.AF.flags.half_carry = 0;

    return res;
}

u8 CPU::rrc(u8 value) {
    auto reg_bitset = std::bitset<8>(value);
    auto original_least_significant = reg_bitset[7];

    auto rotated_reg = std::bitset<8>(std::rotr(value, 1));
    //replace most-significant bit with least significant bit's previous value
    rotated_reg[0] = original_least_significant;
    auto res = static_cast<u8>(rotated_reg.to_ulong());

    registers.AF.flags.zero = res == 0;
    registers.AF.flags.subtract = res < 0;
    registers.AF.flags.carry = original_least_significant;
    registers.AF.flags.half_carry = 0;

    return res;
}

u8 CPU::rlc(u8 value) {
    auto reg_bitset = std::bitset<8>(value);
    auto original_most_significant = reg_bitset[0];

    auto rotated_reg = std::bitset<8>(std::rotr(value, 1));
    //replace most-significant bit with least significant bit's previous value
    rotated_reg[7] = original_most_significant;
    auto res = static_cast<u8>(rotated_reg.to_ulong());

    registers.AF.flags.zero = res == 0;
    registers.AF.flags.subtract = res < 0;
    registers.AF.flags.carry = original_most_significant;
    registers.AF.flags.half_carry = 0;

    return res;
}

u8 CPU::sra(u8 value) {
    auto reg_bitset = std::bitset<8>(value);
    auto original_least_significant = reg_bitset[7];
    auto original_most_significant = reg_bitset[0];

    auto shifted_reg = std::bitset<8>(value >> 1);
    shifted_reg[0] = original_most_significant;
    auto res = static_cast<u8>(shifted_reg.to_ulong());

    registers.AF.flags.zero = res == 0;
    registers.AF.flags.subtract = res < 0;
    registers.AF.flags.carry = original_least_significant;
    registers.AF.flags.half_carry = 0;

    return res;
}

u8 CPU::sla(u8 value) {
    auto reg_bitset = std::bitset<8>(value);
    auto original_most_significant = reg_bitset[0];

    auto shifted_reg = std::bitset<8>(value << 1);
    auto res = static_cast<u8>(shifted_reg.to_ulong());

    registers.AF.flags.zero = res == 0;
    registers.AF.flags.subtract = res < 0;
    registers.AF.flags.carry = original_most_significant;
    registers.AF.flags.half_carry = 0;

    return res;
}

u8 CPU::swap(u8 reg) {
    auto upper = ((reg & 0xf0) >> 4);
    auto lower = (reg & 0x0f);
    auto res = lower | upper; //TODO(Andrea): is it even working?

    registers.AF.flags.zero = res == 0;
    registers.AF.flags.subtract = false;
    registers.AF.flags.carry = 0; //TODO(Andrea): is it correct?
    registers.AF.flags.half_carry = 0;

    return res;
}

u16 CPU::jump(bool should_jump) {
    if (should_jump) {
        auto least_significant = static_cast<u16>(bus.read_byte(pc + 1));
        auto most_significant = static_cast<u16>(bus.read_byte(pc + 2));
        u16 res = (most_significant << 8) | least_significant;
        return res;
    }
    else {
        SafeAdd(pc, static_cast<u16>(3), pc);
    }
}

u16 CPU::jr(bool should_jump) {
    if (should_jump) {
        auto offset = static_cast<i8>(bus.read_byte(pc + 1));
        u16 res = pc + offset;
        return res;
    }
    else {
        SafeAdd(pc, static_cast<u16>(2), pc);
    }
}

u16 CPU::jp_hl() {
    return registers.HL.get();
}

u16 CPU::execute(Instruction instruction) {
    switch (instruction.inst_enum)
    {
    case InstructionEnum::ADD:
    {
        switch (instruction.target) {
        case ArithmeticTarget::A: {
            auto value = registers.AF.first;
            auto new_value = add(value);
            registers.AF.first = new_value;
        }
                                break;
        case ArithmeticTarget::B: {
            auto value = registers.BC.first;
            auto new_value = add(value);
            registers.AF.first = new_value;
        }
        break;
        case ArithmeticTarget::C: {
            auto value = registers.BC.second;
            auto new_value = add(value);
            registers.AF.first = new_value;
        }
        break;
        case ArithmeticTarget::D: {
            auto value = registers.DE.first;
            auto new_value = add(value);
            registers.AF.first = new_value;
        }
        break;
        case ArithmeticTarget::E: {
            auto value = registers.DE.second;
            auto new_value = add(value);
            registers.AF.first = new_value;
        }
        break;
        case ArithmeticTarget::H: {
            auto value = registers.HL.first;
            auto new_value = add(value);
            registers.AF.first = new_value;
        }
        break;
        case ArithmeticTarget::L: {
            auto value = registers.HL.second;
            auto new_value = add(value);
            registers.AF.first = new_value;
        }
        break;
        }
        __assume(false);
    }
    break;
    case InstructionEnum::ADDHL: {
        switch (instruction.target) {
        case ArithmeticTarget::BC: {
            auto value = registers.BC.get();
            auto new_value = addhl(value);
            registers.HL.set(new_value);
        }
        break;
        case ArithmeticTarget::DE: {
            auto value = registers.DE.get();
            auto new_value = addhl(value);
            registers.HL.set(new_value);
        }
        break;
        case ArithmeticTarget::HL: {
                auto value = registers.HL.get();
                auto new_value = addhl(value);
                registers.HL.set(new_value);
            }
            break;
        }
        __assume(false);
    }
    break;
    case InstructionEnum::ADDC:
    {
        switch (instruction.target) {
        case ArithmeticTarget::A: {
            auto value = registers.AF.first;
            auto new_value = adc(value);
            registers.AF.first = new_value;
        }
                                break;
        case ArithmeticTarget::B: {
            auto value = registers.BC.first;
            auto new_value = adc(value);
            registers.AF.first = new_value;
        }
                                break;
        case ArithmeticTarget::C: {
            auto value = registers.BC.second;
            auto new_value = adc(value);
            registers.AF.first = new_value;
        }
                                break;
        case ArithmeticTarget::D: {
            auto value = registers.DE.first;
            auto new_value = adc(value);
            registers.AF.first = new_value;
        }
                                break;
        case ArithmeticTarget::E: {
            auto value = registers.DE.second;
            auto new_value = adc(value);
            registers.AF.first = new_value;
        }
                                break;
        case ArithmeticTarget::H: {
            auto value = registers.HL.first;
            auto new_value = adc(value);
            registers.AF.first = new_value;
        }
                                break;
        case ArithmeticTarget::L: {
            auto value = registers.HL.second;
            auto new_value = adc(value);
            registers.AF.first = new_value;
        }
                                break;
        }
        __assume(false);
    }
    break;
    case InstructionEnum::SUB:
    {
        switch (instruction.target) {
        case ArithmeticTarget::A: {
            auto value = registers.AF.first;
            auto new_value = sub(value);
            //TODO(Andrea): should we store back into the A register?
            registers.AF.first = new_value;
        }
                                break;
        case ArithmeticTarget::B: {
            auto value = registers.BC.first;
            auto new_value = sub(value);
            //TODO(Andrea): should we store back into the A register?
            registers.AF.first = new_value;
        }
                                break;
        case ArithmeticTarget::C: {
            auto value = registers.BC.second;
            auto new_value = sub(value);
            registers.AF.first = new_value;
        }
                                break;
        case ArithmeticTarget::D: {
            auto value = registers.DE.first;
            auto new_value = sub(value);
            registers.AF.first = new_value;
        }
                                break;
        case ArithmeticTarget::E: {
            auto value = registers.DE.second;
            auto new_value = sub(value);
            registers.AF.first = new_value;
        }
                                break;
        case ArithmeticTarget::H: {
            auto value = registers.HL.first;
            auto new_value = sub(value);
            registers.AF.first = new_value;
        }
                                break;
        case ArithmeticTarget::L: {
            auto value = registers.HL.second;
            auto new_value = sub(value);
            registers.AF.first = new_value;
        }
                                break;
        }
        __assume(false);
    }
    break;
    case InstructionEnum::SBC:
    {
        switch (instruction.target) {
        case ArithmeticTarget::A: {
            auto value = registers.AF.first;
            auto new_value = sbc(value);
            //TODO(Andrea): should we store back into the A register?
            registers.AF.first = new_value;
        }
                                break;
        case ArithmeticTarget::B: {
            auto value = registers.BC.first;
            auto new_value = sbc(value);
            //TODO(Andrea): should we store back into the A register?
            registers.AF.first = new_value;
        }
                                break;
        case ArithmeticTarget::C: {
            auto value = registers.BC.second;
            auto new_value = sbc(value);
            registers.AF.first = new_value;
        }
                                break;
        case ArithmeticTarget::D: {
            auto value = registers.DE.first;
            auto new_value = sbc(value);
            registers.AF.first = new_value;
        }
                                break;
        case ArithmeticTarget::E: {
            auto value = registers.DE.second;
            auto new_value = sbc(value);
            registers.AF.first = new_value;
        }
                                break;
        case ArithmeticTarget::H: {
            auto value = registers.HL.first;
            auto new_value = sbc(value);
            registers.AF.first = new_value;
        }
                                break;
        case ArithmeticTarget::L: {
            auto value = registers.HL.second;
            auto new_value = sbc(value);
            registers.AF.first = new_value;
        }
                                break;
        }
        __assume(false);
    }
    break;
    case InstructionEnum::AND:
    {
        switch (instruction.target) {
        case ArithmeticTarget::A: {
            auto value = registers.AF.first;
            auto new_value = bitwise_and(value);
            //TODO(Andrea): should we store back into the A register?
            registers.AF.first = new_value;
        }
                                break;
        case ArithmeticTarget::B: {
            auto value = registers.BC.first;
            auto new_value = bitwise_and(value);
            //TODO(Andrea): should we store back into the A register?
            registers.AF.first = new_value;
        }
                                break;
        case ArithmeticTarget::C: {
            auto value = registers.BC.second;
            auto new_value = bitwise_and(value);
            registers.AF.first = new_value;
        }
                                break;
        case ArithmeticTarget::D: {
            auto value = registers.DE.first;
            auto new_value = bitwise_and(value);
            registers.AF.first = new_value;
        }
                                break;
        case ArithmeticTarget::E: {
            auto value = registers.DE.second;
            auto new_value = bitwise_and(value);
            registers.AF.first = new_value;
        }
                                break;
        case ArithmeticTarget::H: {
            auto value = registers.HL.first;
            auto new_value = bitwise_and(value);
            registers.AF.first = new_value;
        }
                                break;
        case ArithmeticTarget::L: {
            auto value = registers.HL.second;
            auto new_value = bitwise_and(value);
            registers.AF.first = new_value;
        }
                                break;
        }
        __assume(false);
    }
    break;
    case InstructionEnum::OR:
    {
        switch (instruction.target) {
        case ArithmeticTarget::A: {
            auto value = registers.AF.first;
            auto new_value = bitwise_or(value);
            //TODO(Andrea): should we store back into the A register?
            registers.AF.first = new_value;
        }
                                break;
        case ArithmeticTarget::B: {
            auto value = registers.BC.first;
            auto new_value = bitwise_or(value);
            //TODO(Andrea): should we store back into the A register?
            registers.AF.first = new_value;
        }
                                break;
        case ArithmeticTarget::C: {
            auto value = registers.BC.second;
            auto new_value = bitwise_or(value);
            registers.AF.first = new_value;
        }
                                break;
        case ArithmeticTarget::D: {
            auto value = registers.DE.first;
            auto new_value = bitwise_or(value);
            registers.AF.first = new_value;
        }
                                break;
        case ArithmeticTarget::E: {
            auto value = registers.DE.second;
            auto new_value = bitwise_or(value);
            registers.AF.first = new_value;
        }
                                break;
        case ArithmeticTarget::H: {
            auto value = registers.HL.first;
            auto new_value = bitwise_or(value);
            registers.AF.first = new_value;
        }
                                break;
        case ArithmeticTarget::L: {
            auto value = registers.HL.second;
            auto new_value = bitwise_or(value);
            registers.AF.first = new_value;
        }
                                break;
        }
        __assume(false);
    }
    break;
    case InstructionEnum::XOR:
    {
        switch (instruction.target) {
        case ArithmeticTarget::A: {
            auto value = registers.AF.first;
            auto new_value = bitwise_xor(value);
            //TODO(Andrea): should we store back into the A register?
            registers.AF.first = new_value;
        }
                                break;
        case ArithmeticTarget::B: {
            auto value = registers.BC.first;
            auto new_value = bitwise_xor(value);
            //TODO(Andrea): should we store back into the A register?
            registers.AF.first = new_value;
        }
                                break;
        case ArithmeticTarget::C: {
            auto value = registers.BC.second;
            auto new_value = bitwise_xor(value);
            registers.AF.first = new_value;
        }
                                break;
        case ArithmeticTarget::D: {
            auto value = registers.DE.first;
            auto new_value = bitwise_xor(value);
            registers.AF.first = new_value;
        }
                                break;
        case ArithmeticTarget::E: {
            auto value = registers.DE.second;
            auto new_value = bitwise_xor(value);
            registers.AF.first = new_value;
        }
                                break;
        case ArithmeticTarget::H: {
            auto value = registers.HL.first;
            auto new_value = bitwise_xor(value);
            registers.AF.first = new_value;
        }
                                break;
        case ArithmeticTarget::L: {
            auto value = registers.HL.second;
            auto new_value = bitwise_xor(value);
            registers.AF.first = new_value;
        }
                                break;
        }
        __assume(false);
    }
    break;
    case InstructionEnum::CP:
    {
        switch (instruction.target) {
        case ArithmeticTarget::A: {
            auto value = registers.AF.first;
            cp(value);
        }
                                break;
        case ArithmeticTarget::B: {
            auto value = registers.BC.first;
            cp(value);
        }
                                break;
        case ArithmeticTarget::C: {
            auto value = registers.BC.second;
            cp(value);
        }
                                break;
        case ArithmeticTarget::D: {
            auto value = registers.DE.first;
            cp(value);
        }
                                break;
        case ArithmeticTarget::E: {
            auto value = registers.DE.second;
            cp(value);
        }
                                break;
        case ArithmeticTarget::H: {
            auto value = registers.HL.first;
            cp(value);
        }
                                break;
        case ArithmeticTarget::L: {
            auto value = registers.HL.second;
            cp(value);
        }
                                break;
        }
        __assume(false);
    }
    break;
    case InstructionEnum::INC:
    {
        switch (instruction.target) {
        case ArithmeticTarget::A: {
            auto value = registers.AF.first;
            registers.AF.first = inc(value);
        }
                                break;
        case ArithmeticTarget::B: {
            auto value = registers.BC.first;
            registers.BC.first = inc(value);
        }
                                break;
        case ArithmeticTarget::C: {
            auto value = registers.BC.second;
            registers.BC.second = inc(value);
        }
                                break;
        case ArithmeticTarget::D: {
            auto value = registers.DE.first;
            registers.DE.first = inc(value);
        }
                                break;
        case ArithmeticTarget::E: {
            auto value = registers.DE.second;
            registers.DE.second = inc(value);
        }
                                break;
        case ArithmeticTarget::H: {
            auto value = registers.HL.first;
            registers.HL.first = inc(value);
        }
                                break;
        case ArithmeticTarget::L: {
            auto value = registers.HL.second;
            registers.HL.second = inc(value);
        }
                                break;
        }
        __assume(false);
    }
    break;
    case InstructionEnum::DEC:
    {
        switch (instruction.target) {
        case ArithmeticTarget::A: {
            auto value = registers.AF.first;
            registers.AF.first = dec(value);
        }
                                break;
        case ArithmeticTarget::B: {
            auto value = registers.BC.first;
            registers.BC.first = dec(value);
        }
                                break;
        case ArithmeticTarget::C: {
            auto value = registers.BC.second;
            registers.BC.second = dec(value);
        }
                                break;
        case ArithmeticTarget::D: {
            auto value = registers.DE.first;
            registers.DE.first = dec(value);
        }
                                break;
        case ArithmeticTarget::E: {
            auto value = registers.DE.second;
            registers.DE.second = dec(value);
        }
                                break;
        case ArithmeticTarget::H: {
            auto value = registers.HL.first;
            registers.HL.first = dec(value);
        }
                                break;
        case ArithmeticTarget::L: {
            auto value = registers.HL.second;
            registers.HL.second = dec(value);
        }
                                break;
        }
        __assume(false);
    }
    break;
    case InstructionEnum::CCF:
    {
        ccf();
        __assume(false);
    }
    break;
    case InstructionEnum::SCF:
    {
        scf();
        __assume(false);
    }
    break;
    case InstructionEnum::RRA:
    {
        rra();
        __assume(false);
    }
    break;
    case InstructionEnum::RLA:
    {
        rla();
        __assume(false);
    }
    break;
    case InstructionEnum::RRCA:
    {
        rrca();
        __assume(false);
    }
    break;
    case InstructionEnum::RLCA:
    {
        rlca();
        __assume(false);
    }
    break;
    case InstructionEnum::CPL:
    {
        cpl();
        __assume(false);
    }
    break;
    case InstructionEnum::BIT:
    {
        switch (instruction.target) {
        case ArithmeticTarget::A: {
            auto value = registers.AF.first;
            switch (instruction.bit) {
                case RegisterBit::_0:
                    bit(value, 0);
                    break;
                case RegisterBit::_1:
                    bit(value, 1);
                    break;
                case RegisterBit::_2:
                    bit(value, 2);
                    break;
                case RegisterBit::_3:
                    bit(value, 3);
                    break;
                case RegisterBit::_4:
                    bit(value, 4);
                    break;
                case RegisterBit::_5:
                    bit(value, 5);
                    break;
                case RegisterBit::_6:
                    bit(value, 6);
                    break;
                case RegisterBit::_7:
                    bit(value, 7);
                    break;
            }
        }
                                break;
        case ArithmeticTarget::B: {
            auto value = registers.BC.first;
            switch (instruction.bit) {
            case RegisterBit::_0:
                bit(value, 0);
                break;
            case RegisterBit::_1:
                bit(value, 1);
                break;
            case RegisterBit::_2:
                bit(value, 2);
                break;
            case RegisterBit::_3:
                bit(value, 3);
                break;
            case RegisterBit::_4:
                bit(value, 4);
                break;
            case RegisterBit::_5:
                bit(value, 5);
                break;
            case RegisterBit::_6:
                bit(value, 6);
                break;
            case RegisterBit::_7:
                bit(value, 7);
                break;
            }
        }
                                break;
        case ArithmeticTarget::C: {
            auto value = registers.BC.second;
            switch (instruction.bit) {
            case RegisterBit::_0:
                bit(value, 0);
                break;
            case RegisterBit::_1:
                bit(value, 1);
                break;
            case RegisterBit::_2:
                bit(value, 2);
                break;
            case RegisterBit::_3:
                bit(value, 3);
                break;
            case RegisterBit::_4:
                bit(value, 4);
                break;
            case RegisterBit::_5:
                bit(value, 5);
                break;
            case RegisterBit::_6:
                bit(value, 6);
                break;
            case RegisterBit::_7:
                bit(value, 7);
                break;
            }
        }
                                break;
        case ArithmeticTarget::D: {
            auto value = registers.DE.first;
            switch (instruction.bit) {
            case RegisterBit::_0:
                bit(value, 0);
                break;
            case RegisterBit::_1:
                bit(value, 1);
                break;
            case RegisterBit::_2:
                bit(value, 2);
                break;
            case RegisterBit::_3:
                bit(value, 3);
                break;
            case RegisterBit::_4:
                bit(value, 4);
                break;
            case RegisterBit::_5:
                bit(value, 5);
                break;
            case RegisterBit::_6:
                bit(value, 6);
                break;
            case RegisterBit::_7:
                bit(value, 7);
                break;
            }
        }
                                break;
        case ArithmeticTarget::E: {
            auto value = registers.DE.second;
            switch (instruction.bit) {
            case RegisterBit::_0:
                bit(value, 0);
                break;
            case RegisterBit::_1:
                bit(value, 1);
                break;
            case RegisterBit::_2:
                bit(value, 2);
                break;
            case RegisterBit::_3:
                bit(value, 3);
                break;
            case RegisterBit::_4:
                bit(value, 4);
                break;
            case RegisterBit::_5:
                bit(value, 5);
                break;
            case RegisterBit::_6:
                bit(value, 6);
                break;
            case RegisterBit::_7:
                bit(value, 7);
                break;
            }
        }
                                break;
        case ArithmeticTarget::H: {
            auto value = registers.HL.first;
            switch (instruction.bit) {
            case RegisterBit::_0:
                bit(value, 0);
                break;
            case RegisterBit::_1:
                bit(value, 1);
                break;
            case RegisterBit::_2:
                bit(value, 2);
                break;
            case RegisterBit::_3:
                bit(value, 3);
                break;
            case RegisterBit::_4:
                bit(value, 4);
                break;
            case RegisterBit::_5:
                bit(value, 5);
                break;
            case RegisterBit::_6:
                bit(value, 6);
                break;
            case RegisterBit::_7:
                bit(value, 7);
                break;
            }
        }
                                break;
        case ArithmeticTarget::L: {
            auto value = registers.HL.second;
            switch (instruction.bit) {
            case RegisterBit::_0:
                bit(value, 0);
                break;
            case RegisterBit::_1:
                bit(value, 1);
                break;
            case RegisterBit::_2:
                bit(value, 2);
                break;
            case RegisterBit::_3:
                bit(value, 3);
                break;
            case RegisterBit::_4:
                bit(value, 4);
                break;
            case RegisterBit::_5:
                bit(value, 5);
                break;
            case RegisterBit::_6:
                bit(value, 6);
                break;
            case RegisterBit::_7:
                bit(value, 7);
                break;
            }
        }
                                break;
        }
        __assume(false);
    }
    break;
    case InstructionEnum::RESET:
    {
        switch (instruction.target) {
        case ArithmeticTarget::A: {
            auto value = registers.AF.first;
            switch (instruction.bit) {
            case RegisterBit::_0:
                registers.AF.first = reset(value, 0);
                break;
            case RegisterBit::_1:
                registers.AF.first = reset(value, 1);
                break;
            case RegisterBit::_2:
                registers.AF.first = reset(value, 2);
                break;
            case RegisterBit::_3:
                registers.AF.first = reset(value, 3);
                break;
            case RegisterBit::_4:
                registers.AF.first = reset(value, 4);
                break;
            case RegisterBit::_5:
                registers.AF.first = reset(value, 5);
                break;
            case RegisterBit::_6:
                registers.AF.first = reset(value, 6);
                break;
            case RegisterBit::_7:
                registers.AF.first = reset(value, 7);
                break;
            }
        }
                                break;
        case ArithmeticTarget::B: {
            auto value = registers.BC.first;
            switch (instruction.bit) {
            case RegisterBit::_0:
                registers.BC.first = reset(value, 0);
                break;
            case RegisterBit::_1:
                registers.BC.first = reset(value, 1);
                break;
            case RegisterBit::_2:
                registers.BC.first = reset(value, 2);
                break;
            case RegisterBit::_3:
                registers.BC.first = reset(value, 3);
                break;
            case RegisterBit::_4:
                registers.BC.first = reset(value, 4);
                break;
            case RegisterBit::_5:
                registers.BC.first = reset(value, 5);
                break;
            case RegisterBit::_6:
                registers.BC.first = reset(value, 6);
                break;
            case RegisterBit::_7:
                registers.BC.first = reset(value, 7);
                break;
            }
        }
                                break;
        case ArithmeticTarget::C: {
            auto value = registers.BC.second;
            switch (instruction.bit) {
            case RegisterBit::_0:
                registers.BC.second = reset(value, 0);
                break;
            case RegisterBit::_1:
                registers.BC.second = reset(value, 1);
                break;
            case RegisterBit::_2:
                registers.BC.second = reset(value, 2);
                break;
            case RegisterBit::_3:
                registers.BC.second = reset(value, 3);
                break;
            case RegisterBit::_4:
                registers.BC.second = reset(value, 4);
                break;
            case RegisterBit::_5:
                registers.BC.second = reset(value, 5);
                break;
            case RegisterBit::_6:
                registers.BC.second = reset(value, 6);
                break;
            case RegisterBit::_7:
                registers.BC.second = reset(value, 7);
                break;
            }
        }
                                break;
        case ArithmeticTarget::D: {
            auto value = registers.DE.first;
            switch (instruction.bit) {
            case RegisterBit::_0:
                registers.DE.first = reset(value, 0);
                break;
            case RegisterBit::_1:
                registers.DE.first = reset(value, 1);
                break;
            case RegisterBit::_2:
                registers.DE.first = reset(value, 2);
                break;
            case RegisterBit::_3:
                registers.DE.first = reset(value, 3);
                break;
            case RegisterBit::_4:
                registers.DE.first = reset(value, 4);
                break;
            case RegisterBit::_5:
                registers.DE.first = reset(value, 5);
                break;
            case RegisterBit::_6:
                registers.DE.first = reset(value, 6);
                break;
            case RegisterBit::_7:
                registers.DE.first = reset(value, 7);
                break;
            }
        }
                                break;
        case ArithmeticTarget::E: {
            auto value = registers.DE.second;
            switch (instruction.bit) {
            case RegisterBit::_0:
                registers.DE.second = reset(value, 0);
                break;
            case RegisterBit::_1:
                registers.DE.second = reset(value, 1);
                break;
            case RegisterBit::_2:
                registers.DE.second = reset(value, 2);
                break;
            case RegisterBit::_3:
                registers.DE.second = reset(value, 3);
                break;
            case RegisterBit::_4:
                registers.DE.second = reset(value, 4);
                break;
            case RegisterBit::_5:
                registers.DE.second = reset(value, 5);
                break;
            case RegisterBit::_6:
                registers.DE.second = reset(value, 6);
                break;
            case RegisterBit::_7:
                registers.DE.second = reset(value, 7);
                break;
            }
        }
                                break;
        case ArithmeticTarget::H: {
            auto value = registers.HL.first;
            switch (instruction.bit) {
            case RegisterBit::_0:
                registers.HL.first = reset(value, 0);
                break;
            case RegisterBit::_1:
                registers.HL.first = reset(value, 1);
                break;
            case RegisterBit::_2:
                registers.HL.first = reset(value, 2);
                break;
            case RegisterBit::_3:
                registers.HL.first = reset(value, 3);
                break;
            case RegisterBit::_4:
                registers.HL.first = reset(value, 4);
                break;
            case RegisterBit::_5:
                registers.HL.first = reset(value, 5);
                break;
            case RegisterBit::_6:
                registers.HL.first = reset(value, 6);
                break;
            case RegisterBit::_7:
                registers.HL.first = reset(value, 7);
                break;
            }
        }
                                break;
        case ArithmeticTarget::L: {
            auto value = registers.HL.second;
            switch (instruction.bit) {
            case RegisterBit::_0:
                registers.HL.second = reset(value, 0);
                break;
            case RegisterBit::_1:
                registers.HL.second = reset(value, 1);
                break;
            case RegisterBit::_2:
                registers.HL.second = reset(value, 2);
                break;
            case RegisterBit::_3:
                registers.HL.second = reset(value, 3);
                break;
            case RegisterBit::_4:
                registers.HL.second = reset(value, 4);
                break;
            case RegisterBit::_5:
                registers.HL.second = reset(value, 5);
                break;
            case RegisterBit::_6:
                registers.HL.second = reset(value, 6);
                break;
            case RegisterBit::_7:
                registers.HL.second = reset(value, 7);
                break;
            }
        }
                                break;
        }
        __assume(false);
    }
    break;
    case InstructionEnum::SET:
    {
        switch (instruction.target) {
        case ArithmeticTarget::A: {
            auto value = registers.AF.first;
            switch (instruction.bit) {
            case RegisterBit::_0:
                registers.AF.first = set(value, 0);
                break;
            case RegisterBit::_1:
                registers.AF.first = set(value, 1);
                break;
            case RegisterBit::_2:
                registers.AF.first = set(value, 2);
                break;
            case RegisterBit::_3:
                registers.AF.first = set(value, 3);
                break;
            case RegisterBit::_4:
                registers.AF.first = set(value, 4);
                break;
            case RegisterBit::_5:
                registers.AF.first = set(value, 5);
                break;
            case RegisterBit::_6:
                registers.AF.first = set(value, 6);
                break;
            case RegisterBit::_7:
                registers.AF.first = set(value, 7);
                break;
            }
        }
                                break;
        case ArithmeticTarget::B: {
            auto value = registers.BC.first;
            switch (instruction.bit) {
            case RegisterBit::_0:
                registers.BC.first = set(value, 0);
                break;
            case RegisterBit::_1:
                registers.BC.first = set(value, 1);
                break;
            case RegisterBit::_2:
                registers.BC.first = set(value, 2);
                break;
            case RegisterBit::_3:
                registers.BC.first = set(value, 3);
                break;
            case RegisterBit::_4:
                registers.BC.first = set(value, 4);
                break;
            case RegisterBit::_5:
                registers.BC.first = set(value, 5);
                break;
            case RegisterBit::_6:
                registers.BC.first = set(value, 6);
                break;
            case RegisterBit::_7:
                registers.BC.first = set(value, 7);
                break;
            }
        }
                                break;
        case ArithmeticTarget::C: {
            auto value = registers.BC.second;
            switch (instruction.bit) {
            case RegisterBit::_0:
                registers.BC.second = set(value, 0);
                break;
            case RegisterBit::_1:
                registers.BC.second = set(value, 1);
                break;
            case RegisterBit::_2:
                registers.BC.second = set(value, 2);
                break;
            case RegisterBit::_3:
                registers.BC.second = set(value, 3);
                break;
            case RegisterBit::_4:
                registers.BC.second = set(value, 4);
                break;
            case RegisterBit::_5:
                registers.BC.second = set(value, 5);
                break;
            case RegisterBit::_6:
                registers.BC.second = set(value, 6);
                break;
            case RegisterBit::_7:
                registers.BC.second = set(value, 7);
                break;
            }
        }
                                break;
        case ArithmeticTarget::D: {
            auto value = registers.DE.first;
            switch (instruction.bit) {
            case RegisterBit::_0:
                registers.DE.first = set(value, 0);
                break;
            case RegisterBit::_1:
                registers.DE.first = set(value, 1);
                break;
            case RegisterBit::_2:
                registers.DE.first = set(value, 2);
                break;
            case RegisterBit::_3:
                registers.DE.first = set(value, 3);
                break;
            case RegisterBit::_4:
                registers.DE.first = set(value, 4);
                break;
            case RegisterBit::_5:
                registers.DE.first = set(value, 5);
                break;
            case RegisterBit::_6:
                registers.DE.first = set(value, 6);
                break;
            case RegisterBit::_7:
                registers.DE.first = set(value, 7);
                break;
            }
        }
                                break;
        case ArithmeticTarget::E: {
            auto value = registers.DE.second;
            switch (instruction.bit) {
            case RegisterBit::_0:
                registers.DE.second = set(value, 0);
                break;
            case RegisterBit::_1:
                registers.DE.second = set(value, 1);
                break;
            case RegisterBit::_2:
                registers.DE.second = set(value, 2);
                break;
            case RegisterBit::_3:
                registers.DE.second = set(value, 3);
                break;
            case RegisterBit::_4:
                registers.DE.second = set(value, 4);
                break;
            case RegisterBit::_5:
                registers.DE.second = set(value, 5);
                break;
            case RegisterBit::_6:
                registers.DE.second = set(value, 6);
                break;
            case RegisterBit::_7:
                registers.DE.second = set(value, 7);
                break;
            }
        }
                                break;
        case ArithmeticTarget::H: {
            auto value = registers.HL.first;
            switch (instruction.bit) {
            case RegisterBit::_0:
                registers.HL.first = set(value, 0);
                break;
            case RegisterBit::_1:
                registers.HL.first = set(value, 1);
                break;
            case RegisterBit::_2:
                registers.HL.first = set(value, 2);
                break;
            case RegisterBit::_3:
                registers.HL.first = set(value, 3);
                break;
            case RegisterBit::_4:
                registers.HL.first = set(value, 4);
                break;
            case RegisterBit::_5:
                registers.HL.first = set(value, 5);
                break;
            case RegisterBit::_6:
                registers.HL.first = set(value, 6);
                break;
            case RegisterBit::_7:
                registers.HL.first = set(value, 7);
                break;
            }
        }
                                break;
        case ArithmeticTarget::L: {
            auto value = registers.HL.second;
            switch (instruction.bit) {
            case RegisterBit::_0:
                registers.HL.second = set(value, 0);
                break;
            case RegisterBit::_1:
                registers.HL.second = set(value, 1);
                break;
            case RegisterBit::_2:
                registers.HL.second = set(value, 2);
                break;
            case RegisterBit::_3:
                registers.HL.second = set(value, 3);
                break;
            case RegisterBit::_4:
                registers.HL.second = set(value, 4);
                break;
            case RegisterBit::_5:
                registers.HL.second = set(value, 5);
                break;
            case RegisterBit::_6:
                registers.HL.second = set(value, 6);
                break;
            case RegisterBit::_7:
                registers.HL.second = set(value, 7);
                break;
            }
        }
                                break;
        }
        __assume(false);
    }
    break;
    case InstructionEnum::SRL:
    {
        switch (instruction.target) {
        case ArithmeticTarget::A: {
            auto value = registers.AF.first;
            auto new_value = srl(value);
            registers.AF.first = new_value;
        }
                                break;
        case ArithmeticTarget::B: {
            auto value = registers.BC.first;
            auto new_value = srl(value);
            registers.AF.first = new_value;
        }
                                break;
        case ArithmeticTarget::C: {
            auto value = registers.BC.second;
            auto new_value = srl(value);
            registers.AF.first = new_value;
        }
                                break;
        case ArithmeticTarget::D: {
            auto value = registers.DE.first;
            auto new_value = srl(value);
            registers.AF.first = new_value;
        }
                                break;
        case ArithmeticTarget::E: {
            auto value = registers.DE.second;
            auto new_value = srl(value);
            registers.AF.first = new_value;
        }
                                break;
        case ArithmeticTarget::H: {
            auto value = registers.HL.first;
            auto new_value = srl(value);
            registers.AF.first = new_value;
        }
                                break;
        case ArithmeticTarget::L: {
            auto value = registers.HL.second;
            auto new_value = srl(value);
            registers.AF.first = new_value;
        }
                                break;
        }
        __assume(false);
    }
    break;
    case InstructionEnum::RR:
    {
        switch (instruction.target) {
        case ArithmeticTarget::A: {
            auto value = registers.AF.first;
            registers.AF.first = rr(value);
        }
                                break;
        case ArithmeticTarget::B: {
            auto value = registers.BC.first;
            registers.BC.first = rr(value);
        }
                                break;
        case ArithmeticTarget::C: {
            auto value = registers.BC.second;
            registers.BC.second = rr(value);
        }
                                break;
        case ArithmeticTarget::D: {
            auto value = registers.DE.first;
            registers.DE.first = rr(value);
        }
                                break;
        case ArithmeticTarget::E: {
            auto value = registers.DE.second;
            registers.DE.second = rr(value);
        }
                                break;
        case ArithmeticTarget::H: {
            auto value = registers.HL.first;
            registers.HL.first = rr(value);
        }
                                break;
        case ArithmeticTarget::L: {
            auto value = registers.HL.second;
            registers.HL.second = rr(value);
        }
                                break;
        }
        __assume(false);
    }
    break;
    case InstructionEnum::RL:
    {
        switch (instruction.target) {
        case ArithmeticTarget::A: {
            auto value = registers.AF.first;
            registers.AF.first = rl(value);
        }
                                break;
        case ArithmeticTarget::B: {
            auto value = registers.BC.first;
            registers.BC.first = rl(value);
        }
                                break;
        case ArithmeticTarget::C: {
            auto value = registers.BC.second;
            registers.BC.second = rl(value);
        }
                                break;
        case ArithmeticTarget::D: {
            auto value = registers.DE.first;
            registers.DE.first = rl(value);
        }
                                break;
        case ArithmeticTarget::E: {
            auto value = registers.DE.second;
            registers.DE.second = rl(value);
        }
                                break;
        case ArithmeticTarget::H: {
            auto value = registers.HL.first;
            registers.HL.first = rl(value);
        }
                                break;
        case ArithmeticTarget::L: {
            auto value = registers.HL.second;
            registers.HL.second = rl(value);
        }
                                break;
        }
        __assume(false);
    }
    break;
    case InstructionEnum::RRC:
    {
        switch (instruction.target) {
        case ArithmeticTarget::A: {
            auto value = registers.AF.first;
            registers.AF.first = rrc(value);
        }
                                break;
        case ArithmeticTarget::B: {
            auto value = registers.BC.first;
            registers.BC.first = rrc(value);
        }
                                break;
        case ArithmeticTarget::C: {
            auto value = registers.BC.second;
            registers.BC.second = rrc(value);
        }
                                break;
        case ArithmeticTarget::D: {
            auto value = registers.DE.first;
            registers.DE.first = rrc(value);
        }
                                break;
        case ArithmeticTarget::E: {
            auto value = registers.DE.second;
            registers.DE.second = rrc(value);
        }
                                break;
        case ArithmeticTarget::H: {
            auto value = registers.HL.first;
            registers.HL.first = rrc(value);
        }
                                break;
        case ArithmeticTarget::L: {
            auto value = registers.HL.second;
            registers.HL.second = rrc(value);
        }
                                break;
        }
        __assume(false);
    }
    break;
    case InstructionEnum::RLC:
    {
        switch (instruction.target) {
        case ArithmeticTarget::A: {
            auto value = registers.AF.first;
            registers.AF.first = rlc(value);
        }
                                break;
        case ArithmeticTarget::B: {
            auto value = registers.BC.first;
            registers.BC.first = rlc(value);
        }
                                break;
        case ArithmeticTarget::C: {
            auto value = registers.BC.second;
            registers.BC.second = rlc(value);
        }
                                break;
        case ArithmeticTarget::D: {
            auto value = registers.DE.first;
            registers.DE.first = rlc(value);
        }
                                break;
        case ArithmeticTarget::E: {
            auto value = registers.DE.second;
            registers.DE.second = rlc(value);
        }
                                break;
        case ArithmeticTarget::H: {
            auto value = registers.HL.first;
            registers.HL.first = rlc(value);
        }
                                break;
        case ArithmeticTarget::L: {
            auto value = registers.HL.second;
            registers.HL.second = rlc(value);
        }
                                break;
        }
        __assume(false);
    }
    break;
    case InstructionEnum::SRA:
    {
        switch (instruction.target) {
        case ArithmeticTarget::A: {
            auto value = registers.AF.first;
            registers.AF.first = sra(value);
        }
                                break;
        case ArithmeticTarget::B: {
            auto value = registers.BC.first;
            registers.BC.first = sra(value);
        }
                                break;
        case ArithmeticTarget::C: {
            auto value = registers.BC.second;
            registers.BC.second = sra(value);
        }
                                break;
        case ArithmeticTarget::D: {
            auto value = registers.DE.first;
            registers.DE.first = sra(value);
        }
                                break;
        case ArithmeticTarget::E: {
            auto value = registers.DE.second;
            registers.DE.second = sra(value);
        }
                                break;
        case ArithmeticTarget::H: {
            auto value = registers.HL.first;
            registers.HL.first = sra(value);
        }
                                break;
        case ArithmeticTarget::L: {
            auto value = registers.HL.second;
            registers.HL.second = sra(value);
        }
                                break;
        }
        __assume(false);
    }
    break;
    case InstructionEnum::SLA:
    {
        switch (instruction.target) {
        case ArithmeticTarget::A: {
            auto value = registers.AF.first;
            registers.AF.first = sla(value);
        }
                                break;
        case ArithmeticTarget::B: {
            auto value = registers.BC.first;
            registers.BC.first = sla(value);
        }
                                break;
        case ArithmeticTarget::C: {
            auto value = registers.BC.second;
            registers.BC.second = sla(value);
        }
                                break;
        case ArithmeticTarget::D: {
            auto value = registers.DE.first;
            registers.DE.first = sla(value);
        }
                                break;
        case ArithmeticTarget::E: {
            auto value = registers.DE.second;
            registers.DE.second = sla(value);
        }
                                break;
        case ArithmeticTarget::H: {
            auto value = registers.HL.first;
            registers.HL.first = sla(value);
        }
                                break;
        case ArithmeticTarget::L: {
            auto value = registers.HL.second;
            registers.HL.second = sla(value);
        }
                                break;
        }
        __assume(false);
    }
    break;
    case InstructionEnum::SWAP:
    {
        switch (instruction.target) {
        case ArithmeticTarget::A: {
            auto value = registers.AF.first;
            registers.AF.first = swap(value);
        }
                                break;
        case ArithmeticTarget::B: {
            auto value = registers.BC.first;
            registers.BC.first = swap(value);
        }
                                break;
        case ArithmeticTarget::C: {
            auto value = registers.BC.second;
            registers.BC.second = swap(value);
        }
                                break;
        case ArithmeticTarget::D: {
            auto value = registers.DE.first;
            registers.DE.first = swap(value);
        }
                                break;
        case ArithmeticTarget::E: {
            auto value = registers.DE.second;
            registers.DE.second = swap(value);
        }
                                break;
        case ArithmeticTarget::H: {
            auto value = registers.HL.first;
            registers.HL.first = swap(value);
        }
                                break;
        case ArithmeticTarget::L: {
            auto value = registers.HL.second;
            registers.HL.second = swap(value);
        }
                                break;
        }
        __assume(false);
    }
    break;
    case InstructionEnum::JP:
    {
        bool jump_condition;
        switch (instruction.test) {
        case JumpTest::NotZero: {
            jump_condition = !registers.AF.flags.zero;
            return jump(jump_condition);
        }
                              break;
        case JumpTest::NotCarry: {
            jump_condition = !registers.AF.flags.carry;
            return jump(jump_condition);
        }
                               break;
        case JumpTest::Zero: {
            jump_condition = registers.AF.flags.zero;
            return jump(jump_condition);
        }
                           break;
        case JumpTest::Carry: {
            jump_condition = registers.AF.flags.carry;
            return jump(jump_condition);
        }
                            break;
        case JumpTest::Always:
            return jump(true);
        }
                            break;
    }
    break;
    case InstructionEnum::JR:
    {
        bool jump_condition;
        switch (instruction.test) {
        case JumpTest::NotZero: {
            jump_condition = !registers.AF.flags.zero;
            return jr(jump_condition);
        }
                              break;
        case JumpTest::NotCarry: {
            jump_condition = !registers.AF.flags.carry;
            return jr(jump_condition);
        }
                               break;
        case JumpTest::Zero: {
            jump_condition = registers.AF.flags.zero;
            return jr(jump_condition);
        }
                           break;
        case JumpTest::Carry: {
            jump_condition = registers.AF.flags.carry;
            return jr(jump_condition);
        }
                            break;
        case JumpTest::Always:
            return jr(true);
        }
        break;
    }
    break;
    case InstructionEnum::JP_HL:
    {
        return jp_hl();
    }

    return pc += 1;
    }

    //assert(false);
    __assume(false); //msvc, clang-cl
}

void CPU::step() {
    auto instruction_byte = bus.read_byte(pc);
    bool prefixed = instruction_byte == 0xCB;
    if (prefixed) {
        instruction_byte = bus.read_byte(pc + 1);
    }
    
    std::optional<Instruction> instruction = Instruction::from_byte(instruction_byte, prefixed);
    if (instruction) {
        pc = execute(*instruction);

        if (prefixed) {
            pc += 1; //account for extra instruction byte
        }
    }
    else {
        if (prefixed) {
            std::cerr << "Unknown instruction found for: 0xCB" << std::hex << instruction_byte << "\n";
        }
        else {
            std::cerr << "Unknown instruction found for: 0x" << std::hex << instruction_byte << "\n";
        }
        exit(-1);
    }
}

std::optional<Instruction> Instruction::from_byte(u8 byte, bool prefixed) {
    if (prefixed) {
        Instruction::from_byte_prefixed(byte);
    }
    else {
        Instruction::from_byte_not_prefixed(byte);
    }
}

std::optional<Instruction> Instruction::from_byte_not_prefixed(u8 byte) {
    switch (byte) {
    case 0x87:
        return ADD(ArithmeticTarget::A);
    case 0x80:
        return ADD(ArithmeticTarget::B);
    case 0x81:
        return ADD(ArithmeticTarget::C);
    case 0x82:
        return ADD(ArithmeticTarget::D);
    case 0x83:
        return ADD(ArithmeticTarget::E);
    case 0x84:
        return ADD(ArithmeticTarget::H);
    case 0x85:
        return ADD(ArithmeticTarget::L);

    case 0x8F:
        return ADC(ArithmeticTarget::A);
    case 0x88:
        return ADC(ArithmeticTarget::B);
    case 0x89:
        return ADC(ArithmeticTarget::C);
    case 0x8A:
        return ADC(ArithmeticTarget::D);
    case 0x8B:
        return ADC(ArithmeticTarget::E);
    case 0x8C:
        return ADC(ArithmeticTarget::H);
    case 0x8D:
        return ADC(ArithmeticTarget::L);

    case 0x97:
        return SUB(ArithmeticTarget::A);
    case 0x90:
        return SUB(ArithmeticTarget::B);
    case 0x91:
        return SUB(ArithmeticTarget::C);
    case 0x92:
        return SUB(ArithmeticTarget::D);
    case 0x93:
        return SUB(ArithmeticTarget::E);
    case 0x94:
        return SUB(ArithmeticTarget::H);
    case 0x95:
        return SUB(ArithmeticTarget::L);

    case 0x9F:
        return SBC(ArithmeticTarget::A);
    case 0x98:
        return SBC(ArithmeticTarget::B);
    case 0x99:
        return SBC(ArithmeticTarget::C);
    case 0x9A:
        return SBC(ArithmeticTarget::D);
    case 0x9B:
        return SBC(ArithmeticTarget::E);
    case 0x9C:
        return SBC(ArithmeticTarget::H);
    case 0x9D:
        return SBC(ArithmeticTarget::L);
    
    case 0xA7:
        return AND(ArithmeticTarget::A);
    case 0xA0:
        return AND(ArithmeticTarget::B);
    case 0xA1:
        return AND(ArithmeticTarget::C);
    case 0xA2:
        return AND(ArithmeticTarget::D);
    case 0xA3:
        return AND(ArithmeticTarget::E);
    case 0xA4:
        return AND(ArithmeticTarget::H);
    case 0xA5:
        return AND(ArithmeticTarget::L);
    
    case 0xB7:
        return OR(ArithmeticTarget::A);
    case 0xB0:
        return OR(ArithmeticTarget::B);
    case 0xB1:
        return OR(ArithmeticTarget::C);
    case 0xB2:
        return OR(ArithmeticTarget::D);
    case 0xB3:
        return OR(ArithmeticTarget::E);
    case 0xB4:
        return OR(ArithmeticTarget::H);
    case 0xB5:
        return OR(ArithmeticTarget::L);

    case 0xAF:
        return XOR(ArithmeticTarget::A);
    case 0xA8:
        return XOR(ArithmeticTarget::B);
    case 0xA9:
        return XOR(ArithmeticTarget::C);
    case 0xAA:
        return XOR(ArithmeticTarget::D);
    case 0xAB:
        return XOR(ArithmeticTarget::E);
    case 0xAC:
        return XOR(ArithmeticTarget::H);
    case 0xAD:
        return XOR(ArithmeticTarget::L);

    case 0xBF:
        return CP(ArithmeticTarget::A);
    case 0xB8:
        return CP(ArithmeticTarget::B);
    case 0xB9:
        return CP(ArithmeticTarget::C);
    case 0xBA:
        return CP(ArithmeticTarget::D);
    case 0xBB:
        return CP(ArithmeticTarget::E);
    case 0xBC:
        return CP(ArithmeticTarget::H);
    case 0xBD:
        return CP(ArithmeticTarget::L);

    case 0x3C:
        return INC(ArithmeticTarget::A);
    case 0x4:
        return INC(ArithmeticTarget::B);
    case 0xC:
        return INC(ArithmeticTarget::C);
    case 0x14:
        return INC(ArithmeticTarget::D);
    case 0x1C:
        return INC(ArithmeticTarget::E);
    case 0x24:
        return INC(ArithmeticTarget::H);
    case 0x22:
        return INC(ArithmeticTarget::L);

    case 0x3D:
        return DEC(ArithmeticTarget::A);
    case 0x5:
        return DEC(ArithmeticTarget::B);
    case 0xD:
        return DEC(ArithmeticTarget::C);
    case 0x15:
        return DEC(ArithmeticTarget::D);
    case 0x1D:
        return DEC(ArithmeticTarget::E);
    case 0x25:
        return DEC(ArithmeticTarget::H);
    case 0x2D:
        return DEC(ArithmeticTarget::L);

    case 0x3F:
        return CCF();

    case 0x37:
        return SCF();

    case 0x1F:
        return RRA();

    case 0x17:
        return RLA();

    case 0x0F:
        return RRCA();

    case 0x07:
        return RLCA();

    case 0x2F:
        return CPL();

    case 0xC2:
        return JP(JumpTest::NotZero);

    case 0xD2:
        return JP(JumpTest::NotCarry);

    case 0xCA:
        return JP(JumpTest::Zero);

    case 0xDA:
        return JP(JumpTest::Carry);

    case 0xC3:
        return JP(JumpTest::Always);

    case 0x38:
        return JR(JumpTest::Carry);

    case 0x30:
        return JR(JumpTest::NotCarry);

    case 0x28:
        return JR(JumpTest::Zero);

    case 0x20:
        return JR(JumpTest::NotZero);

    case 0x18:
        return JR(JumpTest::Always);

    case 0xE9:
        return JP_HL();
    }

    return std::nullopt;
}

std::optional<Instruction> Instruction::from_byte_prefixed(u8 byte) {
    switch (byte) {
    case 0x47:
        return BIT(RegisterBit::_0, ArithmeticTarget::A);
    case 0x4f:
        return BIT(RegisterBit::_1, ArithmeticTarget::A);
    case 0x57:
        return BIT(RegisterBit::_2, ArithmeticTarget::A);
    case 0x5f:
        return BIT(RegisterBit::_3, ArithmeticTarget::A);
    case 0x67:
        return BIT(RegisterBit::_4, ArithmeticTarget::A);
    case 0x6f:
        return BIT(RegisterBit::_5, ArithmeticTarget::A);
    case 0x77:
        return BIT(RegisterBit::_6, ArithmeticTarget::A);
    case 0x7f:
        return BIT(RegisterBit::_7, ArithmeticTarget::A);

    case 0x40:
        return BIT(RegisterBit::_0, ArithmeticTarget::B);
    case 0x48:
        return BIT(RegisterBit::_1, ArithmeticTarget::B);
    case 0x50:
        return BIT(RegisterBit::_2, ArithmeticTarget::B);
    case 0x58:
        return BIT(RegisterBit::_3, ArithmeticTarget::B);
    case 0x60:
        return BIT(RegisterBit::_4, ArithmeticTarget::B);
    case 0x68:
        return BIT(RegisterBit::_5, ArithmeticTarget::B);
    case 0x70:
        return BIT(RegisterBit::_6, ArithmeticTarget::B);
    case 0x78:
        return BIT(RegisterBit::_7, ArithmeticTarget::B);

    case 0x41:
        return BIT(RegisterBit::_0, ArithmeticTarget::C);
    case 0x49:
        return BIT(RegisterBit::_1, ArithmeticTarget::C);
    case 0x51:
        return BIT(RegisterBit::_2, ArithmeticTarget::C);
    case 0x59:
        return BIT(RegisterBit::_3, ArithmeticTarget::C);
    case 0x61:
        return BIT(RegisterBit::_4, ArithmeticTarget::C);
    case 0x69:
        return BIT(RegisterBit::_5, ArithmeticTarget::C);
    case 0x71:
        return BIT(RegisterBit::_6, ArithmeticTarget::C);
    case 0x79:
        return BIT(RegisterBit::_7, ArithmeticTarget::C);

    case 0x42:
        return BIT(RegisterBit::_0, ArithmeticTarget::D);
    case 0x4a:
        return BIT(RegisterBit::_1, ArithmeticTarget::D);
    case 0x52:
        return BIT(RegisterBit::_2, ArithmeticTarget::D);
    case 0x5a:
        return BIT(RegisterBit::_3, ArithmeticTarget::D);
    case 0x62:
        return BIT(RegisterBit::_4, ArithmeticTarget::D);
    case 0x6a:
        return BIT(RegisterBit::_5, ArithmeticTarget::D);
    case 0x72:
        return BIT(RegisterBit::_6, ArithmeticTarget::D);
    case 0x7a:
        return BIT(RegisterBit::_7, ArithmeticTarget::D);

    case 0x43:
        return BIT(RegisterBit::_0, ArithmeticTarget::E);
    case 0x4b:
        return BIT(RegisterBit::_1, ArithmeticTarget::E);
    case 0x53:
        return BIT(RegisterBit::_2, ArithmeticTarget::E);
    case 0x5b:
        return BIT(RegisterBit::_3, ArithmeticTarget::E);
    case 0x63:
        return BIT(RegisterBit::_4, ArithmeticTarget::E);
    case 0x6b:
        return BIT(RegisterBit::_5, ArithmeticTarget::E);
    case 0x73:
        return BIT(RegisterBit::_6, ArithmeticTarget::E);
    case 0x7b:
        return BIT(RegisterBit::_7, ArithmeticTarget::E);

    case 0x44:
        return BIT(RegisterBit::_0, ArithmeticTarget::H);
    case 0x4c:
        return BIT(RegisterBit::_1, ArithmeticTarget::H);
    case 0x54:
        return BIT(RegisterBit::_2, ArithmeticTarget::H);
    case 0x5c:
        return BIT(RegisterBit::_3, ArithmeticTarget::H);
    case 0x64:
        return BIT(RegisterBit::_4, ArithmeticTarget::H);
    case 0x6c:
        return BIT(RegisterBit::_5, ArithmeticTarget::H);
    case 0x74:
        return BIT(RegisterBit::_6, ArithmeticTarget::H);
    case 0x7c:
        return BIT(RegisterBit::_7, ArithmeticTarget::H);

    case 0x45:
        return BIT(RegisterBit::_0, ArithmeticTarget::L);
    case 0x4d:
        return BIT(RegisterBit::_1, ArithmeticTarget::L);
    case 0x55:
        return BIT(RegisterBit::_2, ArithmeticTarget::L);
    case 0x5d:
        return BIT(RegisterBit::_3, ArithmeticTarget::L);
    case 0x65:
        return BIT(RegisterBit::_4, ArithmeticTarget::L);
    case 0x6d:
        return BIT(RegisterBit::_5, ArithmeticTarget::L);
    case 0x75:
        return BIT(RegisterBit::_6, ArithmeticTarget::L);
    case 0x7d:
        return BIT(RegisterBit::_7, ArithmeticTarget::L);


    case 0x87:
        return RESET(RegisterBit::_0, ArithmeticTarget::A);
    case 0x8f:
        return RESET(RegisterBit::_1, ArithmeticTarget::A);
    case 0x97:
        return RESET(RegisterBit::_2, ArithmeticTarget::A);
    case 0x9f:
        return RESET(RegisterBit::_3, ArithmeticTarget::A);
    case 0xa7:
        return RESET(RegisterBit::_4, ArithmeticTarget::A);
    case 0xaf:
        return RESET(RegisterBit::_5, ArithmeticTarget::A);
    case 0xb7:
        return RESET(RegisterBit::_6, ArithmeticTarget::A);
    case 0xbf:
        return RESET(RegisterBit::_7, ArithmeticTarget::A);

    case 0x80:
        return RESET(RegisterBit::_0, ArithmeticTarget::B);
    case 0x88:
        return RESET(RegisterBit::_1, ArithmeticTarget::B);
    case 0x90:
        return RESET(RegisterBit::_2, ArithmeticTarget::B);
    case 0x98:
        return RESET(RegisterBit::_3, ArithmeticTarget::B);
    case 0xa0:
        return RESET(RegisterBit::_4, ArithmeticTarget::B);
    case 0xa8:
        return RESET(RegisterBit::_5, ArithmeticTarget::B);
    case 0xb0:
        return RESET(RegisterBit::_6, ArithmeticTarget::B);
    case 0xb8:
        return RESET(RegisterBit::_7, ArithmeticTarget::B);

    case 0x81:
        return RESET(RegisterBit::_0, ArithmeticTarget::C);
    case 0x89:
        return RESET(RegisterBit::_1, ArithmeticTarget::C);
    case 0x91:
        return RESET(RegisterBit::_2, ArithmeticTarget::C);
    case 0x99:
        return RESET(RegisterBit::_3, ArithmeticTarget::C);
    case 0xa1:
        return RESET(RegisterBit::_4, ArithmeticTarget::C);
    case 0xa9:
        return RESET(RegisterBit::_5, ArithmeticTarget::C);
    case 0xb1:
        return RESET(RegisterBit::_6, ArithmeticTarget::C);
    case 0xb9:
        return RESET(RegisterBit::_7, ArithmeticTarget::C);

    case 0x82:
        return RESET(RegisterBit::_0, ArithmeticTarget::D);
    case 0x8a:
        return RESET(RegisterBit::_1, ArithmeticTarget::D);
    case 0x92:
        return RESET(RegisterBit::_2, ArithmeticTarget::D);
    case 0x9a:
        return RESET(RegisterBit::_3, ArithmeticTarget::D);
    case 0xa2:
        return RESET(RegisterBit::_4, ArithmeticTarget::D);
    case 0xaa:
        return RESET(RegisterBit::_5, ArithmeticTarget::D);
    case 0xb2:
        return RESET(RegisterBit::_6, ArithmeticTarget::D);
    case 0xba:
        return RESET(RegisterBit::_7, ArithmeticTarget::D);

    case 0x83:
        return RESET(RegisterBit::_0, ArithmeticTarget::E);
    case 0x8b:
        return RESET(RegisterBit::_1, ArithmeticTarget::E);
    case 0x93:
        return RESET(RegisterBit::_2, ArithmeticTarget::E);
    case 0x9b:
        return RESET(RegisterBit::_3, ArithmeticTarget::E);
    case 0xa3:
        return RESET(RegisterBit::_4, ArithmeticTarget::E);
    case 0xab:
        return RESET(RegisterBit::_5, ArithmeticTarget::E);
    case 0xb3:
        return RESET(RegisterBit::_6, ArithmeticTarget::E);
    case 0xbb:
        return RESET(RegisterBit::_7, ArithmeticTarget::E);

    case 0x84:
        return RESET(RegisterBit::_0, ArithmeticTarget::H);
    case 0x8c:
        return RESET(RegisterBit::_1, ArithmeticTarget::H);
    case 0x94:
        return RESET(RegisterBit::_2, ArithmeticTarget::H);
    case 0x9c:
        return RESET(RegisterBit::_3, ArithmeticTarget::H);
    case 0xa4:
        return RESET(RegisterBit::_4, ArithmeticTarget::H);
    case 0xac:
        return RESET(RegisterBit::_5, ArithmeticTarget::H);
    case 0xb4:
        return RESET(RegisterBit::_6, ArithmeticTarget::H);
    case 0xbc:
        return RESET(RegisterBit::_7, ArithmeticTarget::H);

    case 0x85:
        return RESET(RegisterBit::_0, ArithmeticTarget::L);
    case 0x8d:
        return RESET(RegisterBit::_1, ArithmeticTarget::L);
    case 0x95:
        return RESET(RegisterBit::_2, ArithmeticTarget::L);
    case 0x9d:
        return RESET(RegisterBit::_3, ArithmeticTarget::L);
    case 0xa5:
        return RESET(RegisterBit::_4, ArithmeticTarget::L);
    case 0xad:
        return RESET(RegisterBit::_5, ArithmeticTarget::L);
    case 0xb5:
        return RESET(RegisterBit::_6, ArithmeticTarget::L);
    case 0xbd:
        return RESET(RegisterBit::_7, ArithmeticTarget::L);


    case 0xc7:
        return SET(RegisterBit::_0, ArithmeticTarget::A);
    case 0xcf:
        return SET(RegisterBit::_1, ArithmeticTarget::A);
    case 0xd7:
        return SET(RegisterBit::_2, ArithmeticTarget::A);
    case 0xdf:
        return SET(RegisterBit::_3, ArithmeticTarget::A);
    case 0xe7:
        return SET(RegisterBit::_4, ArithmeticTarget::A);
    case 0xef:
        return SET(RegisterBit::_5, ArithmeticTarget::A);
    case 0xf7:
        return SET(RegisterBit::_6, ArithmeticTarget::A);
    case 0xff:
        return SET(RegisterBit::_7, ArithmeticTarget::A);

    case 0xc0:
        return SET(RegisterBit::_0, ArithmeticTarget::B);
    case 0xc8:
        return SET(RegisterBit::_1, ArithmeticTarget::B);
    case 0xd0:
        return SET(RegisterBit::_2, ArithmeticTarget::B);
    case 0xd8:
        return SET(RegisterBit::_3, ArithmeticTarget::B);
    case 0xe0:
        return SET(RegisterBit::_4, ArithmeticTarget::B);
    case 0xe8:
        return SET(RegisterBit::_5, ArithmeticTarget::B);
    case 0xf0:
        return SET(RegisterBit::_6, ArithmeticTarget::B);
    case 0xf8:
        return SET(RegisterBit::_7, ArithmeticTarget::B);

    case 0xc1:
        return SET(RegisterBit::_0, ArithmeticTarget::C);
    case 0xc9:
        return SET(RegisterBit::_1, ArithmeticTarget::C);
    case 0xd1:
        return SET(RegisterBit::_2, ArithmeticTarget::C);
    case 0xd9:
        return SET(RegisterBit::_3, ArithmeticTarget::C);
    case 0xe1:
        return SET(RegisterBit::_4, ArithmeticTarget::C);
    case 0xe9:
        return SET(RegisterBit::_5, ArithmeticTarget::C);
    case 0xf1:
        return SET(RegisterBit::_6, ArithmeticTarget::C);
    case 0xf9:
        return SET(RegisterBit::_7, ArithmeticTarget::C);

    case 0xc2:
        return SET(RegisterBit::_0, ArithmeticTarget::D);
    case 0xca:
        return SET(RegisterBit::_1, ArithmeticTarget::D);
    case 0xd2:
        return SET(RegisterBit::_2, ArithmeticTarget::D);
    case 0xda:
        return SET(RegisterBit::_3, ArithmeticTarget::D);
    case 0xe2:
        return SET(RegisterBit::_4, ArithmeticTarget::D);
    case 0xea:
        return SET(RegisterBit::_5, ArithmeticTarget::D);
    case 0xf2:
        return SET(RegisterBit::_6, ArithmeticTarget::D);
    case 0xfa:
        return SET(RegisterBit::_7, ArithmeticTarget::D);

    case 0xc3:
        return SET(RegisterBit::_0, ArithmeticTarget::E);
    case 0xcb:
        return SET(RegisterBit::_1, ArithmeticTarget::E);
    case 0xd3:
        return SET(RegisterBit::_2, ArithmeticTarget::E);
    case 0xdb:
        return SET(RegisterBit::_3, ArithmeticTarget::E);
    case 0xe3:
        return SET(RegisterBit::_4, ArithmeticTarget::E);
    case 0xeb:
        return SET(RegisterBit::_5, ArithmeticTarget::E);
    case 0xf3:
        return SET(RegisterBit::_6, ArithmeticTarget::E);
    case 0xfb:
        return SET(RegisterBit::_7, ArithmeticTarget::E);

    case 0xc4:
        return SET(RegisterBit::_0, ArithmeticTarget::H);
    case 0xcc:
        return SET(RegisterBit::_1, ArithmeticTarget::H);
    case 0xd4:
        return SET(RegisterBit::_2, ArithmeticTarget::H);
    case 0xdc:
        return SET(RegisterBit::_3, ArithmeticTarget::H);
    case 0xe4:
        return SET(RegisterBit::_4, ArithmeticTarget::H);
    case 0xec:
        return SET(RegisterBit::_5, ArithmeticTarget::H);
    case 0xf4:
        return SET(RegisterBit::_6, ArithmeticTarget::H);
    case 0xfc:
        return SET(RegisterBit::_7, ArithmeticTarget::H);

    case 0xc5:
        return SET(RegisterBit::_0, ArithmeticTarget::L);
    case 0xcd:
        return SET(RegisterBit::_1, ArithmeticTarget::L);
    case 0xd5:
        return SET(RegisterBit::_2, ArithmeticTarget::L);
    case 0xdd:
        return SET(RegisterBit::_3, ArithmeticTarget::L);
    case 0xe5:
        return SET(RegisterBit::_4, ArithmeticTarget::L);
    case 0xed:
        return SET(RegisterBit::_5, ArithmeticTarget::L);
    case 0xf5:
        return SET(RegisterBit::_6, ArithmeticTarget::L);
    case 0xfd:
        return SET(RegisterBit::_7, ArithmeticTarget::L);


    case 0x3f:
        return SRL(ArithmeticTarget::A);
    case 0x38:
        return SRL(ArithmeticTarget::B);
    case 0x39:
        return SRL(ArithmeticTarget::C);
    case 0x3a:
        return SRL(ArithmeticTarget::D);
    case 0x3b:
        return SRL(ArithmeticTarget::E);
    case 0x3c:
        return SRL(ArithmeticTarget::H);
    case 0x3d:
        return SRL(ArithmeticTarget::L);


    case 0x1f:
        return RR(ArithmeticTarget::A);
    case 0x18:
        return RR(ArithmeticTarget::B);
    case 0x19:
        return RR(ArithmeticTarget::C);
    case 0x1a:
        return RR(ArithmeticTarget::D);
    case 0x1b:
        return RR(ArithmeticTarget::E);
    case 0x1c:
        return RR(ArithmeticTarget::H);
    case 0x1d:
        return RR(ArithmeticTarget::L);


    case 0x17:
        return RL(ArithmeticTarget::A);
    case 0x10:
        return RL(ArithmeticTarget::B);
    case 0x11:
        return RL(ArithmeticTarget::C);
    case 0x12:
        return RL(ArithmeticTarget::D);
    case 0x13:
        return RL(ArithmeticTarget::E);
    case 0x14:
        return RL(ArithmeticTarget::H);
    case 0x15:
        return RL(ArithmeticTarget::L);


    case 0xf:
        return RRC(ArithmeticTarget::A);
    case 0x8:
        return RRC(ArithmeticTarget::B);
    case 0x9:
        return RRC(ArithmeticTarget::C);
    case 0xa:
        return RRC(ArithmeticTarget::D);
    case 0xb:
        return RRC(ArithmeticTarget::E);
    case 0xc:
        return RRC(ArithmeticTarget::H);
    case 0xd:
        return RRC(ArithmeticTarget::L);


    case 0x7:
        return RLC(ArithmeticTarget::A);
    case 0x0:
        return RLC(ArithmeticTarget::B);
    case 0x1:
        return RLC(ArithmeticTarget::C);
    case 0x2:
        return RLC(ArithmeticTarget::D);
    case 0x3:
        return RLC(ArithmeticTarget::E);
    case 0x4:
        return RLC(ArithmeticTarget::H);
    case 0x5:
        return RLC(ArithmeticTarget::L);


    case 0x2f:
        return SRA(ArithmeticTarget::A);
    case 0x28:
        return SRA(ArithmeticTarget::B);
    case 0x29:
        return SRA(ArithmeticTarget::C);
    case 0x2a:
        return SRA(ArithmeticTarget::D);
    case 0x2b:
        return SRA(ArithmeticTarget::E);
    case 0x2c:
        return SRA(ArithmeticTarget::H);
    case 0x2d:
        return SRA(ArithmeticTarget::L);


    case 0x27:
        return SLA(ArithmeticTarget::A);
    case 0x20:
        return SLA(ArithmeticTarget::B);
    case 0x21:
        return SLA(ArithmeticTarget::C);
    case 0x22:
        return SLA(ArithmeticTarget::D);
    case 0x23:
        return SLA(ArithmeticTarget::E);
    case 0x24:
        return SLA(ArithmeticTarget::H);
    case 0x25:
        return SLA(ArithmeticTarget::L);


    case 0x37:
        return SWAP(ArithmeticTarget::A);
    case 0x30:
        return SWAP(ArithmeticTarget::B);
    case 0x31:
        return SWAP(ArithmeticTarget::C);
    case 0x32:
        return SWAP(ArithmeticTarget::D);
    case 0x33:
        return SWAP(ArithmeticTarget::E);
    case 0x34:
        return SWAP(ArithmeticTarget::H);
    case 0x35:
        return SWAP(ArithmeticTarget::L);
    }

    return std::nullopt;
}

int main()
{
    std::cout << "Hello DMG-01!\n";
}