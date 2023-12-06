#include <iostream>
#include <cstdint>
#include <bitset>
#include <bit>
#include <array>
#include <optional>
#include "SafeInt.hpp"

using u8 = uint8_t;
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
    ADD, ADDHL, ADDC, SUB, SBC, AND, OR, XOR, CP, INC, DEC, CCF, SCF, RRA, RLA, RRCA, RRLA, CPL, BIT,
    RESET, SET, SRL, RR, RL, RRC, RLC, SRA, SLA, SWAP,
};

enum struct ArithmeticTarget : u8 {
    A, B, C, D, E, H, L,
    AF, BC, DE, HL
};

enum struct RegisterBit : u8 {
    _0, _1, _2, _3, _4, _5, _6, _7
};

struct Instruction {
    InstructionEnum inst_enum;
    ArithmeticTarget target;
    RegisterBit bit;

    Instruction(InstructionEnum inst_enum) : inst_enum(inst_enum) {}

    static std::optional<Instruction> from_byte(u8 byte);
};

struct ADD : Instruction { ADD() : Instruction(InstructionEnum::ADD) {} };
struct ADDHL : Instruction { ADDHL() : Instruction(InstructionEnum::ADDHL) {} };
struct ADDC : Instruction { ADDC() : Instruction(InstructionEnum::ADDC) {} };
struct SUB : Instruction { SUB() : Instruction(InstructionEnum::SUB) {} };
struct SBC : Instruction { SBC() : Instruction(InstructionEnum::SBC) {} };
struct AND : Instruction { AND() : Instruction(InstructionEnum::AND) {} };
struct OR : Instruction { OR() : Instruction(InstructionEnum::OR) {} };
struct XOR : Instruction { XOR() : Instruction(InstructionEnum::XOR) {} };
struct CP : Instruction { CP() : Instruction(InstructionEnum::CP) {} };
struct INC : Instruction { INC() : Instruction(InstructionEnum::INC) {} };
struct DEC : Instruction { DEC() : Instruction(InstructionEnum::DEC) {} };
struct CCF : Instruction { CCF() : Instruction(InstructionEnum::CCF) {} };
struct SCF : Instruction { SCF() : Instruction(InstructionEnum::SCF) {} };
struct RRA : Instruction { RRA() : Instruction(InstructionEnum::RRA) {} };
struct RLA : Instruction { RLA() : Instruction(InstructionEnum::RLA) {} };
struct RRCA : Instruction { RRCA() : Instruction(InstructionEnum::RRCA) {} };
struct RRLA : Instruction { RRLA() : Instruction(InstructionEnum::RRLA) {} };
struct CPL : Instruction { CPL() : Instruction(InstructionEnum::CPL) {} };
struct BIT : Instruction { BIT() : Instruction(InstructionEnum::BIT) {} };
struct RESET : Instruction { RESET() : Instruction(InstructionEnum::RESET) {} };
struct SET : Instruction { SET() : Instruction(InstructionEnum::SET) {} };
struct SRL : Instruction { SRL() : Instruction(InstructionEnum::SRL) {} };
struct RR : Instruction { RR() : Instruction(InstructionEnum::RR) {} };
struct RL : Instruction { RL() : Instruction(InstructionEnum::RL) {} };
struct RRC : Instruction { RRC() : Instruction(InstructionEnum::RRC) {} };
struct RLC : Instruction { RLC() : Instruction(InstructionEnum::RLC) {} };
struct SRA : Instruction { SRA() : Instruction(InstructionEnum::SRA) {} };
struct SLA : Instruction { SLA() : Instruction(InstructionEnum::SLA) {} };
struct SWAP : Instruction { SWAP() : Instruction(InstructionEnum::SWAP) {} };

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

    void execute(Instruction instruction);
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
    void rrla();
    void cpl();
    u8 bit(u8 reg, u8 bit);
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
    registers.AF.flags.subtract = false;
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

void CPU::rrla() {
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

u8 CPU::bit(u8 reg, u8 bit) {
    auto reg_mirror = std::bitset<8>(reg);
    bool res = reg_mirror[bit];

    registers.AF.flags.subtract = false;

    return (res ? 1 : 0);
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
    u8 res = value >> 1;

    registers.AF.flags.zero = res == 0;
    registers.AF.flags.subtract = false;
    registers.AF.flags.carry = false;
    registers.AF.flags.half_carry = (registers.AF.first & 0xf) + (value & 0xf) > 0xf;

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

void CPU::execute(Instruction instruction) {
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
        switch (instruction.target) {
        case ArithmeticTarget::A: {
            ccf();
        }
                                break;
        case ArithmeticTarget::B: {
            ccf();
        }
                                break;
        case ArithmeticTarget::C: {
            ccf();
        }
                                break;
        case ArithmeticTarget::D: {
            ccf();
        }
                                break;
        case ArithmeticTarget::E: {
            ccf();
        }
                                break;
        case ArithmeticTarget::H: {
            ccf();
        }
                                break;
        case ArithmeticTarget::L: {
            ccf();
        }
                                break;
        }
        __assume(false);
    }
    break;
    case InstructionEnum::SCF:
    {
        switch (instruction.target) {
        case ArithmeticTarget::A: {
            scf();
        }
                                break;
        case ArithmeticTarget::B: {
            scf();
        }
                                break;
        case ArithmeticTarget::C: {
            scf();
        }
                                break;
        case ArithmeticTarget::D: {
            scf();
        }
                                break;
        case ArithmeticTarget::E: {
            scf();
        }
                                break;
        case ArithmeticTarget::H: {
            scf();
        }
                                break;
        case ArithmeticTarget::L: {
            scf();
        }
                                break;
        }
        __assume(false);
    }
    break;
    case InstructionEnum::RRA:
    {
        switch (instruction.target) {
        case ArithmeticTarget::A: {
            rra();
        }
                                break;
        case ArithmeticTarget::B: {
            rra();
        }
                                break;
        case ArithmeticTarget::C: {
            rra();
        }
                                break;
        case ArithmeticTarget::D: {
            rra();
        }
                                break;
        case ArithmeticTarget::E: {
            rra();
        }
                                break;
        case ArithmeticTarget::H: {
            rra();
        }
                                break;
        case ArithmeticTarget::L: {
            rra();
        }
                                break;
        }
        __assume(false);
    }
    break;
    case InstructionEnum::RLA:
    {
        switch (instruction.target) {
        case ArithmeticTarget::A: {
            rla();
        }
                                break;
        case ArithmeticTarget::B: {
            rla();
        }
                                break;
        case ArithmeticTarget::C: {
            rla();
        }
                                break;
        case ArithmeticTarget::D: {
            rla();
        }
                                break;
        case ArithmeticTarget::E: {
            rla();
        }
                                break;
        case ArithmeticTarget::H: {
            rla();
        }
                                break;
        case ArithmeticTarget::L: {
            rla();
        }
                                break;
        }
        __assume(false);
    }
    break;
    case InstructionEnum::RRCA:
    {
        switch (instruction.target) {
        case ArithmeticTarget::A: {
            rrca();
        }
                                break;
        case ArithmeticTarget::B: {
            rrca();
        }
                                break;
        case ArithmeticTarget::C: {
            rrca();
        }
                                break;
        case ArithmeticTarget::D: {
            rrca();
        }
                                break;
        case ArithmeticTarget::E: {
            rrca();
        }
                                break;
        case ArithmeticTarget::H: {
            rrca();
        }
                                break;
        case ArithmeticTarget::L: {
            rrca();
        }
                                break;
        }
        __assume(false);
    }
    break;
    case InstructionEnum::RRLA:
    {
        switch (instruction.target) {
        case ArithmeticTarget::A: {
            rrla();
        }
                                break;
        case ArithmeticTarget::B: {
            rrla();
        }
                                break;
        case ArithmeticTarget::C: {
            rrla();
        }
                                break;
        case ArithmeticTarget::D: {
            rrla();
        }
                                break;
        case ArithmeticTarget::E: {
            rrla();
        }
                                break;
        case ArithmeticTarget::H: {
            rrla();
        }
                                break;
        case ArithmeticTarget::L: {
            rrla();
        }
                                break;
        }
        __assume(false);
    }
    break;
    case InstructionEnum::CPL:
    {
        switch (instruction.target) {
        case ArithmeticTarget::A: {
            cpl();
        }
                                break;
        case ArithmeticTarget::B: {
            cpl();
        }
                                break;
        case ArithmeticTarget::C: {
            cpl();
        }
                                break;
        case ArithmeticTarget::D: {
            cpl();
        }
                                break;
        case ArithmeticTarget::E: {
            cpl();
        }
                                break;
        case ArithmeticTarget::H: {
            cpl();
        }
                                break;
        case ArithmeticTarget::L: {
            cpl();
        }
                                break;
        }
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
                    registers.AF.first = bit(value, 0);
                    break;
                case RegisterBit::_1:
                    registers.AF.first = bit(value, 1);
                    break;
                case RegisterBit::_2:
                    registers.AF.first = bit(value, 2);
                    break;
                case RegisterBit::_3:
                    registers.AF.first = bit(value, 3);
                    break;
                case RegisterBit::_4:
                    registers.AF.first = bit(value, 4);
                    break;
                case RegisterBit::_5:
                    registers.AF.first = bit(value, 5);
                    break;
                case RegisterBit::_6:
                    registers.AF.first = bit(value, 6);
                    break;
                case RegisterBit::_7:
                    registers.AF.first = bit(value, 7);
                    break;
            }
        }
                                break;
        case ArithmeticTarget::B: {
            auto value = registers.BC.first;
            switch (instruction.bit) {
            case RegisterBit::_0:
                registers.BC.first = bit(value, 0);
                break;
            case RegisterBit::_1:
                registers.BC.first = bit(value, 1);
                break;
            case RegisterBit::_2:
                registers.BC.first = bit(value, 2);
                break;
            case RegisterBit::_3:
                registers.BC.first = bit(value, 3);
                break;
            case RegisterBit::_4:
                registers.BC.first = bit(value, 4);
                break;
            case RegisterBit::_5:
                registers.BC.first = bit(value, 5);
                break;
            case RegisterBit::_6:
                registers.BC.first = bit(value, 6);
                break;
            case RegisterBit::_7:
                registers.BC.first = bit(value, 7);
                break;
            }
        }
                                break;
        case ArithmeticTarget::C: {
            auto value = registers.BC.second;
            switch (instruction.bit) {
            case RegisterBit::_0:
                registers.BC.second = bit(value, 0);
                break;
            case RegisterBit::_1:
                registers.BC.second = bit(value, 1);
                break;
            case RegisterBit::_2:
                registers.BC.second = bit(value, 2);
                break;
            case RegisterBit::_3:
                registers.BC.second = bit(value, 3);
                break;
            case RegisterBit::_4:
                registers.BC.second = bit(value, 4);
                break;
            case RegisterBit::_5:
                registers.BC.second = bit(value, 5);
                break;
            case RegisterBit::_6:
                registers.BC.second = bit(value, 6);
                break;
            case RegisterBit::_7:
                registers.BC.second = bit(value, 7);
                break;
            }
        }
                                break;
        case ArithmeticTarget::D: {
            auto value = registers.DE.first;
            switch (instruction.bit) {
            case RegisterBit::_0:
                registers.DE.first = bit(value, 0);
                break;
            case RegisterBit::_1:
                registers.DE.first = bit(value, 1);
                break;
            case RegisterBit::_2:
                registers.DE.first = bit(value, 2);
                break;
            case RegisterBit::_3:
                registers.DE.first = bit(value, 3);
                break;
            case RegisterBit::_4:
                registers.DE.first = bit(value, 4);
                break;
            case RegisterBit::_5:
                registers.DE.first = bit(value, 5);
                break;
            case RegisterBit::_6:
                registers.DE.first = bit(value, 6);
                break;
            case RegisterBit::_7:
                registers.DE.first = bit(value, 7);
                break;
            }
        }
                                break;
        case ArithmeticTarget::E: {
            auto value = registers.DE.second;
            switch (instruction.bit) {
            case RegisterBit::_0:
                registers.DE.second = bit(value, 0);
                break;
            case RegisterBit::_1:
                registers.DE.second = bit(value, 1);
                break;
            case RegisterBit::_2:
                registers.DE.second = bit(value, 2);
                break;
            case RegisterBit::_3:
                registers.DE.second = bit(value, 3);
                break;
            case RegisterBit::_4:
                registers.DE.second = bit(value, 4);
                break;
            case RegisterBit::_5:
                registers.DE.second = bit(value, 5);
                break;
            case RegisterBit::_6:
                registers.DE.second = bit(value, 6);
                break;
            case RegisterBit::_7:
                registers.DE.second = bit(value, 7);
                break;
            }
        }
                                break;
        case ArithmeticTarget::H: {
            auto value = registers.HL.first;
            switch (instruction.bit) {
            case RegisterBit::_0:
                registers.HL.first = bit(value, 0);
                break;
            case RegisterBit::_1:
                registers.HL.first = bit(value, 1);
                break;
            case RegisterBit::_2:
                registers.HL.first = bit(value, 2);
                break;
            case RegisterBit::_3:
                registers.HL.first = bit(value, 3);
                break;
            case RegisterBit::_4:
                registers.HL.first = bit(value, 4);
                break;
            case RegisterBit::_5:
                registers.HL.first = bit(value, 5);
                break;
            case RegisterBit::_6:
                registers.HL.first = bit(value, 6);
                break;
            case RegisterBit::_7:
                registers.HL.first = bit(value, 7);
                break;
            }
        }
                                break;
        case ArithmeticTarget::L: {
            auto value = registers.HL.second;
            switch (instruction.bit) {
            case RegisterBit::_0:
                registers.HL.second = bit(value, 0);
                break;
            case RegisterBit::_1:
                registers.HL.second = bit(value, 1);
                break;
            case RegisterBit::_2:
                registers.HL.second = bit(value, 2);
                break;
            case RegisterBit::_3:
                registers.HL.second = bit(value, 3);
                break;
            case RegisterBit::_4:
                registers.HL.second = bit(value, 4);
                break;
            case RegisterBit::_5:
                registers.HL.second = bit(value, 5);
                break;
            case RegisterBit::_6:
                registers.HL.second = bit(value, 6);
                break;
            case RegisterBit::_7:
                registers.HL.second = bit(value, 7);
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
    }

    //assert(false);
    __assume(false); //msvc, clang-cl
}

void CPU::step() {
    auto instruction_byte = bus.read_byte(pc);
    
    std::optional<Instruction> instruction = Instruction::from_byte(instruction_byte);
    if (instruction) {
        pc = execute(*instruction);
    }
    else {
        std::cerr << "Unknown instruction found for: 0x" << std::hex << instruction_byte << "\n";
        exit(-1);
    }
}

static std::optional<Instruction> from_byte(u8 byte) {
    switch (byte) {
        
    }
}

int main()
{
    std::cout << "Hello DMG-01!\n";
}