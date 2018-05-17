#include "cpu.h"

#define OP_RNM(name, ...) ([](parsed_reference_t &reference) -> opcode_name_t { return {(name), __VA_ARGS__}; })
#define OP_NAMESEL(expr, ...) ([](parsed_reference_t &reference) -> opcode_name_t { return {(expr), __VA_ARGS__}; })

#define OP_EXACT_NAMESEL(name) ([](uint8_t bytes[]) -> const char* { return name; })
#define OP_RNM_EXACTL(name, exactl) \
    ([](parsed_reference_t &reference) -> opcode_name_t { return {(name), opcode_arg_t::EXACT, opcode_arg_t::NONE, OP_EXACT_NAMESEL(exactl)}; })
#define OP_RNM_EXACTL_D(name, exactr) \
    ([](parsed_reference_t &reference) -> opcode_name_t { return {(name), opcode_arg_t::EXACT, opcode_arg_t::NONE, exactr}; })
#define OP_RNM_EXACT2(name, exactl, exactr) \
    ([](parsed_reference_t &reference) -> opcode_name_t { return {(name), opcode_arg_t::EXACT, opcode_arg_t::EXACT, OP_EXACT_NAMESEL(exactl), OP_EXACT_NAMESEL(exactr)}; })
#define OP_RNM_EXACT2_D(name, exactl, exactr) \
    ([](parsed_reference_t &reference) -> opcode_name_t { return {(name), opcode_arg_t::EXACT, opcode_arg_t::EXACT, exactl, exactr}; })

#define OP_PRED(expr) ([](uint8_t bytes[]) -> bool { return expr; })
#define OP_ACTION(...) ([](cpu_t &cpu, parsed_reference_t &reference, uint32_t matched[MAX_OPCODE_PATTERN_SIZE], uint8_t bytes[]) -> void {__VA_ARGS__;})
#define OP_ALU_C1(x) (static_cast<cpu_alu_op_t>(x))
#define OP_ALU_C2(x) (static_cast<cpu_alu_op_t>(x + ALU_OP_CLASS2_OFS))
#define OP_ALU_C3(x) (static_cast<cpu_alu_op_t>(x + ALU_OP_CLASS3_OFS))

#define OPW_B ref_width_t::BYTE
#define OPW_W ref_width_t::WORD
#define OPW_D ref_width_t::DWRD

#define OP_MODRM_REG(val) ((val & 0b00111000) >> 3)

const char *alu1_names[] = { "add", "or", "adc", "sbb", "and", "sub", "xor", "cmp" };
const char *alu2_names[] = { "rol", "ror", "rcl", "rcr", "shl", "shr", "sal", "sar" };
const char *alu3_names[] = { "test", "test", "not", "neg", "mul", "imul", "div", "idiv" };

static const char *regNamesByte[] = { "al", "cl", "dl", "bl", "ah", "ch", "dh", "bh" };
static const char *regNamesWord[] = { "ax", "cx", "dx", "bx", "sp", "bp", "si", "di" };
static const char *regNamesDwrd[] = { "eax", "ecx", "edx", "ebx", "esp", "ebp", "esi", "edi" };

static const char *sregNames[] = { "es", "cs", "ss", "ds", "fs", "gs" };

opcode_pattern_t cpu_t::m_opcodePatterns[] = {
    {   // ADD r/m16, r16
        { 0x01, P_MODRM16 }, OPW_W,
        OP_RNM("add", opcode_arg_t::MODRM_REF, opcode_arg_t::EXACT, nullptr, OP_EXACT_NAMESEL(regNamesByte[OP_MODRM_REG(bytes[1])])),
        OP_ACTION(cpu.aluOperation(ALU_OP_ADD, reference, cpu_register_t(reference.modrmRegField), ref_width_t::WORD))
    },
    {   // ADD r8, r/m8
        { 0x02, P_MODRM16 }, OPW_W,
        OP_RNM("add", opcode_arg_t::EXACT, opcode_arg_t::MODRM_REF, OP_EXACT_NAMESEL(regNamesByte[OP_MODRM_REG(bytes[1])])),
        OP_ACTION(cpu.aluOperation(ALU_OP_ADD, cpu_register_t(reference.modrmRegField), reference, ref_width_t::BYTE))
    },
    {   // ADD r16, r/m16
        { 0x03, P_MODRM16 }, OPW_W,
        OP_RNM("add", opcode_arg_t::EXACT, opcode_arg_t::MODRM_REF, OP_EXACT_NAMESEL(regNamesWord[OP_MODRM_REG(bytes[1])])),
        OP_ACTION(cpu.aluOperation(ALU_OP_ADD, cpu_register_t(reference.modrmRegField), reference, ref_width_t::WORD))
    },
    {   // ADD AX, imm16
        { 0x05, P_WORD_VAL }, OPW_W,
        OP_RNM("add", opcode_arg_t::EXACT, opcode_arg_t::IMMEDIATE, OP_EXACT_NAMESEL("ax")),
        OP_ACTION(cpu.aluOperation(ALU_OP_ADD, cpu_register_t::REG_EAX, signExtendWord(matched[0]), ref_width_t::WORD))
    },
    {   // PUSH ES
        { 0x06 }, OPW_W,
        OP_RNM_EXACTL("push", "es"),
        OP_ACTION(cpu.pushw(cpu.es))
    },
    {   // POP ES
        { 0x07 }, OPW_W,
        OP_RNM_EXACTL("pop", "es"),
        OP_ACTION(cpu.es = cpu.popw())
    },
    {   // OR AL, imm8
        { 0x0C, P_BYTE_VAL }, OPW_B,
        OP_RNM("or", opcode_arg_t::EXACT, opcode_arg_t::IMMEDIATE, OP_EXACT_NAMESEL("al")),
        OP_ACTION(cpu.aluOperation(ALU_OP_OR, cpu_register_t::REG_EAX, signExtendWord(matched[0]), ref_width_t::BYTE))
    },
    {
        // JC rel16
        { 0x0F, 0x82, P_WORD_VAL }, OPW_W,
        OP_RNM("jc", opcode_arg_t::RELATIVE),
        OP_ACTION(if (cpu.eflags & CPUFLAG_CARRY) cpu.eip += signExtendWord(matched[0]))
    },
    {   // JC rel16
        { 0x0F, 0x82, P_WORD_VAL }, OPW_W,
        OP_RNM("jc", opcode_arg_t::RELATIVE),
        OP_ACTION(if (cpu.eflags & CPUFLAG_CARRY) cpu.eip += signExtendWord(matched[0]))
    },
    {   // JZ rel16
        { 0x0F, 0x84, P_WORD_VAL }, OPW_W,
        OP_RNM("jz", opcode_arg_t::RELATIVE),
        OP_ACTION(if (cpu.eflags & CPUFLAG_ZERO) cpu.eip += signExtendWord(matched[0]))
    },
    {   // JNZ rel16
        { 0x0F, 0x85, P_WORD_VAL }, OPW_W,
        OP_RNM("jnz", opcode_arg_t::RELATIVE),
        OP_ACTION(if (!(cpu.eflags & CPUFLAG_ZERO)) cpu.eip += signExtendWord(matched[0]))
    },
    {   // JNA rel16
        { 0x0F, 0x86, P_WORD_VAL }, OPW_W,
        OP_RNM("jna", opcode_arg_t::RELATIVE),
        OP_ACTION(if ((cpu.eflags & CPUFLAG_CARRY) || (cpu.eflags & CPUFLAG_ZERO)) cpu.eip += signExtendWord(matched[0]))
    },
    {   // JA rel16
        { 0x0F, 0x87, P_WORD_VAL }, OPW_W,
        OP_RNM("ja", opcode_arg_t::RELATIVE),
        OP_ACTION(if (!(cpu.eflags & CPUFLAG_CARRY) && !(cpu.eflags & CPUFLAG_ZERO)) cpu.eip += signExtendWord(matched[0]))
    },
    {   // JL rel16
        { 0x0F, 0x8c, P_WORD_VAL }, OPW_W,
        OP_RNM("jl", opcode_arg_t::RELATIVE),
        OP_ACTION(if (!!(cpu.eflags & CPUFLAG_CARRY) ^ !!(cpu.eflags & CPUFLAG_OVERFLOW)) cpu.eip += signExtendWord(matched[0]))
    },
    {   // PUSH DS
        { 0x1e }, OPW_W,
        OP_RNM_EXACTL("push", "ds"),
        OP_ACTION(cpu.pushw(cpu.ds))
    },
    {   // POP DS
        { 0x1f }, OPW_W,
        OP_RNM_EXACTL("pop", "ds"),
        OP_ACTION(cpu.ds = cpu.popw())
    },
    {   // AND AL, imm8
        { 0x24, P_BYTE_VAL }, OPW_B,
        OP_RNM("and", opcode_arg_t::EXACT, opcode_arg_t::IMMEDIATE, OP_EXACT_NAMESEL("al")),
        OP_ACTION(cpu.aluOperation(ALU_OP_AND, cpu_register_t::REG_EAX, signExtendByte(matched[0]), ref_width_t::BYTE))
    },
    {   // SUB AL, imm8
        { 0x2C, P_BYTE_VAL }, OPW_B,
        OP_RNM("sub", opcode_arg_t::EXACT, opcode_arg_t::IMMEDIATE, OP_EXACT_NAMESEL("al")),
        OP_ACTION(cpu.aluOperation(ALU_OP_SUB, cpu_register_t::REG_EAX, signExtendWord(matched[0]), ref_width_t::BYTE))
    },
    {   // SUB AX, imm16
        { 0x2D, P_WORD_VAL }, OPW_W,
        OP_RNM("sub", opcode_arg_t::EXACT, opcode_arg_t::IMMEDIATE, OP_EXACT_NAMESEL("ax")),
        OP_ACTION(cpu.aluOperation(ALU_OP_SUB, cpu_register_t::REG_EAX, signExtendWord(matched[0]), ref_width_t::WORD))
    },
    {   // XOR r/m8, r8
        { 0x30, P_MODRM16 }, OPW_B,
        OP_RNM("xor", opcode_arg_t::MODRM_REF, opcode_arg_t::EXACT, nullptr, OP_EXACT_NAMESEL(regNamesByte[OP_MODRM_REG(bytes[1])])),
        OP_ACTION(cpu.aluOperation(ALU_OP_XOR, reference, cpu_register_t(reference.modrmRegField), ref_width_t::BYTE))
    },
    {   // XOR r/m16, r16
        { 0x31, P_MODRM16 }, OPW_W,
        OP_RNM("xor", opcode_arg_t::MODRM_REF, opcode_arg_t::EXACT, nullptr, OP_EXACT_NAMESEL(regNamesByte[OP_MODRM_REG(bytes[1])])),
        OP_ACTION(cpu.aluOperation(ALU_OP_XOR, reference, cpu_register_t(reference.modrmRegField), ref_width_t::WORD))
    },
    {   // CMP r8, r/m8
        { 0x3A, P_MODRM16 }, OPW_B,
        OP_RNM("cmp", opcode_arg_t::EXACT, opcode_arg_t::MODRM_REF, OP_EXACT_NAMESEL(regNamesByte[OP_MODRM_REG(bytes[1])])),
        OP_ACTION(cpu.aluOperation(ALU_OP_CMP, cpu_register_t(reference.modrmRegField), reference, ref_width_t::BYTE))
    },
    {   // CMP r16, r/m16
        { 0x3B, P_MODRM16 }, OPW_W,
        OP_RNM("cmp", opcode_arg_t::EXACT, opcode_arg_t::MODRM_REF, OP_EXACT_NAMESEL(regNamesWord[OP_MODRM_REG(bytes[1])])),
        OP_ACTION(cpu.aluOperation(ALU_OP_CMP, cpu_register_t(reference.modrmRegField), reference, ref_width_t::WORD))
    },
    {   // CMP AL, imm8
        { 0x3C, P_BYTE_VAL }, OPW_B,
        OP_RNM("cmp", opcode_arg_t::EXACT, opcode_arg_t::IMMEDIATE, OP_EXACT_NAMESEL("al")),
        OP_ACTION(cpu.aluOperation(ALU_OP_CMP, cpu_register_t::REG_EAX, signExtendByte(matched[0]), ref_width_t::BYTE))
    },
    {   // CMP AX, imm16
        { 0x3D, P_WORD_VAL }, OPW_W,
        OP_RNM("cmp", opcode_arg_t::EXACT, opcode_arg_t::IMMEDIATE, OP_EXACT_NAMESEL("ax")),
        OP_ACTION(cpu.aluOperation(ALU_OP_CMP, cpu_register_t::REG_EAX, signExtendWord(matched[0]), ref_width_t::WORD))
    },
    {   // INC r16
        { P_BYTE_VAL }, OPW_W,
        OP_RNM_EXACTL("inc", regNamesWord[bytes[0] - 0x40]),
        OP_ACTION(cpu.pokew(cpu_register_t(bytes[0] - 0x40), cpu.peekw(cpu_register_t(bytes[0] - 0x40)) + 1)),
        OP_PRED(bytes[0] >= 0x40 && bytes[0] <= 0x47)
    },
    {   // DEC r16
        { P_BYTE_VAL }, OPW_W,
        OP_RNM_EXACTL("dec", regNamesWord[bytes[0] - 0x48]),
        OP_ACTION(cpu.pokew(cpu_register_t(bytes[0] - 0x48), cpu.peekw(cpu_register_t(bytes[0] - 0x48)) - 1)),
        OP_PRED(bytes[0] >= 0x48 && bytes[0] <= 0x4f)
    },
    {   // PUSH r16
        { P_BYTE_VAL }, OPW_W,
        OP_RNM_EXACTL("push", regNamesWord[bytes[0] - 0x50]),
        OP_ACTION(cpu.pushw(cpu.peekw(cpu_register_t(bytes[0] - 0x50)))),
        OP_PRED(bytes[0] >= 0x50 && bytes[0] <= 0x57)
    },
    {   // POP r16
        { P_BYTE_VAL }, OPW_W,
        OP_RNM_EXACTL("pop", regNamesWord[bytes[0] - 0x58]),
        OP_ACTION(cpu.pokew(cpu_register_t(bytes[0] - 0x58), cpu.popw())),
        OP_PRED(bytes[0] >= 0x58 && bytes[0] <= 0x5f)
    },
    {
        // PUSHA
        { 0x60 }, OPW_W,
        OP_RNM("pusha"),
        OP_ACTION({
            auto esp = cpu.esp;
            cpu.pushw(cpu.eax);
            cpu.pushw(cpu.ecx);
            cpu.pushw(cpu.edx);
            cpu.pushw(cpu.ebx);
            cpu.pushw(esp);
            cpu.pushw(cpu.ebp);
            cpu.pushw(cpu.esi);
            cpu.pushw(cpu.edi);
        })
    },
    {
        // POPA
        { 0x61 }, OPW_W,
        OP_RNM("popa"),
        OP_ACTION({
            auto esp = cpu.esp;
            cpu.eax = cpu.popw();
            cpu.ecx = cpu.popw();
            cpu.edx = cpu.popw();
            cpu.ebx = cpu.popw();
            cpu.popw();
            cpu.ebp = cpu.popw();
            cpu.esi = cpu.popw();
            cpu.edi = cpu.popw();
        })
    },
    {   // JC rel8
        { 0x72, P_BYTE_VAL }, OPW_B,
        OP_RNM("jc", opcode_arg_t::RELATIVE),
        OP_ACTION(if (cpu.eflags & CPUFLAG_CARRY) cpu.eip += signExtendByte(matched[0]))
    },
    {   // JZ rel8
        { 0x74, P_BYTE_VAL }, OPW_B,
        OP_RNM("jz", opcode_arg_t::RELATIVE),
        OP_ACTION(if (cpu.eflags & CPUFLAG_ZERO) cpu.eip += signExtendByte(matched[0]))
    },
    {   // JNZ rel8
        { 0x75, P_BYTE_VAL }, OPW_B,
        OP_RNM("jnz", opcode_arg_t::RELATIVE),
        OP_ACTION(if (!(cpu.eflags & CPUFLAG_ZERO)) cpu.eip += signExtendByte(matched[0]))
    },
    {   // JNA rel8
        { 0x76, P_BYTE_VAL }, OPW_B,
        OP_RNM("jna", opcode_arg_t::RELATIVE),
        OP_ACTION(if ((cpu.eflags & CPUFLAG_CARRY) || (cpu.eflags & CPUFLAG_ZERO)) cpu.eip += signExtendByte(matched[0]))
    },
    {   // JA rel8
        { 0x77, P_BYTE_VAL }, OPW_B,
        OP_RNM("ja", opcode_arg_t::RELATIVE),
        OP_ACTION(if (!(cpu.eflags & CPUFLAG_CARRY) && !(cpu.eflags & CPUFLAG_ZERO)) cpu.eip += signExtendByte(matched[0]))
    },
    {   // JL rel8
        { 0x7c, P_BYTE_VAL }, OPW_B,
        OP_RNM("jl", opcode_arg_t::RELATIVE),
        OP_ACTION(if (!!(cpu.eflags & CPUFLAG_CARRY) ^ !!(cpu.eflags & CPUFLAG_OVERFLOW)) cpu.eip += signExtendByte(matched[0]))
    },
    {   // ALU1 r/m8, imm8
        { 0x80, P_MODRM16, P_BYTE_VAL }, OPW_B,
        OP_NAMESEL(alu1_names[static_cast<size_t>(reference.modrmRegField)], opcode_arg_t::MODRM_REF, opcode_arg_t::IMMEDIATE),
        OP_ACTION(cpu.aluOperation(OP_ALU_C1(reference.modrmRegField), reference, signExtendByte(matched[0]), ref_width_t::BYTE))
    },
    {   // ALU1 r/m16, imm16
        { 0x81, P_MODRM16, P_WORD_VAL }, OPW_W,
        OP_NAMESEL(alu1_names[static_cast<size_t>(reference.modrmRegField)], opcode_arg_t::MODRM_REF, opcode_arg_t::IMMEDIATE),
        OP_ACTION(cpu.aluOperation(OP_ALU_C1(reference.modrmRegField), reference, signExtendWord(matched[0]), ref_width_t::WORD))
    },
    {   // ALU1 r/m16, imm8
        { 0x83, P_MODRM16, P_BYTE_VAL }, OPW_W,
        OP_NAMESEL(alu1_names[static_cast<size_t>(reference.modrmRegField)], opcode_arg_t::MODRM_REF, opcode_arg_t::IMMEDIATE),
        OP_ACTION(cpu.aluOperation(OP_ALU_C1(reference.modrmRegField), reference, signExtendByte(matched[0]), ref_width_t::WORD))
    },
    {   // TEST r/m8, r8
        { 0x84, P_MODRM16 }, OPW_W,
        OP_RNM("test", opcode_arg_t::MODRM_REF, opcode_arg_t::EXACT, nullptr, OP_EXACT_NAMESEL(regNamesByte[OP_MODRM_REG(bytes[1])])),
        OP_ACTION(cpu.aluOperation(cpu_alu_op_t::ALU_OP_TEST, reference, cpu_register_t(reference.modrmRegField), ref_width_t::BYTE))
    },
    {   // TEST r/m16, r16
        { 0x85, P_MODRM16 }, OPW_W,
        OP_RNM("test", opcode_arg_t::MODRM_REF, opcode_arg_t::EXACT, nullptr, OP_EXACT_NAMESEL(regNamesWord[OP_MODRM_REG(bytes[1])])),
        OP_ACTION(cpu.aluOperation(cpu_alu_op_t::ALU_OP_TEST, reference, cpu_register_t(reference.modrmRegField), ref_width_t::WORD))
    },
    {   // MOV r/m8, r8
        { 0x88, P_MODRM16 }, OPW_B,
        OP_RNM("mov", opcode_arg_t::MODRM_REF, opcode_arg_t::EXACT, nullptr, OP_EXACT_NAMESEL(regNamesByte[OP_MODRM_REG(bytes[1])])),
        OP_ACTION(reference_t(reference).setValue(cpu, ref_width_t::BYTE, cpu.peekb(cpu_register_t(reference.modrmRegField))))
    },
    {   // MOV r/m16, r16
        { 0x89, P_MODRM16 }, OPW_W,
        OP_RNM("mov", opcode_arg_t::MODRM_REF, opcode_arg_t::EXACT, nullptr, OP_EXACT_NAMESEL(regNamesWord[OP_MODRM_REG(bytes[1])])),
        OP_ACTION(reference_t(reference).setValue(cpu, ref_width_t::WORD, cpu.peekw(cpu_register_t(reference.modrmRegField))))
    },
    {   // MOV r8, r/m8
        { 0x8a, P_MODRM16 }, OPW_B,
        OP_RNM("mov", opcode_arg_t::EXACT, opcode_arg_t::MODRM_REF, OP_EXACT_NAMESEL(regNamesByte[OP_MODRM_REG(bytes[1])])),
        OP_ACTION(cpu.pokeb(cpu_register_t(reference.modrmRegField), reference_t(reference).getValue(cpu, ref_width_t::BYTE)))
    },
    {   // MOV r16, r/m16
        { 0x8b, P_MODRM16 }, OPW_W,
        OP_RNM("mov", opcode_arg_t::EXACT, opcode_arg_t::MODRM_REF, OP_EXACT_NAMESEL(regNamesWord[OP_MODRM_REG(bytes[1])])),
        OP_ACTION(cpu.pokew(cpu_register_t(reference.modrmRegField), reference_t(reference).getValue(cpu, ref_width_t::WORD)))
    },
    {   // MOV r/m16, sreg
        { 0x8c, P_MODRM16 }, OPW_W,
        OP_RNM("mov", opcode_arg_t::MODRM_REF, opcode_arg_t::EXACT, nullptr, OP_EXACT_NAMESEL(sregNames[OP_MODRM_REG(bytes[1])])),
        OP_ACTION(reference_t(reference).setValue(cpu, ref_width_t::WORD, cpu.peekw(cpu_register_t(REG_SEGMENTR_OFFSET + reference.modrmRegField))))
    },
    {   // MOV sreg, r/m16
        { 0x8e, P_MODRM16 }, OPW_W,
        OP_RNM("mov", opcode_arg_t::EXACT, opcode_arg_t::MODRM_REF, OP_EXACT_NAMESEL(sregNames[OP_MODRM_REG(bytes[1])])),
        OP_ACTION(cpu.pokew(cpu_register_t(REG_SEGMENTR_OFFSET + reference.modrmRegField), reference_t(reference).getValue(cpu, ref_width_t::WORD)))
    },
    {   // LEA r16, m
        { 0x8d, P_MODRM16 }, OPW_W,
        OP_RNM("lea", opcode_arg_t::EXACT, opcode_arg_t::MODRM_REF, OP_EXACT_NAMESEL(regNamesWord[OP_MODRM_REG(bytes[1])])),
        OP_ACTION({
            reference_t xref = reference;
            if (xref.refType != ref_type_t::MEMORY) {
                printf("LEA on non-memory target");
            }
            cpu.pokew(cpu_register_t(reference.modrmRegField), xref.ref.linearAddress.offset);
        })
    },
    {   // NOP
        { 0x90 }, OPW_W,
        OP_RNM("nop"),
        OP_ACTION(),
    },
    {   // XCHG r16/32, AX
        { P_BYTE_VAL }, OPW_W,
        OP_RNM_EXACT2("xchg", regNamesWord[bytes[0] - 0x90], "eax"),
        OP_ACTION({
            auto tempv = cpu.peekw(cpu_register_t(bytes[0] - 0x90));
            cpu.pokew(cpu_register_t(bytes[0] - 0x90), cpu.eax);
            cpu.pokew(REG_EAX, tempv);
        }),
        OP_PRED(bytes[0] >= 0x91 && bytes[0] <= 0x97)
    },
    {   // PUSHF
        { 0x9C }, OPW_W, 
        OP_RNM("pushf"), 
        OP_ACTION(cpu.pushw(cpu.eflags))
    },
    {   // POPF
        { 0x9D }, OPW_W, 
        OP_RNM("popf"), 
        OP_ACTION(cpu.eflags = cpu_flags_t(cpu.popw()))
    },
    {   // MOV moffs16, AX, TODO moffs operand, BUG: segment override not applied
        { 0xA3, P_WORD_VAL }, OPW_W,
        OP_RNM("mov", opcode_arg_t::IMMEDIATE, opcode_arg_t::EXACT, nullptr, OP_EXACT_NAMESEL("ax")),
        OP_ACTION(cpu.pokew(cpu.ds, matched[0], cpu.eax))
    },
    {   // MOVSB
        { 0xA4 }, OPW_B,
        OP_RNM("movsb"),
        OP_ACTION({
            cpu.pokeb(cpu.es, cpu.edi, cpu.peekb(cpu.ds, cpu.esi));
            cpu.edi += (cpu.eflags & CPUFLAG_DIRECTION ? -1 : 1) * sizeof(uint8_t);
            cpu.esi += (cpu.eflags & CPUFLAG_DIRECTION ? -1 : 1) * sizeof(uint8_t);
        })
    },
    {   // STOSW
        { 0xAB }, OPW_W,
        OP_RNM("stosw"),
        OP_ACTION({
            cpu.pokew(cpu.es, cpu.edi, cpu.eax);
            cpu.edi += (cpu.eflags & CPUFLAG_DIRECTION ? -1 : 1) * sizeof(uint16_t);
        })
    },
    {   // MOV reg, imm8
        { P_BYTE_VAL, P_BYTE_VAL }, OPW_B,
        OP_RNM("mov", opcode_arg_t::EXACT, opcode_arg_t::IMMEDIATE, OP_EXACT_NAMESEL(regNamesByte[static_cast<size_t>(bytes[0] - 0xb0)]), nullptr, 0, 1),
        OP_ACTION(cpu.pokeb(cpu_register_t(matched[0] - 0xb0), matched[1])),
        OP_PRED(bytes[0] >= 0xb0 && bytes[0] <= 0xb7)
    },
    {   // MOV reg, imm16
        { P_BYTE_VAL, P_WORD_VAL }, OPW_W,
        OP_RNM("mov", opcode_arg_t::EXACT, opcode_arg_t::IMMEDIATE, OP_EXACT_NAMESEL(regNamesWord[static_cast<size_t>(bytes[0] - 0xb8)]), nullptr, 0, 1),
        OP_ACTION(cpu.pokew(cpu_register_t(matched[0] - 0xb8), matched[1])),
        OP_PRED(bytes[0] >= 0xb8 && bytes[0] <= 0xbf)
    },
    {   // ALU2 r/m8, imm8
        { 0xc0, P_MODRM16, P_BYTE_VAL }, OPW_B,
        OP_NAMESEL(alu2_names[static_cast<size_t>(reference.modrmRegField)], opcode_arg_t::MODRM_REF, opcode_arg_t::IMMEDIATE),
        OP_ACTION(cpu.aluOperation(OP_ALU_C2(reference.modrmRegField), reference, matched[0], ref_width_t::BYTE))
    },
    {
        { 0xC3 }, OPW_W,
        OP_RNM("ret"),
        OP_ACTION(cpu.eip = cpu.popw())
    },
    {
        { 0xCD, P_BYTE_VAL }, OPW_W,
        OP_RNM("int", opcode_arg_t::IMMEDIATE),
        OP_ACTION({
            cpu.nestedInterruptCalls++;
            cpu.pushw(cpu.eflags);
            cpu.pushw(cpu.cs);
            cpu.pushw(cpu.eip);
            auto farptr = cpu.peekd(0, matched[0] * sizeof(uint32_t));
            cpu.cs = farptr >> 16;
            cpu.eip = uint16_t(farptr);
        })
    },
    {
        { 0xCF }, OPW_W,
        OP_RNM("iret"),
        OP_ACTION({
            cpu.eip = cpu.popw();
            cpu.cs = cpu.popw();
            cpu.eflags = cpu_flags_t(cpu.popw());
        })
    },
    {   // ALU2 r/m8, 1
        { 0xd0, P_MODRM16 }, OPW_B,
        OP_NAMESEL(alu2_names[static_cast<size_t>(reference.modrmRegField)], opcode_arg_t::MODRM_REF, opcode_arg_t::EXACT, nullptr, OP_EXACT_NAMESEL("1")),
        OP_ACTION(cpu.aluOperation(OP_ALU_C2(reference.modrmRegField), reference, 1, ref_width_t::BYTE))
    },
    {   // ALU2 r/m16, 1
        { 0xd1, P_MODRM16 }, OPW_W,
        OP_NAMESEL(alu2_names[static_cast<size_t>(reference.modrmRegField)], opcode_arg_t::MODRM_REF, opcode_arg_t::EXACT, nullptr, OP_EXACT_NAMESEL("1")),
        OP_ACTION(cpu.aluOperation(OP_ALU_C2(reference.modrmRegField), reference, 1, ref_width_t::WORD))
    },
    {   // ALU2 r/m16, CL
        { 0xd3, P_MODRM16 }, OPW_W,
        OP_NAMESEL(alu2_names[static_cast<size_t>(reference.modrmRegField)], opcode_arg_t::MODRM_REF, opcode_arg_t::EXACT, nullptr, OP_EXACT_NAMESEL("cl")),
        OP_ACTION(cpu.aluOperation(OP_ALU_C2(reference.modrmRegField), reference, uint8_t(cpu.ecx), ref_width_t::BYTE))
    },
    {   // CALL rel16
        { 0xe8, P_WORD_VAL }, OPW_W,
        OP_RNM("call", opcode_arg_t::RELATIVE),
        OP_ACTION({
            cpu.pushw(cpu.eip);
            cpu.eip += matched[0];
        })
    },
    {   // JMP rel16
        { 0xe9, P_WORD_VAL }, OPW_W,
        OP_RNM("jmp", opcode_arg_t::RELATIVE),
        OP_ACTION(cpu.eip += matched[0])
    },
    {   // JMP rel8
        { 0xeb, P_BYTE_VAL }, OPW_B,
        OP_RNM("jmp", opcode_arg_t::RELATIVE),
        OP_ACTION(cpu.eip += matched[0])
    },
    {   // IN DX, AL
        { 0xec }, OPW_W,
        OP_RNM_EXACT2("in", "dx", "al"),
        OP_ACTION() // NO ACTION
    },
    {   // IN DX, AX
        { 0xed }, OPW_W,
        OP_RNM_EXACT2("in", "dx", "ax"),
        OP_ACTION() // NO ACTION
    },
    {   // OUT DX, AX
        { 0xee }, OPW_W,
        OP_RNM_EXACT2("out", "dx", "ax"),
        OP_ACTION() // NO ACTION
    },
    {   // OUT DX, AL
        { 0xef }, OPW_B,
        OP_RNM_EXACT2("out", "dx", "al"),
        OP_ACTION() // NO ACTION
    },
    {   // MUL, IMUL, DIV, IDIV r/m16
        { 0xF7, P_MODRM16 }, OPW_W,
        OP_NAMESEL(alu3_names[static_cast<size_t>(reference.modrmRegField)], opcode_arg_t::MODRM_REF),
        OP_ACTION(cpu.aluOperation(OP_ALU_C3(reference.modrmRegField), reference, 0, ref_width_t::WORD)),
        OP_PRED(OP_MODRM_REG(bytes[1]) >= 4)
    },
    {   // CLD
        { 0xFC }, OPW_B,
        OP_RNM("cld"),
        OP_ACTION(cpu.eflags = cpu_flags_t(cpu.eflags & ~CPUFLAG_DIRECTION))
    },
    {   // STD
        { 0xFD }, OPW_B,
        OP_RNM("cld"),
        OP_ACTION(cpu.eflags = cpu_flags_t(cpu.eflags | CPUFLAG_DIRECTION))
    },
    {   // CALL r/m16
        { 0xFF, P_MODRM16 }, OPW_W,
        OP_RNM("call", opcode_arg_t::MODRM_REF),
        OP_ACTION({ 
            cpu.pushw(cpu.eip);
            cpu.eip = reference_t(reference).getValue(cpu, ref_width_t::WORD);
        }),
        OP_PRED(OP_MODRM_REG(bytes[1]) == 2)
    },
    {   // CALL far r/m16
        { 0xFF, P_MODRM16 }, OPW_W,
        OP_RNM("call far", opcode_arg_t::MODRM_REF),
        OP_ACTION({ 
            printf("CALL FAR NOT SUPPORTED");
            while (1); // not supported
        }),
        OP_PRED(OP_MODRM_REG(bytes[1]) == 3)
    },
    {   // JMP r/m16
        { 0xFF, P_MODRM16 }, OPW_W,
        OP_RNM("jmp", opcode_arg_t::MODRM_REF),
        OP_ACTION(cpu.eip = reference_t(reference).getValue(cpu, ref_width_t::WORD)),
        OP_PRED(OP_MODRM_REG(bytes[1]) == 4)
    },
    {   // JMP far r/m16
        { 0xFF, P_MODRM16 }, OPW_W,
        OP_RNM("jmp far", opcode_arg_t::MODRM_REF),
        OP_ACTION({ 
            printf("JMP FAR NOT SUPPORTED");
            while (1); // not supported
        }),
        OP_PRED(OP_MODRM_REG(bytes[1]) == 5)
    },
    {   // PUSH r/m16
        { 0xFF, P_MODRM16 }, OPW_W,
        OP_RNM("jmp", opcode_arg_t::MODRM_REF),
        OP_ACTION(cpu.pushw(reference_t(reference).getValue(cpu, ref_width_t::WORD))),
        OP_PRED(OP_MODRM_REG(bytes[1]) == 6)
    },
    {{P_LASTPATTERN}, OPW_B, OP_RNM("LASTPATTERN"), OP_ACTION()}
};