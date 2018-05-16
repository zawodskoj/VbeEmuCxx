#pragma once

#include <cstdint>
#include <cstddef>

static inline uint32_t signExtendByte(uint8_t val) { return static_cast<uint32_t>(static_cast<int32_t>(static_cast<int8_t>(val))); }
static inline uint32_t signExtendWord(uint16_t val) { return static_cast<uint32_t>(static_cast<int32_t>(static_cast<int16_t>(val))); }
static inline uint64_t signExtendDword(uint32_t val) { return static_cast<uint64_t>(static_cast<int64_t>(static_cast<int32_t>(val))); }

const size_t CPU_EMULATED_MEMORY_SIZE = 0x100000;
const size_t MAX_OPCODE_PATTERNS = 0x100;
const size_t MAX_INSTRUCTION_SIZE = 16; // for 16/32 -> Prefixes(4b) + Opcode(1/2b) + Modrm(1b) + Sib(1b) + Disp(max 4b) + Imm(max 4b)
const size_t MAX_OPCODE_PATTERN_SIZE = MAX_INSTRUCTION_SIZE;
const size_t MAX_MODRM_DESC_LENGTH = 32; // xS:[ExX + EyX * 8 + 0x89ABCDEF]

const int P_LASTPATTERN = 0x100;
const int P_ZERO = 0x101;
const int P_MODRM16 = 0x102;
const int P_MODRM32 = 0x103;

const int P_BYTE_VAL = -1;
const int P_WORD_VAL = -2;
const int P_DWRD_VAL = -4;

enum cpu_alu_op_t : uint8_t {
    // class-1 ALU operations (opcodes 0x80, 0x81 etc)
    ALU_OP_CLASS1_OFS = 0,
    ALU_OP_ADD = 0,
    ALU_OP_OR = 1,
    ALU_OP_ADC = 2,
    ALU_OP_SBB = 3,
    ALU_OP_AND = 4,
    ALU_OP_SUB = 5,
    ALU_OP_XOR = 6,
    ALU_OP_CMP = 7,

    // class-2 ALU operations (opcodes 0xc0, 0xc1 etc)
    ALU_OP_CLASS2_OFS = 8,
    ALU_OP_ROL = ALU_OP_CLASS2_OFS + 0,
    ALU_OP_ROR = ALU_OP_CLASS2_OFS + 1,
    ALU_OP_RCL = ALU_OP_CLASS2_OFS + 2,
    ALU_OP_RCR = ALU_OP_CLASS2_OFS + 3,
    ALU_OP_SHL_SAL = ALU_OP_CLASS2_OFS + 4,
    ALU_OP_SHR = ALU_OP_CLASS2_OFS + 5,
    ALU_OP_SAL_SHL = ALU_OP_CLASS2_OFS + 6,
    ALU_OP_SAR = ALU_OP_CLASS2_OFS + 7,

    // class-3 ALU operations (custom)
    ALU_OP_CLASS3_OFS = 16,
    ALU_OP_TEST = ALU_OP_CLASS3_OFS + 0
};

enum cpu_flags_t : uint32_t {
    CPUFLAG_NONE = 0,
    CPUFLAG_CARRY = 0x0001,
    CPUFLAG_PARITY = 0x0004,
    CPUFLAG_ADJUST = 0x0010,
    CPUFLAG_ZERO = 0x0040,
    CPUFLAG_SIGN = 0x0080,
    CPUFLAG_TRAP = 0x0100,
    CPUFLAG_INTERRUPT = 0x0200,
    CPUFLAG_DIRECTION = 0x0400,
    CPUFLAG_OVERFLOW = 0x0800
};

enum cpu_register_t : uint8_t { 
    // 16/32-bit registers (also 8-bit lower byte)
    REG_EAX = 0, REG_ECX = 1, REG_EDX = 2, REG_EBX = 3,
    REG_ESP = 4, REG_EBP = 5, REG_ESI = 6, REG_EDI = 7,
    
    // 8-bit higher byte
    REG_AH = 4, REG_CH = 5, REG_DH = 6, REG_BH = 7,

    // segment registers
    REG_SEGMENTR_OFFSET = 8,
    REG_ES = 0, REG_CS = 1, REG_SS = 2, REG_DS = 3, REG_FS = 4, REG_GS = 5,
    // fixed segment registers (replaced for generic use)
    REG_ES_G = 8, REG_CS_G = 9, REG_SS_G = 10, REG_DS_G = 11, REG_FS_G = 12, REG_GS_G = 13,
    
    REG_NOOVERRIDE = 0xff
};

static inline cpu_register_t generalizeSegmentRegister(cpu_register_t reg) { return cpu_register_t(reg + REG_SEGMENTR_OFFSET); }
static inline cpu_register_t ungeneralizeSegmentRegister(cpu_register_t reg) { return cpu_register_t(reg - REG_SEGMENTR_OFFSET); }

struct modrm_t {
    uint8_t rm: 3;
    cpu_register_t reg : 3;
    uint8_t mod : 2;
};

struct cpu_linear_address_t {
    uint16_t segment;
    uint16_t offset;
};

union united_reference_t {
    cpu_linear_address_t linearAddress;
    uint32_t immediate;
    cpu_register_t referencedRegister;
};

struct parsed_reference_t {
    bool exists;
    char desc[MAX_MODRM_DESC_LENGTH];
    bool isRegisterReference;
    united_reference_t ref;
    uint32_t modrmRegField;
};

enum class ref_type_t { MEMORY, REGISTER, IMMEDIATE };

enum class ref_width_t {
    BYTE = 1,
    WORD = 2,
    DWRD = 4
};

class cpu_t;

struct reference_t {
    ref_type_t refType;
    united_reference_t ref;

    uint32_t getValue(cpu_t &cpu, ref_width_t width);

    void setValue(cpu_t &cpu, ref_width_t width, uint32_t value);

    reference_t(const parsed_reference_t &parsedRef)
        : refType(parsedRef.isRegisterReference ? ref_type_t::REGISTER : ref_type_t::MEMORY)
        , ref(parsedRef.ref) {}

    united_reference_t createImmediateRef(uint32_t imm) {
        united_reference_t uref;
        uref.immediate = imm;
        return uref;
    }

    united_reference_t createRegisterRef(cpu_register_t reg) {
        united_reference_t uref;
        uref.referencedRegister = reg;
        return uref;
    }

    reference_t(uint32_t immediate) : refType(ref_type_t::IMMEDIATE), ref(createImmediateRef(immediate)) {}
    reference_t(cpu_register_t reg) : refType(ref_type_t::REGISTER), ref(createRegisterRef(reg)) {}
};

struct opcode_name_t;

using opcode_pattern_action_t = void(*)(cpu_t &cpu, parsed_reference_t &reference, uint32_t matched[MAX_OPCODE_PATTERN_SIZE], uint8_t bytes[]);
using opcode_name_selector_t = opcode_name_t(*)(parsed_reference_t &reference);
using opcode_predicate_t = bool(*)(uint8_t bytes[]);
using opcode_exact_name_sel_t = const char*(*)(uint8_t bytes[]);

enum class opcode_arg_t {
    NONE,
    IMMEDIATE,
    RELATIVE,
    MODRM_REF,
    EXACT
};

struct opcode_name_t {
    const char *name;
    opcode_arg_t lhs, rhs;
    opcode_exact_name_sel_t exactLhs, exactRhs;
    size_t lhsImmAt, rhsImmAt;
};

struct opcode_pattern_t {
    int pattern[MAX_OPCODE_PATTERN_SIZE];
    ref_width_t defWidth;
    opcode_name_selector_t nameSel;
    opcode_pattern_action_t action;
    opcode_predicate_t predicate;
};

class cpu_t {
private:
    static opcode_pattern_t m_opcodePatterns[MAX_OPCODE_PATTERNS];
    static uint32_t segmentedToLinear(uint16_t segment, uint16_t offset) { return segment * 0x10 + offset; }
public:
    uint32_t eax; uint32_t ecx; uint32_t edx; uint32_t ebx;
    uint32_t esp; uint32_t ebp; uint32_t esi; uint32_t edi;

    uint16_t es; uint16_t cs; uint16_t ss; uint16_t ds;
    uint16_t fs; uint16_t gs;

    uint32_t eip; cpu_flags_t eflags;
private:
    void *m_regPtrs[14] = {&eax, &ecx, &edx, &ebx, &esp, &ebp, &esi, &edi, &es, &cs, &ss, &ds, &fs, &gs};
    parsed_reference_t parseRef16(modrm_t modrm, uint8_t bytes[], size_t &position, ref_width_t refw, cpu_register_t segOverride);
    //parsed_reference_t parseRef32(modrm_t modrm, uint8_t remainingBytes[], size_t &position);
public:

    uint8_t memory[CPU_EMULATED_MEMORY_SIZE];

    uint8_t peekb(uint16_t segment, uint16_t offset) const { return memory[segmentedToLinear(segment, offset)]; }
    uint16_t peekw(uint16_t segment, uint16_t offset) const { return *reinterpret_cast<const uint16_t*>(&memory[segmentedToLinear(segment, offset)]); }
    uint32_t peekd(uint16_t segment, uint16_t offset) const { return *reinterpret_cast<const uint32_t*>(&memory[segmentedToLinear(segment, offset)]); }

    uint8_t peekb(cpu_register_t reg) const {
        if (reg >= 8) // not common register
            return 0; // todo fail (not supported)
        if (reg >= 4) // higher-half
            return reinterpret_cast<uint8_t*>(m_regPtrs[reg - 4])[1];
        else // lower half
            return *reinterpret_cast<uint8_t*>(m_regPtrs[reg]);
    }
    uint16_t peekw(cpu_register_t reg) const { return *reinterpret_cast<uint16_t*>(m_regPtrs[reg]); }
    uint32_t peekd(cpu_register_t reg) const {
        if (reg >= 8) // not common register
            return 0; // todo fail (not supported)
        return *reinterpret_cast<uint32_t*>(m_regPtrs[reg]);
    }

    void pokeb(uint16_t segment, uint16_t offset, uint8_t value) { memory[segmentedToLinear(segment, offset)] = value; }
    void pokew(uint16_t segment, uint16_t offset, uint16_t value) { *reinterpret_cast<uint16_t*>(&memory[segmentedToLinear(segment, offset)]) = value; }
    void poked(uint16_t segment, uint16_t offset, uint32_t value) { *reinterpret_cast<uint32_t*>(&memory[segmentedToLinear(segment, offset)]) = value; }

    void pokeb(cpu_register_t reg, uint8_t value) {
        if (reg >= 8) // not common register
            return; // todo fail (not supported)
        if (reg >= 4) // higher-half
            reinterpret_cast<uint8_t*>(m_regPtrs[reg - 4])[1] = value;
        else // lower half
            *reinterpret_cast<uint8_t*>(m_regPtrs[reg]) = value;
    }
    void pokew(cpu_register_t reg, uint16_t value) { *reinterpret_cast<uint16_t*>(m_regPtrs[reg]) = value; }
    void poked(cpu_register_t reg, uint32_t value) {
        if (reg >= 8) // not common register
            return; // todo fail (not supported)
        *reinterpret_cast<uint32_t*>(m_regPtrs[reg]) = value;
    }

    uint16_t popw() { return peekw(ss, (esp += sizeof(uint16_t)) - sizeof(uint16_t)); }
    uint32_t popd() { return peekd(ss, (esp += sizeof(uint32_t)) - sizeof(uint32_t)); }

    void pushw(uint16_t value) { pokew(ss, esp -= sizeof(uint16_t), value); }
    void pushd(uint32_t value) { poked(ss, esp -= sizeof(uint32_t), value); }

    void executeSingleInstruction();
    void aluOperation(cpu_alu_op_t aluOp, reference_t lhsRef, reference_t rhsRef, ref_width_t refWidth);

    void runToIret() {
        while (peekb(cs, eip) != 0xcf) executeSingleInstruction();
    }
};