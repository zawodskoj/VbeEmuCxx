#include "cpu.h"

uint32_t reference_t::getValue(cpu_t &cpu, ref_width_t width) {
    uint32_t val =
        refType == ref_type_t::REGISTER
        ? width == ref_width_t::BYTE ? cpu.peekb(ref.referencedRegister) : cpu.peekd(ref.referencedRegister)
        : refType == ref_type_t::MEMORY
        ? cpu.peekd(ref.linearAddress.segment, ref.linearAddress.offset)
        : ref.immediate;
    return width == ref_width_t::BYTE ? signExtendByte(val) : width == ref_width_t::WORD ? signExtendWord(val) : val;
}

void reference_t::setValue(cpu_t &cpu, ref_width_t width, uint32_t value) {
    if (refType == ref_type_t::IMMEDIATE) {
        return; // TODO: debug output/crash
    }
    switch (width) {
    case ref_width_t::BYTE:
        if (refType == ref_type_t::REGISTER)
            cpu.pokeb(ref.referencedRegister, value);
        else
            cpu.pokeb(ref.linearAddress.segment, ref.linearAddress.offset, value);
        break;
    case ref_width_t::WORD:
        if (refType == ref_type_t::REGISTER)
            cpu.pokew(ref.referencedRegister, value);
        else
            cpu.pokew(ref.linearAddress.segment, ref.linearAddress.offset, value);
        break;
    case ref_width_t::DWRD:
        if (refType == ref_type_t::REGISTER)
            cpu.poked(ref.referencedRegister, value);
        else
            cpu.poked(ref.linearAddress.segment, ref.linearAddress.offset, value);
        break;
    }
}