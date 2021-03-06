#include "cpu.h"
#include <cstring>

using namespace System::IO;

int main() {
    cpu_t *cpu = new cpu_t();

    FileStream file("C:\\Users\\zawod\\bochs\\conventional.bin", FileMode::Open, FileAccess::Read, FileShare::Read);
    auto mem = gcnew array<uint8_t>(CPU_EMULATED_MEMORY_SIZE);
    file.Read(mem, 0, mem->Length);
    
    pin_ptr<uint8_t> memptr = &mem[0];
    memcpy(cpu->memory, memptr, mem->Length);

    cpu->eax = 3;
    cpu->ss = cpu->ds = 0x7000;
    cpu->esp = 0xffe8;
    cpu->cs = 0xc000;
    cpu->eip = 0x0151;
    cpu->runToIret();

    return 0;
}