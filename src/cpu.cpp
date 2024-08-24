
// Class to contain core CPU functionality
class CPU {

    // Pointer to the combined game and computer memory
    int* memory;

    // 6502 registers; accumulator, X and Y registers, stack pointer
    // status flags, and program counter
    char acc, x, y, sp, status;
    short pc;

    void load_instr(char opcode);
    public:
        CPU(int* rom);
        cycle();
};

// CPU constructor
CPU::CPU(int* rom){
    memory = rom;
}

// Cycle the CPU once.
CPU::cycle(){

}

// Load an instruction from the given opcode.
// If it is an illegal opcode, return -1
CPU::load_instr(char opcode){

    // NOTE: The 6502 contains multiple addressing modes for
    // many instructions, and as such multiple functions can be
    // called depending on the addressing mode. All potential opcodes
    // are organized alphabetically by their instruction names.

    // KEY: IM - Immediate
    //      ABS - Absolute
    //      XABS - X-Indexed Absolute
    //      YABS - Y-Indexed Absolute
    //      ZP - Zero Page
    //      XZP - X-Indexed Zero Page
    //      XZPI - X-Indexed Zero Page Indirect
    //
    //      N/A - Implicit
    switch(opcode){
        case 0x69 : ADC_IM
        case 0x6D : ADC_ABS
        case 0x7D : ADC_XABS
        case 0x79 : ADC_YABS
        case 0x65 : ADC_ZP
        case 0x75 : ADC_XZP
        case 0x65 : ADC_XZPI
        case
    }

}

