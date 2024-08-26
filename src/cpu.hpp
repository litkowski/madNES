// Class to contain core CPU functionality
class CPU {

    // All possible addressing modes for each 6502 opcode
    // KEY: IMPL - Implicit
    //      ACC - Accumulator
    //      IMM - Immediate
    //      ABS - Absolute
    //      XABS - X-Indexed Absolute
    //      YABS - Y-Indexed Absolute
    //      ABSI - Absolute Indirect
    //      ZP - Zero Page
    //      XZP - X-Indexed Zero Page
    //      YZP - Y-Indexed Zero Page
    //      XZPI - X-Indexed Zero Page Indirect
    //      ZPIY - Zero Page Indirect Y-Indexed
    //      REL - Relative
    enum address_mode {
        IMPL, ACC, IMM, ABS, XABS, YABS, ABSI, ZP, XZP, YZP, XZPI, ZPIY, REL
    };

    // All instruction functions, each one will queue a number of single-cycle actions
    // NOTE: This will be expanded to include illegal opcodes in the future
    void ADC (address_mode);
    void AND (address_mode);
    void ASL (address_mode);
    void BCC (address_mode);
    void BCS (address_mode);
    void BEQ (address_mode);
    void BIT (address_mode);
    void BMI (address_mode);
    void BNE (address_mode);
    void BPL (address_mode);
    void BRK (address_mode);
    void BVC (address_mode);
    void BVS (address_mode);
    void CLC (address_mode);
    void CLD (address_mode);
    void CLI (address_mode);
    void CLV (address_mode);
    void CMP (address_mode);
    void CPX (address_mode);
    void CPY (address_mode);
    void DEC (address_mode);
    void DEX (address_mode);
    void DEY (address_mode);
    void EOR (address_mode);
    void INC (address_mode);
    void INX (address_mode);
    void INY (address_mode);
    void JMP (address_mode);
    void LDA (address_mode);
    void LDX (address_mode);
    void LDY (address_mode);
    void LSR (address_mode);
    void NOP (address_mode);
    void ORA (address_mode);
    void PHA (address_mode);
    void PHP (address_mode);
    void PLA (address_mode);
    void PLP (address_mode);
    void ROL (address_mode);
    void ROR (address_mode);
    void RTI (address_mode);
    void RTS (address_mode);
    void SBC (address_mode);
    void SEC (address_mode);
    void SED (address_mode);
    void SEI (address_mode);
    void STA (address_mode);
    void STX (address_mode);
    void STY (address_mode);
    void TAX (address_mode);
    void TAY (address_mode);
    void TSX (address_mode);
    void TXA (address_mode);
    void TXS (address_mode);
    void TYA (address_mode);

    // Pointer to the combined game and computer memory
    int* memory;

    // 6502 data; accumulator, X and Y registers, stack pointer status
    // register, data bus, opcode register, program counter, and address bus
    char acc, x, y, sp, status, dbus, opcode;
    short pc, abus;


    char fetch (short address);

    // Single cycle actions that the CPU can take
    void write ();

    void fetch_opcode ();
    void fetch_data ();
    void fetch_x ();
    void fetch_y ();
    void fetch_pch ();
    void fetch_pcl ();


    void fetch_acc ();

    void set_addh ();
    void set_addl ();

    void adc_acc ();
    void and_acc ();
    void asl_acc ();
    void asl_mem ();
    void sub_acc ();

    void fetch_zp ();
    void nop ();
    void push_pch ();
    void push_pcl ();

    void decode ();



    // Queue to store cycle events in for the current instruction, with a pointer
    // to the next event to be executed
    void (*event_queue[2][8]) ();
    char cur_event;

    public:
        CPU(int* rom);
        cycle();
};
