// This file contains the all CPU functionality for the CPU class defined in "cpu.hpp"
#include <cstring>


// CPU constructor
CPU::CPU (int* rom) {
    memory = rom;
    status = 0b00100000;
    acc = x = y = dbus = 0;
    abus = 0;
    ladd_temp = 0;
    sp = 0x200;
    pc = 0x000;
}

// Cycle the CPU once. Calls both potential operations on
CPU::cycle () {

    // Execute the atomic event or events in the front of the event queue, remove it from the queue
    for (auto atomic_event : events[0]) {
        atomic_event;
    }
    events.pop();
}

// Fetch the byte at the given address and return it
CPU::fetch (short address) {
    return memory[address];
}

// Fetch opcode into the opcode register from PC's address
CPU::fetch_opcode () {
    opcode = dbus = fetch(pc);
    pc++;
}

// Fetch data into the data bus from the address bus
CPU::fetch_data_abus () {
    dbus = fetch(abus);
}

// Fetch data into the data bus from PC
CPU::fetch_data_pc () {
    dbus = fetch(pc);
}

// Fetch data into the data bus from the low byte of the address bus
CPU::fetch_zp () {
    zp_add = abus &= 0x0F;
    dbus = fetch(zp_add);
}

// Fetch data into the X register from the address bus
CPU::fetch_x () {
    x = dbus = fetch(abus);
}

// Fetch data into the Y register from the address bus
CPU::fetch_y () {
    y = dbus = fetch(abus);
}

// Fetch data into the low byte of the address bus from the parameter.
// If the parameter is pc, increment pc.
CPU::fetch_ladd (short address) {
    dbus = fetch(address);
    abus = (abus &= 0xF0) | (dbus);

    if (address == pc) {
        pc++;
    }
}

// Fetch data into the high byte of the address bus from the parameter.
// If the parameter is pc, increment pc.
CPU::fetch_hadd (short address) {
    dbus = fetch(address);
    abus = (abus &= 0x0F) | (dbus << 8);

    if (address == pc) {
        pc++
    }
}

// If the addition to the low byte of address has carried, add
// one to the high byte. Else skip the next cycle
CPU::fix_add () {
    if () {

    }
}

// Fetch data at abus into the accumulator
CPU::fetch_acc () {
    acc = dbus = fetch(abus);
}

// Set abus to the parameter
CPU::set_abus (short address) {
    abus = address;
}

// Add the data from dbus into the accumulator with carry
CPU::adc_acc () {

    int old_acc = acc;
    acc += dbus;

    if (acc < old_acc) {
        status |= 0b00000001;
    } else {
        status &= 0b11111110;
    }

    if (acc < 0) {
        status |= 0b10000000;
    } else {
        status &= 0b01111111;

        if (acc == 0) {
            status |= 0b00000010;
        } else {
            status &= 0b11111101;
        }

    }

}


// Add the data to the accumulator, possibly set carry flag
CPU::ADC (address_mode mode) {

    // Add different single-cycle actions to the event queue depending on
    // the addressing mode
    switch (mode) {
        case IMM:
            events.insert({{&fetch_data_pc, &adc_acc}, {&fetch_opcode, &decode}});
            break;

        case ABS:
            events.insert({{&fetch_ladd_pc}, {&fetch_hadd_pc},
                            {&fetch_data_abus, &adc_acc}, {&fetch_opcode, &decode}});
            break;

        case XABS:
            events.insert({{&fetch_ladd_pc}, {&fetch_hadd_pc, &add_x_ladd},
                {&fetch_data_abus, &fix_add_skip}, {&fetch_data_abus, &adc_acc},
                {&fetch_opcode, &decode}});
            break;

        case YABS:
            events.insert({{&fetch_ladd_pc}, {&fetch_hadd_pc, &add_y_ladd},
                {&fetch_data_abus, &fix_add_skip},
                {&fetch_data_abus, &adc_acc}, {&fetch_opcode, &decode}})
            break;

        case ZP:
            events.insert({{&fetch_ladd_pc}, {&fetch_data_zp, &adc_acc},
                {&fetch_opcode, &decode}});
            break;

        case XZP:
            events.insert({{&fetch_ladd_pc}, {&fetch_ladd_zp, &add_x_ladd},
                {&fetch_zp_abus, &adc_acc}, {&fetch_opcode, &decode}};
            break;

        case XZPI:
            events.insert({{&fetch_ladd_pc}, {&fetch_ladd_zp, &add_x_ladd},
                {&fetch_ladd_zp, &inc_ladd}, {&fetch_hadd_zp}, {&fetch_data_abus, &adc_acc},
                {&fetch_opcode, &decode}};
            break;

        case ZPIY:
            events.insert({{&fetch_ladd_pc}, {&fetch_ladd_zp, &inc_ladd},
                {&fetch_hadd_zp, &add_y_ladd}, {&fetch_data_abus, &fix_add_skip},
                {&fetch_data_abus, &adc_acc}, {&fetch_opcode, &decode}};
            break;
    }
}

// Perform a bitwise AND on the data and the accumulator
CPU::AND (address_mode mode) {

    // Add different single-cycle actions to the event queue depending on
    // the addressing mode
    switch (mode) {
        case IMM:
            events.insert({{&fetch_data_pc, &and_acc}, {&fetch_opcode, &decode}});
            break;

        case ABS:
            events.insert({{&fetch_ladd_pc}, {&fetch_hadd_pc},
                {&fetch_data_abus, &and_acc}, {&fetch_opcode, &decode}});
            break;

        case XABS:
            events.insert({{&fetch_ladd_pc}, {&fetch_hadd_pc, &add_x_ladd},
                {&fetch_data_abus, &fix_add_skip}, {&fetch_data_abus, &and_acc},
                {&fetch_opcode, &decode}});
            break;

        case YABS:
            events.insert({{&fetch_ladd_pc}, {&fetch_hadd_pc, &add_y_ladd},
                {&fetch_data_abus, &fix_add_skip}, {&fetch_data_abus, &and_acc},
                {&fetch_opcode, &decode}};
            break;

        case ZP:
            events.insert({{&fetch_ladd_pc}, {&fetch_data_zp, &and_acc},
                {&fetch_opcode, &decode}});
            break;

        case XZP:
            events.insert({{&fetch_ladd_pc}, {&fetch_ladd_zp, &add_x_ladd},
                {&fetch_zp_abus, &and_acc}, {&fetch_opcode, &decode}});
            break;

        case XZPI:
            events.insert({{&fetch_ladd_pc}, {&fetch_ladd_zp, &add_x_ladd},
                {&fetch_ladd_zp, &inc_ladd}, {&fetch_hadd_zp}, {&fetch_data_abus, &and_acc},
                {&fetch_opcode, &decode}};
            break;

        case ZPIY:
            events.insert({{&fetch_ladd_pc}, {&fetch_ladd_zp, &inc_ladd},
                {&fetch_hadd_zp, &add_y_ladd}, {&fetch_data_abus, &fix_add_skip},
                {&fetch_data_abus, &and_acc}, {&fetch_opcode, &decode}};
            break;
    }

}

// Shifts the accumulator (or data at the specified location)
// left one bit, resetting bit 0 and storing bit 7 in the carry flag
CPU::ASL(address_mode mode){

    // Add different single-cycle actions to the event queue depending on
    // the addressing mode
    switch(mode){
        case ACC:
            events.insert(events.end(), {{&asl_acc}, {&fetch_opcode, &decode}});
            break;

        case ABS:
            events.insert(events.end(), {{&fetch_ladd_pc}, {&fetch_hadd_pc},
                {&fetch_data_abus}, {&write_data_abus, &asl_data}, {&write_data_abus},
                {&fetch_opcode, &decode}});
            break;

        case XABS:
            events.insert(events.end(), {{&fetch_ladd_pc}, {&fetch_hadd_pc, &add_xladd},
                {&fetch_data_abus, &fix_add_skip}, {&fetch_data_abus},
                {&write_data_abus, &asl_data}, {&write_data_abus},
                {&fetch_opcode, &decode}});
            break;

        case ZP:
            events.insert(events.end(), {{&fetch_ladd_pc}, {&fetch_data_zp},
                {&write_data_zp, &asl_data}, {&write_data_zp}, {&fetch_opcode, &decode}});
            break;

        case XIZP:
            events.insert(events.end(), {{&fetch_ladd_pc}, {&fetch_ladd_zp, &add_x_ladd},
                {fetch_data_zp}, {&write_data_zp, &asl_data}, {&write_data_zp},
                {&fetch_opcode, &decode}};
            break;

    }
}


// Branch if the carry bit is set to 0
CPU::BCC(address_mode mode){
    events.insert(events.end(), {{&fetch_data_pc}, {&eval_branch(CC), &add_dbus_pcl},
        {&fetch_opcode, &fix_pc}, {&fetch_opcode, &decode}});
}

// Branch if the carry bit is set to 1
CPU::BCS(address_mode mode){
    void (*events[][]) () =
        {{&fetch_data_pc}, {&eval_branch(CS), &add_dbus_pcl}, {&fetch_opcode, &fix_pc}, {&fetch_opcode, &decode};

    std::memcpy(event_queue, events, 8 * sizeof(void*));
}

// Branch if the Z flag is set to 1 (i.e. the previous result is zero)
CPU::BEQ(address_mode mode){
    void (*events[][]) () =
        {{&fetch_data_pc}, {&eval_branch(EQ), &add_dbus_pcl}, {&fetch_opcode, &fix_pc}, {&fetch_opcode, &decode};

    std::memcpy(event_queue, events, 8 * sizeof(void*));
}

// Perform a bitwise and operation on the specified location and the
// accumulator, sets status flags accordingly. Sets the N flag to the
CPU::BIT(address_mode mode){

    switch (mode ) {

        case ABS:
            void (*events[][]) () =
                {{&fetch_ladd_pc, NULL}, {&fetch_hadd_pc, NULL},
                {&fetch_data_abus, &test_dbus_acc}, {&fetch_opcode, &decode}};

            std::memcpy(event_queue, events, 8 * sizeof(void*));
            break;

        case ZP:
            void (*events[][]) () =
                {{&fetch_ladd_pc, NULL}, {&fetch_data_zp, &test_dbus_acc}, {&fetch_opcode, &decode}};

            std::memcpy(event_queue, events, 6 * sizeof(void*));
            break;


    }

}

// Branch if the N bit is set to 1, (i.e the previous result is negative)
CPU::BMI(address_mode mode){
    void (*events[][]) () =
        {{&fetch_data_pc, NULL}, {&eval_branch(MI), &add_dbus_pcl}, {&fetch_opcode, &fix_pc}, {&fetch_opcode, &decode};

    std::memcpy(event_queue, events, 8 * sizeof(void*));
}

// Branch if the Z flag is set to 1 (i.e. the previous result is nonzero)
CPU::BNE(address_mode mode){
    void (*events[][]) () =
        {{&fetch_data_pc, NULL}, {&eval_branch(NE), &add_dbus_pcl}, {&fetch_opcode, &fix_pc}, {&fetch_opcode, &decode};

    std::memcpy(event_queue, events, 8 * sizeof(void*));
}

// Branch if the N bit is set to 0, (i.e. the previous result is positive)
CPU::BPL(address_mode mode){
    void (*events[][]) () =
        {{&fetch_data_pc, NULL}, {&eval_branch(CC), &add_dbus_pcl}, {&fetch_opcode, &fix_pc}, {&fetch_opcode, &decode};

    std::memcpy(event_queue, events, 8 * sizeof(void*));
}

// Issue an interrupt, store the next PC address on the stack.
CPU::BRK(address_mode mode){
    void(*events[][]) () =
        {{&fetch_from_pc, NULL}, {&push_pch, NULL}, {&push_pcl, NULL},
        {&push_status_b, &set_abus(0xFFFE)}, {&fetch_pcl_abus, NULL}, {&fetch_pch, NULL},
        {&fetch_opcode, NULL};

    std::memcpy(event_queue, events, 14 * sizeof(void*));
}

// Branch if the V flag is not set (if there is no overflow)
CPU::BVC(address_mode mode){
    void (*events[][]) () =
        {{&fetch_data_pc, NULL}, {&eval_branch(CC), &add_dbus_pcl}, {&fetch_opcode, &fix_pc}, {&fetch_opcode, &decode};

    std::memcpy(event_queue, events, 8 * sizeof(void*));
}

// Branch if the V flag is set (if there is an overflow)
CPU::BVS(address_mode mode){
    void (*events[][]) () =
        {{&fetch_data_pc, NULL}, {&eval_branch(CC), &add_dbus_pcl}, {&fetch_opcode, &fix_pc}, {&fetch_opcode, &decode};

    std::memcpy(event_queue, events, 8 * sizeof(void*));
}


// Increment the X register
CPU::INX(address_mode mode){
    void (*events[][]) () =
            {{&fetch_opcode, &inx}, {&decode, NULL}};

    std::memcpy(event_queue, events, 4);
    break;
}

// Set the carry flag to 0
CPU::CLC(address_mode mode){
    void (*events[][]) () =
            {{&clear_carry, NULL}, {&fetch_opcode, &decode}};

    std::memcpy(event_queue, events, 4);
}

// Set the decimal mode flag to 0
CPU::CLD(address_mode mode){
    void (*events[][]) () =
            {{&clear_decimal, NULL}, {&fetch_opcode, &decode}};

    std::memcpy(event_queue, events, 4);
}

// Set the interrupt disable mode flag to 0
CPU::CLI(address_mode mode){
    void (*events[][]) () =
            {{&clear_interrupt, NULL}, {&fetch_opcode, &decode}};

    std::memcpy(event_queue, events, 4);
}

// Set the overflow flag to 0
CPU::CLV(address_mode mode){
    void (*events[][]) () =
            {{&clear_overflow, NULL}, {&fetch_opcode, &decode}};

    std::memcpy(event_queue, events, 4);
}

// Compare the specified memory location and the accumulator.
//
CPU::CMP (address_mode mode) {

    // Add different single-cycle actions to the event queue depending on
    // the addressing mode
    switch (mode) {
        case IMM:
            void (*events[][]) () =
                {{&fetch_data_pc, &cmp_acc}, {&fetch_opcode, &decode}};

            std::memcpy(event_queue, events, 4 * sizeof(void*));
            break;

        case ABS:
            void (*events[][]) () =
                {{&fetch_ladd_pc, NULL}, {&fetch_hadd_pc, NULL},
                {&fetch_data_abus, &cmp_acc}, {&fetch_opcode, &decode}};

            std::memcpy(event_queue, events, 8 * sizeof(void*));
            break;

        case XABS:
            void (*events[][]) () =
                {{&fetch_ladd_pc, NULL}, {&fetch_hadd_pc, &add_x_ladd}, {&fetch_data_abus, &fix_add_skip},
                {&fetch_data_abus, &cmp_acc}, {&fetch_opcode, &decode}};

            std::memcpy(event_queue, events, 10 * sizeof(void*));
            break;

        case YABS:
            void (*events[][]) () =
                {{&fetch_ladd_pc, NULL}, {&fetch_hadd_pc, &add_y_ladd}, {&fetch_data_abus, &fix_add_skip},
                {&fetch_data_abus, &cmp_acc}, {&fetch_opcode, &decode}};

            std::memcpy(event_queue, events, 10 * sizeof(void*));
            break;

        case ZP:
            void (*events[][]) () =
                {{&fetch_ladd_pc, NULL}, {&fetch_data_zp, &cmp_acc}, {&fetch_opcode, &decode}};

            std::memcpy(event_queue, events, 6 * sizeof(void*));
            break;

        case XZP:
            void (*events[][]) () =
                {{&fetch_ladd_pc, NULL}, {&fetch_ladd_zp, &add_x_ladd},
                {&fetch_zp_abus, &cmp_acc}, {&fetch_opcode, &decode}};

            std::memcpy(event_queue, events, 8 * sizeof(void*));
            break;

        case XZPI:
            void (*events[][]) () =
                {{&fetch_ladd_pc, NULL}, {&fetch_ladd_zp, &add_x_ladd}, {&fetch_ladd_zp, &inc_ladd},
                {&fetch_hadd_zp, NULL}, {&fetch_data_abus, &cmp_acc}, {&fetch_opcode, &decode}};

            std::memcpy(event_queue, events, 12 * sizeof(void*));
            break;

        case ZPIY:
            void (*events[][]) () =
                {{&fetch_ladd_pc, NULL}, {&fetch_ladd_zp, &inc_ladd}, {&fetch_hadd_zp, &add_y_ladd},
                {&fetch_data_abus, &fix_add_skip}, {&fetch_data_abus, &cmp_acc}, {&fetch_opcode, &decode}};

            std::memcpy(event_queue, events, 12 * sizeof(void*));
            break;
    }
}

// Compare register X to the specified memory location and set
// flags accordingly.
CPU::CPX (address_mode mode) {
    switch (mode) {

        case IMM:
            void (*events [][]) () =
                {{&fetch_data_pc, &cmp_x}, {&fetch_opcode, &decode}};

            std::memcpy(event_queue, events, 4 * sizeof(void*));
            break;

        case ABS:
            void (*events [][]) () =
                {{&fetch_ladd_pc, NULL}, {&fetch_hadd_pc, NULL},
                {&fetch_data_abus, &cmp_x}, {&fetch_opcode, &decode}};

            std::memcpy(event_queue, events, 8 * sizeof(void*));
            break;

        case ZP:
            void (*events [][]) () =
                {{&fetch_ladd_pc, NULL}, {&fetch_data_zp, &cmp_x}, {&fetch_opcode, &decode}};

            std::memcpy(event_queue, events, 6 * sizeof(void*));
            break;
}

// Compare register Y to the specified memory location and set
// flags accordingly.
CPU::CPY (address_mode mode) {
    switch (mode) {

        case IMM:
            void (*events [][]) () =
                {{&fetch_data_pc, &cmp_y}, {&fetch_opcode, &decode}};

            std::memcpy(event_queue, events, 4 * sizeof(void*));
            break;

        case ABS:
            void (*events [][]) () =
                {{&fetch_ladd_pc, NULL}, {&fetch_hadd_pc, NULL},
                {&fetch_data_abus, &cmp_y}, {&fetch_opcode, &decode}};

            std::memcpy(event_queue, events, 8 * sizeof(void*));
            break;

        case ZP:
            void (*events [][]) () =
                {{&fetch_ladd_pc, NULL}, {&fetch_data_zp, &cmp_y}, {&fetch_opcode, &decode}};

            std::memcpy(event_queue, events, 6 * sizeof(void*));
            break;
}

// Decrement the specified memory location by one
CPU::DEC (address_mode mode) {

    switch (mode) {

        case ABS:
            void (*events [][]) () =
                {{&fetch_ladd_pc, NULL}, {fetch_hadd_pc, NULL}, {&fetch_data_abus, NULL},
                {&write_data_abus, &dec_dbus}, {&write_data_abus, NULL}, {&fetch_opcode, &decode}};

            std::memcpy(event_queue, events, 12 * sizeof(void*));
            break;

        case XABS:
            void (*events [][]) () =
                {{&fetch_ladd_pc, NULL}, {fetch_hadd_pc, &add_x_ladd}, {&fetch_data_abus, &fix_add},
                {&fetch_data_abus, NULL}, {&write_data_abus, &dec_dbus}, {&write_data_abus, NULL},
                {&fetch_opcode, &decode}};

            std::memcpy(event_queue, events, 14 * sizeof(void*));
            break;
    }
}

// Increment the Y register
CPU::INY (address_mode mode) {
    events.insert({{&fetch_opcode, &iny}, {&decode, NULL}});
    break;
}

// Perform no operation, fetch next opcode
CPU::NOP (address_mode mode) {
    events.insert({{&fetch_opcode, NULL}, {&decode, NULL}});
    break;
}



// Load an instruction from the given opcode and queue its needed events
CPU::decode () {

    // NOTE: The 6502 contains multiple addressing modes for
    // many instructions, and as such functions can be
    // called depending on the addressing mode. All potential opcodes
    // are organized alphabetically by their instruction names.
    switch (opcode) {

        case 0x69:  ADC(IMM);   break;
        case 0x6D:  ADC(ABS);   break;
        case 0x7D:  ADC(XABS);  break;
        case 0x79:  ADC(YABS);  break;
        case 0x65:  ADC(ZP);    break;
        case 0x75:  ADC(XZP);   break;
        case 0x61:  ADC(XZPI);  break;
        case 0x71:  ADC(ZPIY);  break;

        case 0x29:  AND(IMM);   break;
        case 0x2D:  AND(ABS);   break;
        case 0x3D:  AND(XABS);  break;
        case 0x39:  AND(YABS);  break;
        case 0x25:  AND(ZP);    break;
        case 0x35:  AND(XZP);   break;
        case 0x21:  AND(XZPI);  break;
        case 0x32:  AND(ZPIY);  break;

        case 0x0A:  ASL(ACC);   break;
        case 0x0E:  ASL(ABS);   break;
        case 0x1E:  ASL(XABS);  break;
        case 0x06:  ASL(ZP);    break;
        case 0x16:  ASL(XZP);   break;

        case 0x90:  BCC(REL);   break;

        case 0xB0:  BCS(REL);   break;

        case 0xF0:  BEQ(REL);   break;

        case 0x2C:  BIT(ABS);   break;
        case 0x24:  BIT(ZP);    break;

        case 0x30:  BMI(REL);   break;

        case 0xD0:  BNE(REL);   break;

        case 0x10:  BPL(REL);   break;

        case 0x00:  BRK(IMPL);  break;

        case 0x50:  BVC(REL);   break;

        case 0x70:  BVS(REL);   break;

        case 0x18:  CLC(IMPL);  break;

        // NOTE: The decimal mode is disabled on the NES
        case 0xD8:  CLD(IMPL);  break;

        case 0x58:  CLI(IMPL);  break;

        case 0xB8:  CLV(IMPL);  break;

        case 0xC9:  CMP(IMM);   break;
        case 0xCD:  CMP(ABS);   break;
        case 0xDD:  CMP(XABS);  break;
        case 0xD9:  CMP(YABS);  break;
        case 0xC5:  CMP(ZP);    break;
        case 0xD5:  CMP(XZP);   break;
        case 0xC1:  CMP(XZPI);  break;
        case 0xD1:  CMP(ZPIY);  break;

        case 0xE0:  CPX(IMM);   break;
        case 0xEC:  CPX(ABS);   break;
        case 0xE4:  CPX(ZP);    break;

        case 0xC0:  CPY(IMM):   break;
        case 0xCC:  CPY(ABS):   break;
        case 0xC4:  CPY(ZP):    break;

        case 0xCE:  DEC(ABS):   break;
        case 0xDE:  DEC(XABS):  break;
        case 0xC6:  DEC(ZP):    break;
        case 0xD6:  DEC(XZP):   break;

        case 0xCA:  DEX(IMPL):  break;

        case 0x88:  DEY(IMPL):  break;

        case 0x49:  EOR(IMM):   break;
        case 0x4D:  EOR(ABS):   break;
        case 0x5D:  EOR(XABS):  break;
        case 0x59:  EOR(YABS):  break;
        case 0x45:  EOR(ZP):    break;
        case 0x55:  EOR(XZP):   break;
        case 0x41:  EOR(XZPI):  break;
        case 0x51:  EOR(ZPIY):  break;

        case 0xEE:  INC(ABS):   break;
        case 0xFE:  INC(XABS):  break;
        case 0xE6:  INC(ZP):    break;
        case 0xF6:  INC(XZP):   break;

        case 0xE8:  INX(IMPL):  break;

        case 0xC8:  INY(IMPL):  break;

        case 0x4C:  JMP(ABS):   break;
        case 0x6C:  JMP(ABSI):  break;

        case 0x20:  JSR(ABS):   break;

        case 0xA9:  LDA(IMM):   break;
        case 0xAD:  LDA(ABS):   break;
        case 0xBD:  LDA(XABS):  break;
        case 0xB9:  LDA(YABS):  break;
        case 0xA5:  LDA(ZP):    break;
        case 0xB5:  LDA(XZP):   break;
        case 0xA1:  LDA(XZPI):  break;
        case 0xB1:  LDA(ZPIY):  break;

        case 0xA2:  LDX(IMM):   break;
        case 0xAE:  LDX(ABS):   break;
        case 0xBE:  LDX(YABS):  break;
        case 0xA6:  LDX(ZP):    break;
        case 0xB6:  LDX(YZP):   break;

        case 0xA0:  LDY(IMM):   break;
        case 0xAC:  LDY(ABS):   break;
        case 0xBC:  LDY(XABS):  break;
        case 0xA4:  LDY(ZP):    break;
        case 0xB4:  LDY(XZP):   break;

        case 0x4A:  LSR(ACC):   break;
        case 0x4E:  LSR(ABS):   break;
        case 0x5E:  LSR(XABS):  break;
        case 0x46:  LSR(ZP):    break;
        case 0x56:  LSR(XZP):   break;

        case 0xEA:  NOP(IMPL):  break;

        case 0x09:  ORA(IMM):   break;
        case 0x0D:  ORA(ABS):   break;
        case 0x1D:  ORA(XABS):  break;
        case 0x19:  ORA(YABS):  break;
        case 0x05:  ORA(ZP):    break;
        case 0x15:  ORA(XZP):   break;
        case 0x01:  ORA(XZPI):  break;
        case 0x11:  ORA(ZPIY):  break;

        case 0x48:  PHA(IMPL):  break;

        case 0x08:  PHP(IMPL):  break;

        case 0x68:  PLA(IMPL):  break;

        case 0x28:  PLP(IMPL):  break;

        case 0x2A:  ROL(ACC):   break;
        case 0x2E:  ROL(ABS):   break;
        case 0x3E:  ROL(XABS):  break;
        case 0x26:  ROL(ZP):    break;
        case 0x36:  ROL(XZP):   break;

        case 0x6A:  ROR(ACC):   break;
        case 0x6E:  ROR(ABS):   break;
        case 0x7E:  ROR(XABS):  break;
        case 0x66:  ROR(ZP):    break;
        case 0x76:  ROR(XZP):   break;

        case 0x40:  RTI(IMPL):  break;

        case 0x60:  RTS(IMPL):  break;

        case 0xE9:  SBC(IMM):   break;
        case 0xED:  SBC(ABS):   break;
        case 0xFD:  SBC(XABS):  break;
        case 0xF9:  SBC(YABS):  break;
        case 0xE5:  SBC(ZP):    break;
        case 0xF5:  SBC(XZP):   break;
        case 0xE1:  SBC(XZPI):  break;
        case 0xF1:  SBC(ZPIY):  break;

        case 0x38:  SEC(IMPL):  break;

        case 0xF8:  SEC(IMPL):  break;

        case 0x78:  SEI(IMPL):  break;

        case 0x8D:  STA(ABS):   break;
        case 0x9D:  STA(XABS):  break;
        case 0x99:  STA(YABS):  break;
        case 0x85:  STA(ZP):    break;
        case 0x95:  STA(XZP):   break;
        case 0x81:  STA(XZPI):  break;
        case 0x91:  STA(ZPIY):  break;

        case 0x8E:  STX(ABS):   break;
        case 0x86:  STX(ZP):    break;
        case 0x96:  STX(YZP):   break;

        case 0x8C:  STY(ABS):   break;
        case 0x84:  STY(ZP):    break;
        case 0x94:  STY(YZP):   break;

        case 0xAA:  TAX(IMPL):  break;

        case 0xA8:  TAY(IMPL):  break;

        case 0xBA:  TSX(IMPL):  break;

        case 0x8A:  TXA(IMPL):  break;

        case 0x9A:  TXS(IMPL):  break;

        case 0x98:  TYA(IMPL):  break;

        default:    NOP(IMPL);  break;
    }
}
