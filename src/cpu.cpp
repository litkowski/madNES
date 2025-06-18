#include <deque>
#include <array>
#include <bitset>
#include <iostream>
#include <sstream>
#include <fstream>
#include "cpu.hpp"

#define CARRY_FLAG 0b00000001
#define ZERO_FLAG 0b00000010
#define INTERRUPT_FLAG 0b00000100
#define DECIMAL_FLAG 0b00001000
#define B_FLAG 0b00010000
#define EXTRA_FLAG 0b00100000
#define OVERFLOW_FLAG 0b01000000
#define NEGATIVE_FLAG 0b10000000

// Log
std::ofstream cpu_log;

// Mapper
Cartridge* game;

// All 6502 registers and buses
uint8_t acc, x, y, sp, status, opcode, dbus;
uint8_t pcl;
uint16_t pc, abus;

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


void push_events_read (address_mode mode, void (*func) ());
void push_events_read_mod_write (address_mode mode, void (*func) ());
void push_events_write (address_mode mode, void (*func) ());

void adc_acc ();
void and_acc ();
void asl_acc ();
void asl_dbus ();
void bcc ();
void bcs ();
void bne ();
void beq ();
void bpl ();
void bmi ();
void bvc ();
void bvs ();
void bit ();
void clc ();
void cld ();
void cli ();
void clv ();
void cmp ();
void cpx ();
void cpy ();
void dec ();
void dex ();
void dey ();
void eor ();
void inc ();
void inx ();
void iny ();
void lda ();
void ldx ();
void ldy ();
void lsr_acc ();
void lsr_dbus ();
void ora ();
void push_acc ();
void pop_acc ();
void rol_acc ();
void rol_dbus ();
void ror_acc ();
void ror_dbus ();
void sbc ();
void sec ();
void sed ();
void sei ();
void sta ();
void stx ();
void sty ();
void tax ();
void tay ();
void tsx ();
void txa ();
void txs ();
void tya ();

void nop ();
void fetch_data_abus ();
void fetch_data_pc ();
void fetch_ladd_pc ();
void fetch_hadd_pc ();
void fetch_ladd_abus ();
void fetch_hadd_abus ();
void fetch_opcode ();
void write_data_abus ();
void add_dbus_pcl ();
void add_x_ladd ();
void add_x_ladd_skip ();
void add_y_ladd ();
void add_y_ladd_skip ();
void add_x_zp ();
void add_y_zp ();
void fix_add ();
void fix_pch ();
void inc_abus ();
void dec_sp ();
void inc_sp ();
void push_pch ();
void pop_pch ();
void push_pcl ();
void pop_pcl ();
void push_p_irq ();
void push_p_nmi ();
void pop_p ();
void fetch_pcl_irq ();
void fetch_pch_irq ();
void fetch_pcl_nmi ();
void fetch_pch_nmi ();
void copy_dbus_pcl ();
void fetch_pc_to_pch ();
void fetch_abus_to_pch ();
void inc_pc ();
void inc_ladd_no_fix ();
void set_zero_and_neg (uint8_t result);

void decode ();

// All instruction functions, each one will queue a number of single-cycle actions
// NOTE: This will be expanded to include illegal opcodes in the future
void ADC (address_mode mode);
void AND (address_mode mode);
void ASLA(address_mode mode);
void ASL (address_mode mode);
void BCC (address_mode mode);
void BCS (address_mode mode);
void BEQ (address_mode mode);
void BIT (address_mode mode);
void BMI (address_mode mode);
void BNE (address_mode mode);
void BPL (address_mode mode);
void BRK (address_mode mode);
void BVC (address_mode mode);
void BVS (address_mode mode);
void CLC (address_mode mode);
void CLD (address_mode mode);
void CLI (address_mode mode);
void CLV (address_mode mode);
void CMP (address_mode mode);
void CPX (address_mode mode);
void CPY (address_mode mode);
void DEC (address_mode mode);
void DEX (address_mode mode);
void DEY (address_mode mode);
void EOR (address_mode mode);
void INC (address_mode mode);
void INX (address_mode mode);
void INY (address_mode mode);
void JMP (address_mode mode);
void JSR (address_mode mode);
void LDA (address_mode mode);
void LDX (address_mode mode);
void LDY (address_mode mode);
void LSRA(address_mode mode);
void LSR (address_mode mode);
void NOP (address_mode mode);
void ORA (address_mode mode);
void PHA (address_mode mode);
void PHP (address_mode mode);
void PLA (address_mode mode);
void PLP (address_mode mode);
void ROLA(address_mode mode);
void ROL (address_mode mode);
void RORA(address_mode mode);
void ROR (address_mode mode);
void RTI (address_mode mode);
void RTS (address_mode mode);
void SBC (address_mode mode);
void SEC (address_mode mode);
void SED (address_mode mode);
void SEI (address_mode mode);
void STA (address_mode mode);
void STX (address_mode mode);
void STY (address_mode mode);
void TAX (address_mode mode);
void TAY (address_mode mode);
void TSX (address_mode mode);
void TXA (address_mode mode);
void TXS (address_mode mode);
void TYA (address_mode mode);

// Helper function; push the given events to the event queue
void push_events (std::vector<std::array<void (*) (), 2>> events);

// Queue to store atomic events in.
// NOTE: Each CPU cycle includes a maximum of two actions, and thus two function
// pointers occupy a single space in the queue
std::deque<std::array<void (*) (), 2>> event_queue;

// INitialize the CCPU with the given mapper
void Init_CPU (Cartridge* mapper) {

	game = mapper;

	// Assign all registers to their startup states
	acc = x = y = 0;
	pc = 0;
    pc |= game->cpu_read(0xFFFC);
    pc |= game->cpu_read(0xFFFD) << 8;
	sp = 0xFD;
	status = 0b00100100;

    // Start the CPU at the first instruction
    push_events({});

    cpu_log.open("./log.txt", std::ios::trunc);
}

// Helper function to push a range of atomic operations to the event queue
// NOTE: Adds fetch and decode operations to the queue,
// ONLY USE when adding a new instruction!!!
void push_events (std::vector<std::array<void (*) (), 2>> events) {
	for (int i = 0; i < events.size(); i++) {
		event_queue.push_back(events[i]);
	}
	event_queue.push_back({&fetch_opcode, &decode});
}

/*
 * Execute one CPU cycle.
 */
void cycle_cpu () {

    std::array<void (*) (), 2> functions =  event_queue.front();
    event_queue.pop_front();

    (*functions[0])();

    if (functions[1] != NULL) {
        (*functions[1])();
    }
}

// Set the processor to start an interrupt after the current instuction finishes
void signal_nmi () {
    // Remove the fetch/decode cycle that's currently queued
    event_queue.pop_back();
    push_events({{&nop, NULL}, {&push_pch, &dec_sp}, {&push_pcl, &dec_sp},
                {&push_p_nmi, &dec_sp}, {&fetch_pcl_nmi, NULL}, {&fetch_pch_nmi, NULL}});
}

// Signal to the CPU to wait until OAMDMA is finished
// TODO: This should take either 513 or 514 cycles, currently only supports 513
void signal_oamdma () {
    for (int i = 0; i < 513; i++) {
        event_queue.push_front({&nop, NULL});
    }
}

// Do nothing
void nop () {
}

// Fetch the data at the address bus's pointer into the data bus
void fetch_data_abus () {
	dbus = game->cpu_read(abus);
    cpu_log << std::hex << (int) dbus << "\n";
}

// Fetch the data at the program counter into the data bus
void fetch_data_pc () {
	dbus = game->cpu_read(pc);
	pc++;
}

// Fetch the data at the program counter into the low byte of the address bus
void fetch_ladd_pc () {
	dbus = game->cpu_read(pc);
	abus = 0;
	abus |= dbus;
    pc++;
}

// Fetch the high address of the address bus
void fetch_hadd_pc () {
	dbus = game->cpu_read(pc);
	abus &= 0x00FF;
	abus |= (dbus << 8);
    pc++;
}

// Fetch the data at the abus into the low byte of the address bus
void fetch_ladd_abus () {
	dbus = game->cpu_read(abus);
	abus = 0;
	abus |= dbus;
}

// Fetch the data at the abus into the high byte of the address bus
void fetch_hadd_abus () {
    uint16_t abus_temp = abus;
    abus = 0;
    abus |= dbus;
	dbus = game->cpu_read(abus_temp);
	abus &= 0x00FF;
	abus |= (dbus << 8);
}


// Fetch next opcode from PC
void fetch_opcode () {
    cpu_log << std::hex << pc << ": ";
	opcode = dbus = game->cpu_read(pc);
	pc++;
}

void write_data_abus () {
    game->cpu_write(abus, dbus);
    cpu_log << std::hex << (int) dbus << "\n";
}

/*
 * Add the contents of the data bus to the low bytes of PC
 * NOTE: If there is a page crossing, an additional cycle will be added
 */
void add_dbus_pcl () {

    if (((pc + (int8_t) dbus) & 0xF0) != (pc & 0xF0)) {
        status |= CARRY_FLAG;
        event_queue.push_front({&nop, NULL});
    }

    pc += (int8_t) dbus;
}

/*
 * Add the contents of the X register to the low byte of the address bus
 * NOTE: This version of the function will NOT skip if fixing is unnecessary
 */
void add_x_ladd () {

	uint8_t ladd = (uint8_t) abus;
	abus &= 0xFF00;

	if (ladd + x > 0x00FF) {
		status |= CARRY_FLAG;
	}

	ladd += x;
	abus |= ladd;
}

/*
 * Add the contents of the X register to the low byte of the address bus
 * NOTE: This version of the function WILL skip a cycle if fixing is unnecessary
 */
 void add_x_ladd_skip () {

	uint8_t ladd = (uint8_t) abus;
	abus &= 0xFF00;

	if (ladd + x > 0x00FF) {
		status |= CARRY_FLAG;
        event_queue.push_front({&fetch_data_abus, &fix_add});
	}

	ladd += x;
	abus |= ladd;
}

/*
 * Add the contents of the Y register to the low byte of the address bus
 * NOTE: This version of the function will NOT skip if fixing is unnecessary
 */
void add_y_ladd () {

	uint8_t ladd = (uint8_t) abus;
	abus &= 0xFF00;

	if (ladd + y > 0x00FF) {
		status |= CARRY_FLAG;
	}

	ladd += y;
	abus |= ladd;
}

/*
 * Add the contents of the Y register to the low byte of the address bus
 * NOTE: This version of the function WILL skip a cycle if fixing is unnecessary
 */
 void add_y_ladd_skip () {

	uint8_t ladd = (uint8_t) abus;
	abus &= 0xFF00;

	if (ladd + y > 0x00FF) {
		status |= CARRY_FLAG;
        event_queue.push_front({&fetch_data_abus, &fix_add});
	}

	ladd += y;
	abus |= ladd;
}

// Add the contents of the X register to a zero page address
void add_x_zp () {
	abus &= 0xFF;
	uint8_t* ladd = (uint8_t*) &abus;
	*ladd += x;
}

// Add the contents of the Y register to a zero page address
void add_y_zp () {
	abus &= 0xFF;
	uint8_t* ladd = (uint8_t*) &abus;
	*ladd += y;
}

// Fix the high byte of the address bus.
// Only executed in cases of page crossings.
void fix_add () {
    if (status &= CARRY_FLAG) {
        abus += 0x100;
    }
}

// Fix the high byte of the program counter.
// Only executed in cases of page crossings.
void fix_pch () {
    if (status &= CARRY_FLAG) {
        pc += 0x100;
    }
}

// Increment the abus by one
void inc_abus () {
	abus++;
}

// Decrement the stack pointer by one byte
void dec_sp () {
    sp--;
}

// Increment the stack pointer by one byte
void inc_sp () {
    sp++;
}

// Push the high byte of PC onto the stack
void push_pch () {
    game->cpu_write(0x100 + sp, (uint8_t) ((pc & 0xFF00) >> 8));
}

// Pop the high byte of PC from the stack
void pop_pch () {
    pc &= 0x00FF;
    pc |= game->cpu_read(0x100 + sp) << 8;
}

// Push the low byte of PC onto the stack
void push_pcl () {
    game->cpu_write(0x100 + sp, (uint8_t) (pc & 0x00FF));
}

// Pop the low byte of PC from the stack
void pop_pcl () {
    pc &= 0xFF00;
    pc |= game->cpu_read(0x100 + sp);
}

// Push the status register to the stack with the B flag set
void push_p_irq () {
    game->cpu_write(0x100 + sp, status | B_FLAG | EXTRA_FLAG);
}

// Push the status register to the stack with the B flag unset
void push_p_nmi () {
    game->cpu_write(0x100 + sp, (status | EXTRA_FLAG) & ~B_FLAG);
}

// Pop the status register from the stack
void pop_p () {
    status = game->cpu_read(0x100 + sp);
}

// Fetch the low byte of the IRQ interrupt handler
void fetch_pcl_irq  () {
    pc &= 0xFF00;
    pc |= game->cpu_read(0xFFFE);
}

// Fetch the high byte of the IRQ interrupt handler
void fetch_pch_irq () {
    pc &= 0x00FF;
    pc |= (game->cpu_read(0xFFFF) << 8);
}

// Fetch the low byte of the NMI interrupt handler
void fetch_pcl_nmi () {
    pc &= 0xFF00;
    pc |= game->cpu_read(0xFFFA);
}

// Fetch the high byte of the NMI interrupt handler
void fetch_pch_nmi () {
    pc &= 0x00FF;
    pc |= (game->cpu_read(0xFFFB) << 8);
}

// Copy the dbus to the low byte of PC
void copy_dbus_pcl () {
    pc &= 0xFF00;
    pc |= dbus;
}

// Fetch data from PC to the high byte of PC
void fetch_pc_to_pch () {
    uint16_t pch = game->cpu_read(pc) << 8;
    pc &= 0x00FF;
    pc |= pch;
}

// Fetch data from the abus's address to the low byte of PC
void fetch_abus_to_pch () {
    pc &= 0x00FF;
    pc |= game->cpu_read(abus) << 8;
}

// Increment the PC by one
void inc_pc () {
    pc++;
}

// Increment the low address of the address bus, do not fix it
void inc_ladd_no_fix () {
    uint8_t ladd = 0x00FF & abus;
    ladd++;
    abus &= 0xFF00;
    abus |= ladd;
}

// Set zero and negative flags depending on the result
// NOTE: Used in a variety of atomic operations on many registers
void set_zero_and_neg (uint8_t result) {
    if (!result) {
        status |= ZERO_FLAG;
    } else {
        status &= ~ZERO_FLAG;
    }

    status &= ~NEGATIVE_FLAG;
    status |= (NEGATIVE_FLAG & result);
}

/* NOTE: Specialized operations for particular instructions begin here */

// Add the contents of the data bus to the accumulator, with carry
void adc_acc () {
    uint8_t old_acc = acc;
    acc += dbus;

	if (old_acc > acc) {
		status |= CARRY_FLAG;
	} else {
        status &= ~CARRY_FLAG;
    }

    if ((old_acc ^ acc) & 0b10000000) {
        status |= OVERFLOW_FLAG;
    } else {
        status &= ~OVERFLOW_FLAG;
    }
}

// Perform a bitwise AND operation on the accumulator
// Sets the zero flag if the result is zero, sets the negative
// flag if the result is a signed negative integer
void and_acc () {
    acc &= dbus;
    set_zero_and_neg(acc);
}

/*
 * Shift the accumulator left one bit.
 * Save the 7th bit in the carry flag
 */
void asl_acc () {

    if (acc & 0b1000000) {
        status |= CARRY_FLAG;
    } else {
        status &= ~CARRY_FLAG;
    }

    acc = acc << 1;
    acc &= 0b11111110;
}

/*
 * Shift the data bus left one bit.
 * Save the 7th bit in the carry flag
 */
void asl_dbus () {

    if (dbus & 0b10000000) {
        status |= CARRY_FLAG;
    } else {
        status &= ~CARRY_FLAG;
    }

    dbus = dbus << 1;
    dbus &= 0b11111110;
}



/*  BRANCH INSTRUCTIONS: If the condition is passed, the branch is taken. */
/*  NOTE: If the branch is taken, add_dbus_pcl() will add a cycle if a page is crossed */
/*  KEY:
 *      BCC - Branch if carry flag is clear
 *      BCS - Branch if the carry flag is set
 *      BEQ - Branch if the zero flag is set
 *      BMI - Branch if the negative flag is set
 *      BNE - Branch if the zero flag is not set
 *      BPL - Branch if the negative flag is not set
 *      BVC - Branch if the overflow flag is not set
 *      BVS - Branch if the overflow flag is set
 */
void bcc () {
    if (!(status & CARRY_FLAG)) {
        event_queue.push_front({&add_dbus_pcl, NULL});
    }
}

void bcs () {
    if (status & CARRY_FLAG) {
        event_queue.push_front({&add_dbus_pcl, NULL});
    }
}

void beq () {
    if (status & ZERO_FLAG) {
        event_queue.push_front({&add_dbus_pcl, NULL});
    }
}

void bmi () {
    if (status & NEGATIVE_FLAG) {
        event_queue.push_front({&add_dbus_pcl, NULL});
    }
}

void bne () {
    if (!(status & ZERO_FLAG)) {
        event_queue.push_front({&add_dbus_pcl, NULL});
    }
}

void bpl () {
    if (!(status & ZERO_FLAG)) {
        event_queue.push_front({&add_dbus_pcl, NULL});
    }
}

void bvc () {
    if (!(status & OVERFLOW_FLAG)) {
        event_queue.push_front({&add_dbus_pcl, NULL});
    }
}

void bvs () {
    if (status & OVERFLOW_FLAG) {
        event_queue.push_front({&add_dbus_pcl, NULL});
    }
}


// AND the data in the dbus with the accumulator. If the result is zero,
// set the zero flag. If not, reset the flag. No matter what, copy the
// 6th and 7th bits of the memory address into the status register
void bit () {
    uint8_t result = dbus & acc;

    if (result) {
        status &= ~ZERO_FLAG;
    } else {
        status |= ZERO_FLAG;
    }

    status &= 0b00111111;
    status |= (dbus & 0b11000000);
}

// Clear the carry flag
void clc () {
    status &= ~CARRY_FLAG;
}

// Clear the decimal flag (disabled on the NES)
void cld () {
    status &= ~DECIMAL_FLAG;
}

// Clear the interrupt disable flag
void cli () {
    status &= ~INTERRUPT_FLAG;
}

// Clear the overflow flag
void clv () {
    status &= ~OVERFLOW_FLAG;
}

// Compare the value of the specified location with the accumulator, set the status flags accordingly
void cmp () {
    // Check to set the carry flag
    if (dbus <= acc) {
        status |= CARRY_FLAG;
    } else {
        status &= ~ZERO_FLAG;
    }

    // Perform SIGNED subtraction on the accumulator and dbus
    uint8_t result = acc - dbus;

    // Check for zero and negative results
    set_zero_and_neg(result);
}

// Compare the value of the specified location with the X register, set the status flags accordingly
void cpx () {
    // Check to set the carry flag
    if (dbus <= x) {
        status |= CARRY_FLAG;
    } else {
        status &= ~ZERO_FLAG;
    }

    // Perform SIGNED subtraction on the accumulator and dbus
    uint8_t result = x - dbus;

    // Check for zero and negative results
    set_zero_and_neg(result);
}

// Compare the value of the specified location with the Y register, set the status flags accordingly
void cpy () {

    // Check to set the carry flag
    if (dbus <= y) {
        status |= CARRY_FLAG;
    } else {
        status &= ~ZERO_FLAG;
    }

    // Perform SIGNED subtraction on the accumulator and dbus
    uint8_t result = y - dbus;

    // Check for zero and negative results
    set_zero_and_neg(result);
}

// Decrement the dbus by one, set flags accordingly
void dec () {
    dbus -= 1;
    set_zero_and_neg(dbus);
}

// Decrement X by one, set flags accordingly
void dex () {
    x -= 1;
    set_zero_and_neg(x);
}

// Decrement Y by one, set flags accordingly
void dey () {
    y -= 1;
    set_zero_and_neg(y);
}

// Perform an XOR on the accumulator and dbus, store in acc
void eor () {
    acc ^= dbus;
    set_zero_and_neg(acc);
}

// Increment the dbus by one
void inc () {
    dbus += 1;
    set_zero_and_neg(dbus);
}

// Increment X by one
void inx () {
    x += 1;
    set_zero_and_neg(x);
}

// Increment Y by one
void iny () {
    y += 1;
    set_zero_and_neg(y);
}

// Load the accumulator with the memory in dbus, set flags accordingly
void lda () {
    acc = dbus;
    set_zero_and_neg(acc);
}

// Load X with the memory in dbus, set flags accordingly
void ldx () {
    x = dbus;
    set_zero_and_neg(x);
}

// Load Y with the memory in dbus, set flags accordingly
void ldy () {
    y = dbus;
    set_zero_and_neg(y);
}

// Shift the accumulator right one bit. Store the low bit in the carry flag.
// Reset the negative flag.
void lsr_acc () {
    if (acc & 0x01) {
        status |= CARRY_FLAG;
    } else {
        status &= ~CARRY_FLAG;
    }

    acc = acc >> 1;
    acc &= 0b01111111;
    if (!acc) {
        status |= ZERO_FLAG;
    } else {
        status &= ~ZERO_FLAG;
    }

    status &= ~NEGATIVE_FLAG;
}

// Shift the dbus right one bit. Store the low bit in the carry flag.
// Reset the negative flag.
void lsr_dbus () {
    if (dbus & 0x01) {
        status |= CARRY_FLAG;
    } else {
        status &= ~CARRY_FLAG;
    }

    dbus = dbus >> 1;
    dbus &= 0b01111111;
    if (!dbus) {
        status |= ZERO_FLAG;
    } else {
        status &= ~ZERO_FLAG;
    }

    status &= ~NEGATIVE_FLAG;
}

// Perform a bitwise OR with the dbus and accumulator
void ora () {
    acc = dbus | acc;
    set_zero_and_neg(acc);
}

// Push the accumulator onto the stack
void push_acc () {
    game->cpu_write(0x100 + sp, acc);
}

// Pop the accumulator from the stack
void pop_acc () {
    acc = game->cpu_read(0x100 + sp);
}

// Rotate the accumulator left; i.e. store the 7th bit in the carry flag,
// store the initial carry flag in the 0th bit, and shift left one bit.
void rol_acc () {
    uint8_t temp_status = status;

    if (acc & 0b10000000) {
        status |= CARRY_FLAG;
    } else {
        status &= ~CARRY_FLAG;
    }

    acc << 1;
    if (temp_status &= CARRY_FLAG) {
        acc |= 0b00000001;
    } else {
        acc &= 0b11111110;
    }
}

// Rotate the dbus left; i.e. store the 7th bit in the carry flag,
// store the initial carry flag in the 0th bit, and shift left one bit.
void rol_dbus () {
    uint8_t temp_status = status;

    if (dbus & 0b10000000) {
        status |= CARRY_FLAG;
    } else {
        status &= ~CARRY_FLAG;
    }

    dbus << 1;
    if (temp_status &= CARRY_FLAG) {
        dbus |= 0b00000001;
    } else {
        dbus &= 0b11111110;
    }
}

// Rotate the accumulator right; i.e. store the 7th bit in the carry flag,
// store the initial carry flag in the 0th bit, and shift left one bit.
void ror_acc () {
    uint8_t temp_status = status;

    if (acc & 0b00000001) {
        status |= CARRY_FLAG;
    } else {
        status &= ~CARRY_FLAG;
    }

    acc >> 1;
    if (temp_status &= CARRY_FLAG) {
        acc |= 0b10000000;
    } else {
        acc &= 0b01111111;
    }
}

// Rotate the dbus right; i.e. store the 7th bit in the carry flag,
// store the initial carry flag in the 0th bit, and shift left one bit.
void ror_dbus () {
    uint8_t temp_status = status;

    if (dbus & 0b00000001) {
        status |= CARRY_FLAG;
    } else {
        status &= ~CARRY_FLAG;
    }

    dbus >> 1;
    if (temp_status &= CARRY_FLAG) {
        dbus |= 0b10000000;
    } else {
        dbus &= 0b01111111;
    }
}

// Subtract the value in the dbus from the accumulator.
void sbc () {
    int8_t result = ((int8_t) acc - (int8_t) dbus);
    int16_t real_result = ((int16_t) acc) - ((int16_t) dbus);

    if (result >= 0) {
        status |= CARRY_FLAG;
    } else {
        status &= ~CARRY_FLAG;
    }

    if (result < real_result || result > real_result) {
        status |= OVERFLOW_FLAG;
    } else {
        status &= ~OVERFLOW_FLAG;
    }

    set_zero_and_neg(result);
    acc = (uint8_t) result;
}

// Set the carry flag
void sec () {
    status |= CARRY_FLAG;
}

// Set the decimal flag
void sed () {
    status |= DECIMAL_FLAG;
}

// Set the interrupt disable flag
void sei () {
    status |= INTERRUPT_FLAG;
}

// Write the accumulator to the abus location
void sta () {
    game->cpu_write(abus, acc);
}

// Write X to the abus location
void stx () {
    game->cpu_write(abus, x);
}

// Write Y to the abus location
void sty () {
    game->cpu_write(abus, y);
}

// Transfer acc to X
void tax () {
    x = acc;
    set_zero_and_neg(x);
}

// Transfer acc to Y
void tay () {
    y = acc;
    cpu_log << std::hex << (int) acc << "\n";
    set_zero_and_neg(y);
}

// Transfer SP to X
void tsx () {
    x = sp;
    set_zero_and_neg(x);
}

// Transfer X to acc
void txa () {
    acc = x;
    set_zero_and_neg(acc);
}

// Transfer X to SP
void txs () {
    sp = x;
    set_zero_and_neg(sp);
}

// Transfer Y to acc
void tya () {
    acc = y;
    set_zero_and_neg(acc);
}

/* NOTE: Helper functions for generic instruction types begin here */

// Generic read type function
void push_events_read (address_mode mode, void (*func) ()) {
    switch (mode) {
        case IMM:
			push_events({{&fetch_data_pc, func}});
			break;
		case ABS:
			push_events({{&fetch_ladd_pc, NULL}, {&fetch_hadd_pc, NULL},
				{&fetch_data_abus, func}});
			break;
		case XABS:
			push_events({{&fetch_ladd_pc, NULL}, {&fetch_hadd_pc, &add_x_ladd_skip},
				{&fetch_data_abus, func}});
			break;
		case YABS:
			push_events({{&fetch_ladd_pc, NULL}, {&fetch_hadd_pc, &add_y_ladd_skip},
				{&fetch_data_abus, &fix_add}, {&fetch_data_abus, func}});
			break;
		case ZP:
			push_events({{&fetch_ladd_pc, NULL}, {&fetch_data_abus, func}});
			break;
		case XZP:
			push_events({{&fetch_ladd_pc, NULL}, {&fetch_data_abus, &add_x_zp},
				{&fetch_data_abus, func}});
			break;
        case YZP:
            push_events({{&fetch_ladd_pc, NULL}, {&fetch_data_abus, &add_y_zp},
				{&fetch_data_abus, func}});
			break;
		case XZPI:
			push_events({{&fetch_ladd_pc, NULL}, {&fetch_ladd_abus, &add_x_zp},
				{&fetch_ladd_abus, &inc_abus}, {&fetch_hadd_abus, NULL},
				{&fetch_data_abus, func}});
            break;
		case ZPIY:
			push_events({{&fetch_ladd_pc, NULL}, {&fetch_data_abus, &inc_abus},
				{&fetch_hadd_abus, &add_y_ladd_skip}, {&fetch_data_abus, func}});
            break;
    }
}

// Generic read-modify-write type function
void push_events_read_mod_write (address_mode mode, void (*func) ()) {
    switch (mode) {
        case ABS:
            push_events({{&fetch_ladd_pc, NULL}, {&fetch_hadd_pc, NULL},
                {&fetch_data_abus, NULL}, {&write_data_abus, func},
                {&write_data_abus, NULL}});
            break;
        case XABS:
            push_events({{&fetch_ladd_pc, NULL}, {&fetch_ladd_pc, &add_x_ladd},
                {&fetch_data_abus, &fix_add}, {&fetch_data_abus, NULL},
                {&write_data_abus, func}, {&write_data_abus, NULL}});
            break;
        case ZP:
            push_events({{&fetch_ladd_pc, NULL}, {&fetch_data_abus, NULL},
                {&write_data_abus, func}, {&write_data_abus, NULL}});
            break;
        case XZP:
			push_events({{&fetch_ladd_pc, NULL}, {&fetch_data_abus, &add_x_zp},
				{&fetch_data_abus, func}});
			break;
    }
}

// Generic simple write type functin
void push_events_write (address_mode mode, void (*func) ()) {
    switch (mode) {
        case ABS:
            push_events({{&fetch_ladd_pc, NULL}, {&fetch_hadd_pc, NULL}, {func, NULL}});
            break;
        case XABS:
            push_events({{&fetch_ladd_pc, NULL}, {&fetch_hadd_pc, &add_x_ladd}, {&fetch_data_abus, &fix_add}, {func, NULL}});
            break;
        case YABS:
            push_events({{&fetch_ladd_pc, NULL}, {&fetch_hadd_pc, &add_y_ladd}, {&fetch_data_abus, &fix_add}, {func, NULL}});
            break;
        case ZP:
            push_events({{&fetch_ladd_pc, NULL}, {func, NULL}});
            break;
        case XZP:
            push_events({{&fetch_ladd_pc, NULL}, {&fetch_data_abus, &add_x_zp}, {func, NULL}});
            break;
        case XZPI:
			push_events({{&fetch_ladd_pc, NULL}, {&fetch_ladd_abus, &add_x_zp},
				{&fetch_ladd_abus, &inc_abus}, {&fetch_hadd_abus, NULL},
                {func, NULL}});
            break;
        case ZPIY:
			push_events({{&fetch_ladd_pc, NULL}, {&fetch_data_abus, &inc_abus},
				{&fetch_hadd_abus, &add_y_ladd}, {&fetch_data_abus, &fix_add},
				{func, NULL}});
            break;
    }
}

/* NOTE: Opcode handlers begin here */

// Add the data, as specified through addressing, to the accumulator
void ADC (address_mode mode) {
    cpu_log << "ADC\n";
    push_events_read(mode, &adc_acc);
}

// AND the specified data to the accumulator
void AND (address_mode mode) {
    cpu_log << "AND\n";
    push_events_read(mode, &and_acc);
}

// Shift the accumulator left one bit
void ASLA (address_mode mode) {
    cpu_log << "ASLA\n";
    push_events({{&asl_acc, NULL}});
}

// Shift the data left by one bit
void ASL (address_mode mode) {
    cpu_log << "ASL\n";
    push_events_read_mod_write(mode, &asl_dbus);
}

/* BRANCH INSTRUCTIONS */
void BCC (address_mode mode) {
    cpu_log << "BCC\n";
    push_events({{&fetch_data_pc, &bcc}});
}

void BCS (address_mode mode) {
    cpu_log << "BCS\n";
    push_events({{&fetch_data_pc, &bcs}});
}

void BEQ (address_mode mode) {
    cpu_log << "BEQ\n";
    push_events({{&fetch_data_pc, &beq}});
}

void BMI (address_mode mode) {
    cpu_log << "BMI\n";
    push_events({{&fetch_data_pc, &bmi}});
}

void BNE (address_mode mode) {
    cpu_log << "BNE\n";
    push_events({{&fetch_data_pc, &bne}});
}

void BPL (address_mode mode) {
    cpu_log << "BPL\n";
    push_events({{&fetch_data_pc, &bpl}});
}

void BVC (address_mode mode) {
    cpu_log << "BVC\n";
    push_events({{&fetch_data_pc, &bvc}});
}

void BVS (address_mode mode) {
    cpu_log << "BVS\n";
    push_events({{&fetch_data_pc, &bvs}});
}

// AND the memory location with the accumulator, but do not save the result
void BIT (address_mode mode) {
    cpu_log << "BIT\n";
    switch (mode) {
        case ABS:
            push_events({{&fetch_ladd_pc, NULL}, {&fetch_hadd_pc, NULL}, {&fetch_data_abus, &bit}});
            break;
        case ZP:
            push_events({{&fetch_ladd_pc, NULL}, {&fetch_data_abus, &bit}});
            break;
    }
}

// Undergo a voluntary interrupt. Pushes PC to the stack first, then status,
// and finally reads the handler address from 0xFFFE - 0xFFFF
void BRK (address_mode mode) {
    cpu_log << "BRK PC = " << pc <<"\n";
    push_events({{&fetch_opcode, NULL}, {&push_pch, &dec_sp}, {&push_pcl, &dec_sp},
                {&push_p_irq, &dec_sp}, {&fetch_pcl_irq, NULL}, {&fetch_pch_irq, NULL}});
}

// Clear the carry bit
void CLC (address_mode mode) {
    cpu_log << "CLC\n";
    push_events({{&clc, NULL}});
}

// Clear the decimal flag. NOTE: The decimal mode is disabled on the NES
void CLD (address_mode mode) {
    cpu_log << "CLD\n";
    push_events({{&cld, NULL}});
}

// Clear the interrupt disable flag
void CLI (address_mode mode) {
    cpu_log << "CLI\n";
    push_events({{&cli, NULL}});
}

// Clear the overflow flag
void CLV (address_mode mode) {
    cpu_log << "CLV\n";
    push_events({{&clv, NULL}});
}

// Compare the value of the memory location with the accumulator
void CMP (address_mode mode) {
    cpu_log << "CMP\n";
    push_events_read(mode, &cmp);
}

// Compare the memory with X
void CPX (address_mode mode) {
    cpu_log << "CPX\n";
    push_events_read(mode, &cpx);
}

// Compare the memory with Y
void CPY (address_mode mode) {
    cpu_log << "CPY\n";
    push_events_read(mode, &cpy);
}

// Decrement the memory location by one
void DEC (address_mode mode) {
    cpu_log << "DEC\n";
    push_events_read_mod_write(mode, &dec);
}

// Decrement the X register by one
void DEX (address_mode mode) {
    cpu_log << "DEX\n";
    push_events({{&dex, NULL}});
}

// Decrement the Y register by one
void DEY (address_mode mode) {
    cpu_log << "DEY\n";
    push_events({{&dey, NULL}});
}

// Perform an XOR on the memory location with acc, save it in acc
void EOR (address_mode mode) {
    cpu_log << "EOR\n";
    push_events_read(mode, &eor);
}

// Increment the memory location by one
void INC (address_mode mode) {
    cpu_log << "INC\n";
    push_events_read_mod_write(mode, &inc);
}

// Increment X by one
void INX (address_mode mode) {
    cpu_log << "INX\n";
    push_events({{&inx, NULL}});
}

// Increment Y by one
void INY (address_mode mode) {
    cpu_log << "INY\n";
    push_events({{&iny, NULL}});
}

// Set PC to a new location
void JMP (address_mode mode) {
    cpu_log << "JMP\n";
    switch (mode) {
        case ABS:
            push_events({{&fetch_data_pc, NULL}, {&fetch_pc_to_pch, &copy_dbus_pcl}});
            break;
        case ABSI:
            push_events({{&fetch_ladd_pc, NULL}, {&fetch_hadd_pc, NULL},
                {&fetch_data_abus, &inc_ladd_no_fix}, {&copy_dbus_pcl, &fetch_abus_to_pch}});
            break;
    }
}

// Jump to subroutine; i.e. set PC to a new location and push current state on the stack
void JSR (address_mode mode) {
    cpu_log << "JSR\n";
    push_events({{&fetch_data_pc, NULL}, {&nop, NULL}, {&push_pch, &dec_sp},
        {&push_pcl, &dec_sp}, {&fetch_pc_to_pch, &copy_dbus_pcl}});
}

// Load the accumulator with the memory from the address specified
void LDA (address_mode mode) {
    cpu_log << "LDA\n";
    push_events_read(mode, &lda);
}

// Load X with the memory from the address specified
void LDX (address_mode mode) {
    cpu_log << "LDX\n";
    push_events_read(mode, &ldx);
}

// Load Y with the memory from the address specified
void LDY (address_mode mode) {
    cpu_log << "LDY\n";
    push_events_read(mode, &ldy);
}

// Shift the accumulator right by one
void LSRA (address_mode mode) {
    cpu_log << "LSRA\n";
    push_events({{&lsr_acc, NULL}});
}

// Shift the memory location right by one
void LSR (address_mode mode) {
    cpu_log << "LSR\n";
    push_events_read(mode, &lsr_dbus);
}

// Do nothing for a cycle
void NOP (address_mode mode) {
    cpu_log << "NOP\n";
    push_events({{&nop, NULL}});
}

// Perform a bitwise OR with the memory location and accumulator
void ORA (address_mode mode) {
    cpu_log << "ORA\n";
    push_events_read(mode, &ora);
}

// Push the accumulator onto the stack
void PHA (address_mode mode) {
    cpu_log << "PHA\n";
    push_events({{&nop, NULL}, {&push_acc, &dec_sp}});
}

// Push the status register onto the stack
void PHP (address_mode mode) {
    cpu_log << "PHP\n";
    push_events({{&nop, NULL}, {&push_p_irq, &dec_sp}});
}

// Pop the accumulator from the stack
void PLA (address_mode mode) {
    cpu_log << "PLA\n";
    push_events({{&nop, NULL}, {&inc_sp, NULL}, {&pop_acc, NULL}});
}

// Pop the status register from the stack
void PLP (address_mode mode) {
    cpu_log << "PLP\n";
    push_events({{&nop, NULL}, {&inc_sp, NULL}, {&pop_p, NULL}});
}

// Rotate the accumulator left.
void ROLA (address_mode mode) {
    cpu_log << "ROLA\n";
    push_events({{&rol_acc, NULL}});
}

// Rotate the memory address left
void ROL (address_mode mode) {
    cpu_log << "ROL\n";
    push_events_read_mod_write(mode, &rol_dbus);
}

// Rotate the accumulator right
void RORA (address_mode mode) {
    cpu_log << "RORA\n";
    push_events({{&ror_acc, NULL}});
}

// Rotate the memory address right
void ROR (address_mode mode) {
    cpu_log << "ROR\n";
    push_events_read_mod_write(mode, &ror_dbus);
}

// Return from interrupt. Restores all status flags
void RTI (address_mode mode) {
    cpu_log << "RTI\n";
    push_events({{&nop, NULL}, {&inc_sp, NULL}, {&pop_p, &inc_sp},
        {&pop_pcl, &inc_sp}, {&pop_pch, NULL}});
}

// Return from subroutine. Does not restore flags, simply pops PC from the stack
void RTS (address_mode mode) {
    cpu_log << "RTS\n";
    push_events({{&nop, NULL}, {&inc_sp, NULL}, {&pop_pcl, &inc_sp},
        {&pop_pch, NULL}, {&inc_pc, NULL}});
}

// Subtract the value in memory from the accumulator, store in accumulator
void SBC (address_mode mode) {
    cpu_log << "SBC\n";
    push_events_read(mode, &sbc);
}

// Set the carry flag
void SEC (address_mode mode) {
    cpu_log << "SEC\n";
    push_events({{&sec, NULL}});
}

// Set the decimal mode flag
void SED (address_mode mode) {
    cpu_log << "SED\n";
    push_events({{&sed, NULL}});
}

// Set the interrupt disable flag
void SEI (address_mode mode) {
    cpu_log << "SEI\n";
    push_events({{&sei, NULL}});
}

// Store the value of the accumulator in the memory address
void STA (address_mode mode) {
    cpu_log << "STA\n";
    push_events_write(mode, &sta);
}

// Store X in the memory address
void STX (address_mode mode) {
    cpu_log << "STX\n";
    push_events_write(mode, &stx);
}

// Store Y in the memory address
void STY (address_mode mode) {
    cpu_log << "STY\n";
    push_events_write(mode, &sty);
}

// Transfer the accumulator to X
void TAX (address_mode mode) {
    cpu_log << "TAX\n";
    push_events({{&tax, NULL}});
}

// Transfer the accumulator to Y
void TAY (address_mode mode) {
    cpu_log << "TAY\n";
    push_events({{&tay, NULL}});
}

// Transfer SP to X
void TSX (address_mode mode) {
    cpu_log << "TSX\n";
    push_events({{&tsx, NULL}});
}

// Transfer X to the accumulator
void TXA (address_mode mode) {
    cpu_log << "TXA\n";
    push_events({{&txa, NULL}});
}

// Transfer X to the stack pointer
void TXS (address_mode mode) {
    cpu_log << "TXS\n";
    push_events({{&txs, NULL}});
}

// Transfer Y to the accumulator
void TYA (address_mode mode) {
    cpu_log << "TYA\n";
    push_events({{&tya, NULL}});
}

// Decode the currently held opcode
void decode () {

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

        case 0x0A:  ASLA(ACC);  break;
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

        case 0x00:  BRK(IMPL); std::cout << "BRK";  break;

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

        case 0xC0:  CPY(IMM);   break;
        case 0xCC:  CPY(ABS);   break;
        case 0xC4:  CPY(ZP);   	break;

        case 0xCE:  DEC(ABS);   break;
        case 0xDE:  DEC(XABS);  break;
        case 0xC6:  DEC(ZP);    break;
        case 0xD6:  DEC(XZP);   break;

        case 0xCA:  DEX(IMPL);  break;

        case 0x88:  DEY(IMPL);  break;

        case 0x49:  EOR(IMM);   break;
        case 0x4D:  EOR(ABS);   break;
        case 0x5D:  EOR(XABS);  break;
        case 0x59:  EOR(YABS);  break;
        case 0x45:  EOR(ZP);    break;
        case 0x55:  EOR(XZP);   break;
        case 0x41:  EOR(XZPI);  break;
        case 0x51:  EOR(ZPIY);  break;

        case 0xEE:  INC(ABS);   break;
        case 0xFE:  INC(XABS);  break;
        case 0xE6:  INC(ZP);    break;
        case 0xF6:  INC(XZP);   break;

        case 0xE8:  INX(IMPL);  break;

        case 0xC8:  INY(IMPL);  break;

        case 0x4C:  JMP(ABS);   break;
        case 0x6C:  JMP(ABSI);  break;

        case 0x20:  JSR(ABS);   break;

        case 0xA9:  LDA(IMM);   break;
        case 0xAD:  LDA(ABS);   break;
        case 0xBD:  LDA(XABS);  break;
        case 0xB9:  LDA(YABS);  break;
        case 0xA5:  LDA(ZP);    break;
        case 0xB5:  LDA(XZP);   break;
        case 0xA1:  LDA(XZPI);  break;
        case 0xB1:  LDA(ZPIY);  break;

        case 0xA2:  LDX(IMM);   break;
        case 0xAE:  LDX(ABS);   break;
        case 0xBE:  LDX(YABS);  break;
        case 0xA6:  LDX(ZP);    break;
        case 0xB6:  LDX(YZP);   break;

        case 0xA0:  LDY(IMM);   break;
        case 0xAC:  LDY(ABS);   break;
        case 0xBC:  LDY(XABS);  break;
        case 0xA4:  LDY(ZP);    break;
        case 0xB4:  LDY(XZP);   break;

        case 0x4A:  LSRA(ACC);  break;
        case 0x4E:  LSR(ABS);   break;
        case 0x5E:  LSR(XABS);  break;
        case 0x46:  LSR(ZP);    break;
        case 0x56:  LSR(XZP);   break;

        case 0xEA:  NOP(IMPL);  break;

        case 0x09:  ORA(IMM);   break;
        case 0x0D:  ORA(ABS);   break;
        case 0x1D:  ORA(XABS);  break;
        case 0x19:  ORA(YABS);  break;
        case 0x05:  ORA(ZP);    break;
        case 0x15:  ORA(XZP);   break;
        case 0x01:  ORA(XZPI);  break;
        case 0x11:  ORA(ZPIY);  break;

        case 0x48:  PHA(IMPL);  break;

        case 0x08:  PHP(IMPL);  break;

        case 0x68:  PLA(IMPL);  break;

        case 0x28:  PLP(IMPL);  break;

        case 0x2A:  ROLA(ACC);  break;
        case 0x2E:  ROL(ABS);   break;
        case 0x3E:  ROL(XABS);  break;
        case 0x26:  ROL(ZP);    break;
        case 0x36:  ROL(XZP);   break;

        case 0x6A:  RORA(ACC);  break;
        case 0x6E:  ROR(ABS);   break;
        case 0x7E:  ROR(XABS);  break;
        case 0x66:  ROR(ZP);    break;
        case 0x76:  ROR(XZP);   break;

        case 0x40:  RTI(IMPL);  break;

        case 0x60:  RTS(IMPL);  break;

        case 0xE9:  SBC(IMM);   break;
        case 0xED:  SBC(ABS);   break;
        case 0xFD:  SBC(XABS);  break;
        case 0xF9:  SBC(YABS);  break;
        case 0xE5:  SBC(ZP);    break;
        case 0xF5:  SBC(XZP);   break;
        case 0xE1:  SBC(XZPI);  break;
        case 0xF1:  SBC(ZPIY);  break;

        case 0x38:  SEC(IMPL);  break;

        case 0xF8:  SEC(IMPL);  break;

        case 0x78:  SEI(IMPL);  break;

        case 0x8D:  STA(ABS);   break;
        case 0x9D:  STA(XABS);  break;
        case 0x99:  STA(YABS);  break;
        case 0x85:  STA(ZP);    break;
        case 0x95:  STA(XZP);   break;
        case 0x81:  STA(XZPI);  break;
        case 0x91:  STA(ZPIY);  break;

        case 0x8E:  STX(ABS);   break;
        case 0x86:  STX(ZP);    break;
        case 0x96:  STX(YZP);   break;

        case 0x8C:  STY(ABS);   break;
        case 0x84:  STY(ZP);    break;
        case 0x94:  STY(YZP);   break;

        case 0xAA:  TAX(IMPL);  break;

        case 0xA8:  TAY(IMPL);  break;

        case 0xBA:  TSX(IMPL);  break;

        case 0x8A:  TXA(IMPL);  break;

        case 0x9A:  TXS(IMPL);  break;

        case 0x98:  TYA(IMPL);  break;

        default:    NOP(IMPL);  break;
	}
}

