#include "cpu.hpp"

CPU::CPU (Cartridge* game) {

	this->game = game;

	// Assign all registers to their startup states
	acc = x = y = 0;
	pc = 0xFFFC;
	sp = 0xFD;
	status = 0b00100100;
}

// Helper function to push a range of atomic operations to the event queue
// NOTE: Adds fetch and decode operations to the queue,
// ONLY USE when adding a new instruction!!!
void CPU::push_events (std::vector<std::array<void (CPU::*) (), 2>> events) {
	for (int i = 0; i < events.size(); i++) {
		event_queue.push_back(events[i]);
	}
	event_queue.push_back({&CPU::fetch_opcode, &CPU::decode});
}

/*
 * Execute one CPU cycle.
 */
 void CPU::cycle () {

    (this->*event_queue.front()[0])();

    if (event_queue.front()[1] != NULL) {
        (this->*event_queue.front()[1])();
    }

    event_queue.pop_front();
}

// Do nothing
void CPU::nop () {
}

// Fetch the data at the address bus's pointer into the data bus
void CPU::fetch_data_abus () {
	dbus = game->cpu_read(abus);
}

// Fetch the data at the program counter into the data bus
void CPU::fetch_data_pc () {
	dbus = game->cpu_read(pc);
	pc++;
}

// Fetch the data at the program counter into the low byte of the address bus
void CPU::fetch_ladd_pc () {
	dbus = game->cpu_read(pc);
	abus = 0;
	abus |= dbus;
}

// Fetch the high address of the address bus
void CPU::fetch_hadd_pc () {
	dbus = game->cpu_read(pc);
	abus &= 0x00FF;
	abus |= (dbus << 8);
}

// Fetch the data at the abus into the low byte of the address bus
void CPU::fetch_ladd_abus () {
	dbus = game->cpu_read(abus);
	abus = 0;
	abus |= dbus;
}

// Fetch the data at the abus into the high byte of the address bus
void CPU::fetch_hadd_abus () {
	dbus = game->cpu_read(abus);
	abus &= 0x00FF;
	abus |= (dbus << 8);
}


// Fetch next opcode from PC
void CPU::fetch_opcode () {
	opcode = dbus = game->cpu_read(pc);
	pc++;
}

void CPU::write_data_abus () {
    game->cpu_write(abus, dbus);
}

/*
 * Add the contents of the data bus to the low bytes of PC
 * NOTE: If there is a page crossing, an additional cycle will be added
 */
void CPU::add_dbus_pcl () {

    uint8_t pcl = (uint8_t) pc & 0xFF;
    pcl += dbus;

    if (pc & 0xFF > pcl) {
        status |= CARRY_FLAG;
        event_queue.push_front({&CPU::fetch_opcode, &CPU::fix_pch});
    }

    pc &= 0xFF00;
    pc |= pcl;
}

/*
 * Add the contents of the X register to the low byte of the address bus
 * NOTE: This version of the function will NOT skip if fixing is unnecessary
 */
void CPU::add_x_ladd () {

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
 void CPU::add_x_ladd_skip () {

	uint8_t ladd = (uint8_t) abus;
	abus &= 0xFF00;

	if (ladd + x > 0x00FF) {
		status |= CARRY_FLAG;
        event_queue.push_front({&CPU::fetch_data_abus, &CPU::fix_add});
	}

	ladd += x;
	abus |= ladd;
}

/*
 * Add the contents of the Y register to the low byte of the address bus
 * NOTE: This version of the function will NOT skip if fixing is unnecessary
 */
void CPU::add_y_ladd () {

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
 void CPU::add_y_ladd_skip () {

	uint8_t ladd = (uint8_t) abus;
	abus &= 0xFF00;

	if (ladd + y > 0x00FF) {
		status |= CARRY_FLAG;
        event_queue.push_front({&CPU::fetch_data_abus, &CPU::fix_add});
	}

	ladd += y;
	abus |= ladd;
}

// Add the contents of the X register to a zero page address
void CPU::add_x_zp () {
	abus &= 0xFF;
	uint8_t* ladd = (uint8_t*) &abus;
	*ladd += x;
}

// Add the contents of the Y register to a zero page address
void CPU::add_y_zp () {
	abus &= 0xFF;
	uint8_t* ladd = (uint8_t*) &abus;
	*ladd += y;
}

// Fix the high byte of the address bus.
// Only executed in cases of page crossings.
void CPU::fix_add () {
    if (status &= CARRY_FLAG) {
        abus += 0x100;
    }
}

// Fix the high byte of the program counter.
// Only executed in cases of page crossings.
void CPU::fix_pch () {
    if (status &= CARRY_FLAG) {
        pc += 0x100;
    }
}

// Increment the abus by one
void CPU::inc_abus () {
	abus++;
}

// Decrement the stack pointer by one byte
void CPU::dec_sp () {
    sp--;
}

// Increment the stack pointer by one byte
void CPU::inc_sp () {
    sp++;
}

// Push the high byte of PC onto the stack
void CPU::push_pch () {
    game->cpu_write(sp, (uint8_t) (pc & 0xFF00) >> 8);
}

// Pop the high byte of PC from the stack TODO
void CPU::pop_pch () {

}

// Push the low byte of PC onto the stack
void CPU::push_pcl () {
    game->cpu_write(sp, (uint8_t) (pc & 0x00FF));
}

// Pop the low byte of PC from the stack TODO
void CPU::pop_pcl () {

}

// Push the status register to the stack with the B flag set
void CPU::push_p () {
    game->cpu_write(sp, status | B_FLAG | EXTRA_FLAG);
}

// Pop the status register from the stack
void CPU::pop_p () {
    p = game->cpu_read(sp - 1);
}

// Fetch the low byte of the IRQ interrupt handler
void CPU::fetch_pcl_irq  () {
    dbus = game->cpu_read(0xFFFE);
}

// Fetch the high byte of the IRQ interrupt handler
void CPU::fetch_pch_irq () {
    dbus = game->cpu_read(0xFFFF);
}

// Fetch the low byte of the NMI interrupt handler
void CPU::fetch_pcl_nmi () {
    dbus = game->cpu_read(0xFFFA);
}

// Fetch the high byte of the NMI interrupt handler
void CPU::fetch_pch_nmi () {
    dbus = game->cpu_read(0xFFFB);
}

// Copy the dbus to the low byte of PC
void CPU::copy_dbus_pcl () {
    pc &= 0xFF00;
    pc |= dbus;
}

// Fetch data from PC to the high byte of PC
void CPU::fetch_pc_to_pch () {
    uint8_t pch = game->cpu_read(pc) << 8;
    pc &= 0x00FF;
    pc |= pch << 8;
}

// Fetch data from the abus's address to the low byte of PC
void CPU::fetch_abus_to_pch () {
    pc &= 0x00FF;
    pc |= game->cpu_read(abus) << 8;
}

// Increment the PC by one
void CPU::inc_pc () {
    pc++;
}

// Increment the low address of the address bus, do not fix it
void CPU::inc_ladd_no_fix () {
    uint8_t ladd = 0x00FF & abus;
    ladd++;
    abus &= 0xFF00;
    abus |= ladd;
}

// Set zero and negative flags depending on the result
// NOTE: Used in a variety of atomic operations on many registers
void CPU::set_zero_and_neg (uint8_t result) {
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
void CPU::adc_acc () {
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
void CPU::and_acc () {
    acc &= dbus;
    set_zero_and_neg(acc);
}

/*
 * Shift the accumulator left one bit.
 * Save the 7th bit in the carry flag
 */
void CPU::asl_acc () {

    if (acc & 0b1000000) {
        status |= CARRY_FLAG;
    } else {
        status &= ~CARRY_FLAG;
    }

    acc << 1;
    acc &= 0b11111110;
}

/*
 * Shift the data bus left one bit.
 * Save the 7th bit in the carry flag
 */
void CPU::asl_dbus () {

    if (dbus & 0b10000000) {
        status |= CARRY_FLAG;
    } else {
        status &= ~CARRY_FLAG;
    }

    dbus << 1;
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
void CPU::bcc () {
    if (!(status & CARRY_FLAG)) {
        event_queue.push_front({&CPU::fetch_opcode, &CPU::add_dbus_pcl});
    }
}

void CPU::bcs () {
    if (status & CARRY_FLAG) {
        event_queue.push_front({&CPU::fetch_opcode, &CPU::add_dbus_pcl});
    }
}

void CPU::beq () {
    if (status & ZERO_FLAG) {
        event_queue.push_front({&CPU::fetch_opcode, &CPU::add_dbus_pcl});
    }
}

void CPU::bmi () {
    if (status & NEGATIVE_FLAG) {
        event_queue.push_front({&CPU::fetch_opcode, &CPU::add_dbus_pcl});
    }
}

void CPU::bne () {
    if (!(status & ZERO_FLAG)) {
        event_queue.push_front({&CPU::fetch_opcode, &CPU::add_dbus_pcl});
    }
}

void CPU::bpl () {
    if (!(status & ZERO_FLAG)) {
        event_queue.push_front({&CPU::fetch_opcode, &CPU::add_dbus_pcl});
    }
}

void CPU::bvc () {
    if (!(status & OVERFLOW_FLAG)) {
        event_queue.push_front({&CPU::fetch_opcode, &CPU::add_dbus_pcl});
    }
}

void CPU::bvs () {
    if (status & OVERFLOW_FLAG) {
        event_queue.push_front({&CPU::fetch_opcode, &CPU::add_dbus_pcl});
    }
}


// AND the data in the dbus with the accumulator. If the result is zero,
// set the zero flag. If not, reset the flag. No matter what, copy the
// 6th and 7th bits of the memory address into the status register
void CPU::bit () {
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
void CPU::clc () {
    status &= ~CARRY_FLAG;
}

// Clear the decimal flag (disabled on the NES)
void CPU::cld () {
    status &= ~DECIMAL_FLAG;
}

// Clear the interrupt disable flag
void CPU::cli () {
    status &= ~INTERRUPT_FLAG;
}

// Clear the overflow flag
void CPU::clv () {
    status &= ~OVERFLOW_FLAG;
}

// Compare the value of the specified location with the accumulator, set the status flags accordingly
void CPU::cmp () {
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
void CPU::cpx () {
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

// Compare the value of the specified location with the Y register, set the status flags accordingly
void CPU::cpy () {
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

// Decrement the dbus by one, set flags accordingly
void CPU::dec () {
    dbus -= 1;
    set_zero_and_neg(dbus);
}

// Decrement X by one, set flags accordingly
void CPU::dex () {
    x -= 1;
    set_zero_and_neg(x);
}

// Decrement Y by one, set flags accordingly
void CPU::dey () {
    y -= 1;
    set_zero_and_neg(y);
}

// Perform an XOR on the accumulator and dbus, store in acc
void CPU::eor () {
    acc ^= dbus;
    set_zero_and_neg(acc);
}

// Increment the dbus by one
void CPU::inc () {
    dbus += 1;
    set_zero_and_neg(dbus);
}

// Increment X by one
void CPU::inx () {
    x += 1;
    set_zero_and_neg(x);
}

// Increment Y by one
void CPU::iny () {
    y += 1;
    set_zero_and_neg(y);
}

// Load the accumulator with the memory in dbus, set flags accordingly
void CPU::lda () {
    acc  = dbus;
    set_zero_and_neg(acc);
}

// Load X with the memory in dbus, set flags accordingly
void CPU::ldx () {
    x = dbus;
    set_zero_and_neg(x);
}

// Shift the accumulator right one bit. Store the low bit in the carry flag.
// Reset the negative flag.
void CPU::lsr_acc () {
    if (acc & 0x01) {
        status |= CARRY_FLAG;
    } else {
        status &= ~CARRY_FLAG;
    }

    acc >> 1;
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
void CPU::lsr () {
    if (dbus & 0x01) {
        status |= CARRY_FLAG;
    } else {
        status &= ~CARRY_FLAG;
    }

    dbus >> 1;
    dbus &= 0b01111111;
    if (!dbus) {
        status |= ZERO_FLAG;
    } else {
        status &= ~ZERO_FLAG;
    }

    status &= ~NEGATIVE_FLAG;
}

// Perform a bitwise OR with the dbus and accumulator
void CPU::ora () {
    acc = dbus | acc;
    set_zero_and_neg(acc);
}

// Push the accumulator onto the stack
void CPU::push_acc () {
    game->cpu_write(sp, acc);
}

// Pop the accumulator from the stack
void CPU::pop_acc () {
    acc = game->cpu_read(sp - 1);
}

// Rotate the accumulator left; i.e. store the 7th bit in the carry flag,
// store the initial carry flag in the 0th bit, and shift left one bit.
void CPU::rol_acc () {
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
void CPU::rol_acc () {
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
void CPU::ror_acc () {
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
void CPU::ror_dbus () {
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

// TODO
void CPU::sbc () {

}

// TODO
void CPU::sec () {

}

// TODO
void CPU::sed () {

}

// TODO
void CPU::sei () {

}

/* NOTE: Helper functions for generic instruction types begin here */

// Generic read type function
void push_events_read (address_mode mode, void (CPU::*func) ()) {
    switch (mode) {
        case IMM:
			push_events({{&CPU::fetch_data_pc, func}});
			break;
		case ABS:
			push_events({{&CPU::fetch_ladd_pc, NULL}, {&CPU::fetch_hadd_pc, NULL},
				{&CPU::fetch_data_abus, func}});
			break;
		case XABS:
			push_events({{&CPU::fetch_ladd_pc, NULL}, {&CPU::fetch_hadd_pc, &CPU::add_x_ladd_skip},
				{&CPU::fetch_data_abus, func}});
			break;
		case YABS:
			push_events({{&CPU::fetch_ladd_pc, NULL}, {&CPU::fetch_hadd_pc, &CPU::add_y_ladd_skip},
				{&CPU::fetch_data_abus, &CPU::fix_add}, {&CPU::fetch_data_abus, func}});
			break;
		case ZP:
			push_events({{&CPU::fetch_ladd_pc, NULL}, {&CPU::fetch_data_abus, func}});
			break;
		case XZP:
			push_events({{&CPU::fetch_ladd_pc, NULL}, {&CPU::fetch_data_abus, &CPU::add_x_zp},
				{&CPU::fetch_data_abus, func}});
			break;
        case YZP:
            push_events({{&CPU::fetch_ladd_pc, NULL}, {&CPU::fetch_data_abus, &CPU::add_y_zp},
				{&CPU::fetch_data_abus, func}});
			break;
		case XZPI:
			push_events({{&CPU::fetch_ladd_pc, NULL}, {&CPU::fetch_ladd_abus, &CPU::add_x_zp},
				{&CPU::fetch_ladd_abus, &CPU::inc_abus}, {&CPU::fetch_hadd_abus, NULL},
				{&CPU::fetch_data_abus, func}});
		case ZPIY:
			push_events({{&CPU::fetch_ladd_pc, NULL}, {&CPU::fetch_ladd_abus, &CPU::inc_abus},
				{&CPU::fetch_hadd_abus, &CPU::add_x_ladd}, {&CPU::fetch_data_abus, &CPU::fix_add},
				{&CPU::fetch_data_abus, func}});
    }
}

// Generic read-modify-write type function
void push_events_read_mod_write (address_mode mode, void (CPU::*func) ()) {
    switch (mode) {
        case ABS:
            push_events({{&CPU::fetch_ladd_pc, NULL}, {&CPU::fetch_hadd_pc, NULL},
                {&CPU::fetch_data_abus, &CPU::asl_dbus}, {&CPU::write_data_abus, func},
                {&CPU::write_data_abus, NULL}});
            break;
        case XABS:
            push_events({{&CPU::fetch_ladd_pc, NULL}, {&CPU::fetch_ladd_pc, &CPU::add_x_ladd},
                {&CPU::fetch_data_abus, &CPU::fix_add}, {&CPU::fetch_data_abus, NULL},
                {&CPU::write_data_abus, func}, {&CPU::write_data_abus, NULL}});
            break;
        case ZP:
            push_events({{&CPU::fetch_ladd_pc, NULL}, {&CPU::fetch_data_abus, NULL},
                {&CPU::write_data_abus, func}, {&CPU::write_data_abus, NULL}});
            break;
        case XZP:
            push_events({{&CPU::fetch_ladd_pc, NULL}, {&CPU::fetch_data_abus, &CPU::add_x_zp},
                {&CPU::fetch_data_abus, NULL}, {&CPU::write_data_abus, func},
                {&CPU::write_data_abus, NULL}});
            break;
    }
}

/* NOTE: Opcode handlers begin here */

// Add the data, as specified through addressing, to the accumulator
void CPU::ADC (address_mode mode) {
    push_events_read(mode, &CPU::adc_acc);
}

// AND the specified data to the accumulator
void CPU::AND (address_mode mode) {
    push_events_read(mode, &CPU::and_acc);
}

// Shift the accumulator left one bit
void CPU::ASLA (address_mode mode) {
    push_events({{&CPU::asl_acc, NULL}});
}

// Shift the data left by one bit
void CPU::ASL (address_mode mode) {
    push_events_read_mod_write(mode, &CPU::asl_dbus);
}

/* BRANCH INSTRUCTIONS */
void CPU::BCC (address_mode mode) {
    push_events({{&CPU::fetch_data_pc, &CPU::bcc}});
}

void CPU::BCS (address_mode mode) {
    push_events({{&CPU::fetch_data_pc, &CPU::bcs}});
}

void CPU::BEQ (address_mode mode) {
    push_events({{&CPU::fetch_data_pc, &CPU::beq}});
}

void CPU::BMI (address_mode mode) {
    push_events({{&CPU::fetch_data_pc, &CPU::bmi}});
}

void CPU::BNE (address_mode mode) {
    push_events({{&CPU::fetch_data_pc, &CPU::bne}});
}

void CPU::BPL (address_mode mode) {
    push_events({{&CPU::fetch_data_pc, &CPU::bpl}});
}

void CPU::BVC (address_mode mode) {
    push_events({{&CPU::fetch_data_pc, &CPU::bvc}});
}

void CPU::BVS (address_mode mode) {
    push_events({{&CPU::fetch_data_pc, &CPU::bvs}});
}

// AND the memory location with the accumulator, but do not save the result
void CPU::BIT (address_mode mode) {
    case ABS:
        push_events({{&CPU::fetch_ladd_pc, NULL}, {&CPU::fetch_hadd_pc, NULL}, {&CPU::fetch_data_abus, &CPU::bit_acc}});
    case ZP:
        push_events({{&CPU::fetch_ladd_pc, NULL}, {&CPU::fetch_data_abus, &CPU::bit_acc}});
}

// Undergo a voluntary interrupt. Pushes PC to the stack first, then status,
// and finally reads the handler address from 0xFFFE - 0xFFFF
void CPU::BRK (address_mode mode) {
    push_events({{&CPU::fetch_opcode, NULL}, {&CPU::push_pch, &CPU::dec_sp}, {&CPU::push_pcl, &CPU::dec_sp},
                {&CPU::push_p, &CPU::dec_sp}, {&CPU::fetch_pcl_irq, NULL}, {&CPU::fetch_pch_irq, NULL});
}

// Clear the carry bit
void CPU::CLC (address_mode mode) {
    push_events({{&CPU::fetch_ladd_pc, &CPU::clc}});
}

// Clear the decimal flag. NOTE: The decimal mode is disabled on the NES
void CPU::CLD (address_mode mode) {
    push_events({&CPU::fetch_ladd_pc, &CPU::cld});
}

// Clear the interrupt disable flag
void CPU::CLI (address_mode mode) {
    push_events({&CPU::fetch_ladd_pc, &CPU::cli});
}

// Clear the overflow flag
void CPU::CLV (address_mode mode) {
    push_events({&CPU::fetch_ladd_pc, &CPU::clv});
}

// Compare the value of the memory location with the accumulator
void CPU::CMP (address_mode mode) {
    push_events_read(mode, &CPU::cmp);
}

// Compare the memory with X
void CPU::CPX (address_mode mode) {
    push_events_read(mode, &CPU::cpx);
}

// Compare the memory with Y
void CPU::CPY (address_mode mode) {
    push_events_read(mode, &CPU::cpy);
}

// Decrement the memory location by one
void CPU::DEC (address_mode mode) {
    push_events_read_mod_write(mode, &CPU::dec);
}

// Decrement the X register by one
void CPU::DEX (address_mode mode) {
    push_events({{&CPU::dex, NULL}});
}

// Decrement the Y register by one
void CPU::DEY (address_mode mode) {
    push_events({{&CPU::dey, NULL}});
}

// Perform an XOR on the memory location with acc, save it in acc
void CPU::EOR (address_mode mode) {
    push_events_read(mode, &CPU::eor);
}

// Increment the memory location by one
void CPU::INC (address_mode mode) {
    push_events_read_mod_write(mode, &CPU::inc);
}

// Increment X by one
void CPU::INX (address_mode mode) {
    push_events({{&CPU::inx, NULL}});
}

// Increment Y by one
void CPU::INY (address_mode mode) {
    push_events({{&CPU::iny, NULL}});
}

// Set PC to a new location
void CPU::JMP (address_mode mode) {
    switch (mode) {
        case ABS:
            push_events({{&CPU::fetch_data_pc, NULL}, {&CPU::fetch_pc_to_pch, &CPU::copy_dbus_pcl}});
        case ABSI:
            push_events({{&CPU::fetch_ladd_pc, &CPU::inc_pc}, {&CPU::fetch_hadd_pc, &CPU::inc_pc},
                {&CPU::fetch_data_abus, &CPU::inc_ladd_no_fix}, {&CPU::copy_dbus_pcl, &CPU::fetch_abus_to_pch}})
    }
}

// Jump to subroutine; i.e. set PC to a new location and push current state on the stack
void CPU::JSR (address_mode mode) {
    push_events({{&CPU::fetch_data_pc, NULL}, {&CPU::nop, NULL}, {&CPU::push_pch, &CPU::dec_sp}
        {&CPU::push_pcl, &CPU::dec_sp}, {&CPU::fetch_pc_to_pch, &CPU::copy_dbus_pcl}});
}

// Load the accumulator with the memory from the address specified
void CPU::LDA (address_mode mode) {
    push_events_read(mode, &CPU::lda);
}

// Load X with the memory from the address specified
void CPU::LDX (address_mode mode) {
    push_events_read(mode, &CPU::ldx);
}

// Load Y with the memory from the address specified
void CPU::LDY (address_mode mode) {
    push_events_read(mode, &CPU::ldy);
}

// Shift the accumulator right by one
void CPU::LSRA (address_mode mode) {
    push_events({{&CPU::lsr_acc, NULL}});
}

// Shift the memory location right by one
void CPU::LSR (address_mode mode) {
    push_events_read(mode, &CPU::lsr_dbus);
}

// Do nothing for a cycle
void CPU::NOP (address_mode mode) {
    push_events({{&CPU::nop, NULL}});
}

// Perform a bitwise OR with the memory location and accumulator
void CPU::ORA (address_mode mode) {
    push_events_read(mode, &CPU::ora);
}

// Push the accumulator onto the stack
void CPU::PHA (address_mode mode) {
    push_events({{&CPU::nop, NULL}, {&CPU::push_acc, &CPU::dec_sp}});
}

// Push the status register onto the stack
void CPU::PHP (address_mode mode) {
    push_events({{&CPU::nop, NULL}, {&CPU::push_p, &CPU::dec_sp}});
}

// Pop the accumulator from the stack
void CPU::PLA (address_mode mode) {
    push_events({{&CPU::nop, NULL}, {&CPU::inc_sp, NULL}, {&CPU::pop_acc, NULL}});
}

// Pop the status register from the stack
void CPU::PLP (address_mode mode) {
    push_events({{&CPU::nop, NULL}, {&CPU::inc_sp, NULL}, {&CPU::pop_p, NULL}});
}

// NOTE: Rotation is specified in the rol_acc/dbus and ror_acc/dbus functions
// Rotate the accumulator left.
void CPU::ROLA (address_mode mode) {
    push_events({&CPU::rol_acc, NULL});
}

// Rotate the memory address left
void CPU::ROL (address_mode mode) {
    push_events_read_mod_write(mode, &CPU::rol_dbus);
}

// Rotate the accumulator right
void CPU::RORA (address_mode mode) {
    push_events({&CPU::ror_acc, NULL});
}

// Rotate the memory address right
void CPU::ROR (address_mode mode) {
    push_events_read_mod_write(mode, &CPU::ror_dbus);
}

// Return from interrupt. Restores all status flags TODO
void CPU::RTI () {

}

// Return from subroutine. Does not restore flags, simply pops PC from the stack TODO
void CPU::RTS (address_mode mode) {
}

void CPU::SBC (address_mode mode) {
}

// Set the carry flag
void CPU::SEC (address_mode mode) {
    push_events({{&CPU::sec, NULL});
}

// Set the decimal mode flag
void CPU::SED (address_mode mode) {
    push_events({{&CPU::sed, NULL});
}

// Set the interrupt disable flag
void CPU::SEI (address_mode mode) {
    push_events({{&CPU::sei, NULL});
}

void CPU::STA (address_mode mode) {
}

void CPU::STX (address_mode mode) {
}

void CPU::STY (address_mode mode) {
}

void CPU::TAX (address_mode mode) {
}

void CPU::TAY (address_mode mode) {
}

void CPU::TSX (address_mode mode) {
}

void CPU::TXA (address_mode mode) {
}

void CPU::TXS (address_mode mode) {
}

void CPU::TYA (address_mode mode) {
}


// Decode the currently held opcode
void CPU::decode () {
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

