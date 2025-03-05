#include "cpu.hpp"
#define CARRY_FLAG 0b00000001

CPU::CPU (Cartridge* game) {

	this->game = game;

	// Assign all registers to their startup states
	a = x = y = 0;
	pc = 0xFFFC;
	sp = 0xFD;
	status = 0b00100100;

	// Initialize the event queue to fetch and decode the first instruction
	event_queue.push({&CPU::fetch_opcode, &CPU::decode});
}



// Execute one CPU cycle
void CPU::cycle () {
	(this->*event_queue.front()[0])();

	if (event_queue.front()[1] != NULL) {
		(this->*event_queue.front()[1])();
	}

	event_queue.pop();
}



// Add the contents of the data bus to the accumulator, with carry
void CPU::adc_acc () {

	if (a + dbus < a) {
		status |= CARRY_FLAG;
	}

	a += dbus;
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

// Add the contents of the X register to the low byte of the address bus
void CPU::add_x_ladd () {

	uint8_t ladd = (uint8_t) abus;
	abus &= 0xFF00;

	if (ladd + x > 0xFF) {
		status |= CARRY_FLAG;
	} else {
		event_queue.pop();
	}

	ladd += x;
	abus |= ladd;
}

// Add the contents of the Y register to the low byte of the address bus
void CPU::add_y_ladd () {

	uint8_t ladd = (uint8_t) abus;
	abus &= 0xFF00;

	if (ladd + y > 0xFF) {
		status |= CARRY_FLAG;
	} else {
		event_queue.pop();
	}

	ladd += y;
	abus |= ladd;
}

// Add the contents of the X register to a zero page address
void CPU::add_x_zp () {

	abus &= 0xFF;
	uint8_t ladd = (uint8_t) (abus);
	ladd += x;

}

// Fix the high byte of the address bus.
// Only executed in cases of page crossings.
void CPU::fix_add () {
	abus += 0x100;
}

// Increment the abus by one
void CPU::inc_abus () {
	abus++;
}

// Add the data, as specified through addressing, to the accumulator
void CPU::ADC (address_mode mode) {
	switch (mode) {
		case IMM:
			push_events({{&CPU::fetch_data_pc, &CPU::adc_acc}, {&CPU::fetch_opcode, &CPU::decode}});
			break;
		case ABS:
			push_events({{&CPU::fetch_ladd_pc, NULL}, {&CPU::fetch_hadd_pc, NULL},
				{&CPU::fetch_data_abus, &CPU::adc_acc}, {&CPU::fetch_opcode, &CPU::decode}});
			break;
		case XABS:
			push_events({{&CPU::fetch_ladd_pc, NULL}, {&CPU::fetch_hadd_pc, &CPU::add_x_ladd},
				{&CPU::fetch_data_abus, &CPU::fix_add}, {&CPU::fetch_data_abus, &CPU::adc_acc},
				{&CPU::fetch_opcode, &CPU::decode}});
			break;
		case YABS:
			push_events({{&CPU::fetch_ladd_pc, NULL}, {&CPU::fetch_hadd_pc, &CPU::add_y_ladd},
				{&CPU::fetch_data_abus, &CPU::fix_add}, {&CPU::fetch_data_abus, &CPU::adc_acc},
				{&CPU::fetch_opcode, &CPU::decode}});
			break;
		case ZP:
			push_events({{&CPU::fetch_ladd_pc, NULL}, {&CPU::fetch_opcode, &CPU::decode}});
			break;
		case XZP:
			push_events({{&CPU::fetch_ladd_pc, NULL}, {&CPU::fetch_data_abus, &CPU::add_x_zp},
				{&CPU::fetch_data_abus, &CPU::adc_acc}, {&CPU::fetch_opcode, &CPU::decode}});
			break;
		case XZPI:
			push_events({{&CPU::fetch_ladd_pc, NULL}, {&CPU::fetch_ladd_abus, &CPU::add_x_zp},
				{&CPU::fetch_ladd_abus, &CPU::inc_abus}, {&CPU::fetch_hadd_abus, NULL},
				{&CPU::fetch_data_abus, &CPU::adc_acc}});
		case ZPIY:
			push_events({{&CPU::fetch_ladd_pc, NULL}, {&CPU::fetch_ladd_abus, &CPU::inc_abus},
				{&CPU::fetch_hadd_abus, &CPU::add_x_ladd}, {&CPU::fetch_data_abus, &CPU::fix_add},
				{&CPU::fetch_data_abus, &CPU::adc_acc}, {&CPU::fetch_opcode, &CPU::decode}});
	}
}

void CPU::AND (address_mode mode) {
}

void CPU::ASL (address_mode mode) {
}

void CPU::BCC (address_mode mode) {
}

void CPU::BCS (address_mode mode) {
}

void CPU::BEQ (address_mode mode) {
}

void CPU::BIT (address_mode mode) {
}

void CPU::BMI (address_mode mode) {
}

void CPU::BNE (address_mode mode) {
}

void CPU::BPL (address_mode mode) {
}

void CPU::BRK (address_mode mode) {
}

void CPU::BVC (address_mode mode) {
}

void CPU::BVS (address_mode mode) {
}

void CPU::CLC (address_mode mode) {
}

void CPU::CLD (address_mode mode) {
}

void CPU::CLI (address_mode mode) {
}

void CPU::CLV (address_mode mode) {
}

void CPU::CMP (address_mode mode) {
}

void CPU::CPX (address_mode mode) {
}

void CPU::CPY (address_mode mode) {
}

void CPU::DEC (address_mode mode) {
}

void CPU::DEX (address_mode mode) {
}

void CPU::DEY (address_mode mode) {
}

void CPU::EOR (address_mode mode) {
}

void CPU::INC (address_mode mode) {
}

void CPU::INX (address_mode mode) {
}

void CPU::INY (address_mode mode) {
}

void CPU::JMP (address_mode mode) {
}

void CPU::JSR (address_mode mode) {
}

void CPU::LDA (address_mode mode) {
}

void CPU::LDX (address_mode mode) {
}

void CPU::LDY (address_mode mode) {
}

void CPU::LSR (address_mode mode) {
}

void CPU::NOP (address_mode mode) {
}

void CPU::ORA (address_mode mode) {
}

void CPU::PHA (address_mode mode) {
}

void CPU::PHP (address_mode mode) {
}

void CPU::PLA (address_mode mode) {
}

void CPU::PLP (address_mode mode) {
}

void CPU::ROL (address_mode mode) {
}

void CPU::ROR (address_mode mode) {
}

void CPU::RTI (address_mode mode) {
}

void CPU::RTS (address_mode mode) {
}

void CPU::SBC (address_mode mode) {
}

void CPU::SEC (address_mode mode) {
}

void CPU::SED (address_mode mode) {
}

void CPU::SEI (address_mode mode) {
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

        case 0x4A:  LSR(ACC);   break;
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

        case 0x2A:  ROL(ACC);   break;
        case 0x2E:  ROL(ABS);   break;
        case 0x3E:  ROL(XABS);  break;
        case 0x26:  ROL(ZP);    break;
        case 0x36:  ROL(XZP);   break;

        case 0x6A:  ROR(ACC);   break;
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

// Helper function to push a range of atomic operations to the event queue
void CPU::push_events (std::vector<std::array<void (CPU::*) (), 2>> events) {
	for (int i = 0; i < events.size(); i++) {
		event_queue.push(events[i]);
	}
}
