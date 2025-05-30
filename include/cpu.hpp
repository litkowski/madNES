#ifndef CPU_H
#define CPU_H

#define CARRY_FLAG 0b00000001
#define ZERO_FLAG 0b00000010
#define INTERRUPT_FLAG 0b00000100
#define DECIMAL_FLAG 0b00001000
#define B_FLAG 0b00010000
#define EXTRA_FLAG 0b00100000
#define OVERFLOW_FLAG 0b01000000
#define NEGATIVE_FLAG 0b10000000

#include <deque>
#include <array>

#include "mappers.hpp"

class CPU {
	private:

		Cartridge* game;

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

		// All 6502 registers and buses
		uint8_t acc, x, y, sp, status, opcode, dbus;
		uint8_t pcl;
		uint16_t pc, abus;

		// Queue to store atomic events in.
		// NOTE: Each CPU cycle includes a maximum of two actions, and thus two function
		// pointers occupy a single space in the queue
		std::deque<std::array<void (CPU::*) (), 2>> event_queue;

		void push_events_read (address_mode mode, void (CPU::*func) ());
		void push_events_read_mod_write (address_mode mode, void (CPU::*func) ());
		void push_events_write (address_mode mode, void (CPU::*func) ());

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
		void push_p ();
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
		void push_events (std::vector<std::array<void (CPU::*) (), 2>> events);
	public:
		CPU (Cartridge* game_cartridge);
		void cycle ();
};

#endif
