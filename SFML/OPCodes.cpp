#include "CPU.h"
// JR, N - skok relatywny bezwarunkowy o N
void CPU::jump_n(){
	int8_t signedn = static_cast<int8_t>((Memory.readMemory(this->pc)));
	this->pc++;
	this->pc += signedn;
	cycles += 12;
}
// JR Z, n - jump o n jest Z=1
void CPU::jump_zero(){
	if (getBit(FLAG_Z) == 1) {
		jump_n();
		return;
	}
	pc++;
	cycles += 8;
}
// JR C,n  - jump o n jest C=1
void CPU::jump_carry(){
	if (getBit(FLAG_C) == 1) {
		jump_n();
		return;
	}
	pc++;
	cycles += 8;
}
// JR NZ, n - jump o n jest Z=0
void CPU::jump_notzero(){
	if (getBit(FLAG_Z) == 0) {
		jump_n();
		return;
	}
	pc++;
	cycles += 8;
}
// JR, NC - jump o n jest C=0
void CPU::jump_notcarry(){
	if (getBit(FLAG_C) == 0) {
		jump_n();
		return;
	}
	pc++;
	cycles += 8;
}
// JP nn - jump bezwarunkwoy o nn
void CPU::jump_abs(){
	uint16_t jump = readTwoBytes();
	this->pc = jump;
	cycles += 16;
}
// JP NZ, nn - jump jesli Z=0
void CPU::jump_absFalse(int flag){
	if (getBit(flag) == 0) {
		jump_abs();
		return;
	}
	pc += 2;
}
// JP Z, nn - jump jesli Z=0
void CPU::jump_absTrue(int flag){
	if (getBit(flag)) {
		jump_abs();
		return;
	}
	pc += 2;
}
// JP (HL) - jump do lokacji w pamieci
void CPU::jump_mmu(uint16_t reg){
	this->pc = reg;
	cycles += 4;
}
// LD A - zaladowanie A
void CPU::mmu_loadA(){
	registerAF.hi = Memory.readMemory((0xFF00) + registerBC.lo);
	cycles += 8;
}
// LDI (HL), A - Load location (DE) with location (HL), incr DE,HL; decr BC.
void CPU::mmu_ldi(uint16_t& address, uint8_t& data){
	mmu_load8(address, data);
	incr_reg(address);

}
// LDD (HL), A -  Load location (DE) with location (HL), decrement DE,HL,BC.
void CPU::mmu_ldd(uint16_t& address, uint8_t  data){
	mmu_load8(address, data);
	decr_reg(address);
}
// LD (BC), A - akumulator do lokacji (BC)
void CPU::mmu_load8(uint16_t address, uint8_t data){
	Memory.writeMemory(address, data);
	cycles += 8;
}
// LD B, n - wartosc do rejestru 8 bitowego
void CPU::load8_imm(uint8_t& reg){
	uint8_t data = Memory.readMemory(this->pc);
	pc++;
	reg = data;
	cycles += 8;
}
// CALL nn - podprogram z nn
void CPU::opcode_callNN(){
	uint16_t val = readTwoBytes();
	writeToStack(this->pc);
	this->pc = val;
	cycles += 24;
}
//JMP nn - jump do nn
void CPU::opcode_cb(){
	uint8_t address = Memory.readMemory(this->pc);
	pc++;
	extended_opcodes(address);
	cycles += 4;
}
// nic nie rób
void CPU::opcode_nop() {
	cycles += 4;
}
// CPL - ustawic akumulator na 1
void CPU::opcode_cpl(uint8_t& value){
	value = ~value;
	bitset(FLAG_N);
	bitset(FLAG_H);
	cycles += 4;
}
// SCF - ustawic carry na 1
void CPU::opcode_scf(){
	bitset(FLAG_C);
	bitreset(FLAG_N);
	bitreset(FLAG_H);
	cycles += 4;
}
// CP B - porownanie rejestru z akumulatorem
void CPU::opcode_CP(uint8_t reg1, uint8_t reg2){
	uint8_t before = reg1;
	uint8_t ztest = reg1 - reg2;
	registerAF.lo = 0;
	if (ztest == 0) {
		bitset(FLAG_Z);
	}
	int16_t test = before & 0xF;
	test -= (reg2 & 0xF);
	if (test < 0) {
		bitset(FLAG_H);
	}
	if (reg2 > reg1) {
		bitset(FLAG_C);
	}
	bitset(FLAG_N);
	cycles += 4;
}
//CP (HL) - porownanie rejestru z wartoscia z pamieci
void CPU::opcode_CPmmu(uint8_t reg1, uint16_t pointer){
	uint8_t val2 = Memory.readMemory(pointer);
	opcode_CP(reg1, val2);
	// opcode_CP already adds 4 cycles and this one requires 8 thus we add 4 more
	cycles += 4;
}
// w³aczenie przerwan
void CPU::enable_interrupts(){
	IME = true;
	cycles += 4;
}
// wy³¹czenie przerwañ
void CPU::opcode_di(){
	IME = false;
	cycles += 4;
}
// w³aczenie przerwan
void CPU::opcode_ei(){
	IME = true;
	cycles += 4;
}
// wy³¹czenie przerwañ
void CPU::opcode_resetIME(){
	IME = false;
	cycles += 4;
}
// ustawienie CF na 1
void CPU::opcode_ccf(){
	if (getBit(FLAG_C)) {
		bitreset(FLAG_C);
	}
	else {
		bitset(FLAG_C);
	}
	bitreset(FLAG_N);
	bitreset(FLAG_H);
}
// POP BC - zdjecie adresu ze stosu
void CPU::opcode_pop(uint16_t& address) {
	address = readFromStack();
	cycles += 12;
}
// stop
void CPU::opcode_stop(){
	stop = true;
	cycles += 4;
}
// halt
void CPU::opcode_halt(){
	halt = true;
}
//CALL - jesli flaga jest rowna 0
void CPU::call_false(int flag){
	if (getBit(flag) == 0) {
		call_nn();
		cycles += 12;
		return;
	}
	pc += 2;
	cycles += 12;
}
// CALL Z, nn - call jesli Z=1
void CPU::call_true(int flag){
	if (getBit(flag) == 1) {
		call_nn();
		cycles += 12;
		return;
	}
	pc += 2;
	cycles += 12;
}
// CALL nn - wywolaj podprogram z nn
void CPU::call_nn(){
	uint16_t imd = readTwoBytes();
	writeToStack(pc);
	this->pc = imd;
	cycles += 24;
}
// PUSH BC - rejestr na stos
void CPU::push_reg16(uint16_t reg){
	writeToStack(reg);
	cycles += 16;
}
// DAA - decymalne ustawienie akumulatora
void CPU::opcode_bcd(uint8_t& value){
	if (!getBit(FLAG_N)) {
		if (getBit(FLAG_C) || value > 0x99) { value += 0x60; bitset(FLAG_C); }
		if (getBit(FLAG_H) || (value & 0x0F) > 0x09) { value += 0x6; }
	}
	else {
		if (getBit(FLAG_C)) { value -= 0x60; }
		if (getBit(FLAG_H)) { value -= 0x6; }
	}
	registerAF.hi == 0 ? bitset(FLAG_Z) : bitreset(FLAG_Z);
	bitreset(FLAG_H);
	cycles += 4;
}
// LD A, (BC) - zaladowanie danych z lokacji do akumulatoraa
void CPU::opcode_load8(uint16_t address, uint8_t& destination) {
	uint8_t val = Memory.readMemory(address);
	destination = val;
	cycles += 8;
}
// LDI A, (HL) - Load location (DE) with location (HL), incr DE,HL; decr BC.
void CPU::opcode_ldi8(uint16_t& address, uint8_t& destination){
	opcode_load8(address, destination);
	address++;
}
// LDD A, (HL) - Load location (DE) with location (HL), decrement DE,HL,BC.
void CPU::opcode_ldd8(uint16_t& address, uint8_t& destination){
	opcode_load8(address, destination);
	address--;
}
// LD (BC), A - za³adowanie akumulatora pod lokacje
void CPU::opcode_mmucopy8(uint16_t mmulocation, uint8_t data){
	Memory.writeMemory(mmulocation, data);
	cycles += 8;
}
// zaldowanie do pamieci 8 bitowego
void CPU::ld_reg8(uint8_t loc, uint8_t data){
	mmu_load8((0xFF00 + loc), data);
}
// LDH (n), A - wartosc A pod adres
void CPU::ldh_imm(uint8_t reg){
	uint8_t n = Memory.readMemory(this->pc);
	pc++;
	mmu_load8((0xFF00 + n), reg);
	cycles += 4;
}
// LDH A, (n) - adres do A
void CPU::ldh_a(uint8_t& reg){
	uint8_t n = Memory.readMemory(this->pc);
	pc++;
	uint8_t val = Memory.readMemory(0xFF00 + n);
	reg = val;
	cycles += 12;
}
// LD (nn), SP - SP pod adres
void CPU::load_SP(){
	writeRegToMemory(sp);
	cycles += 20;
}
// LD (nn), A - A pod adres
void CPU::load_nnReg8(uint8_t reg){
	uint16_t address = readTwoBytes();
	Memory.writeMemory(address, reg);
	cycles += 12;
}
// LDHL SP, d - HL = SP
void CPU::opcode_ldhl(){
	int8_t signedInt = static_cast<int8_t>(Memory.readMemory(this->pc));
	pc++;
	if (this->sp.reg + signedInt > 0xFFFF) {
		bitset(FLAG_C);
	}
	else {
		bitreset(FLAG_C);
	}

	if (this->sp.reg + signedInt > 0x0FFF) {
		bitset(FLAG_H);
	}
	else {
		bitreset(FLAG_H);
	}
	registerHL.reg = signedInt + sp.reg;
	bitreset(FLAG_Z);
	bitreset(FLAG_N);
	cycles += 12;
}
// SP = HL
void CPU::load_SP_HL()
{	this->sp.reg = registerHL.reg;
	cycles += 8;
}
// LD B, n - ladowanie do 8 bitowego rejestru
void CPU::copy_reg8(uint8_t& destination, uint8_t source){
	destination = source;
	cycles += 4;
}
//ld do pamieci
void CPU::mmu_imm(uint16_t address){
	uint8_t data = Memory.readMemory(this->pc);
	pc++;
	mmu_load8(address, data);
}
// LD BC, nn - 16 bit
void CPU::reg16_load(uint16_t& reg){
	uint16_t twoBytes = readTwoBytes();
	reg = twoBytes;
	cycles += 12;
}
// LD C,n - 8 bit
void CPU::reg8_load(uint8_t& address){
	address = Memory.readMemory(this->pc);
	this->pc += 1;
	cycles += 8;
}
// ADD HL, BC - dodoawanie wartosci do 16 bitowych
void CPU::add_16(uint16_t& destination, uint16_t source){
	if ((destination & 0x0FFF) + (source & 0x0FFF) > 0x0FFF) {
		bitset(FLAG_H);
	}
	else {
		bitreset(FLAG_H);
	}
	if (destination + source > 0xFFFF) {
		bitset(FLAG_C);
	}
	else {
		bitreset(FLAG_C);
	}
	bitreset(FLAG_N);
	destination += source;
	cycles += 8;
}
// ADD H, L - dodoawanie wartosci do 8 bitowych
void CPU::add_8(uint8_t value, uint8_t& destination){
	uint8_t before = destination;
	destination += value;
	registerAF.lo = 0;
	if (destination == 0) {
		bitset(FLAG_Z);
	}
	if ((before & 0x0f) + (value & 0x0F) > 0x0f) {
		bitset(FLAG_H);
	}
	if ((before + value) > 0xFF) {
		bitset(FLAG_C);
	}
	cycles += 4;
}
// ADD A, (HL) - dodowanie pod adres
void CPU::add_mmu(uint16_t value, uint8_t& dest){
	uint8_t valueToSum = Memory.readMemory(value);
	add_8(valueToSum, dest);
	cycles += 8;
}
// ADC A, B - dodaj z carry
void CPU::adc_reg8(uint8_t source, uint8_t& destination){
	int carry = getBit(FLAG_C);
	uint8_t before = destination;
	destination += source;
	destination += carry;
	registerAF.lo = 0;
	if (destination == 0) {
		bitset(FLAG_Z);
	}
	if ((before & 0x0f) + (source & 0x0F) + carry > 0x0f) {
		bitset(FLAG_H);
	}

	if ((before + source + carry) > 0xFF) {
		bitset(FLAG_C);
	}
	cycles += 4;
}
// ADC A, (HL) - dodaj z pod lokacji z carry 
void CPU::adc_imm(uint16_t pointer, uint8_t& destination){
	uint8_t source = Memory.readMemory(pointer);
	this->adc_reg8(source, destination);
	cycles += 8;
}
// ADC A, n - dodaj wartosc z carry
void CPU::adc_n(uint8_t& destination){
	uint8_t data = Memory.readMemory(this->pc);
	this->pc++;
	adc_reg8(data, destination);
	cycles += 4;
}
// ADD A, n - dodaj wartosc
void CPU::add_n(uint8_t& destination){
	uint8_t data = Memory.readMemory(this->pc);
	this->pc++;
	add_8(data, destination);
	cycles += 4;
}
// ADD SP, d - dodaj do SP
void CPU::add_signedToSP(){
	int8_t signedInt = static_cast<int8_t>(Memory.readMemory(this->pc));
	pc++;
	bitreset(FLAG_Z);
	bitreset(FLAG_N);
	if ((this->sp.reg & 0x0FFF) + signedInt > 0x0FFF) {
		bitset(FLAG_H);
	}
	else {
		bitreset(FLAG_H);
	}

	if (this->sp.reg + signedInt > 0xFFFF) {
		bitset(FLAG_C);
	}
	else {
		bitreset(FLAG_C);
	}

	sp.reg += signedInt;
	cycles += 16;
}
// SUB A, D - odejmij wartosc
void CPU::sub_reg8(uint8_t value, uint8_t& destination){
	registerAF.lo = 0;
	uint8_t original = destination;
	if (destination - value == 0) {
		bitset(FLAG_Z);
	}
	bitset(FLAG_N);
	if ((value & 0xf) > (original & 0xf)) {
		bitset(FLAG_H);
	}
	if (destination < value) {
		bitset(FLAG_C);
	}
	destination -= value;
	cycles += 4;
}
// SUB A, (HL) - odejmij z lokacji pamieci
void CPU::sub_mmu(uint16_t value, uint8_t& destination){
	uint8_t val = Memory.readMemory(value);
	sub_reg8(val, destination);
	// add extra 4 cycles for total of 8 required due to memory access
	cycles += 4;
}
// SBC A, (HL) - odejmij z carry
void CPU::sbc_imm(uint16_t pointer, uint8_t& destination){
	uint8_t value = Memory.readMemory(pointer);
	sbc_reg8(value, destination);
	// add extra 4 cycles for total of 8 required due to memory access
	cycles += 4;
}
// SBC A, n - odejmij z carry
void CPU::sbc_n(uint8_t& destination){
	uint8_t data = Memory.readMemory(this->pc);
	this->pc++;
	sbc_reg8(data, destination);
	// add extra 4 cycles for total of 8 required due to memory access
	cycles += 4;
}
// SBC A, B - odejmij z carry
void CPU::sbc_reg8(uint8_t data, uint8_t& destination){
	uint8_t carry = getBit(FLAG_C);
	uint8_t value = data + carry;
	registerAF.lo = 0;
	uint8_t original = destination;
	if (destination - value == 0) {
		bitset(FLAG_Z);
	}
	bitset(FLAG_N);
	if ((data & 0xf) + carry > (original & 0xf)) {
		bitset(FLAG_H);
	}
	int dest = destination;
	if ((dest - (data + carry)) < 0) {
		bitset(FLAG_C);
	}
	destination -= value;
}
// SUB A, n - odejmij
void CPU::sub_n(uint8_t& destination){
	uint8_t data = Memory.readMemory(this->pc);
	this->pc++;
	sub_reg8(data, destination);
	// add extra 4 cycles for total of 8 required due to memory access
	cycles += 4;
}
// INC BC - inkrementacja 16 bitowy
void CPU::incr_reg(uint16_t& address){
	address += 1;
	cycles += 8;
}
// INC B - inkrementacja 8 bitowy
void CPU::incr_reg(uint8_t& address){
	int original = address;
	address += 1;
	if (address == 0) {
		bitset(FLAG_Z);
	}
	else {
		bitreset(FLAG_Z);
	}
	if ((original & 0xF) == 0xF) {
		bitset(FLAG_H);
	}
	else {
		bitreset(FLAG_H);
	}
	bitreset(FLAG_N);
	cycles += 4;
}
// INC (HL) - inkrementacja z pod adresu
void CPU::incp_reg(int16_t address){
	uint8_t value = Memory.readMemory(address);
	incr_reg(value);
	Memory.writeMemory(address, value);
	cycles += 12;
}
// AND B - and dla 8 bitow
void CPU::and_reg8(uint8_t value, uint8_t& destination){
	destination &= value;
	if (destination == 0) {
		bitset(FLAG_Z);
	}
	else {
		bitreset(FLAG_Z);
	}
	bitreset(FLAG_N);
	bitset(FLAG_H);
	bitreset(FLAG_C);
	cycles += 4;
}
// AND, n - and z wartoscia
void CPU::and_n(uint8_t& destination){
	uint8_t data = Memory.readMemory(this->pc);
	pc++;
	and_reg8(data, destination);
	cycles += 4;
}
// AND (HL) - and z lokacja
void CPU::and_mmu(uint16_t value, uint8_t& destination){
	uint8_t a = Memory.readMemory(value);
	this->and_reg8(a, destination);
	cycles += 4;
}
// XOR B - xor z rejestrem
void CPU::xor_reg8(uint8_t value, uint8_t& destination){
	destination ^= value;
	registerAF.lo = 0;
	if (destination == 0) {
		bitset(FLAG_Z);
	}
	cycles += 4;
}
// XOR (HL) - xor z lokacja
void CPU::xor_mmu(uint16_t value, uint8_t& destination){
	uint8_t val2 = Memory.readMemory(value);
	xor_reg8(val2, destination);
	cycles += 4;
}
// XOR, n - xor z wartoscia
void CPU::xor_n(uint8_t& destination){
	uint8_t data = Memory.readMemory(this->pc);
	pc++;
	xor_reg8(data, destination);
	// add extra 4 cycles for total of 8 required due to memory access
	cycles += 4;
}
// OR B - or z rejestrem
void CPU::or_reg8(uint8_t value, uint8_t& destination){
	destination |= value;
	if (destination == 0) {
		bitset(FLAG_Z);
	}
	else {
		bitreset(FLAG_Z);
	}
	bitreset(FLAG_N);
	bitreset(FLAG_H);
	bitreset(FLAG_C);
	cycles += 4;
}
// OR (HL) - or z lokacja
void CPU::or_mmu(uint16_t pointer, uint8_t& destination){
	uint8_t value = Memory.readMemory(pointer);
	or_reg8(value, destination);
	// add extra 4 cycles for total of 8 required due to memory access
	cycles += 4;
}
// OR, n - or z wartoscia
void CPU::or_n(uint8_t& destination){
	uint8_t data = Memory.readMemory(this->pc);
	pc++;
	or_reg8(data, destination);
	// add extra 4 cycles for total of 8 required due to memory access
	cycles += 4;

}
// DEC B - rejestr 8 bit
void CPU::decr_reg(uint8_t& address){
	int original = address;
	address -= 1;
	if (address == 0) {
		bitset(FLAG_Z);
	}
	else {
		bitreset(FLAG_Z);
	}

	if ((original & 0x0F) == 0) {
		bitset(FLAG_H);
	}
	else {
		bitreset(FLAG_H);
	}
	bitset(FLAG_N);
	cycles += 4;
}
// DEC BC - rejestr 16 bit
void CPU::decr_reg(uint16_t& address){
	address -= 1;
	cycles += 8;

}
// DEC (HL)- z lokacji
void CPU::decp_reg(uint16_t value){
	uint8_t newValue = Memory.readMemory(value);
	decr_reg(newValue);
	Memory.writeMemory(value, newValue);
	cycles += 12;
}
// restart 
void CPU::restart(uint8_t n){
	writeToStack(this->pc);
	this->pc = n;
	cycles += 32;
}
//RLA lewo z carry
void CPU::rl_reg8(uint8_t& address){
	uint8_t original = address;
	uint8_t oldCarry = getBit(FLAG_C);
	address <<= 1;
	uint8_t msb = getBit(7, original);
	registerAF.lo = 0;
	if (msb) {
		bitset(FLAG_C);
	}
	if (oldCarry) {
		address = set_bit(address, 0);
	}
	cycles += 8;
}
//RRC prawo z carry
void CPU::rrc_reg8(uint8_t& address){
	uint8_t original = address;
	address = original >> 1;
	uint8_t lsb = getBit(0, original);
	registerAF.lo = 0;
	if (lsb) {
		bitset(FLAG_C);
		address = set_bit(address, 7);
	}
	else {
		bitreset(FLAG_C);
	}
	cycles += 8;
}
// RLC, A lewo z carry
void CPU::rlc_reg8(uint8_t& address){
	uint8_t bitSeven = getBit(7, address);
	registerAF.lo = 0;
	address <<= 1;
	if (bitSeven) {
		bitset(FLAG_C);
		address = set_bit(address, 0);
	}
	cycles += 4;
}
// RR A prawo bez carry ale z jej uwzglednieniem
void CPU::rr_reg8(uint8_t& address){
	uint8_t original = address;
	uint8_t bitZero = getBit(0, address);
	uint8_t carry = getBit(FLAG_C);
	address >>= 1;
	registerAF.lo = 0;
	if (bitZero) {
		bitset(FLAG_C);
	}
	if (carry) {
		address = set_bit(address, 7);
	}
}
// RET powrot z podprogramu 
void CPU::opcode_ret(){
	this->pc = readFromStack();
	cycles += 16;
}
// RET Z - jesli flaga = 0
void CPU::opcode_retFalse(int flag){
	if (getBit(flag) == 0) {
		opcode_ret();
		cycles += 4;
		return;
	}
	cycles += 8;
}
// RET NZ - jesli flaga = 1
void CPU::opcode_retTrue(int flag){
	if (getBit(flag)) {
		opcode_ret();
		cycles += 4;
		return;
	}
	cycles += 8;
}