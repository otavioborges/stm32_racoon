/*
 * GZ80 - Zilog Z80 Emulator
 * Copyright (C) 2014  Ignacio Sanchez Gines

 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see http://www.gnu.org/licenses/ 
 * 
 */

#ifndef GZ80_CORE_H_
#define	GZ80_CORE_H_

#include "gz80_definitions.h"
#include "gz80_eight_bit_register.h"
#include "gz80_sixteen_bit_register.h"
#include "gz80_memory_interface.h"
#include "gz80_ioports_interface.h"

namespace gz80
{
   
class GZ80
{
public:
    GZ80();
    ~GZ80();
    void Init();
    void Reset();
    unsigned int RunFor(unsigned int t_states);
    unsigned int Tick();
    void RequestINT(bool assert);
    void RequestNMI();
    void SetIOPOrtsImpl(IOPortsInterface* io_ports_impl);
    void SetMemoryImpl(MemoryInterface* memory_impl);

private:
    typedef void (GZ80::*OPCptr) (void);
    OPCptr opcodes_[256];
    OPCptr opcodes_cb_[256];
    OPCptr opcodes_ed_[256];
    SixteenBitRegister AF_;
    SixteenBitRegister BC_;
    SixteenBitRegister DE_;
    SixteenBitRegister HL_;
    SixteenBitRegister AF2_;
    SixteenBitRegister BC2_;
    SixteenBitRegister DE2_;
    SixteenBitRegister HL2_;
    SixteenBitRegister IX_;
    SixteenBitRegister IY_;
    SixteenBitRegister SP_;
    SixteenBitRegister PC_;
    SixteenBitRegister XY_;  // MEMPTR register for XY flags
    EightBitRegister I_;
    EightBitRegister R_;
    bool iff1_;
    bool iff2_;
    bool halt_;
    bool branched_;
    unsigned int t_states_;
    bool just_after_ei_;
    int interrupt_mode_;
    u8 actual_prefix_;
    bool interrupt_requested_;
    bool nmi_interrupt_requested_;
    bool prefixed_cb_opcode_;
    u8 prefixed_cb_value_;
    IOPortsInterface* io_ports_impl_;
    MemoryInterface* memory_impl_;

private:
    u8 FetchOPCode();
    u16 FetchArg16();
    void ExecuteOPCode();
    void LeaveHalt();
    void ClearAllFlags();
    void ToggleZeroFlagFromResult(u16 result);
    void ToggleSignFlagFromResult(u8 result);
    void ToggleXYFlagsFromResult(u8 result);
    void ToggleParityFlagFromResult(u8 result);
    void SetFlag(u8 flag);
    void FlipFlag(u8 flag);
    void ToggleFlag(u8 flag);
    void ClearFlag(u8 flag);
    bool IsSetFlag(u8 flag);
    void StackPush(SixteenBitRegister* reg);
    void StackPop(SixteenBitRegister* reg);
    void SetInterruptMode(int mode);
    void IncreaseR();
    void InvalidOPCode();
    void UndocumentedOPCode();
    SixteenBitRegister* GetPrefixedRegister();
    u16 GetEffectiveAddress();
    bool IsPrefixedInstruction();
    void OPCodes_LD(EightBitRegister* reg1, u8 value);
    void OPCodes_LD(EightBitRegister* reg, u16 address);
    void OPCodes_LD(u16 address, u8 reg);
    void OPCodes_LD_dd_nn(SixteenBitRegister* reg);
    void OPCodes_LD_nn_dd(SixteenBitRegister* reg);
    void OPCodes_LDI();
    void OPCodes_LDD();
    void OPCodes_RST(u16 address);
    void OPCodes_CALL_nn();
    void OPCodes_CALL_nn_Conditional(bool condition);
    void OPCodes_JP_nn();
    void OPCodes_JP_nn_Conditional(bool condition);
    void OPCodes_JR_n();
    void OPCodes_JR_n_conditional(bool condition);
    void OPCodes_RET();
    void OPCodes_RET_Conditional(bool condition);
    void OPCodes_IN_C(EightBitRegister* reg);
    void OPCodes_INI();
    void OPCodes_IND();
    void OPCodes_OUT_C(EightBitRegister* reg);
    void OPCodes_OUTI();
    void OPCodes_OUTD();
    void OPCodes_EX(SixteenBitRegister* reg1, SixteenBitRegister* reg2);
    void OPCodes_OR(u8 number);
    void OPCodes_XOR(u8 number);
    void OPCodes_AND(u8 number);
    void OPCodes_CP(u8 number);
    void OPCodes_CPI();
    void OPCodes_CPD();
    void OPCodes_INC(EightBitRegister* reg);
    void OPCodes_INC_HL();
    void OPCodes_DEC(EightBitRegister* reg);
    void OPCodes_DEC_HL();
    void OPCodes_ADD(u8 number);
    void OPCodes_ADC(u8 number);
    void OPCodes_SUB(u8 number);
    void OPCodes_SBC(u8 number);
    void OPCodes_ADD_HL(u16 number);
    void OPCodes_ADC_HL(u16 number);
    void OPCodes_SBC_HL(u16 number);
    void OPCodes_SLL(EightBitRegister* reg);
    void OPCodes_SLL_HL();
    void OPCodes_SLA(EightBitRegister* reg);
    void OPCodes_SLA_HL();
    void OPCodes_SRA(EightBitRegister* reg);
    void OPCodes_SRA_HL();
    void OPCodes_SRL(EightBitRegister* reg);
    void OPCodes_SRL_HL();
    void OPCodes_RLC(EightBitRegister* reg, bool register_A = false);
    void OPCodes_RLC_HL();
    void OPCodes_RL(EightBitRegister* reg, bool register_A = false);
    void OPCodes_RL_HL();
    void OPCodes_RRC(EightBitRegister* reg, bool register_A = false);
    void OPCodes_RRC_HL();
    void OPCodes_RR(EightBitRegister* reg, bool register_A = false);
    void OPCodes_RR_HL();
    void OPCodes_BIT(EightBitRegister* reg, int bit);
    void OPCodes_BIT_HL(int bit);
    void OPCodes_SET(EightBitRegister* reg, int bit);
    void OPCodes_SET_HL(int bit);
    void OPCodes_RES(EightBitRegister* reg, int bit);
    void OPCodes_RES_HL(int bit);
    void InitOPCodeFunctors();

    void OPCode0x00();
    void OPCode0x01();
    void OPCode0x02();
    void OPCode0x03();
    void OPCode0x04();
    void OPCode0x05();
    void OPCode0x06();
    void OPCode0x07();
    void OPCode0x08();
    void OPCode0x09();
    void OPCode0x0A();
    void OPCode0x0B();
    void OPCode0x0C();
    void OPCode0x0D();
    void OPCode0x0E();
    void OPCode0x0F();
    void OPCode0x10();
    void OPCode0x11();
    void OPCode0x12();
    void OPCode0x13();
    void OPCode0x14();
    void OPCode0x15();
    void OPCode0x16();
    void OPCode0x17();
    void OPCode0x18();
    void OPCode0x19();
    void OPCode0x1A();
    void OPCode0x1B();
    void OPCode0x1C();
    void OPCode0x1D();
    void OPCode0x1E();
    void OPCode0x1F();
    void OPCode0x20();
    void OPCode0x21();
    void OPCode0x22();
    void OPCode0x23();
    void OPCode0x24();
    void OPCode0x25();
    void OPCode0x26();
    void OPCode0x27();
    void OPCode0x28();
    void OPCode0x29();
    void OPCode0x2A();
    void OPCode0x2B();
    void OPCode0x2C();
    void OPCode0x2D();
    void OPCode0x2E();
    void OPCode0x2F();
    void OPCode0x30();
    void OPCode0x31();
    void OPCode0x32();
    void OPCode0x33();
    void OPCode0x34();
    void OPCode0x35();
    void OPCode0x36();
    void OPCode0x37();
    void OPCode0x38();
    void OPCode0x39();
    void OPCode0x3A();
    void OPCode0x3B();
    void OPCode0x3C();
    void OPCode0x3D();
    void OPCode0x3E();
    void OPCode0x3F();
    void OPCode0x40();
    void OPCode0x41();
    void OPCode0x42();
    void OPCode0x43();
    void OPCode0x44();
    void OPCode0x45();
    void OPCode0x46();
    void OPCode0x47();
    void OPCode0x48();
    void OPCode0x49();
    void OPCode0x4A();
    void OPCode0x4B();
    void OPCode0x4C();
    void OPCode0x4D();
    void OPCode0x4E();
    void OPCode0x4F();
    void OPCode0x50();
    void OPCode0x51();
    void OPCode0x52();
    void OPCode0x53();
    void OPCode0x54();
    void OPCode0x55();
    void OPCode0x56();
    void OPCode0x57();
    void OPCode0x58();
    void OPCode0x59();
    void OPCode0x5A();
    void OPCode0x5B();
    void OPCode0x5C();
    void OPCode0x5D();
    void OPCode0x5E();
    void OPCode0x5F();
    void OPCode0x60();
    void OPCode0x61();
    void OPCode0x62();
    void OPCode0x63();
    void OPCode0x64();
    void OPCode0x65();
    void OPCode0x66();
    void OPCode0x67();
    void OPCode0x68();
    void OPCode0x69();
    void OPCode0x6A();
    void OPCode0x6B();
    void OPCode0x6C();
    void OPCode0x6D();
    void OPCode0x6E();
    void OPCode0x6F();
    void OPCode0x70();
    void OPCode0x71();
    void OPCode0x72();
    void OPCode0x73();
    void OPCode0x74();
    void OPCode0x75();
    void OPCode0x76();
    void OPCode0x77();
    void OPCode0x78();
    void OPCode0x79();
    void OPCode0x7A();
    void OPCode0x7B();
    void OPCode0x7C();
    void OPCode0x7D();
    void OPCode0x7E();
    void OPCode0x7F();
    void OPCode0x80();
    void OPCode0x81();
    void OPCode0x82();
    void OPCode0x83();
    void OPCode0x84();
    void OPCode0x85();
    void OPCode0x86();
    void OPCode0x87();
    void OPCode0x88();
    void OPCode0x89();
    void OPCode0x8A();
    void OPCode0x8B();
    void OPCode0x8C();
    void OPCode0x8D();
    void OPCode0x8E();
    void OPCode0x8F();
    void OPCode0x90();
    void OPCode0x91();
    void OPCode0x92();
    void OPCode0x93();
    void OPCode0x94();
    void OPCode0x95();
    void OPCode0x96();
    void OPCode0x97();
    void OPCode0x98();
    void OPCode0x99();
    void OPCode0x9A();
    void OPCode0x9B();
    void OPCode0x9C();
    void OPCode0x9D();
    void OPCode0x9E();
    void OPCode0x9F();
    void OPCode0xA0();
    void OPCode0xA1();
    void OPCode0xA2();
    void OPCode0xA3();
    void OPCode0xA4();
    void OPCode0xA5();
    void OPCode0xA6();
    void OPCode0xA7();
    void OPCode0xA8();
    void OPCode0xA9();
    void OPCode0xAA();
    void OPCode0xAB();
    void OPCode0xAC();
    void OPCode0xAD();
    void OPCode0xAE();
    void OPCode0xAF();
    void OPCode0xB0();
    void OPCode0xB1();
    void OPCode0xB2();
    void OPCode0xB3();
    void OPCode0xB4();
    void OPCode0xB5();
    void OPCode0xB6();
    void OPCode0xB7();
    void OPCode0xB8();
    void OPCode0xB9();
    void OPCode0xBA();
    void OPCode0xBB();
    void OPCode0xBC();
    void OPCode0xBD();
    void OPCode0xBE();
    void OPCode0xBF();
    void OPCode0xC0();
    void OPCode0xC1();
    void OPCode0xC2();
    void OPCode0xC3();
    void OPCode0xC4();
    void OPCode0xC5();
    void OPCode0xC6();
    void OPCode0xC7();
    void OPCode0xC8();
    void OPCode0xC9();
    void OPCode0xCA();
    void OPCode0xCB();
    void OPCode0xCC();
    void OPCode0xCD();
    void OPCode0xCE();
    void OPCode0xCF();
    void OPCode0xD0();
    void OPCode0xD1();
    void OPCode0xD2();
    void OPCode0xD3();
    void OPCode0xD4();
    void OPCode0xD5();
    void OPCode0xD6();
    void OPCode0xD7();
    void OPCode0xD8();
    void OPCode0xD9();
    void OPCode0xDA();
    void OPCode0xDB();
    void OPCode0xDC();
    void OPCode0xDD();
    void OPCode0xDE();
    void OPCode0xDF();
    void OPCode0xE0();
    void OPCode0xE1();
    void OPCode0xE2();
    void OPCode0xE3();
    void OPCode0xE4();
    void OPCode0xE5();
    void OPCode0xE6();
    void OPCode0xE7();
    void OPCode0xE8();
    void OPCode0xE9();
    void OPCode0xEA();
    void OPCode0xEB();
    void OPCode0xEC();
    void OPCode0xED();
    void OPCode0xEE();
    void OPCode0xEF();
    void OPCode0xF0();
    void OPCode0xF1();
    void OPCode0xF2();
    void OPCode0xF3();
    void OPCode0xF4();
    void OPCode0xF5();
    void OPCode0xF6();
    void OPCode0xF7();
    void OPCode0xF8();
    void OPCode0xF9();
    void OPCode0xFA();
    void OPCode0xFB();
    void OPCode0xFC();
    void OPCode0xFD();
    void OPCode0xFE();
    void OPCode0xFF();

    void OPCodeCB0x00();
    void OPCodeCB0x01();
    void OPCodeCB0x02();
    void OPCodeCB0x03();
    void OPCodeCB0x04();
    void OPCodeCB0x05();
    void OPCodeCB0x06();
    void OPCodeCB0x07();
    void OPCodeCB0x08();
    void OPCodeCB0x09();
    void OPCodeCB0x0A();
    void OPCodeCB0x0B();
    void OPCodeCB0x0C();
    void OPCodeCB0x0D();
    void OPCodeCB0x0E();
    void OPCodeCB0x0F();
    void OPCodeCB0x10();
    void OPCodeCB0x11();
    void OPCodeCB0x12();
    void OPCodeCB0x13();
    void OPCodeCB0x14();
    void OPCodeCB0x15();
    void OPCodeCB0x16();
    void OPCodeCB0x17();
    void OPCodeCB0x18();
    void OPCodeCB0x19();
    void OPCodeCB0x1A();
    void OPCodeCB0x1B();
    void OPCodeCB0x1C();
    void OPCodeCB0x1D();
    void OPCodeCB0x1E();
    void OPCodeCB0x1F();
    void OPCodeCB0x20();
    void OPCodeCB0x21();
    void OPCodeCB0x22();
    void OPCodeCB0x23();
    void OPCodeCB0x24();
    void OPCodeCB0x25();
    void OPCodeCB0x26();
    void OPCodeCB0x27();
    void OPCodeCB0x28();
    void OPCodeCB0x29();
    void OPCodeCB0x2A();
    void OPCodeCB0x2B();
    void OPCodeCB0x2C();
    void OPCodeCB0x2D();
    void OPCodeCB0x2E();
    void OPCodeCB0x2F();
    void OPCodeCB0x30();
    void OPCodeCB0x31();
    void OPCodeCB0x32();
    void OPCodeCB0x33();
    void OPCodeCB0x34();
    void OPCodeCB0x35();
    void OPCodeCB0x36();
    void OPCodeCB0x37();
    void OPCodeCB0x38();
    void OPCodeCB0x39();
    void OPCodeCB0x3A();
    void OPCodeCB0x3B();
    void OPCodeCB0x3C();
    void OPCodeCB0x3D();
    void OPCodeCB0x3E();
    void OPCodeCB0x3F();
    void OPCodeCB0x40();
    void OPCodeCB0x41();
    void OPCodeCB0x42();
    void OPCodeCB0x43();
    void OPCodeCB0x44();
    void OPCodeCB0x45();
    void OPCodeCB0x46();
    void OPCodeCB0x47();
    void OPCodeCB0x48();
    void OPCodeCB0x49();
    void OPCodeCB0x4A();
    void OPCodeCB0x4B();
    void OPCodeCB0x4C();
    void OPCodeCB0x4D();
    void OPCodeCB0x4E();
    void OPCodeCB0x4F();
    void OPCodeCB0x50();
    void OPCodeCB0x51();
    void OPCodeCB0x52();
    void OPCodeCB0x53();
    void OPCodeCB0x54();
    void OPCodeCB0x55();
    void OPCodeCB0x56();
    void OPCodeCB0x57();
    void OPCodeCB0x58();
    void OPCodeCB0x59();
    void OPCodeCB0x5A();
    void OPCodeCB0x5B();
    void OPCodeCB0x5C();
    void OPCodeCB0x5D();
    void OPCodeCB0x5E();
    void OPCodeCB0x5F();
    void OPCodeCB0x60();
    void OPCodeCB0x61();
    void OPCodeCB0x62();
    void OPCodeCB0x63();
    void OPCodeCB0x64();
    void OPCodeCB0x65();
    void OPCodeCB0x66();
    void OPCodeCB0x67();
    void OPCodeCB0x68();
    void OPCodeCB0x69();
    void OPCodeCB0x6A();
    void OPCodeCB0x6B();
    void OPCodeCB0x6C();
    void OPCodeCB0x6D();
    void OPCodeCB0x6E();
    void OPCodeCB0x6F();
    void OPCodeCB0x70();
    void OPCodeCB0x71();
    void OPCodeCB0x72();
    void OPCodeCB0x73();
    void OPCodeCB0x74();
    void OPCodeCB0x75();
    void OPCodeCB0x76();
    void OPCodeCB0x77();
    void OPCodeCB0x78();
    void OPCodeCB0x79();
    void OPCodeCB0x7A();
    void OPCodeCB0x7B();
    void OPCodeCB0x7C();
    void OPCodeCB0x7D();
    void OPCodeCB0x7E();
    void OPCodeCB0x7F();
    void OPCodeCB0x80();
    void OPCodeCB0x81();
    void OPCodeCB0x82();
    void OPCodeCB0x83();
    void OPCodeCB0x84();
    void OPCodeCB0x85();
    void OPCodeCB0x86();
    void OPCodeCB0x87();
    void OPCodeCB0x88();
    void OPCodeCB0x89();
    void OPCodeCB0x8A();
    void OPCodeCB0x8B();
    void OPCodeCB0x8C();
    void OPCodeCB0x8D();
    void OPCodeCB0x8E();
    void OPCodeCB0x8F();
    void OPCodeCB0x90();
    void OPCodeCB0x91();
    void OPCodeCB0x92();
    void OPCodeCB0x93();
    void OPCodeCB0x94();
    void OPCodeCB0x95();
    void OPCodeCB0x96();
    void OPCodeCB0x97();
    void OPCodeCB0x98();
    void OPCodeCB0x99();
    void OPCodeCB0x9A();
    void OPCodeCB0x9B();
    void OPCodeCB0x9C();
    void OPCodeCB0x9D();
    void OPCodeCB0x9E();
    void OPCodeCB0x9F();
    void OPCodeCB0xA0();
    void OPCodeCB0xA1();
    void OPCodeCB0xA2();
    void OPCodeCB0xA3();
    void OPCodeCB0xA4();
    void OPCodeCB0xA5();
    void OPCodeCB0xA6();
    void OPCodeCB0xA7();
    void OPCodeCB0xA8();
    void OPCodeCB0xA9();
    void OPCodeCB0xAA();
    void OPCodeCB0xAB();
    void OPCodeCB0xAC();
    void OPCodeCB0xAD();
    void OPCodeCB0xAE();
    void OPCodeCB0xAF();
    void OPCodeCB0xB0();
    void OPCodeCB0xB1();
    void OPCodeCB0xB2();
    void OPCodeCB0xB3();
    void OPCodeCB0xB4();
    void OPCodeCB0xB5();
    void OPCodeCB0xB6();
    void OPCodeCB0xB7();
    void OPCodeCB0xB8();
    void OPCodeCB0xB9();
    void OPCodeCB0xBA();
    void OPCodeCB0xBB();
    void OPCodeCB0xBC();
    void OPCodeCB0xBD();
    void OPCodeCB0xBE();
    void OPCodeCB0xBF();
    void OPCodeCB0xC0();
    void OPCodeCB0xC1();
    void OPCodeCB0xC2();
    void OPCodeCB0xC3();
    void OPCodeCB0xC4();
    void OPCodeCB0xC5();
    void OPCodeCB0xC6();
    void OPCodeCB0xC7();
    void OPCodeCB0xC8();
    void OPCodeCB0xC9();
    void OPCodeCB0xCA();
    void OPCodeCB0xCB();
    void OPCodeCB0xCC();
    void OPCodeCB0xCD();
    void OPCodeCB0xCE();
    void OPCodeCB0xCF();
    void OPCodeCB0xD0();
    void OPCodeCB0xD1();
    void OPCodeCB0xD2();
    void OPCodeCB0xD3();
    void OPCodeCB0xD4();
    void OPCodeCB0xD5();
    void OPCodeCB0xD6();
    void OPCodeCB0xD7();
    void OPCodeCB0xD8();
    void OPCodeCB0xD9();
    void OPCodeCB0xDA();
    void OPCodeCB0xDB();
    void OPCodeCB0xDC();
    void OPCodeCB0xDD();
    void OPCodeCB0xDE();
    void OPCodeCB0xDF();
    void OPCodeCB0xE0();
    void OPCodeCB0xE1();
    void OPCodeCB0xE2();
    void OPCodeCB0xE3();
    void OPCodeCB0xE4();
    void OPCodeCB0xE5();
    void OPCodeCB0xE6();
    void OPCodeCB0xE7();
    void OPCodeCB0xE8();
    void OPCodeCB0xE9();
    void OPCodeCB0xEA();
    void OPCodeCB0xEB();
    void OPCodeCB0xEC();
    void OPCodeCB0xED();
    void OPCodeCB0xEE();
    void OPCodeCB0xEF();
    void OPCodeCB0xF0();
    void OPCodeCB0xF1();
    void OPCodeCB0xF2();
    void OPCodeCB0xF3();
    void OPCodeCB0xF4();
    void OPCodeCB0xF5();
    void OPCodeCB0xF6();
    void OPCodeCB0xF7();
    void OPCodeCB0xF8();
    void OPCodeCB0xF9();
    void OPCodeCB0xFA();
    void OPCodeCB0xFB();
    void OPCodeCB0xFC();
    void OPCodeCB0xFD();
    void OPCodeCB0xFE();
    void OPCodeCB0xFF();

    void OPCodeED0x40();
    void OPCodeED0x41();
    void OPCodeED0x42();
    void OPCodeED0x43();
    void OPCodeED0x44();
    void OPCodeED0x45();
    void OPCodeED0x46();
    void OPCodeED0x47();
    void OPCodeED0x48();
    void OPCodeED0x49();
    void OPCodeED0x4A();
    void OPCodeED0x4B();
    void OPCodeED0x4C();
    void OPCodeED0x4D();
    void OPCodeED0x4E();
    void OPCodeED0x4F();
    void OPCodeED0x50();
    void OPCodeED0x51();
    void OPCodeED0x52();
    void OPCodeED0x53();
    void OPCodeED0x54();
    void OPCodeED0x55();
    void OPCodeED0x56();
    void OPCodeED0x57();
    void OPCodeED0x58();
    void OPCodeED0x59();
    void OPCodeED0x5A();
    void OPCodeED0x5B();
    void OPCodeED0x5C();
    void OPCodeED0x5D();
    void OPCodeED0x5E();
    void OPCodeED0x5F();
    void OPCodeED0x60();
    void OPCodeED0x61();
    void OPCodeED0x62();
    void OPCodeED0x63();
    void OPCodeED0x64();
    void OPCodeED0x65();
    void OPCodeED0x66();
    void OPCodeED0x67();
    void OPCodeED0x68();
    void OPCodeED0x69();
    void OPCodeED0x6A();
    void OPCodeED0x6B();
    void OPCodeED0x6C();
    void OPCodeED0x6D();
    void OPCodeED0x6E();
    void OPCodeED0x6F();
    void OPCodeED0x70();
    void OPCodeED0x71();
    void OPCodeED0x72();
    void OPCodeED0x73();
    void OPCodeED0x74();
    void OPCodeED0x75();
    void OPCodeED0x76();
    void OPCodeED0x78();
    void OPCodeED0x79();
    void OPCodeED0x7A();
    void OPCodeED0x7B();
    void OPCodeED0x7C();
    void OPCodeED0x7D();
    void OPCodeED0x7E();
    void OPCodeED0xA0();
    void OPCodeED0xA1();
    void OPCodeED0xA2();
    void OPCodeED0xA3();
    void OPCodeED0xA8();
    void OPCodeED0xA9();
    void OPCodeED0xAA();
    void OPCodeED0xAB();
    void OPCodeED0xB0();
    void OPCodeED0xB1();
    void OPCodeED0xB2();
    void OPCodeED0xB3();
    void OPCodeED0xB8();
    void OPCodeED0xB9();
    void OPCodeED0xBA();
    void OPCodeED0xBB();
};

const bool kZ80ParityTable[256] = {
    true, false, false, true, false, true, true, false,
    false, true, true, false, true, false, false, true,
    false, true, true, false, true, false, false, true,
    true, false, false, true, false, true, true, false,
    false, true, true, false, true, false, false, true,
    true, false, false, true, false, true, true, false,
    true, false, false, true, false, true, true, false,
    false, true, true, false, true, false, false, true,
    false, true, true, false, true, false, false, true,
    true, false, false, true, false, true, true, false,
    true, false, false, true, false, true, true, false,
    false, true, true, false, true, false, false, true,
    true, false, false, true, false, true, true, false,
    false, true, true, false, true, false, false, true,
    false, true, true, false, true, false, false, true,
    true, false, false, true, false, true, true, false,
    false, true, true, false, true, false, false, true,
    true, false, false, true, false, true, true, false,
    true, false, false, true, false, true, true, false,
    false, true, true, false, true, false, false, true,
    true, false, false, true, false, true, true, false,
    false, true, true, false, true, false, false, true,
    false, true, true, false, true, false, false, true,
    true, false, false, true, false, true, true, false,
    true, false, false, true, false, true, true, false,
    false, true, true, false, true, false, false, true,
    false, true, true, false, true, false, false, true,
    true, false, false, true, false, true, true, false,
    false, true, true, false, true, false, false, true,
    true, false, false, true, false, true, true, false,
    true, false, false, true, false, true, true, false,
    false, true, true, false, true, false, false, true
};

} // namespace gz80

#include "gz80_core_inl.h"

#endif // GZ80_CORE_H_

