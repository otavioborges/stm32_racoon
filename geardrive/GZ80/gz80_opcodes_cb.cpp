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

#include "gz80_core.h"

namespace gz80
{
   
void GZ80::OPCodeCB0x00()
{
    // RLC B
    OPCodes_RLC(BC_.GetHighRegister());
}

void GZ80::OPCodeCB0x01()
{
    // RLC C
    OPCodes_RLC(BC_.GetLowRegister());
}

void GZ80::OPCodeCB0x02()
{
    // RLC D
    OPCodes_RLC(DE_.GetHighRegister());
}

void GZ80::OPCodeCB0x03()
{
    // RLC E
    OPCodes_RLC(DE_.GetLowRegister());
}

void GZ80::OPCodeCB0x04()
{
    // RLC H
    OPCodes_RLC(HL_.GetHighRegister());
}

void GZ80::OPCodeCB0x05()
{
    // RLC L
    OPCodes_RLC(HL_.GetLowRegister());
}

void GZ80::OPCodeCB0x06()
{
    // RLC (HL)
    OPCodes_RLC_HL();
}

void GZ80::OPCodeCB0x07()
{
    // RLC A
    OPCodes_RLC(AF_.GetHighRegister());
}

void GZ80::OPCodeCB0x08()
{
    // RRC B
    OPCodes_RRC(BC_.GetHighRegister());
}

void GZ80::OPCodeCB0x09()
{
    // RRC C
    OPCodes_RRC(BC_.GetLowRegister());
}

void GZ80::OPCodeCB0x0A()
{
    // RRC D
    OPCodes_RRC(DE_.GetHighRegister());
}

void GZ80::OPCodeCB0x0B()
{
    // RRC E
    OPCodes_RRC(DE_.GetLowRegister());
}

void GZ80::OPCodeCB0x0C()
{
    // RRC H
    OPCodes_RRC(HL_.GetHighRegister());
}

void GZ80::OPCodeCB0x0D()
{
    // RRC L
    OPCodes_RRC(HL_.GetLowRegister());
}

void GZ80::OPCodeCB0x0E()
{
    // RRC (HL)
    OPCodes_RRC_HL();
}

void GZ80::OPCodeCB0x0F()
{
    // RRC A
    OPCodes_RRC(AF_.GetHighRegister());
}

void GZ80::OPCodeCB0x10()
{
    // RL B
    OPCodes_RL(BC_.GetHighRegister());
}

void GZ80::OPCodeCB0x11()
{
    // RL C
    OPCodes_RL(BC_.GetLowRegister());
}

void GZ80::OPCodeCB0x12()
{
    // RL D
    OPCodes_RL(DE_.GetHighRegister());
}

void GZ80::OPCodeCB0x13()
{
    // RL E
    OPCodes_RL(DE_.GetLowRegister());
}

void GZ80::OPCodeCB0x14()
{
    // RL H
    OPCodes_RL(HL_.GetHighRegister());
}

void GZ80::OPCodeCB0x15()
{
    // RL L
    OPCodes_RL(HL_.GetLowRegister());
}

void GZ80::OPCodeCB0x16()
{
    // RL (HL)
    OPCodes_RL_HL();
}

void GZ80::OPCodeCB0x17()
{
    // RL A
    OPCodes_RL(AF_.GetHighRegister());
}

void GZ80::OPCodeCB0x18()
{
    // RR B
    OPCodes_RR(BC_.GetHighRegister());
}

void GZ80::OPCodeCB0x19()
{
    // RR C
    OPCodes_RR(BC_.GetLowRegister());
}

void GZ80::OPCodeCB0x1A()
{
    // RR D
    OPCodes_RR(DE_.GetHighRegister());
}

void GZ80::OPCodeCB0x1B()
{
    // RR E
    OPCodes_RR(DE_.GetLowRegister());
}

void GZ80::OPCodeCB0x1C()
{
    // RR H
    OPCodes_RR(HL_.GetHighRegister());
}

void GZ80::OPCodeCB0x1D()
{
    // RR L
    OPCodes_RR(HL_.GetLowRegister());
}

void GZ80::OPCodeCB0x1E()
{
    // RR (HL)
    OPCodes_RR_HL();
}

void GZ80::OPCodeCB0x1F()
{
    // RR A
    OPCodes_RR(AF_.GetHighRegister());
}

void GZ80::OPCodeCB0x20()
{
    // SLA B
    OPCodes_SLA(BC_.GetHighRegister());
}

void GZ80::OPCodeCB0x21()
{
    // SLA C
    OPCodes_SLA(BC_.GetLowRegister());
}

void GZ80::OPCodeCB0x22()
{
    // SLA D
    OPCodes_SLA(DE_.GetHighRegister());
}

void GZ80::OPCodeCB0x23()
{
    // SLA E
    OPCodes_SLA(DE_.GetLowRegister());
}

void GZ80::OPCodeCB0x24()
{
    // SLA H
    OPCodes_SLA(HL_.GetHighRegister());
}

void GZ80::OPCodeCB0x25()
{
    // SLA L
    OPCodes_SLA(HL_.GetLowRegister());
}

void GZ80::OPCodeCB0x26()
{
    // SLA (HL)
    OPCodes_SLA_HL();
}

void GZ80::OPCodeCB0x27()
{
    // SLA A
    OPCodes_SLA(AF_.GetHighRegister());
}

void GZ80::OPCodeCB0x28()
{
    // SRA B
    OPCodes_SRA(BC_.GetHighRegister());
}

void GZ80::OPCodeCB0x29()
{
    // SRA C
    OPCodes_SRA(BC_.GetLowRegister());
}

void GZ80::OPCodeCB0x2A()
{
    // SRA D
    OPCodes_SRA(DE_.GetHighRegister());
}

void GZ80::OPCodeCB0x2B()
{
    // SRA E
    OPCodes_SRA(DE_.GetLowRegister());
}

void GZ80::OPCodeCB0x2C()
{
    // SRA H
    OPCodes_SRA(HL_.GetHighRegister());
}

void GZ80::OPCodeCB0x2D()
{
    // SRA L
    OPCodes_SRA(HL_.GetLowRegister());
}

void GZ80::OPCodeCB0x2E()
{
    // SRA (HL)
    OPCodes_SRA_HL();
}

void GZ80::OPCodeCB0x2F()
{
    // SRA A
    OPCodes_SRA(AF_.GetHighRegister());
}

void GZ80::OPCodeCB0x30()
{
    // SLL B
    OPCodes_SLL(BC_.GetHighRegister());
}

void GZ80::OPCodeCB0x31()
{
    // SLL C
    OPCodes_SLL(BC_.GetLowRegister());
}

void GZ80::OPCodeCB0x32()
{
    // SLL D
    OPCodes_SLL(DE_.GetHighRegister());
}

void GZ80::OPCodeCB0x33()
{
    // SLL E
    OPCodes_SLL(DE_.GetLowRegister());
}

void GZ80::OPCodeCB0x34()
{
    // SLL H
    OPCodes_SLL(HL_.GetHighRegister());
}

void GZ80::OPCodeCB0x35()
{
    // SLL L
    OPCodes_SLL(HL_.GetLowRegister());
}

void GZ80::OPCodeCB0x36()
{
    // SLL (HL)
    OPCodes_SLL_HL();
}

void GZ80::OPCodeCB0x37()
{
    // SLL A
    OPCodes_SLL(AF_.GetHighRegister());
}

void GZ80::OPCodeCB0x38()
{
    // SRL B
    OPCodes_SRL(BC_.GetHighRegister());
}

void GZ80::OPCodeCB0x39()
{
    // SRL C
    OPCodes_SRL(BC_.GetLowRegister());
}

void GZ80::OPCodeCB0x3A()
{
    // SRL D
    OPCodes_SRL(DE_.GetHighRegister());
}

void GZ80::OPCodeCB0x3B()
{
    // SRL E
    OPCodes_SRL(DE_.GetLowRegister());
}

void GZ80::OPCodeCB0x3C()
{
    // SRL H
    OPCodes_SRL(HL_.GetHighRegister());
}

void GZ80::OPCodeCB0x3D()
{
    // SRL L
    OPCodes_SRL(HL_.GetLowRegister());
}

void GZ80::OPCodeCB0x3E()
{
    // SRL (HL)
    OPCodes_SRL_HL();
}

void GZ80::OPCodeCB0x3F()
{
    // SRL A
    OPCodes_SRL(AF_.GetHighRegister());
}

void GZ80::OPCodeCB0x40()
{
    // BIT 0 B
    OPCodes_BIT(BC_.GetHighRegister(), 0);
}

void GZ80::OPCodeCB0x41()
{
    // BIT 0 C
    OPCodes_BIT(BC_.GetLowRegister(), 0);
}

void GZ80::OPCodeCB0x42()
{
    // BIT 0 D
    OPCodes_BIT(DE_.GetHighRegister(), 0);
}

void GZ80::OPCodeCB0x43()
{
    // BIT 0 E
    OPCodes_BIT(DE_.GetLowRegister(), 0);
}

void GZ80::OPCodeCB0x44()
{
    // BIT 0 H
    OPCodes_BIT(HL_.GetHighRegister(), 0);
}

void GZ80::OPCodeCB0x45()
{
    // BIT 0 L
    OPCodes_BIT(HL_.GetLowRegister(), 0);
}

void GZ80::OPCodeCB0x46()
{
    // BIT 0 (HL)
    OPCodes_BIT_HL(0);
}

void GZ80::OPCodeCB0x47()
{
    // BIT 0 A
    OPCodes_BIT(AF_.GetHighRegister(), 0);
}

void GZ80::OPCodeCB0x48()
{
    // BIT 1 B
    OPCodes_BIT(BC_.GetHighRegister(), 1);
}

void GZ80::OPCodeCB0x49()
{
    // BIT 1 C
    OPCodes_BIT(BC_.GetLowRegister(), 1);
}

void GZ80::OPCodeCB0x4A()
{
    // BIT 1 D
    OPCodes_BIT(DE_.GetHighRegister(), 1);
}

void GZ80::OPCodeCB0x4B()
{
    // BIT 1 E
    OPCodes_BIT(DE_.GetLowRegister(), 1);
}

void GZ80::OPCodeCB0x4C()
{
    // BIT 1 H
    OPCodes_BIT(HL_.GetHighRegister(), 1);
}

void GZ80::OPCodeCB0x4D()
{
    // BIT 1 L
    OPCodes_BIT(HL_.GetLowRegister(), 1);
}

void GZ80::OPCodeCB0x4E()
{
    // BIT 1 (HL)
    OPCodes_BIT_HL(1);
}

void GZ80::OPCodeCB0x4F()
{
    // BIT 1 A
    OPCodes_BIT(AF_.GetHighRegister(), 1);
}

void GZ80::OPCodeCB0x50()
{
    // BIT 2 B
    OPCodes_BIT(BC_.GetHighRegister(), 2);
}

void GZ80::OPCodeCB0x51()
{
    // BIT 2 C
    OPCodes_BIT(BC_.GetLowRegister(), 2);
}

void GZ80::OPCodeCB0x52()
{
    // BIT 2 D
    OPCodes_BIT(DE_.GetHighRegister(), 2);
}

void GZ80::OPCodeCB0x53()
{
    // BIT 2 E
    OPCodes_BIT(DE_.GetLowRegister(), 2);
}

void GZ80::OPCodeCB0x54()
{
    // BIT 2 H
    OPCodes_BIT(HL_.GetHighRegister(), 2);
}

void GZ80::OPCodeCB0x55()
{
    // BIT 2 L
    OPCodes_BIT(HL_.GetLowRegister(), 2);
}

void GZ80::OPCodeCB0x56()
{
    // BIT 2 (HL)
    OPCodes_BIT_HL(2);
}

void GZ80::OPCodeCB0x57()
{
    // BIT 2 A
    OPCodes_BIT(AF_.GetHighRegister(), 2);
}

void GZ80::OPCodeCB0x58()
{
    // BIT 3 B
    OPCodes_BIT(BC_.GetHighRegister(), 3);
}

void GZ80::OPCodeCB0x59()
{
    // BIT 3 C
    OPCodes_BIT(BC_.GetLowRegister(), 3);
}

void GZ80::OPCodeCB0x5A()
{
    // BIT 3 D
    OPCodes_BIT(DE_.GetHighRegister(), 3);
}

void GZ80::OPCodeCB0x5B()
{
    // BIT 3 E
    OPCodes_BIT(DE_.GetLowRegister(), 3);
}

void GZ80::OPCodeCB0x5C()
{
    // BIT 3 H
    OPCodes_BIT(HL_.GetHighRegister(), 3);
}

void GZ80::OPCodeCB0x5D()
{
    // BIT 3 L
    OPCodes_BIT(HL_.GetLowRegister(), 3);
}

void GZ80::OPCodeCB0x5E()
{
    // BIT 3 (HL)
    OPCodes_BIT_HL(3);
}

void GZ80::OPCodeCB0x5F()
{
    // BIT 3 A
    OPCodes_BIT(AF_.GetHighRegister(), 3);
}

void GZ80::OPCodeCB0x60()
{
    // BIT 4 B
    OPCodes_BIT(BC_.GetHighRegister(), 4);
}

void GZ80::OPCodeCB0x61()
{
    // BIT 4 C
    OPCodes_BIT(BC_.GetLowRegister(), 4);
}

void GZ80::OPCodeCB0x62()
{
    // BIT 4 D
    OPCodes_BIT(DE_.GetHighRegister(), 4);
}

void GZ80::OPCodeCB0x63()
{
    // BIT 4 E
    OPCodes_BIT(DE_.GetLowRegister(), 4);
}

void GZ80::OPCodeCB0x64()
{
    // BIT 4 H
    OPCodes_BIT(HL_.GetHighRegister(), 4);
}

void GZ80::OPCodeCB0x65()
{
    // BIT 4 L
    OPCodes_BIT(HL_.GetLowRegister(), 4);
}

void GZ80::OPCodeCB0x66()
{
    // BIT 4 (HL)
    OPCodes_BIT_HL(4);
}

void GZ80::OPCodeCB0x67()
{
    // BIT 4 A
    OPCodes_BIT(AF_.GetHighRegister(), 4);
}

void GZ80::OPCodeCB0x68()
{
    // BIT 5 B
    OPCodes_BIT(BC_.GetHighRegister(), 5);
}

void GZ80::OPCodeCB0x69()
{
    // BIT 5 C
    OPCodes_BIT(BC_.GetLowRegister(), 5);
}

void GZ80::OPCodeCB0x6A()
{
    // BIT 5 D
    OPCodes_BIT(DE_.GetHighRegister(), 5);
}

void GZ80::OPCodeCB0x6B()
{
    // BIT 5 E
    OPCodes_BIT(DE_.GetLowRegister(), 5);
}

void GZ80::OPCodeCB0x6C()
{
    // BIT 5 H
    OPCodes_BIT(HL_.GetHighRegister(), 5);
}

void GZ80::OPCodeCB0x6D()
{
    // BIT 5 L
    OPCodes_BIT(HL_.GetLowRegister(), 5);
}

void GZ80::OPCodeCB0x6E()
{
    // BIT 5 (HL)
    OPCodes_BIT_HL(5);
}

void GZ80::OPCodeCB0x6F()
{
    // BIT 5 A
    OPCodes_BIT(AF_.GetHighRegister(), 5);
}

void GZ80::OPCodeCB0x70()
{
    // BIT 6 B
    OPCodes_BIT(BC_.GetHighRegister(), 6);
}

void GZ80::OPCodeCB0x71()
{
    // BIT 6 C
    OPCodes_BIT(BC_.GetLowRegister(), 6);
}

void GZ80::OPCodeCB0x72()
{
    // BIT 6 D
    OPCodes_BIT(DE_.GetHighRegister(), 6);
}

void GZ80::OPCodeCB0x73()
{
    // BIT 6 E
    OPCodes_BIT(DE_.GetLowRegister(), 6);
}

void GZ80::OPCodeCB0x74()
{
    // BIT 6 H
    OPCodes_BIT(HL_.GetHighRegister(), 6);
}

void GZ80::OPCodeCB0x75()
{
    // BIT 6 L
    OPCodes_BIT(HL_.GetLowRegister(), 6);
}

void GZ80::OPCodeCB0x76()
{
    // BIT 6 (HL)
    OPCodes_BIT_HL(6);
}

void GZ80::OPCodeCB0x77()
{
    // BIT 6 A
    OPCodes_BIT(AF_.GetHighRegister(), 6);
}

void GZ80::OPCodeCB0x78()
{
    // BIT 7 B
    OPCodes_BIT(BC_.GetHighRegister(), 7);
}

void GZ80::OPCodeCB0x79()
{
    // BIT 7 C
    OPCodes_BIT(BC_.GetLowRegister(), 7);
}

void GZ80::OPCodeCB0x7A()
{
    // BIT 7 D
    OPCodes_BIT(DE_.GetHighRegister(), 7);
}

void GZ80::OPCodeCB0x7B()
{
    // BIT 7 E
    OPCodes_BIT(DE_.GetLowRegister(), 7);
}

void GZ80::OPCodeCB0x7C()
{
    // BIT 7 H
    OPCodes_BIT(HL_.GetHighRegister(), 7);
}

void GZ80::OPCodeCB0x7D()
{
    // BIT 7 L
    OPCodes_BIT(HL_.GetLowRegister(), 7);
}

void GZ80::OPCodeCB0x7E()
{
    // BIT 7 (HL)
    OPCodes_BIT_HL(7);
}

void GZ80::OPCodeCB0x7F()
{
    // BIT 7 A
    OPCodes_BIT(AF_.GetHighRegister(), 7);
}

void GZ80::OPCodeCB0x80()
{
    // RES 0 B
    OPCodes_RES(BC_.GetHighRegister(), 0);
}

void GZ80::OPCodeCB0x81()
{
    // RES 0 C
    OPCodes_RES(BC_.GetLowRegister(), 0);
}

void GZ80::OPCodeCB0x82()
{
    // RES 0 D
    OPCodes_RES(DE_.GetHighRegister(), 0);
}

void GZ80::OPCodeCB0x83()
{
    // RES 0 E
    OPCodes_RES(DE_.GetLowRegister(), 0);
}

void GZ80::OPCodeCB0x84()
{
    // RES 0 H
    OPCodes_RES(HL_.GetHighRegister(), 0);
}

void GZ80::OPCodeCB0x85()
{
    // RES 0 L
    OPCodes_RES(HL_.GetLowRegister(), 0);
}

void GZ80::OPCodeCB0x86()
{
    // RES 0 (HL)
    OPCodes_RES_HL(0);
}

void GZ80::OPCodeCB0x87()
{
    // RES 0 A
    OPCodes_RES(AF_.GetHighRegister(), 0);
}

void GZ80::OPCodeCB0x88()
{
    // RES 1 B
    OPCodes_RES(BC_.GetHighRegister(), 1);
}

void GZ80::OPCodeCB0x89()
{
    // RES 1 C
    OPCodes_RES(BC_.GetLowRegister(), 1);
}

void GZ80::OPCodeCB0x8A()
{
    // RES 1 D
    OPCodes_RES(DE_.GetHighRegister(), 1);
}

void GZ80::OPCodeCB0x8B()
{
    // RES 1 E
    OPCodes_RES(DE_.GetLowRegister(), 1);
}

void GZ80::OPCodeCB0x8C()
{
    // RES 1 H
    OPCodes_RES(HL_.GetHighRegister(), 1);
}

void GZ80::OPCodeCB0x8D()
{
    // RES 1 L
    OPCodes_RES(HL_.GetLowRegister(), 1);
}

void GZ80::OPCodeCB0x8E()
{
    // RES 1 (HL)
    OPCodes_RES_HL(1);
}

void GZ80::OPCodeCB0x8F()
{
    // RES 1 A
    OPCodes_RES(AF_.GetHighRegister(), 1);
}

void GZ80::OPCodeCB0x90()
{
    // RES 2 B
    OPCodes_RES(BC_.GetHighRegister(), 2);
}

void GZ80::OPCodeCB0x91()
{
    // RES 2 C
    OPCodes_RES(BC_.GetLowRegister(), 2);
}

void GZ80::OPCodeCB0x92()
{
    // RES 2 D
    OPCodes_RES(DE_.GetHighRegister(), 2);
}

void GZ80::OPCodeCB0x93()
{
    // RES 2 E
    OPCodes_RES(DE_.GetLowRegister(), 2);
}

void GZ80::OPCodeCB0x94()
{
    // RES 2 H
    OPCodes_RES(HL_.GetHighRegister(), 2);
}

void GZ80::OPCodeCB0x95()
{
    // RES 2 L
    OPCodes_RES(HL_.GetLowRegister(), 2);
}

void GZ80::OPCodeCB0x96()
{
    // RES 2 (HL)
    OPCodes_RES_HL(2);
}

void GZ80::OPCodeCB0x97()
{
    // RES 2 A
    OPCodes_RES(AF_.GetHighRegister(), 2);
}

void GZ80::OPCodeCB0x98()
{
    // RES 3 B
    OPCodes_RES(BC_.GetHighRegister(), 3);
}

void GZ80::OPCodeCB0x99()
{
    // RES 3 C
    OPCodes_RES(BC_.GetLowRegister(), 3);
}

void GZ80::OPCodeCB0x9A()
{
    // RES 3 D
    OPCodes_RES(DE_.GetHighRegister(), 3);
}

void GZ80::OPCodeCB0x9B()
{
    // RES 3 E
    OPCodes_RES(DE_.GetLowRegister(), 3);
}

void GZ80::OPCodeCB0x9C()
{
    // RES 3 H
    OPCodes_RES(HL_.GetHighRegister(), 3);
}

void GZ80::OPCodeCB0x9D()
{
    // RES 3 L
    OPCodes_RES(HL_.GetLowRegister(), 3);
}

void GZ80::OPCodeCB0x9E()
{
    // RES 3 (HL)
    OPCodes_RES_HL(3);
}

void GZ80::OPCodeCB0x9F()
{
    // RES 3 A
    OPCodes_RES(AF_.GetHighRegister(), 3);
}

void GZ80::OPCodeCB0xA0()
{
    // RES 4 B
    OPCodes_RES(BC_.GetHighRegister(), 4);
}

void GZ80::OPCodeCB0xA1()
{
    // RES 4 C
    OPCodes_RES(BC_.GetLowRegister(), 4);
}

void GZ80::OPCodeCB0xA2()
{
    // RES 4 D
    OPCodes_RES(DE_.GetHighRegister(), 4);
}

void GZ80::OPCodeCB0xA3()
{
    // RES 4 E
    OPCodes_RES(DE_.GetLowRegister(), 4);
}

void GZ80::OPCodeCB0xA4()
{
    // RES 4 H
    OPCodes_RES(HL_.GetHighRegister(), 4);
}

void GZ80::OPCodeCB0xA5()
{
    // RES 4 L
    OPCodes_RES(HL_.GetLowRegister(), 4);
}

void GZ80::OPCodeCB0xA6()
{
    // RES 4 (HL)
    OPCodes_RES_HL(4);
}

void GZ80::OPCodeCB0xA7()
{
    // RES 4 A
    OPCodes_RES(AF_.GetHighRegister(), 4);
}

void GZ80::OPCodeCB0xA8()
{
    // RES 5 B
    OPCodes_RES(BC_.GetHighRegister(), 5);
}

void GZ80::OPCodeCB0xA9()
{
    // RES 5 C
    OPCodes_RES(BC_.GetLowRegister(), 5);
}

void GZ80::OPCodeCB0xAA()
{
    // RES 5 D
    OPCodes_RES(DE_.GetHighRegister(), 5);
}

void GZ80::OPCodeCB0xAB()
{
    // RES 5 E
    OPCodes_RES(DE_.GetLowRegister(), 5);
}

void GZ80::OPCodeCB0xAC()
{
    // RES 5 H
    OPCodes_RES(HL_.GetHighRegister(), 5);
}

void GZ80::OPCodeCB0xAD()
{
    // RES 5 L
    OPCodes_RES(HL_.GetLowRegister(), 5);
}

void GZ80::OPCodeCB0xAE()
{
    // RES 5 (HL)
    OPCodes_RES_HL(5);
}

void GZ80::OPCodeCB0xAF()
{
    // RES 5 A
    OPCodes_RES(AF_.GetHighRegister(), 5);
}

void GZ80::OPCodeCB0xB0()
{
    // RES 6 B
    OPCodes_RES(BC_.GetHighRegister(), 6);
}

void GZ80::OPCodeCB0xB1()
{
    // RES 6 C
    OPCodes_RES(BC_.GetLowRegister(), 6);
}

void GZ80::OPCodeCB0xB2()
{
    // RES 6 D
    OPCodes_RES(DE_.GetHighRegister(), 6);
}

void GZ80::OPCodeCB0xB3()
{
    // RES 6 E
    OPCodes_RES(DE_.GetLowRegister(), 6);
}

void GZ80::OPCodeCB0xB4()
{
    // RES 6 H
    OPCodes_RES(HL_.GetHighRegister(), 6);
}

void GZ80::OPCodeCB0xB5()
{
    // RES 6 L
    OPCodes_RES(HL_.GetLowRegister(), 6);
}

void GZ80::OPCodeCB0xB6()
{
    // RES 6 (HL)
    OPCodes_RES_HL(6);
}

void GZ80::OPCodeCB0xB7()
{
    // RES 6 A
    OPCodes_RES(AF_.GetHighRegister(), 6);
}

void GZ80::OPCodeCB0xB8()
{
    // RES 7 B
    OPCodes_RES(BC_.GetHighRegister(), 7);
}

void GZ80::OPCodeCB0xB9()
{
    // RES 7 C
    OPCodes_RES(BC_.GetLowRegister(), 7);
}

void GZ80::OPCodeCB0xBA()
{
    // RES 7 D
    OPCodes_RES(DE_.GetHighRegister(), 7);
}

void GZ80::OPCodeCB0xBB()
{
    // RES 7 E
    OPCodes_RES(DE_.GetLowRegister(), 7);
}

void GZ80::OPCodeCB0xBC()
{
    // RES 7 H
    OPCodes_RES(HL_.GetHighRegister(), 7);
}

void GZ80::OPCodeCB0xBD()
{
    // RES 7 L
    OPCodes_RES(HL_.GetLowRegister(), 7);
}

void GZ80::OPCodeCB0xBE()
{
    // RES 7 (HL)
    OPCodes_RES_HL(7);
}

void GZ80::OPCodeCB0xBF()
{
    // RES 7 A
    OPCodes_RES(AF_.GetHighRegister(), 7);
}

void GZ80::OPCodeCB0xC0()
{
    // SET 0 B
    OPCodes_SET(BC_.GetHighRegister(), 0);
}

void GZ80::OPCodeCB0xC1()
{
    // SET 0 C
    OPCodes_SET(BC_.GetLowRegister(), 0);
}

void GZ80::OPCodeCB0xC2()
{
    // SET 0 D
    OPCodes_SET(DE_.GetHighRegister(), 0);
}

void GZ80::OPCodeCB0xC3()
{
    // SET 0 E
    OPCodes_SET(DE_.GetLowRegister(), 0);
}

void GZ80::OPCodeCB0xC4()
{
    // SET 0 H
    OPCodes_SET(HL_.GetHighRegister(), 0);
}

void GZ80::OPCodeCB0xC5()
{
    // SET 0 L
    OPCodes_SET(HL_.GetLowRegister(), 0);
}

void GZ80::OPCodeCB0xC6()
{
    // SET 0 (HL)
    OPCodes_SET_HL(0);
}

void GZ80::OPCodeCB0xC7()
{
    // SET 0 A
    OPCodes_SET(AF_.GetHighRegister(), 0);
}

void GZ80::OPCodeCB0xC8()
{
    // SET 1 B
    OPCodes_SET(BC_.GetHighRegister(), 1);
}

void GZ80::OPCodeCB0xC9()
{
    // SET 1 C
    OPCodes_SET(BC_.GetLowRegister(), 1);
}

void GZ80::OPCodeCB0xCA()
{
    // SET 1 D
    OPCodes_SET(DE_.GetHighRegister(), 1);
}

void GZ80::OPCodeCB0xCB()
{
    // SET 1 E
    OPCodes_SET(DE_.GetLowRegister(), 1);
}

void GZ80::OPCodeCB0xCC()
{
    // SET 1 H
    OPCodes_SET(HL_.GetHighRegister(), 1);
}

void GZ80::OPCodeCB0xCD()
{
    // SET 1 L
    OPCodes_SET(HL_.GetLowRegister(), 1);
}

void GZ80::OPCodeCB0xCE()
{
    // SET 1 (HL)
    OPCodes_SET_HL(1);
}

void GZ80::OPCodeCB0xCF()
{
    // SET 1 A
    OPCodes_SET(AF_.GetHighRegister(), 1);
}

void GZ80::OPCodeCB0xD0()
{
    // SET 2 B
    OPCodes_SET(BC_.GetHighRegister(), 2);
}

void GZ80::OPCodeCB0xD1()
{
    // SET 2 C
    OPCodes_SET(BC_.GetLowRegister(), 2);
}

void GZ80::OPCodeCB0xD2()
{
    // SET 2 D
    OPCodes_SET(DE_.GetHighRegister(), 2);
}

void GZ80::OPCodeCB0xD3()
{
    // SET 2 E
    OPCodes_SET(DE_.GetLowRegister(), 2);
}

void GZ80::OPCodeCB0xD4()
{
    // SET 2 H
    OPCodes_SET(HL_.GetHighRegister(), 2);
}

void GZ80::OPCodeCB0xD5()
{
    // SET 2 L
    OPCodes_SET(HL_.GetLowRegister(), 2);
}

void GZ80::OPCodeCB0xD6()
{
    // SET 2 (HL)
    OPCodes_SET_HL(2);
}

void GZ80::OPCodeCB0xD7()
{
    // SET 2 A
    OPCodes_SET(AF_.GetHighRegister(), 2);
}

void GZ80::OPCodeCB0xD8()
{
    // SET 3 B
    OPCodes_SET(BC_.GetHighRegister(), 3);
}

void GZ80::OPCodeCB0xD9()
{
    // SET 3 C
    OPCodes_SET(BC_.GetLowRegister(), 3);
}

void GZ80::OPCodeCB0xDA()
{
    // SET 3 D
    OPCodes_SET(DE_.GetHighRegister(), 3);
}

void GZ80::OPCodeCB0xDB()
{
    // SET 3 E
    OPCodes_SET(DE_.GetLowRegister(), 3);
}

void GZ80::OPCodeCB0xDC()
{
    // SET 3 H
    OPCodes_SET(HL_.GetHighRegister(), 3);
}

void GZ80::OPCodeCB0xDD()
{
    // SET 3 L
    OPCodes_SET(HL_.GetLowRegister(), 3);
}

void GZ80::OPCodeCB0xDE()
{
    // SET 3 (HL)
    OPCodes_SET_HL(3);
}

void GZ80::OPCodeCB0xDF()
{
    // SET 3 A
    OPCodes_SET(AF_.GetHighRegister(), 3);
}

void GZ80::OPCodeCB0xE0()
{
    // SET 4 B
    OPCodes_SET(BC_.GetHighRegister(), 4);
}

void GZ80::OPCodeCB0xE1()
{
    // SET 4 C
    OPCodes_SET(BC_.GetLowRegister(), 4);
}

void GZ80::OPCodeCB0xE2()
{
    // SET 4 D
    OPCodes_SET(DE_.GetHighRegister(), 4);
}

void GZ80::OPCodeCB0xE3()
{
    // SET 4 E
    OPCodes_SET(DE_.GetLowRegister(), 4);
}

void GZ80::OPCodeCB0xE4()
{
    // SET 4 H
    OPCodes_SET(HL_.GetHighRegister(), 4);
}

void GZ80::OPCodeCB0xE5()
{
    // SET 4 L
    OPCodes_SET(HL_.GetLowRegister(), 4);
}

void GZ80::OPCodeCB0xE6()
{
    // SET 4 (HL)
    OPCodes_SET_HL(4);
}

void GZ80::OPCodeCB0xE7()
{
    // SET 4 A
    OPCodes_SET(AF_.GetHighRegister(), 4);
}

void GZ80::OPCodeCB0xE8()
{
    // SET 5 B
    OPCodes_SET(BC_.GetHighRegister(), 5);
}

void GZ80::OPCodeCB0xE9()
{
    // SET 5 C
    OPCodes_SET(BC_.GetLowRegister(), 5);
}

void GZ80::OPCodeCB0xEA()
{
    // SET 5 D
    OPCodes_SET(DE_.GetHighRegister(), 5);
}

void GZ80::OPCodeCB0xEB()
{
    // SET 5 E
    OPCodes_SET(DE_.GetLowRegister(), 5);
}

void GZ80::OPCodeCB0xEC()
{
    // SET 5 H
    OPCodes_SET(HL_.GetHighRegister(), 5);
}

void GZ80::OPCodeCB0xED()
{
    // SET 5 L
    OPCodes_SET(HL_.GetLowRegister(), 5);
}

void GZ80::OPCodeCB0xEE()
{
    // SET 5 (HL)
    OPCodes_SET_HL(5);
}

void GZ80::OPCodeCB0xEF()
{
    // SET 5 A
    OPCodes_SET(AF_.GetHighRegister(), 5);
}

void GZ80::OPCodeCB0xF0()
{
    // SET 6 B
    OPCodes_SET(BC_.GetHighRegister(), 6);
}

void GZ80::OPCodeCB0xF1()
{
    // SET 6 C
    OPCodes_SET(BC_.GetLowRegister(), 6);
}

void GZ80::OPCodeCB0xF2()
{
    // SET 6 D
    OPCodes_SET(DE_.GetHighRegister(), 6);
}

void GZ80::OPCodeCB0xF3()
{
    // SET 6 E
    OPCodes_SET(DE_.GetLowRegister(), 6);
}

void GZ80::OPCodeCB0xF4()
{
    // SET 6 H
    OPCodes_SET(HL_.GetHighRegister(), 6);
}

void GZ80::OPCodeCB0xF5()
{
    // SET 6 L
    OPCodes_SET(HL_.GetLowRegister(), 6);
}

void GZ80::OPCodeCB0xF6()
{
    // SET 6 (HL)
    OPCodes_SET_HL(6);
}

void GZ80::OPCodeCB0xF7()
{
    // SET 6 A
    OPCodes_SET(AF_.GetHighRegister(), 6);
}

void GZ80::OPCodeCB0xF8()
{
    // SET 7 B
    OPCodes_SET(BC_.GetHighRegister(), 7);
}

void GZ80::OPCodeCB0xF9()
{
    // SET 7 C
    OPCodes_SET(BC_.GetLowRegister(), 7);
}

void GZ80::OPCodeCB0xFA()
{
    // SET 7 D
    OPCodes_SET(DE_.GetHighRegister(), 7);
}

void GZ80::OPCodeCB0xFB()
{
    // SET 7 E
    OPCodes_SET(DE_.GetLowRegister(), 7);
}

void GZ80::OPCodeCB0xFC()
{
    // SET 7 H
    OPCodes_SET(HL_.GetHighRegister(), 7);
}

void GZ80::OPCodeCB0xFD()
{
    // SET 7 L
    OPCodes_SET(HL_.GetLowRegister(), 7);
}

void GZ80::OPCodeCB0xFE()
{
    // SET 7 (HL)
    OPCodes_SET_HL(7);
}

void GZ80::OPCodeCB0xFF()
{
    // SET 7 A
    OPCodes_SET(AF_.GetHighRegister(), 7);
}

} // namespace gz80
