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

#ifndef GZ80_OPCODEEDNAMES_H_
#define	GZ80_OPCODEEDNAMES_H_

namespace gz80
{
   
#ifdef GZ80_DISASM

static const char* kOPCodeEDNames[256] = {
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",

    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",

    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",

    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",

    "IN B,(C)",
    "OUT (C),B",
    "SBC HL,BC",
    "LD (nn),BC",
    "NEG",
    "RETN",
    "IM 0",
    "LD I,A",
    "IN C,(C)",
    "OUT (C),C",
    "ADC HL,BC",
    "LD BC,(nn)",
    "NEG*",
    "RETI",
    "IM 0*",
    "LD R,A",

    "IN D,(C)",
    "OUT (C),D",
    "SBC HL,DE",
    "LD (nn),DE",
    "NEG*",
    "RETN*",
    "IM 1",
    "LD A,I",
    "IN E,(C)",
    "OUT (C),E",
    "ADC HL,DE",
    "LD DE,(nn)",
    "NEG*",
    "RETN*",
    "IM 2",
    "LD A,R",

    "IN H,(C)",
    "OUT (C),H",
    "SBC HL,HL",
    "LD (nn),HL",
    "NEG*",
    "RETN*",
    "IM 0*",
    "RRD",
    "IN L,(C)",
    "OUT (C),L",
    "ADC HL,HL",
    "LD HL,(nn)",
    "NEG*",
    "RETN*",
    "IM 0*",
    "RLD",

    "IN F,(C)*",
    "OUT (C),0*",
    "SBC HL,SP",
    "LD (nn),SP",
    "NEG*",
    "RETN*",
    "IM 1*",
    "INVALID",
    "IN A,(C)",
    "OUT (C),A",
    "ADC HL,SP",
    "LD SP,(nn)",
    "NEG*",
    "RETN*",
    "IM 2*",
    "INVALID",

    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",

    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",

    "LDI",
    "CPI",
    "INI",
    "OUTI",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "LDD",
    "CPD",
    "IND",
    "OUTD",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",

    "LDIR",
    "CPIR",
    "INIR",
    "OTIR",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "LDDR",
    "CPDR",
    "INDR",
    "OTDR",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",

    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",

    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",

    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",

    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
    "INVALID",
};

#endif

} // namespace gz80

#endif // GZ80_OPCODEEDNAMES_H_

