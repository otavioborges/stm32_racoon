#include "geardrive.h"
#include "genesis.h"

#define CARTRIDGE_LOCATION

GeardriveCore genesis;

void GENESIS_Init(uint32_t addr, uint32_t size){
	genesis.Init();
	Cartridge* c = genesis.GetCartridge();
	c->LoadFromBuffer((uint8_t *)addr, size);

	genesis.ResetROM();
}

extern "C"{
void GENESIS_CInit(uint32_t addr, uint32_t size){
	GENESIS_Init(addr, size);
}
}
