/*
 * genesis.h
 *
 *  Created on: Dec 18, 2018
 *      Author: otavio
 */

#ifndef INC_GENESIS_H_
#define INC_GENESIS_H_

#include <stdint.h>

void GENESIS_Init(uint32_t addr, uint32_t size);

#ifdef __cplusplus
extern "C"{
#endif
void GENESIS_CInit(uint32_t addr, uint32_t size);
#ifdef __cplusplus
}
#endif

#endif /* INC_GENESIS_H_ */
