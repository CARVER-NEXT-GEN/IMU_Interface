/*
 * SRAM4.h
 *
 *  Created on: Nov 6, 2024
 *      Author: tim
 */

#ifndef SRAM4_H_
#define SRAM4_H_

#include "main.h"
#define SHARED_MEMORY_ADDRESS_BNO086 0x38000000 //SRAM4 address
#define SHARED_MEMORY_ADDRESS_BNO055 0x38002000 //SRAM4 address

typedef struct{
	uint8_t State1;
	double Data[40]; //Out of Complier scope So can bigger sa it not more than 64kB which not notice us

}SharedMemory;


extern volatile SharedMemory *const SRAM4;

#endif /* SRAM4_H_ */
