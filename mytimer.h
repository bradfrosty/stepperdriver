#ifndef MYTIMER_H_  // Only define once
#define MYTIMER_H_  // Only define once

#include "CMSIS/a2fxxxm3.h"

#define MYTIMER_BASE (FPGA_FABRIC_BASE + 0x0)

// The technique of using a structure declaration
// to describe the device register layout and names is
// very common practice. Notice that there aren't actually
// any objects of that type defined, so the declaration
// simply indicates the structure without using up any store.

typedef struct
{
    uint32_t overflow; // Offset 0x0
    uint32_t counter; // Offset 0x4
    uint32_t control; // Offset 0x8
} mytimer_t;

#define MYTIMER_ENABLE_MASK 0x00000001UL

// Using the mytimer_t structure we can make the
// compiler do the offset mapping for us.
// To access the device registers, an appropriately
// cast constant is used as if it were pointing to
// such a structure, but of course it points to memory addresses instead.
// Look at at mytimer.c
// Look at the the functions's disassembly
// in .lst file under the Debug folder

#define MYTIMER ((mytimer_t *) MYTIMER_BASE)

/**
 * Initialize the MYTIMER
*/
void MYTIMER_init();

/**
 * Start MYTIMER
*/

void MYTIMER_enable();
/**
 * Stop MYTIMER
*/

void MYTIMER_disable();

/**
 * Set the limit to which the timer counts.
*/

void MYTIMER_setOverflowVal(uint32_t value);
/**
 * Read the counter value of the timer.
*/

uint32_t MYTIMER_getCounterVal();

#endif /* MYTIMER_H_ */
