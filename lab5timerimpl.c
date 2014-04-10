#include "mytimer.h"

void MYTIMER_init()
{
    // we don't have to do anything.
}

void MYTIMER_enable()
{
    MYTIMER->control |= MYTIMER_ENABLE_MASK;
}

void MYTIMER_disable()
{
    MYTIMER->control &= ~MYTIMER_ENABLE_MASK;
}

void MYTIMER_setOverflowVal(uint32_t value)
{
	// Yes it's inefficient, but it's written this way to
	// show you the C to assembly mapping.
    uint32_t * timerAddr = (uint32_t*)(MYTIMER);
    *timerAddr = value; // overflowReg is at offset 0x0
}

uint32_t MYTIMER_getCounterVal()
{

// Yes it's inefficient, but it's written this way to
// show you the C to assembly mapping.
    uint32_t * timerAddr = (uint32_t*)(MYTIMER);
    return *(timerAddr+1); // counterReg is at offset 0x4
}


/*********************************************************************/

/**
 * Enable all interrupts
 */
void MYTIMER_enable_allInterrupts() {
	MYTIMER->control |= ALLINT_ENABLE_MASK;
}
/**
 * Disable all interrupts
 */

void MYTIMER_disable_allInterrupts(){
	MYTIMER->control &= ~ALLINT_ENABLE_MASK;
}
/**
 * Enable compare interrupt
 */

void MYTIMER_enable_compareInt(){
	MYTIMER->control |= CMPINT_ENABLE_MASK;
}
/**
 * Disable compare interrupt
 */

void MYTIMER_disable_compareInt(){
	MYTIMER->control &= ~CMPINT_ENABLE_MASK;
}
/**
 * Set Compare value
 */

void MYTIMER_setCompareVal(uint32_t compare){
	if(compare < MYTIMER->overflow)
		MYTIMER->compare = compare;
}
/**
 * Enable overflow interrupt
 */

void MYTIMER_enable_overflowInt(){
	MYTIMER->control |= OVERFLOWINT_ENABLE_MASK;
}
/**
 * Disable overflow interrupt
 */

void MYTIMER_disable_overflowInt(){
	MYTIMER->control &= ~OVERFLOWINT_ENABLE_MASK;
}
/**
  * Interrupt status
*/

uint32_t MYTIMER_getInterrupt_status(){
	return MYTIMER->status;
}

/**
 * Enable Capture
 */
void MYTIMER_enable_capture() {
	MYTIMER->control |= CAPTUREINT_ENABLE_MASK;
}
/**
 * Disable Capture
 */
void MYTIMER_disable_capture(){
	MYTIMER->control &= ~CAPTUREINT_ENABLE_MASK;
}
/**
 * Read the synchronous capture value
*/

uint32_t MYTIMER_get_sync_capture() {
	return MYTIMER->sync;
}
/**
 * Read the asynchronous capture value
*/

uint32_t MYTIMER_get_async_capture() {
	return MYTIMER->async;
}
