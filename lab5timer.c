#include <stdio.h>
#include "drivers/mss_uart/mss_uart.h"
#include "mytimer.h"

volatile uint32_t count;
volatile uint32_t compareVal;
volatile uint32_t firstEdge;
__attribute__ ((interrupt)) void Fabric_IRQHandler( void )
{
    //uint32_t time = MYTIMER_getCounterVal();
   // uint32_t status = MYTIMER_getInterrupt_status();
    uint32_t getSync = MYTIMER_get_sync_capture();
    //uint32_t getAsync = MYTIMER_get_async_capture();

    uint32_t latency = 0;
    uint32_t distance = 0;

    if(getSync > firstEdge) latency = getSync - firstEdge;
    else latency = 10000000;//firstEdge - getSync;
    if(latency < 1000000) {
    	latency = latency * 40e-6 * 1e3;
    	distance = latency / 58;
    	printf("Measure distance from target is: %lu\n\r",distance);
    }
    firstEdge = getSync;
   /* printf("Sync Capture Time: %lu\n\rAsync Capture Time: %lu\n\r",getSync,getAsync);
    printf("Sync and Async Latency %lu\n\r",latency);*/
	count--;

   /* printf("Interrupt occurred at %lu FABINT \n\r", time);

    if(status & 0x01)
    {
    	compareVal -= 4000;
    	MYTIMER_setCompareVal(compareVal);
    	if(compareVal == 0) compareVal = 400000;
       // printf("Overflow latency %ld\n\r", 0-time);
    }
    if(status & 0x02)
    {
        //printf("Compare latency %ld\n\r", (1<<27) - time);
    }
    */

    NVIC_ClearPendingIRQ( Fabric_IRQn );
}
int main()
{
	compareVal = 2400000 - 400;
	firstEdge = 0;
   /* Setup MYTIMER */
    MYTIMER_init();
    MYTIMER_setOverflowVal(2400000); // count required to make 100 Hz clock from 40 MHz


    MYTIMER_setCompareVal(compareVal); // reset compare Val
    //MYTIMER_enable_overflowInt();
   // MYTIMER_enable_compareInt();
    //MYTIMER_enable_allInterrupts();
    MYTIMER_enable_capture();
    NVIC_EnableIRQ(Fabric_IRQn);
    MYTIMER_enable();

    count = 10000000;
    while(1){
    	if(!count){ // Global variable checking
	// count decremented inside the interrupt service routine
    		MYTIMER_disable(); // Disable after 1e6 interrupts
    	}
    }
    return 0;
}
