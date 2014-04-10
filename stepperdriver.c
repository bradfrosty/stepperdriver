#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include "drivers/mss_gpio/mss_gpio.h"
#include "drivers/mss_timer/mss_timer.h"
#include "drivers/mss_uart/mss_uart.h"
#include "mytimer.h"




#define CW 0										//motor direction: clockwise
#define CCW 1										//counterclockwise

#define ANGLEPERSTEP  1.8 							//enter angle per step of motor here
#define T1_FREQ	100000								//timer freq at 500 kHz
#define FSPR 360/ANGLEPERSTEP						//steps per revolution

#define ALPHA (2*3.14159)/FSPR						//motor step angle
#define A_T_x100 ((long)(ALPHA*T1_FREQ*100))		//(ALPHA / T1_FREQ)*100
#define T1_FREQ_148 ((uint32_t)((T1_FREQ*0.676)/100)) 	// divided by 100 and scaled by 0.676
#define A_SQ (long)(ALPHA*2*10000000000)         	// ALPHA*2*10000000000
#define A_x20000 (uint32_t)(ALPHA*20000)              	// ALPHA*20000

typedef struct {
  //! What part of the speed ramp we are in.
  unsigned char run_state : 3;
  //! Direction stepper motor should move.
  unsigned char dir : 1;
  //! Period of next timer delay. At start this value sets the acceleration rate.
  uint32_t step_delay;
  //! What step_pos to start deceleration
  uint32_t decel_start;
  //! Sets deceleration rate.
  int32_t decel_val;
  //! Minimum time delay (max speed)
  int32_t min_delay;
  //! Counter used when accelerating/decelerating to calculate step_delay.
  int32_t accel_count;
} speedRampData;

// Speed ramp states
#define STOP  0
#define ACCEL 1
#define DECEL 2
#define RUN   3



int32_t stepPosition;					//motor position

//array of different positions within a step
//if using full stepping, increment array index by 2
//otherwise, increment array index by 1 for half stepping

unsigned char stepOut[] = { MSS_GPIO_0_MASK |
							MSS_GPIO_3_MASK,
							MSS_GPIO_0_MASK |
							MSS_GPIO_2_MASK,
							MSS_GPIO_1_MASK |
							MSS_GPIO_2_MASK,
							MSS_GPIO_1_MASK |
							MSS_GPIO_3_MASK
							};

/*unsigned char stepOut[] = { MSS_GPIO_0_MASK,
							MSS_GPIO_3_MASK,
							MSS_GPIO_1_MASK,
							MSS_GPIO_2_MASK
							};
*/

//Driver functions
void initGPIO(void);
void initTimer(void);
void initMotor(void);
unsigned char stepCounter(char dir);
void outputStep(unsigned char stepSeq);

//Speed Control Functions
speedRampData srd;
void move(signed int step, unsigned int accel, unsigned int decel, unsigned int speed);
static unsigned long sqrt(unsigned long x);
unsigned int min(unsigned int x, unsigned int y);


int main()
{
		initGPIO();
		initMotor();
		initTimer();

		//uint32_t timerVal = MSS_TIM1_get_current_value();

		/* Setup MYTIMER */
		MYTIMER_init();
		MYTIMER_setOverflowVal((1<<31)); // Set compare to a big number
		MYTIMER_enable();




		while(1) {
			//timerVal = MSS_TIM1_get_current_value();
			//printf("%lu\n\r",timerVal);
			uint32_t i;
			printf("Time: %lu\r\n", MYTIMER_getCounterVal()); // Standard printf() now work
			for(i=1e6; i>0; i--); // busy wait
		}



		return 0;
}


void initGPIO() {
	//initalize MSS gpio block
	MSS_GPIO_init();

	//configure MSS gpio pins 0 to 3 as outputs
	MSS_GPIO_config(MSS_GPIO_0, MSS_GPIO_OUTPUT_MODE);
	MSS_GPIO_config(MSS_GPIO_1, MSS_GPIO_OUTPUT_MODE);
	MSS_GPIO_config(MSS_GPIO_2, MSS_GPIO_OUTPUT_MODE);
	MSS_GPIO_config(MSS_GPIO_3, MSS_GPIO_OUTPUT_MODE);

}

void initTimer() {
	MSS_TIM1_init(MSS_TIMER_PERIODIC_MODE); 			// initialize Timer 1 in the MSS in periodic mode
	MSS_TIM1_load_background( T1_FREQ ); 				//load value to count down from
	MSS_TIM1_enable_irq();								//enable interrupts to be fired at end of timer count (enables motor)
	MSS_TIM1_start();
}


char dir=CW;
__attribute__ ((interrupt)) void Timer1_IRQHandler(void) {

	// Holds next delay period.
	unsigned int new_step_delay;
	// Remember the last step delay used when accelrating.
	static int last_accel_delay;
	// Counting steps when moving.
	static unsigned int step_count = 0;
	// Keep track of remainder from new_step-delay calculation to incrase accurancy
	static unsigned int rest = 0;

	/*
	 * set pwm value here for acceleration change
	 */

	switch(srd.run_state) {
	    case STOP:
	      step_count = 0;
	      rest = 0;
	      /*
	       * DISABLE TIMER HERE TO STOP MOTOR FROM STEPPING
	       */


	      break;

	    case ACCEL:
	      stepCounter(srd.dir);
	      step_count++;
	      srd.accel_count++;
	      new_step_delay = srd.step_delay - (((2 * (long)srd.step_delay) + rest)/(4 * srd.accel_count + 1));
	      rest = ((2 * (long)srd.step_delay)+rest)%(4 * srd.accel_count + 1);
	      // Check if we should start deceleration.
	      if(step_count >= srd.decel_start) {
	        srd.accel_count = srd.decel_val;
	        srd.run_state = DECEL;
	      }
	      // Check if we hit max speed.
	      else if(new_step_delay <= srd.min_delay) {
	        last_accel_delay = new_step_delay;
	        new_step_delay = srd.min_delay;
	        rest = 0;
	        srd.run_state = RUN;
	      }
	      break;

	    case RUN:
	      stepCounter(srd.dir);
	      step_count++;
	      new_step_delay = srd.min_delay;
	      // Check if we should start decelration.
	      if(step_count >= srd.decel_start) {
	        srd.accel_count = srd.decel_val;
	        // Start deceleration with same delay as accel ended with.
	        new_step_delay = last_accel_delay;
	        srd.run_state = DECEL;
	      }
	      break;

	    case DECEL:
	      stepCounter(srd.dir);
	      step_count++;
	      srd.accel_count++;
	      new_step_delay = srd.step_delay - (((2 * (long)srd.step_delay) + rest)/(4 * srd.accel_count + 1));
	      rest = ((2 * (long)srd.step_delay)+rest)%(4 * srd.accel_count + 1);
	      // Check if we are at last step
	      if(srd.accel_count >= 0){
	        srd.run_state = STOP;
	      }
	      break;
	  }
	  srd.step_delay = new_step_delay;





	//stepCounter(dir);

	 MSS_TIM1_clear_irq();

}



/*********************************************************************************************/
//DRIVER FUNCTIONS
/*********************************************************************************************/


//initializes the motor pins to 0
void initMotor(void) {

	uint32_t gpio_outputs =  MSS_GPIO_get_outputs();
	gpio_outputs &= ~( MSS_GPIO_0_MASK | MSS_GPIO_1_MASK | MSS_GPIO_2_MASK | MSS_GPIO_3_MASK );
	MSS_GPIO_set_outputs(  gpio_outputs );
	srd.run_state = STOP;

}


//keeps track of the step position and increments the counter in the step drive.
//pulls the position of step sequence from the stepOut array.
unsigned char counter = 0;

unsigned char stepCounter(char dir) {


	if(dir == CW) {
		//stepPosition++;
		counter++;
	}
	else {
		//stepPosition--;
		counter--;
	}
	//keeps the counter within the range of the array
	counter &= 0x03;

	outputStep(counter);
	return counter;

}


//sends the step data to the motor
void outputStep(unsigned char stepSeq) {

	unsigned char temp = stepOut[stepSeq];

	MSS_GPIO_set_outputs(temp);

	return;
}


/*********************************************************************************************/
//Speed Control Functions
/*********************************************************************************************/



void move(signed int step, unsigned int accel, unsigned int decel, unsigned int speed) {

	//! Number of steps before we hit max speed.
	unsigned int max_s_lim;
	//! Number of steps before we must start deceleration (if accel does not hit max speed).
	unsigned int accel_lim;

	// Set direction from sign on step value.
	if(step < 0){
	srd.dir = CCW;
	step = -step;
	}
	else{
	srd.dir = CW;
	}

	// If moving only 1 step.
	if(step == 1){
		// Move one step...
		srd.accel_count = -1;
		// ...in DECEL state.
		srd.run_state = DECEL;
		// Just a short delay so main() can act on 'running'.
		srd.step_delay = 1000;
		/*
		Run timer once HERE
		*/
	}
	else if(step != 0){
	    // Refer to documentation for detailed information about these calculations.

	    // Set max speed limit, by calc min_delay to use in timer.
	    // min_delay = (alpha / tt)/ w
	    srd.min_delay = A_T_x100 / speed;

	    // Set accelration by calc the first (c0) step delay .
	    // step_delay = 1/tt * sqrt(2*alpha/accel)
	    // step_delay = ( tfreq*0.676/100 )*100 * sqrt( (2*alpha*10000000000) / (accel*100) )/10000
	    srd.step_delay = (T1_FREQ_148 * sqrt(A_SQ / accel))/100;

	    // Find out after how many steps does the speed hit the max speed limit.
	    // max_s_lim = speed^2 / (2*alpha*accel)
	    max_s_lim = (long)speed*speed/(long)(((long)A_x20000*accel)/100);
	    // If we hit max speed limit before 0,5 step it will round to 0.
	    // But in practice we need to move atleast 1 step to get any speed at all.
	    if(max_s_lim == 0){
	      max_s_lim = 1;
	    }

	    // Find out after how many steps we must start deceleration.
	    // n1 = (n1+n2)decel / (accel + decel)
	    accel_lim = ((long)step*decel) / (accel+decel);
	    // We must accelrate at least 1 step before we can start deceleration.
	    if(accel_lim == 0){
	      accel_lim = 1;
	    }

	    // Use the limit we hit first to calc decel.
	    if(accel_lim <= max_s_lim){
	      srd.decel_val = accel_lim - step;
	    }
	    else{
	      srd.decel_val = -((long)max_s_lim*accel)/decel;
	    }
	    // We must decelrate at least 1 step to stop.
	    if(srd.decel_val == 0){
	      srd.decel_val = -1;
	    }

	    // Find step to start decleration.
	    srd.decel_start = step + srd.decel_val;

	    // If the maximum speed is so low that we don't need to go via acceleration state.
	    if(srd.step_delay <= srd.min_delay){
	      srd.step_delay = srd.min_delay;
	      srd.run_state = RUN;
	    }
	    else{
	      srd.run_state = ACCEL;
	    }

	    // Reset counter.
	    srd.accel_count = 0;
	    /*
	     * RUN TIMER HERE WITH SPECIFIED COMPARE VAL
	     */

	  }
	}


static unsigned long sqrt(unsigned long x)
{
  register unsigned long xr;  // result register
  register unsigned long q2;  // scan-bit register
  register unsigned char f;   // flag (one bit)

  xr = 0;                     // clear result
  q2 = 0x40000000L;           // higest possible result bit
  do
  {
    if((xr + q2) <= x)
    {
      x -= xr + q2;
      f = 1;                  // set flag
    }
    else{
      f = 0;                  // clear flag
    }
    xr >>= 1;
    if(f){
      xr += q2;               // test flag
    }
  } while(q2 >>= 2);          // shift twice
  if(xr < x){
    return xr +1;             // add for rounding
  }
  else{
    return xr;
  }
}

/*! \brief Find minimum value.
 *
 *  Returns the smallest value.
 *
 *  \return  Min(x,y).
 */
unsigned int min(unsigned int x, unsigned int y)
{
  if(x < y){
    return x;
  }
  else{
    return y;
  }
}
