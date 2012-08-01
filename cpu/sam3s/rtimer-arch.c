#include <stdio.h>
#include <board.h>
#include "rtimer-arch.h"
#include "sys/rtimer.h"

static int time_msb = 0;
static volatile rtimer_clock_t next_rtimer_time = 0;
static volatile uint8_t armed = 0;

void
TC1_IrqHandler(void)
{
  uint32_t status = TC0->TC_CHANNEL[1].TC_SR;
  uint32_t mask = TC0->TC_CHANNEL[1].TC_IMR;
  status = status & mask;

  if( status & TC_IER_CPAS ){
    TC0->TC_CHANNEL[ 1 ].TC_IDR = TC_IDR_CPAS;
    rtimer_run_next();
  }

  if( status & TC_IER_COVFS ){
    time_msb++;
    rtimer_clock_t now =  ((rtimer_clock_t)time_msb << 16);//|TIM1_CNT;
    rtimer_clock_t clock_to_wait = next_rtimer_time - now;
    //if(armed && clock_to_wait <= 0x10000 && clock_to_wait > 0){
    if(clock_to_wait <= 0x10000 && clock_to_wait > 0){
        TC0->TC_CHANNEL[1].TC_RA = clock_to_wait;
        TC0->TC_CHANNEL[ 1 ].TC_IER = TC_IER_CPAS;
        //armed = 0;
    }
    return;
  }
}

void
rtimer_arch_init(void)
{
  //enable clock
  PMC->PMC_PCER0 = 1 << ID_TC1;

  //free running, 128 prescaler
  TC_Configure( TC0, 1, TC_CMR_TCCLKS_TIMER_CLOCK4 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC );
  TC_Start( TC0, 1 );
  NVIC_EnableIRQ( (IRQn_Type)ID_TC1 );
  TC0->TC_CHANNEL[ 1 ].TC_IER = TC_IER_COVFS;

  TC0->TC_CHANNEL[ 1 ].TC_RA = 0;
}

rtimer_clock_t rtimer_arch_now(void)
{
    return ((rtimer_clock_t)time_msb << 16) + TC0->TC_CHANNEL[ 1 ].TC_CV;
}

void
rtimer_arch_schedule(rtimer_clock_t t)
{
    next_rtimer_time = t;
    rtimer_clock_t now = rtimer_arch_now();
    rtimer_clock_t clock_to_wait = t - now;
    if(clock_to_wait <= 0x10000){
      TC0->TC_CHANNEL[1].TC_RA = (uint16_t)now + (uint16_t)clock_to_wait;
      TC0->TC_CHANNEL[1].TC_IER = TC_IER_CPAS;
    }else{
      armed = 1;
    } 
}

