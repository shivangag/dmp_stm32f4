#ifndef _DMP_CLOCK_H_
#define _DMP_CLOCK_H_



/**
 *  @brief      Set the frequency of MCLK, SMCLK, and ACLK.
 *  @param[in]  mclk    Frequency of master clock.
 *  @param[in]  xt      1 if XT1 is present, 2 if XT2 is present, 0 otherwise.
 *  @return     0 if successful.
 */

int clock_init(void);

/**
 *  @brief  Enable the millisecond timer.
 *  This function is automatically called by @e msp430_clock_init. It should be
 *  used to re-enable the timer after @e msp430_clock_disable is called.
 *  @return 0 if successful.
 */

int clock_enable(void);

/**
 *  @brief  Disable the millisecond timer.
 *  This function should be used prior to entering a low-power mode.
 *  @return 0 if successful.
 */

int clock_disable(void);

/**
 *  @brief      Get current clock count.
 *  Timer overflow will occur after 2^32 milliseconds.
 *  @param[out] count   Timer count in milliseconds.
 *  @return      0 if successful.
 */

int get_clock_ms(unsigned long *count);

/**
 *  @brief      Perform a blocking delay.
 *  @param[in]  num_ms  Number of milliseconds to delay.
 *  @return     0 if successful.
 */

int delay_ms(unsigned long num_ms);

/**
 *  @brief      Slow down the timer.
 *  By default, a millisecond timer is used for timing/scheduling purposes.
 *  This API can be used to slow down the interval at which this clock is
 *  updated, saving power by reducing interrupts.
 *  @param[in]  slow    1 to slow down timer.
 *  @return     0 if successful.
 */

//int msp430_slow_timer(unsigned char slow);

/**

 *  @brief      Register a one-time timer event.
 *  Only one event can be registered. If this function is called before the
 *  current timer ends, the new event will be registered and the current one
 *  will be discarded.
 *  @param[in]  timer_cb    Function called when timer is expired.
 *  @param[in]  num_ms      Number of milliseconds before function is called.
 *  @return     0 if successful.
 */

int register_timer_cb(void (*timer_cb)(void), unsigned long num_ms);

#endif  /* _MSP430_CLOCK_H_ */
