#ifndef _DMP_INT_H_
#define _DMP_INT_H_


void (*callback)(void);

int stm_reg_int_cb(void (*cb)(void));


/**
 *  @brief  Enable interrupts.
 *  @return 0 if successful.
 */
int int_enable(void);


/**
 *  @brief  Disable interrupts.
 *  @return 0 if successful.
 */
int int_disable(void);

#endif
