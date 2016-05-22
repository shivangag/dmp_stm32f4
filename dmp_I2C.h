#ifndef _DMP_I2C_H_
#define _DMP_I2C_H_


/**

 *  @brief	Set up the I2C port and configure the MSP430 as the master.

 *  @return	0 if successful.

 */

int i2c_enable(void);

/**
 *  @brief  Disable I2C communication.
 *  This function will disable the I2C hardware and should be called prior to
 *  entering low-power mode.
 *  @return 0 if successful.
 */

int i2c_disable(void);

/**
 *  @brief      Write to a device register.
 *
 *  @param[in]  slave_addr  Slave address of device.
 *  @param[in]  reg_addr	Slave register to be written to.
 *  @param[in]  length      Number of bytes to write.
 *  @param[out] data        Data to be written to register.
 *
 *  @return     0 if successful.
 */

int i2c_write(unsigned char slave_addr,
			 unsigned char reg_addr,
			 unsigned char length,
			 unsigned char const *data);

/**
 *  @brief      Read from a device.
 *
 *  @param[in]  slave_addr  Slave address of device.
 *  @param[in]  reg_addr	Slave register to be read from.
 *  @param[in]  length      Number of bytes to read.
 *  @param[out] data        Data from register.
 *
 *  @return     0 if successful.
 */

int i2c_read(unsigned char slave_addr,
			 unsigned char reg_addr,
			 unsigned char length,
			 unsigned char *data);

int16_t i2c_start(unsigned char reg_addr, uint8_t direction, uint8_t ack);

void i2c_writedata(unsigned char data);

unsigned char i2c_readack();

unsigned char i2c_readnack();

uint8_t i2c_stop();

#endif  /* _STM32F4_I2C_H_ */
