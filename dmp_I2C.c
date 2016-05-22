#include <stm32f4xx.h>
#include <stm32f4xx_i2c.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include "dmp_I2C.h"

#define I2C_TIMEOUT_MS  (2500)


/* Private variables */
static uint32_t I2C_Timeout;
/* Private defines */
#ifndef I2C_TIMEOUT
#define I2C_TIMEOUT					20000
#endif


#define I2C_TRANSMITTER_MODE   0
#define I2C_RECEIVER_MODE      1
#define I2C_ACK_ENABLE         1
#define I2C_ACK_DISABLE        0


int i2c_enable(void){

	GPIO_InitTypeDef GPIO_InitStruct;
	I2C_InitTypeDef I2C_InitStruct;

	// enable APB1 peripheral clock for I2C1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	// enable clock for SCL and SDA pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* setup SCL and SDA pins
	 * You can connect I2C1 to two different
	 * pairs of pins:
	 * 1. SCL on PB6 and SDA on PB7
	 * 2. SCL on PB8 and SDA on PB9
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // we are going to use PB6 and PB7
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;			// set pins to alternate function
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// set GPIO speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;			// set output to open drain --> the line has to be only pulled low, not driven high
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// enable pull up resistors
	GPIO_Init(GPIOB, &GPIO_InitStruct);					// init GPIOB

	// Connect I2C1 pins to AF
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);	// SCL
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1); // SDA


	I2C_InitStruct.I2C_ClockSpeed = 100000; 		// 100kHz
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;			// I2C mode
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;	// 50% duty cycle --> standard
	I2C_InitStruct.I2C_OwnAddress1 = 0x00;			// own address, not relevant in master mode
	I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;		// disable acknowledge when reading (can be changed later on)
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // set address length to 7 bit addresses
	I2C_Cmd(I2C1, DISABLE);
	I2C_Init(I2C1, &I2C_InitStruct);				// init I2C1

	// enable I2C1
	I2C_Cmd(I2C1, ENABLE);
	return 0;
}

int i2c_disable(void){
	I2C_Cmd(I2C1, DISABLE);
	I2C_DeInit(I2C1);
	return 0;
}

int i2c_read(unsigned char slave_addr,
			 unsigned char reg_addr,
			 unsigned char length,
			 unsigned char *data) {

	uint8_t i;
	i2c_start(slave_addr<<1, I2C_TRANSMITTER_MODE, I2C_ACK_ENABLE);
	i2c_writedata(reg_addr);
	//TM_I2C_Stop(I2Cx);
	i2c_start(slave_addr<<1, I2C_RECEIVER_MODE, I2C_ACK_ENABLE);
	for (i = 0; i < length; i++) {
		if (i == (length - 1)) {
			/* Last byte */
			data[i] = i2c_readnack();
		} else {
			data[i] = i2c_readack();
		}
	}
	return 0;
}

int i2c_write(unsigned char slave_addr,
			 unsigned char reg_addr,
			 unsigned char length,
			 unsigned char const *data){

	uint8_t i=0;
	//for(i=0;i<length;++i){
	if(length==1){
		i2c_start(slave_addr<<1, I2C_TRANSMITTER_MODE, I2C_ACK_DISABLE);
		i2c_writedata(reg_addr);
		i2c_writedata(*data);
		i2c_stop();
	}
	else {uint8_t i;
	i2c_start(slave_addr<<1, I2C_TRANSMITTER_MODE, I2C_ACK_DISABLE);
	i2c_writedata(reg_addr);
	for (i = 0; i < length; i++) {
		i2c_writedata(data[i]);
	}
	i2c_stop();
	}
	return 0;

}


/* Private functions */

int16_t i2c_start(unsigned char slave_addr, uint8_t direction, uint8_t ack) {

	/* Generate I2C start pulse */
	I2C1->CR1 |= I2C_CR1_START;

	I2C_Timeout = I2C_TIMEOUT;
	while (!(I2C1->SR1 & I2C_SR1_SB)) {
		if (--I2C_Timeout == 0x00) {
			return 1;
		}
	}

	/* Enable ack if we select it */
	if (ack) {
		I2C1->CR1 |= I2C_CR1_ACK;
	}

	if (direction == I2C_TRANSMITTER_MODE) {
		/* Send address with zero last bit */
		I2C1->DR = slave_addr & ~I2C_OAR1_ADD0;
		/* Wait till finished */
		I2C_Timeout = I2C_TIMEOUT;
		while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
			if (--I2C_Timeout == 0x00) {
				return 1;
			}
		}
	}
	else if (direction == I2C_RECEIVER_MODE) {
		/* Send address with 1 last bit */
		I2C1->DR = slave_addr | I2C_OAR1_ADD0;

		/* Wait till finished */
		I2C_Timeout = I2C_TIMEOUT;
		while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
			if (--I2C_Timeout == 0x00) {
				return 1;
			}
		}

	}

	/* Read status register to clear ADDR flag */
	I2C1->SR2;

	/* Return 0, everything ok */
	return 0;

}



void i2c_writedata(unsigned char data) {

	/* Wait till I2C is not busy anymore */
	I2C_Timeout = I2C_TIMEOUT;
	while (!(I2C1->SR1 & I2C_SR1_TXE) && I2C_Timeout) {
		I2C_Timeout--;
	}

	/* Send I2C data */
	I2C1->DR = data;

}



unsigned char i2c_readack() {

	uint8_t data;

	/* Enable ACK */
	I2C1->CR1 |= I2C_CR1_ACK;

	/* Wait till not received */
	I2C_Timeout = I2C_TIMEOUT;
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
		if (--I2C_Timeout == 0x00) {
			return 1;
		}
	}

	/* Read data */
	data = I2C1->DR;

	/* Return data */
	return data;

}



unsigned char i2c_readnack() {

	uint8_t data;

	/* Disable ACK */
	I2C1->CR1 &= ~I2C_CR1_ACK;

	/* Generate stop */
	I2C1->CR1 |= I2C_CR1_STOP;

	/* Wait till received */
	I2C_Timeout = I2C_TIMEOUT;
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
		if (--I2C_Timeout == 0x00) {
			return 1;
		}
	}
	/* Read data */
	data = I2C1->DR;

	/* Return data */
	return data;

}



uint8_t i2c_stop() {

	/* Wait till transmitter not empty */
	I2C_Timeout = I2C_TIMEOUT;
	while (((!(I2C1->SR1 & I2C_SR1_TXE)) || (!(I2C1->SR1 & I2C_SR1_BTF)))) {
		if (--I2C_Timeout == 0x00) {
			return 1;
		}
	}

	/* Generate stop */
	I2C1->CR1 |= I2C_CR1_STOP;

	/* Return 0, everything ok */
	return 0;
}


/*
uint8_t TM_I2C_IsDeviceConnected(I2C_TypeDef* I2Cx, uint8_t address) {

	uint8_t connected = 0;
	/* Try to start, function will return 0 in case device will send ACK
	if (!TM_I2C_Start(I2Cx, address, I2C_TRANSMITTER_MODE, I2C_ACK_ENABLE)) {
		connected = 1;
	}
	/* STOP I2C
	TM_I2C_Stop(I2Cx);
	/* Return status
	return connected;
}
*/
