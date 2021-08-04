#ifndef NRF24_H_
#define NRF24_H_

//------------------------------------------------
#include "stm32f4xx_hal.h"
#include "stm32f411xe.h"


//extern volatile uint32_t NRF24_IRQ;
//extern volatile uint32_t SR501_IRQ;


extern SPI_HandleTypeDef hspi1;

#define SYSTEM_CLOCK 180000000

#define TX_ADR_WIDTH 3
#define RX_ADR_WIDTH 3

#define TX_PLOAD_WIDTH 27
#define RX_PLOAD_WIDTH 27

//------------------------------------------------
#define CE_ON HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET)
#define CE_OFF HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET)

#define CS_ON HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET)
#define CS_OFF HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET)

#define READ_IRQ HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)

//------------------------------------------------
#define ACTIVATE 0x50 //
#define R_RX_PAYLOAD 0x61 // Define RX payload register address
#define W_TX_PAYLOAD 0xA0 // Define TX payload register address
#define FLUSH_TX 0xE1
#define FLUSH_RX 0xE2
//------------------------------------------------
#define CONFIG 0x00 //'Config' register address
#define EN_AA 0x01 //'Enable Auto Acknowledgment' register address
#define EN_RXADDR 0x02 //'Enabled RX addresses' register address
#define SETUP_AW 0x03 //'Setup address width' register address
#define SETUP_RETR 0x04 //'Setup Auto. Retrans' register address
#define RF_CH 0x05 //'RF channel' register address
#define RF_SETUP 0x06 //'RF setup' register address
#define STATUS 0x07 //'Status' register address
#define OBSERVE_TX 0x08 //'Transmit observe' register
#define RX_ADDR_P0 0x0A //'RX address pipe0' register address
#define RX_ADDR_P1 0x0B //'RX address pipe1' register address
#define TX_ADDR 0x10 //'TX address' register address
#define RX_PW_P0 0x11 //'RX payload width, pipe0' register address
#define RX_PW_P1 0x12 //'RX payload width, pipe1' register address
#define FIFO_STATUS 0x17 //'FIFO Status Register' register address
#define DYNPD 0x1C
#define FEATURE 0x1D
//------------------------------------------------
#define PRIM_RX 0x00 //RX/TX control (1: PRX, 0: PTX)
#define PWR_UP 0x01 //1: POWER UP, 0:POWER DOWN

#define RX_DR 0x06 //Data Ready RX FIFO interrupt
#define TX_DS 0x05 //Data Sent TX FIFO interrupt
#define MAX_RT 0x04 //Maximum number of TX retransmits interrupt
//------------------------------------------------
#define W_REGISTER 0x20
//------------------------------------------------
void NRF24_init();
void delay_micro(volatile uint32_t num);
uint8_t NRF24_ReadReg(uint8_t addr);
void NRF24_WriteReg(uint8_t addr, uint8_t dt);
void NRF24_Read_Buf(uint8_t addr, uint8_t *pBuf, uint8_t bytes);
void NRF24_Write_Buf(uint8_t addr,uint8_t *pBuf,uint8_t bytes);
void NRF24_FlushRX(void);
void NRF24_FlushTX(void);
void NRF24_RX_Mode(void);
void NRF24_TX_Mode(void);
void NRF24_PWR_UP_ON(void);
void NRF24_PWR_UP_OFF(void);
uint8_t NRF24_Transmit(uint8_t *pBuf, uint8_t bytes, uint32_t trys);
uint8_t NRF24_Receive(uint8_t *pBuf, uint8_t bytes, uint32_t delay_mili);
void NRF24_Transmit_base(uint8_t *pBuf, uint8_t bytes);
void NRF24_Receive_base();
//------------------------------------------------
#endif /* NRF24_H_ */
