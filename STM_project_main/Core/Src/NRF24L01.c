#include "NRF24L01.h"


volatile uint32_t NRF24_IRQ = 0;
//volatile uint32_t SR501_IRQ = 0;


uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {0x01,0x02,0x03};
uint8_t RX_ADDRESS[RX_ADR_WIDTH] = {0x01,0x02,0x03};

uint8_t TX_BUF[TX_PLOAD_WIDTH] = {0};
uint8_t RX_BUF[RX_PLOAD_WIDTH] = {0};


////--------------------------------------------------
//
void NRF24_init()
{
	CE_OFF;
	delay_micro(10300);

	NRF24_WriteReg(CONFIG, (uint8_t)0X0C); // mask, CRC, enable(off), TX
	NRF24_WriteReg(EN_AA, (uint8_t)0X01); // Enable auto acknowledgement data pipe 0
	NRF24_WriteReg(EN_RXADDR, (uint8_t)0X01); // Enable data pipe 0.
	NRF24_WriteReg(SETUP_AW, (uint8_t)0X01); // '01' - 3 bytes RX/TX Address field width
	NRF24_WriteReg(SETUP_RETR, (uint8_t)0X0F); // ‘1111’ – Up to 15 Re-Transmit on fail of AA
	NRF24_WriteReg(RF_CH, (uint8_t)0X64); // Sets the frequency channel nRF24L01 operates on 2500Mgc
	NRF24_WriteReg(RF_SETUP, (uint8_t)0X26); // 2Mbps, 0dBm Setup, LNA gain; olde 0X0F
	NRF24_WriteReg(STATUS, (uint8_t)0X70); // reset interupt

	NRF24_Write_Buf(RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH); // Receive address data pipe 0.
	NRF24_Write_Buf(TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH); // Transmit address.
	NRF24_WriteReg(RX_PW_P0, (uint8_t)RX_PLOAD_WIDTH); // number of data byte arived 6


	NRF24_PWR_UP_ON();
	delay_micro(1500);
}


////--------------------------------------------------
//
void delay_micro(volatile uint32_t num)
{
	num *= SYSTEM_CLOCK / 1000000;

	while (num--);
}


////--------------------------------------------------
//
uint8_t NRF24_ReadReg(uint8_t addr)
{
  uint8_t dt=0;
  uint8_t cmd = 0XFF;

  CS_ON;

  HAL_SPI_TransmitReceive(&hspi1, &addr, &dt, 1, 1000);

  if (addr!=STATUS)
    HAL_SPI_TransmitReceive(&hspi1, &cmd, &dt, 1, 1000);

  CS_OFF;

  return dt;
}


////------------------------------------------------
//
void NRF24_WriteReg(uint8_t addr, uint8_t dt)
{
  addr |= W_REGISTER;

  CS_ON;

  HAL_SPI_Transmit(&hspi1, &addr, 1, 1000);
  HAL_SPI_Transmit(&hspi1, &dt, 1, 1000);

  CS_OFF;
}


////------------------------------------------------
//
void NRF24_Read_Buf(uint8_t addr, uint8_t *pBuf, uint8_t bytes)
{
  CS_ON;

  HAL_SPI_Transmit(&hspi1, &addr, 1, 1000);
  HAL_SPI_Receive(&hspi1, pBuf, bytes, 1000);

  CS_OFF;
}


////------------------------------------------------
//
void NRF24_Write_Buf(uint8_t addr,uint8_t *pBuf,uint8_t bytes)
{
  addr |= W_REGISTER;

  CS_ON;

  HAL_SPI_Transmit(&hspi1, &addr, 1, 1000);
  HAL_SPI_Transmit(&hspi1, pBuf, bytes, 1000);

  CS_OFF;
}


////------------------------------------------------
//
void NRF24_FlushRX(void)
{
  uint8_t dt = (uint8_t)FLUSH_RX;

  CS_ON;

  HAL_SPI_Transmit(&hspi1, &dt, 1, 1000);

  CS_OFF;
}


////------------------------------------------------
//
void NRF24_FlushTX(void)
{
  uint8_t dt = (uint8_t)FLUSH_TX;

  CS_ON;

  HAL_SPI_Transmit(&hspi1, &dt, 1, 1000);

  CS_OFF;
}


////------------------------------------------------
//
void NRF24_RX_Mode(void)
{
  uint8_t regval=0x00;

  CS_ON;

  regval = NRF24_ReadReg(CONFIG);
  regval |= (1UL<<PRIM_RX); // RX_mode
  NRF24_WriteReg(CONFIG,regval);

  CS_OFF;

  NRF24_FlushRX();
}


////------------------------------------------------
//
void NRF24_TX_Mode(void)
{
  uint8_t regval=0x00;

  CS_ON;

  regval = NRF24_ReadReg(CONFIG);
  regval &= ~(1UL<<PRIM_RX); // TX_mode
  NRF24_WriteReg(CONFIG,regval);

  CS_OFF;

  NRF24_FlushTX();
}


////------------------------------------------------
//
void NRF24_PWR_UP_ON(void)
{
  uint8_t regval=0x00;

  CS_ON;

  regval = NRF24_ReadReg(CONFIG);
  regval |= (1UL<<PWR_UP); // ON
  NRF24_WriteReg(CONFIG,regval);

  CS_OFF;
}


////------------------------------------------------
//
void NRF24_PWR_UP_OFF(void)
{
  uint8_t regval=0x00;

  CS_ON;

  regval = NRF24_ReadReg(CONFIG);
  regval &= ~(1UL<<PWR_UP); // OFF
  NRF24_WriteReg(CONFIG,regval);

  CS_OFF;
}


////------------------------------------------------
//
uint8_t NRF24_Transmit(uint8_t *pBuf, uint8_t bytes, uint32_t trys)
{
	uint8_t w_tx_payload = (uint8_t)W_TX_PAYLOAD;
	uint8_t ret = 0;

	if (trys == 0)
	{
		while (1)
		{
			CE_OFF;
			delay_micro(300);

			NRF24_TX_Mode();

			NRF24_Write_Buf(w_tx_payload, pBuf, bytes);

			NRF24_WriteReg(STATUS, NRF24_ReadReg((uint8_t)STATUS)); // clear interupt

			CE_ON;

			while (!NRF24_IRQ);
			NRF24_IRQ = 0;

			CE_OFF;

			if (NRF24_ReadReg((uint8_t)STATUS) & (1UL << TX_DS))
			{
				ret = 1;
				break;
			}
		}
	}
	else
	{
		for (uint32_t i = 0; i < trys; i++)
		{
			CE_OFF;
			delay_micro(300);

			NRF24_TX_Mode();

			NRF24_Write_Buf(w_tx_payload, pBuf, bytes);

			NRF24_WriteReg(STATUS, NRF24_ReadReg((uint8_t)STATUS)); // cleer interup

			CE_ON;

			while (!NRF24_IRQ);
			NRF24_IRQ = 0;

			CE_OFF;

			if (NRF24_ReadReg((uint8_t)STATUS) & (1UL << TX_DS))
			{
				ret = 1;
				break;
			}
		}
	}

	NRF24_WriteReg(STATUS, NRF24_ReadReg((uint8_t)STATUS)); // clear interupt

	return ret;
}


////------------------------------------------------
//
uint8_t NRF24_Receive(uint8_t *pBuf, uint8_t bytes, uint32_t delay_mili)
{
	uint8_t r_rx_payload = (uint8_t)R_RX_PAYLOAD;
	uint8_t ret = 0;

	CE_OFF;
	delay_micro(300);

	NRF24_RX_Mode();

	NRF24_WriteReg(STATUS, NRF24_ReadReg((uint8_t)STATUS)); // clear interupt

	CE_ON;

	if (delay_mili == 0)
	{
		while (!NRF24_IRQ);
		NRF24_IRQ = 0;

		if (NRF24_ReadReg((uint8_t)STATUS) & (1UL << RX_DR))
			ret = 1;
	}
	else
	{
		for (uint32_t i = 0; (i < delay_mili * (SYSTEM_CLOCK / 1000)) && (NRF24_IRQ == 0); i++);
		NRF24_IRQ = 0;

		if (NRF24_ReadReg((uint8_t)STATUS) & (1UL << RX_DR))
			ret = 1;
	}

	CE_OFF;
	delay_micro(300);

	if (ret)
	{
		NRF24_Read_Buf(r_rx_payload, pBuf, bytes);
	}

	NRF24_WriteReg(STATUS, NRF24_ReadReg((uint8_t)STATUS)); // clear interupt

	return ret;
}


////------------------------------------------------
//
void NRF24_Transmit_base(uint8_t *pBuf, uint8_t bytes)
{
	uint8_t w_tx_payload = (uint8_t)W_TX_PAYLOAD;

	CE_OFF;
	delay_micro(300);

	NRF24_TX_Mode();

	NRF24_Write_Buf(w_tx_payload, pBuf, bytes);

	NRF24_WriteReg(STATUS, NRF24_ReadReg((uint8_t)STATUS) | 0x70); // clear interupt

	CE_ON;
}


////------------------------------------------------
//
void NRF24_Receive_base()
{
	CE_OFF;
	delay_micro(300);

	NRF24_RX_Mode();

	NRF24_WriteReg(STATUS, NRF24_ReadReg((uint8_t)STATUS) | 0x70); // clear interupt

	CE_ON;
}
