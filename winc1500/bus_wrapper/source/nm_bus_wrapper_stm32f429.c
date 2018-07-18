/**
 *
 * \file
 *
 * \brief This module contains NMC1000 bus wrapper APIs implementation.
 *
 * Copyright (c) 2016-2017 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>     /* Included for uint_t */

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"
#include "bsp/include/nm_bsp.h"
#include "common/include/nm_common.h"
#include "bus_wrapper/include/nm_bus_wrapper.h"
#include "conf_winc.h"

#define NM_BUS_MAX_TRX_SZ	256

/* Declare STM32 SPIx communication handler variable to winc1500 */
SPI_HandleTypeDef hspiWifi;

/* spi_rw variables */
static uint8 spiDummyBuf[300] = {0};

tstrNmBusCapabilities egstrNmBusCapabilities =
{
	NM_BUS_MAX_TRX_SZ
};

#ifdef CONF_WINC_USE_SPI
/*
*	@fn		spi_select_slave
*	@brief	Select slave chip select: true - select, false - deselect
*	@return	None
*/
static void spi_select_slave(const uint8_t select)
{
    if (select)
    {
        HAL_GPIO_WritePin(SPI_WIFI_CS_GPIO_PORT,SPI_WIFI_CS_PIN,GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(SPI_WIFI_CS_GPIO_PORT,SPI_WIFI_CS_PIN,GPIO_PIN_SET);
    }
}

/*
*	@fn		spi_rw
*	@brief	transmit and/or receive data buffer via spi
*	@return	status
*/

#if 0

//struct spi_module master;
//struct spi_slave_inst slave_inst;

static sint8 spi_rw(uint8* pu8Mosi, uint8* pu8Miso, uint16 u16Sz)
{
	uint8 u8Dummy = 0;
	uint8 u8SkipMosi = 0, u8SkipMiso = 0;
	uint8_t txd_data = 0;
	uint8_t rxd_data = 0;

	if (!pu8Mosi) {
		pu8Mosi = &u8Dummy;
		u8SkipMosi = 1;
	}
	else if(!pu8Miso) {
		pu8Miso = &u8Dummy;
		u8SkipMiso = 1;
	}
	else {
		return M2M_ERR_BUS_FAIL;
	}

	spi_select_slave(true);


	while (u16Sz) {
		txd_data = *pu8Mosi;
		//printf("\nsend %d",txd_data);
		HAL_SPI_TransmitReceive(&hspiWifi,&txd_data,&rxd_data,1,1000);
		//HAL_SPI_Transmit(&hspi1,&txd_data,1,1000);
		//HAL_SPI_Receive(&hspi1,&rxd_data,1,1000);
//		while (!spi_is_ready_to_write(&master))
//			;
//		while(spi_write(&master, txd_data) != STATUS_OK)
//			;

//		/* Read SPI master data register. */
//		while (!spi_is_ready_to_read(&master))
//			;
//		while (spi_read(&master, &rxd_data) != STATUS_OK)
//			;
		*pu8Miso = rxd_data;
//printf("\nrecv %d",rxd_data);
		u16Sz--;
		if (!u8SkipMiso)
			pu8Miso++;
		if (!u8SkipMosi)
			pu8Mosi++;
	}

//	while (!spi_is_write_complete(&master))
//		;

spi_select_slave(false);

	return M2M_SUCCESS;
}
#else

static sint8 spi_rw(uint8* pu8Mosi, uint8* pu8Miso, uint16 u16Sz)
{
   HAL_StatusTypeDef status;
   
    /* Start SPI transaction - polling method */
  	spi_select_slave(true);
    
    
    /* Transmit/Recieve */
    if (pu8Mosi == NULL)
	{
		status = HAL_SPI_TransmitReceive(&hspiWifi,spiDummyBuf,pu8Miso,u16Sz,1000);
    }
    else if(pu8Miso == NULL)
    {
        status = HAL_SPI_TransmitReceive(&hspiWifi,pu8Mosi,spiDummyBuf,u16Sz,1000);
        memset(spiDummyBuf,0, u16Sz);
    }
    else
    {     
        status = HAL_SPI_TransmitReceive(&hspiWifi,pu8Mosi,pu8Miso,u16Sz,1000);
    } 
    
    /* Handle Transmit/Recieve error */
    if (status != HAL_OK)
    {
        M2M_ERR("%s: HAL_SPI_TransmitReceive failed. error (%d)\n",__FUNCTION__,status);
        return status;
    }
    
  	spi_select_slave(false);

	return M2M_SUCCESS;
}
#endif
#endif //CONF_WINC_USE_SPI

void nm_bus_wifi_spi_init(SPI_HandleTypeDef *hspi)
{
    GPIO_InitTypeDef  GPIO_InitStruct;

    /* Peripheral clock enable */
    SPI_WIFI_CLK_ENABLE();

    /* Configure GPIO pin : PA4 - we are using ST GPIO definitions for winc1500 */
    GPIO_InitStruct.Pin   = SPI_WIFI_CS_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = 0;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(SPI_WIFI_CS_GPIO_PORT,SPI_WIFI_CS_PIN,GPIO_PIN_SET);

    /**SPIx GPIO Configuration
    PB3     ------> SPI_WIFI_SCK
    PB4     ------> SPI_WIFI_MISO
    PB5     ------> SPI_WIFI_MOSI
    */
    GPIO_InitStruct.Pin = SPI_WIFI_SCK_PIN|SPI_WIFI_MISO_PIN|SPI_WIFI_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
   GPIO_InitStruct.Alternate = SPI3_WIFI_AF;
    HAL_GPIO_Init(SPI_WIFI_MOSI_GPIO_PORT, &GPIO_InitStruct);
}


/*
*	@fn		nm_bus_init
*	@brief	Initialize the bus wrapper
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*/
sint8 nm_bus_init(void *pvinit)
{
	sint8 result = M2M_SUCCESS;

	 /* WiFi SPI init function - called from nm_bus_init() */

	hspiWifi.Instance			   = SPI_WIFI;
	hspiWifi.Init.Mode			   = SPI_MODE_MASTER;
	hspiWifi.Init.Direction 	   = SPI_DIRECTION_2LINES;
	hspiWifi.Init.DataSize		   = SPI_DATASIZE_8BIT;
	hspiWifi.Init.CLKPolarity	   = SPI_POLARITY_LOW;
	hspiWifi.Init.CLKPhase		   = SPI_PHASE_1EDGE;
	hspiWifi.Init.NSS			   = SPI_NSS_SOFT;
	hspiWifi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
	hspiWifi.Init.FirstBit		   = SPI_FIRSTBIT_MSB;
	hspiWifi.Init.TIMode		   = SPI_TIMODE_DISABLE;
	hspiWifi.Init.CRCCalculation   = SPI_CRCCALCULATION_DISABLE;
	hspiWifi.Init.CRCPolynomial    = 10;
//	  hspiWifi.Init.CRCLength		 = SPI_CRC_LENGTH_DATASIZE;
//	  hspiWifi.Init.NSSPMode		 = SPI_NSS_PULSE_DISABLE;
	if (HAL_SPI_Init(&hspiWifi) != HAL_OK)
	{
		M2M_ERR("SPI bus Initialization error\r\n");
	}

	HAL_SPI_MspInit(&hspiWifi);
	return result;
}


/*
*	@fn		nm_bus_ioctl
*	@brief	send/receive from the bus
*	@param[IN]	u8Cmd
*					IOCTL command for the operation
*	@param[IN]	pvParameter
*					Arbitrary parameter depenging on IOCTL
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@note	For SPI only, it's important to be able to send/receive at the same time
*/
sint8 nm_bus_ioctl(uint8 u8Cmd, void* pvParameter)
{
	sint8 s8Ret = 0;
	switch(u8Cmd)
	{
		case NM_BUS_IOCTL_RW: {
			tstrNmSpiRw *pstrParam = (tstrNmSpiRw *)pvParameter;
			s8Ret = spi_rw(pstrParam->pu8InBuf, pstrParam->pu8OutBuf, pstrParam->u16Sz);
		}
		break;
		default:
			s8Ret = -1;
			M2M_ERR("invalide ioclt cmd\n");
			break;
	}

	return s8Ret;
}

/*
*	@fn		nm_bus_deinit
*	@brief	De-initialize the bus wrapper
*/
sint8 nm_bus_deinit(void)
{
	return M2M_SUCCESS;
}

/*
*	@fn			nm_bus_reinit
*	@brief		re-initialize the bus wrapper
*	@param [in]	void *config
*					re-init configuration data
*	@return		M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*/
sint8 nm_bus_reinit(void* config)
{
	return M2M_SUCCESS;
}

