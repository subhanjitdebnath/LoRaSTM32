/*
 * sx126x_hal.c
 *
 *  Created on: Oct 13, 2024
 *      Author: Subhanjit Debnath
 */

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stdbool.h>
#include "sx126x_hal.h"

uint8_t _null[20];
/**
 * Radio data transfer - write
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context          Radio implementation parameters
 * @param [in] command          Pointer to the buffer to be transmitted
 * @param [in] command_length   Buffer size to be transmitted
 * @param [in] data             Pointer to the buffer to be transmitted
 * @param [in] data_length      Buffer size to be transmitted
 *
 * @returns Operation status
 */
sx126x_hal_status_t sx126x_hal_write( const void* context, const uint8_t* command, const uint16_t command_length,
                                      const uint8_t* data, const uint16_t data_length )

{
	const LoRaConfig* sx126x_context = ( const LoRaConfig* ) context;
	HAL_StatusTypeDef status_cmd ,status_data;
	sx126x_hal_status_t status = SX126X_HAL_STATUS_SPI_FAULT;

	sx126x_hal_wait_on_busy( sx126x_context );

	HAL_GPIO_WritePin(sx126x_context->NSS_port, sx126x_context->NSS_pin, GPIO_PIN_RESET);
	if(command != NULL && command_length > 0)
	{
		status_cmd = HAL_SPI_TransmitReceive(sx126x_context->hSPIx, command, _null, command_length, 500);
	}
	else
	{
		status_cmd = HAL_OK;
	}

	//status_cmd = HAL_SPI_Transmit(sx126x_context->hSPIx, command, command_length, 500);
	if(data != NULL && data_length > 0)
	{
		status_data = HAL_SPI_TransmitReceive(sx126x_context->hSPIx, data, _null,data_length, 500);
	}
	else
	{
		status_data = HAL_OK;
	}

	//status_data = HAL_SPI_Transmit(sx126x_context->hSPIx, data, data_length, 500);

	HAL_GPIO_WritePin(sx126x_context->NSS_port, sx126x_context->NSS_pin, GPIO_PIN_SET);

	if(status_data == HAL_OK && status_cmd == HAL_OK )
	{
		status = SX126X_HAL_STATUS_OK;
	}

	return status;
}

/**
 * Radio data transfer - read
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context          Radio implementation parameters
 * @param [in] command          Pointer to the buffer to be transmitted
 * @param [in] command_length   Buffer size to be transmitted
 * @param [in] data             Pointer to the buffer to be received
 * @param [in] data_length      Buffer size to be received
 *
 * @returns Operation status
 */
sx126x_hal_status_t sx126x_hal_read( const void* context, const uint8_t* command, const uint16_t command_length,
                                     uint8_t* data, const uint16_t data_length )
{

	const LoRaConfig* sx126x_context = ( const LoRaConfig* ) context;
		HAL_StatusTypeDef status_cmd ,status_data;
		sx126x_hal_status_t status = SX126X_HAL_STATUS_SPI_FAULT;

		sx126x_hal_wait_on_busy( sx126x_context );

		HAL_GPIO_WritePin(sx126x_context->NSS_port, sx126x_context->NSS_pin, GPIO_PIN_RESET);

		if(command != NULL && command_length > 0)
		{
			status_cmd = HAL_SPI_TransmitReceive(sx126x_context->hSPIx, command, _null, command_length, 500);
		}
		else
		{
			status_cmd = HAL_OK;
		}

		if(data != NULL && data_length > 0)
		{
			status_data = HAL_SPI_TransmitReceive(sx126x_context->hSPIx, _null, data, data_length, 500);
		}
		else
		{
			status_data = HAL_OK;
		}

		HAL_GPIO_WritePin(sx126x_context->NSS_port, sx126x_context->NSS_pin, GPIO_PIN_SET);

		if(status_data == HAL_OK && status_cmd == HAL_OK )
		{
			status = SX126X_HAL_STATUS_OK;
		}

		return status;
}

/**
 * Reset the radio
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context Radio implementation parameters
 *
 * @returns Operation status
 */
sx126x_hal_status_t sx126x_hal_reset( const void* context )
{
	const LoRaConfig* sx126x_context = ( const LoRaConfig* ) context;

	HAL_GPIO_WritePin(sx126x_context->RST_port, sx126x_context->RST_pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(sx126x_context->RST_port, sx126x_context->RST_pin, GPIO_PIN_SET);
	HAL_Delay(1);

	return SX126X_HAL_STATUS_OK;
}

/**
 * Wake the radio up.
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context Radio implementation parameters
 *
 * @returns Operation status
 */
sx126x_hal_status_t sx126x_hal_wakeup( const void* context )
{
	const LoRaConfig* sx126x_context = ( const LoRaConfig* ) context;

	HAL_GPIO_WritePin(sx126x_context->NSS_port, sx126x_context->NSS_pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(sx126x_context->NSS_port, sx126x_context->NSS_pin, GPIO_PIN_SET);
	HAL_Delay(1);

	return SX126X_HAL_STATUS_OK;

}

/**
 * Wait for the Busy state to go down
 *
 *
 */
void sx126x_hal_wait_on_busy( const void* context )
{
	const LoRaConfig* sx126x_context = ( const LoRaConfig* ) context;

    while(HAL_GPIO_ReadPin(sx126x_context->BUSY_port, sx126x_context->BUSY_pin)== GPIO_PIN_SET)
    {

    }
}


#ifdef __cplusplus
}
#endif


/* --- EOF ------------------------------------------------------------------ */
