/*
 * sx1262_B_common.c
 *
 *  Created on: Oct 13, 2024
 *      Author: Subhanjit Debnath
 */
#include "sx126x.h"
#include "sx126x_hal.h"
#include "sx1262_B_common.h"


/**
 *
 *  Intialization of the sx1262-B
 * @param [in] context Radio implementation parameters
 *
 * @returns Operation status
 *
 */

sx126x_hal_status_t sx126x_init( const void* context )
{
	static sx126x_chip_status_t radio_status;
	sx126x_hal_status_t status = SX126X_HAL_STATUS_CFG_FAULT;
	uint8_t successcount = 0;

	if(sx126x_reset( context ) == SX126X_STATUS_OK)
	{
		successcount++;
	}

	/*if(sx126x_init_retention_list( context ) == SX126X_STATUS_OK)
	{
		successcount++;
	}*/

	if(sx126x_set_reg_mode( context, SX126X_REG_MODE_LDO )== SX126X_STATUS_OK)
	{
		successcount++;
	}

	if(sx126x_set_dio2_as_rf_sw_ctrl( context, true )== SX126X_STATUS_OK)
	{
		successcount++;
	}

	if(sx126x_set_dio3_as_tcxo_ctrl( context, SX126X_TCXO_CTRL_3_0V, 300 )== SX126X_STATUS_OK)
	{
		successcount++;
	}

	sx126x_reset( context );
	sx126x_get_status( context, &radio_status );

	if(radio_status.chip_mode != SX126X_CHIP_MODE_STBY_RC)
	{
		if(sx126x_cal( context, SX126X_CAL_ALL )== SX126X_STATUS_OK)
		{
			successcount++;
		}
	}

	if(successcount == 6)
	{
		status = SX126X_HAL_STATUS_OK;
	}
	return status;

}

/**
 *
 *  Initialization of the radio settings
 * @param [in] context Radio implementation parameters
 *
 * @returns Operation status
 *
 */
void Radio_init(const void* context)
{
		//SX126X power amplifier configuration
		sx126x_pa_cfg_params_t 			pa_cfg;

		static sx126x_mod_params_lora_t lora_mod_params = {
		    .sf   = SX126X_LORA_SF7,
		    .bw   = SX126X_LORA_BW_125,
		    .cr   = SX126X_LORA_CR_4_5,
		    .ldro = 0,  // Will be initialized during radio init
		};

		const sx126x_pkt_params_lora_t lora_pkt_params = {
		    .preamble_len_in_symb = LORA_PREAMBLE_LENGTH,
		    .header_type          = LORA_PKT_LEN_MODE,
		    .pld_len_in_bytes     = PAYLOAD_LENGTH,
		    .crc_is_on            = LORA_CRC,
		    .invert_iq_is_on      = LORA_IQ,
		};

/*	***********************************************************************************************************
 * 	Mode 	*	OutputPower	*	paDutyCycle	*	hpMax	*	deviceSel 	*	paLut 	*	Value inSetTxParams1
 *  ***********************************************************************************************************
 *	SX1262	*  	+17dBm 		*		0x02 	*	0x03 	*	 0x00 		*    0x01 	*	 +22 dBm
	***********************************************************************************************************
*/
		pa_cfg.device_sel 		= 0x00;
		pa_cfg.pa_lut 			= 0x01;
		pa_cfg.pa_duty_cycle 	= 0x02;
		pa_cfg.hp_max 			= 0x03;





	  sx126x_set_standby( context, SX126X_STANDBY_CFG_RC);
	  sx126x_set_pkt_type( context, SX126X_PKT_TYPE_LORA );
	  sx126x_set_rf_freq( context, RF_FREQ_IN_HZ );

	  sx126x_set_pa_cfg( context, &pa_cfg);
	  sx126x_set_tx_params( context, TX_OUTPUT_POWER_DBM, PA_RAMP_TIME );

	  sx126x_set_rx_tx_fallback_mode( context, FALLBACK_MODE );
	  sx126x_cfg_rx_boosted( context, ENABLE_RX_BOOST_MODE );

	  //For LORA
	  lora_mod_params.ldro = 1;
	  sx126x_set_lora_mod_params( context, &lora_mod_params );
	  sx126x_set_lora_pkt_params( context, &lora_pkt_params );
	  sx126x_set_lora_sync_word( context, LORA_SYNCWORD );
}
