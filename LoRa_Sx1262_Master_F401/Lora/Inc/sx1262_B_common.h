/*
 * sx1262_B_common.h
 *
 *  Created on: Oct 13, 2024
 *      Author: Subhanjit Debnath
 */

#ifndef INC_SX1262_B_COMMON_H_
#define INC_SX1262_B_COMMON_H_

/*!
 * @brief LoRa sync word
 */
#ifndef LORA_SYNCWORD_PRIVATE_NTW
#define LORA_SYNCWORD_PRIVATE_NTW   0x12 // 0x12 Private Network
#endif
#ifndef LORA_SYNCWORD_PUBLIC_NTW
#define LORA_SYNCWORD_PUBLIC_NTW    0x34 // 0x34 Public Network
#endif
#ifndef LORA_SYNCWORD
#define LORA_SYNCWORD LORA_SYNCWORD_PRIVATE_NTW
#endif

/*!
 * @brief Packet parameters for LoRa packets
 */
#ifndef LORA_PREAMBLE_LENGTH
#define LORA_PREAMBLE_LENGTH 8
#endif
#ifndef LORA_PKT_LEN_MODE
#define LORA_PKT_LEN_MODE SX126X_LORA_PKT_EXPLICIT
#endif
#ifndef LORA_IQ
#define LORA_IQ false
#endif
#ifndef LORA_CRC
#define LORA_CRC false
#endif


/*!
 * @brief General parameters
 */
#ifndef PACKET_TYPE
#define PACKET_TYPE SX126X_PKT_TYPE_LORA
#endif
#ifndef RF_FREQ_IN_HZ
#define RF_FREQ_IN_HZ 490000000
#endif
#ifndef TX_OUTPUT_POWER_DBM
#define TX_OUTPUT_POWER_DBM 22  // range [-17, +22]
#endif
#ifndef PA_RAMP_TIME
#define PA_RAMP_TIME SX126X_RAMP_40_US
#endif
#ifndef FALLBACK_MODE
#define FALLBACK_MODE SX126X_FALLBACK_STDBY_RC
#endif
#ifndef ENABLE_RX_BOOST_MODE
#define ENABLE_RX_BOOST_MODE false
#endif
#ifndef PAYLOAD_LENGTH
#define PAYLOAD_LENGTH 7
#endif

sx126x_hal_status_t sx126x_init( const void* context );
void Radio_init(const void* context );


#endif /* INC_SX1262_B_COMMON_H_ */
