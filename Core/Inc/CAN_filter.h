/*
 * CAN_filter.h
 *
 *  Created on: Apr 21, 2025
 *      Author: viggo
 */

#ifndef INC_CAN_FILTER_H_
#define INC_CAN_FILTER_H_
#include "PacketIdInfo.h"

#ifdef __cplusplus
#else  // C
  typedef struct PacketIdInfo
		PacketIdInfo;
#endif

#ifdef __cplusplus
extern "C" {
#endif
extern void CAN_FilterWrite( uint8_t* data, uint16_t len );
extern unsigned int CAN_ShouldNotify( uint32_t packetId );
#ifdef __cplusplus
}
#endif

#endif /* INC_CAN_FILTER_H_ */
