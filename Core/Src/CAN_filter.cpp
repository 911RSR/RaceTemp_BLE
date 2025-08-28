/*
 * CAN_filter.cpp
 *
 * With code copied from GitHub aollin / racechrono-ble-diy-device
 *
 *  Created on: Apr 21, 2025
 *      Author: viggo
 */
#include "CAN_filter.h"
#include <stddef.h>
#include <stdio.h>
#include "dbg_trace.h"
#define debug(...)     do{printf("\r[%s][%s][%d] ", DbgTraceGetFileName(__FILE__),__FUNCTION__,__LINE__);printf(__VA_ARGS__);}while(0);
#define debugln(...)   do{printf("\r[%s][%s][%d]\r\n", DbgTraceGetFileName(__FILE__),__FUNCTION__,__LINE__);printf(__VA_ARGS__);}while(0);


uint8_t tempData[20];

static const int CAN_BUS_CMD_DENY_ALL = 0;
static const int CAN_BUS_CMD_ALLOW_ALL = 1;
static const int CAN_BUS_CMD_ADD_PID = 2;
PacketIdInfo canBusPacketIdInfo;
bool canBusAllowUnknownPackets = false;
uint32_t canBusLastNotifyMs = 0;
bool isCanBusConnected = false;

unsigned int CAN_ShouldNotify( uint32_t packetId ) {
	PacketIdInfoItem* infoItem = canBusPacketIdInfo.findItem(packetId, canBusAllowUnknownPackets);
	if (infoItem && infoItem->shouldNotify()) {
		infoItem->markNotified();
		return 1;
	} else {
		return 0;
	}
}

void CAN_FilterWrite( uint8_t* data, uint16_t len) {
    if (len < 1) {
        return;
    }
    uint8_t command = data[0];
    switch (command) {
        case CAN_BUS_CMD_DENY_ALL:
            if (len == 1) {
                canBusPacketIdInfo.reset();
                canBusAllowUnknownPackets = false;
                debugln("CAN FILTER command DENY ALL");
            } else debug("CAN FILTER ERROR: DENY ALL length mismatch\n\r");

            break;
        case CAN_BUS_CMD_ALLOW_ALL:
            if (len == 3) {
                canBusPacketIdInfo.reset();
                uint16_t notifyIntervalMs = data[1] << 8 | data[2];
                canBusPacketIdInfo.setDefaultNotifyInterval(notifyIntervalMs);
                canBusAllowUnknownPackets = true;
                debug("CAN FILTER ALLOW ALL interval: %u\r\n",notifyIntervalMs);
            } else debug("CAN FILTER ERROR: ALLOW ALL length mismatch\n\r");
            break;
        case CAN_BUS_CMD_ADD_PID:
            if (len == 7) {
                uint16_t notifyIntervalMs = data[1] << 8 | data[2];
                uint32_t pid = data[3] << 24 | data[4] << 16 | data[5] << 8 | data[6];
                canBusPacketIdInfo.setNotifyInterval(pid, notifyIntervalMs);
                debug("CAN FILTER ADD PID: %lu,  interval: %u",pid, notifyIntervalMs);
            } else debug("CAN FILTER ERROR: ADD PID length mismatch\n\r");
            break;
        default:
        	debug("CAN FILTER ERROR: Unknown command\n\r");
            break;
    }
}


