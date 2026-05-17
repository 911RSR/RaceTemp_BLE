/*
 * RaceChrono.h
 *
 * RaceChrono BLE payload and filter handling.
 */

#ifndef INC_RACECHRONO_H_
#define INC_RACECHRONO_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void RaceChrono_SetCanMainNotificationsEnabled(uint8_t enabled);
void RaceChrono_CanMainUpdate(void);
void RaceChrono_CanMainSendNotification(void);
void RaceChrono_CanFilterWrite(uint8_t *data, uint16_t len);
void RaceChrono_SendCanMessage(uint32_t id, const uint8_t *payload, uint8_t len);
void RaceChrono_SendFloat(uint32_t id, float value);

#ifdef __cplusplus
}
#endif

#endif /* INC_RACECHRONO_H_ */
