/*
 * RaceChrono.c
 *
 * RaceChrono BLE packet construction and filter handling.
 */

#include "RaceChrono.h"

#include "CAN_filter.h"
#include "app_common.h"
#include "ble.h"
#include "custom_stm.h"
#include "dbg_trace.h"
#include <string.h>

static uint8_t BLE_msg[512];
static uint8_t BLE_msg_len = 8;
static uint8_t Can_main_Notification_Status;

void RaceChrono_SetCanMainNotificationsEnabled(uint8_t enabled)
{
  Can_main_Notification_Status = enabled ? 1U : 0U;

  if (Can_main_Notification_Status == 1U)
  {
    APP_DBG_MSG("-- CAN MAIN : NOTIFICATION ENABLED\n\r");
  }
  else
  {
    APP_DBG_MSG("-- CAN MAIN : NOTIFICATION DISABLED\n\r");
  }
}

void RaceChrono_CanMainUpdate(void)
{
  Custom_STM_App_Update_Char_Variable_Length(CUSTOM_STM_CAN_MAIN, BLE_msg, BLE_msg_len);
}

void RaceChrono_CanMainSendNotification(void)
{
  if (Can_main_Notification_Status == 1U)
  {
    Custom_STM_App_Update_Char_Variable_Length(CUSTOM_STM_CAN_MAIN, BLE_msg, BLE_msg_len);
  }
  else
  {
    APP_DBG_MSG("-- CUSTOM APPLICATION : CAN'T INFORM CLIENT -  NOTIFICATION DISABLED\n ");
  }
}

void RaceChrono_CanFilterWrite(uint8_t *data, uint16_t len)
{
  APP_DBG_MSG("-- CAN FILTER WRITE, DataTransfered.Length=%u", len);
  CAN_FilterWrite(data, len);
}

void RaceChrono_SendCanMessage(uint32_t id, const uint8_t *payload, uint8_t len)
{
  if ((len > (SizeCan_Main - sizeof(id))) || ((len > 0U) && (payload == NULL)) || (CAN_ShouldNotify(id) == 0U))
  {
    return;
  }

  memcpy(&BLE_msg[0], &id, sizeof(id));
  if (len > 0U)
  {
    memcpy(&BLE_msg[sizeof(id)], payload, len);
  }

  BLE_msg_len = (uint8_t)(sizeof(id) + len);
  RaceChrono_CanMainSendNotification();
}

void RaceChrono_SendFloat(uint32_t id, float value)
{
  RaceChrono_SendCanMessage(id, (uint8_t *)&value, sizeof(value));
}
