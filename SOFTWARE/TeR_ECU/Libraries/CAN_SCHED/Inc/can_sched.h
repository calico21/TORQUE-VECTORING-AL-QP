/*
 * can_sched.h
 *
 *  Created on: Oct 7, 2025
 *      Author: pieroebs
 */

#ifndef CAN_SCHED_INC_CAN_SCHED_H_
#define CAN_SCHED_INC_CAN_SCHED_H_
#include "cmsis_os2.h"
#include "stm32f4xx_hal.h"
#include "ter.h"
#include "hvbms.h"
#include "TeR_CAN.h"

#define CAN_SCHEDULER_HEAP_CAPACITY 80
typedef struct CanMessage {
	uint8_t content[8];
	uint8_t len;
	uint32_t id;
	uint32_t next_when;
	uint32_t period;
	void (*callback)(struct CanMessage*); // forward declaration
} CanMessage_t;

typedef struct {
	CanMessage_t data[CAN_SCHEDULER_HEAP_CAPACITY];
	int size;
} CanSchedulerHeap;
extern CanSchedulerHeap g_can_scheduler_heap;

bool can_scheduler_insert_msg_with_phase(uint8_t *msg, uint8_t len, uint32_t id,
		uint32_t period_ms, void (*callback)(CanMessage_t*));
bool can_scheduler_insert_non_periodic_msg(uint8_t *msg, uint8_t len,
		uint32_t id, uint32_t timeout);
bool can_scheduler_insert_msg_with_timeout(uint8_t *msg, uint8_t len,
		uint32_t id, uint32_t period_ms, int32_t timeout,
		void (*callback)(CanMessage_t*));
bool can_scheduler_insert_msg(uint8_t *msg, uint8_t len, uint32_t id,
		uint32_t period_ms, void (*callback)(CanMessage_t*));
bool can_scheduler_get_next(CanSchedulerHeap *heap, CanMessage_t *out);
bool can_scheduler_insert_built_msg(CanMessage_t can_msg);
// Aqui define el prototipo de tu callback para que pueda ser pasado desde fuera

void ter_status_callback(CanMessage_t *msg);
void ter_wheel_info_callback(CanMessage_t *msg);
void ter_inverter_info_callback(CanMessage_t *msg);
void ter_ang_rate_callback(CanMessage_t *msg);
void ter_accel_callback(CanMessage_t *msg);
void ter_gps_lat_callback(CanMessage_t *msg);
void ter_ypr_callback(CanMessage_t *msg);
void ter_vel_body_callback(CanMessage_t *msg);
void hvbms_bms_rx_ctrl_1_callback(CanMessage_t *msg);
void ter_tv_debug_callback(CanMessage_t *msg);
void ams_bms_req_callback(CanMessage_t *msg);
void ter_asb_brake_req_callback(CanMessage_t *msg);
void ter_dv_system_status_callback(CanMessage_t *msg);
void ter_dv_config_callback(CanMessage_t *msg);
void ter_ebs_state_req_callback(CanMessage_t *msg);


#endif /* CAN_SCHED_INC_CAN_SCHED_H_ */
