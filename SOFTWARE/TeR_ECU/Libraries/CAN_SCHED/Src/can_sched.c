/* ---------------------------[MAIN CAN TX Scheduler, Author: Asempere]-------------------------- */

// Example usage:
// // Every 100ms
// uint8_t raw_msg[8] = {0};
// ter_ter_status_pack(&raw_msg, &TeR.status, TER_TER_STATUS_LENGTH);
// can_scheduler_insert_msg(&raw_msg, TER_TER_STATUS_FRAME_ID, 100, NULL);
// // Every 300ms
// uint8_t other_raw_msg[8] = {0};
// hvbms_bms_rx_ctrl_1_pack(&other_raw_msg, &TeR.BmsAppReq, HVBMS_BMS_RX_CTRL_1_LENGTH);
// can_scheduler_insert_msg(&other_raw_msg, HVBMS_BMS_RX_CTRL_1_FRAME_ID, 300, NULL);
// // Only once
// uint8_t once_raw_msg[8] = {0};
// hvbms_bms_rx_ctrl_1_pack(&once_raw_msg, &TeR.BmsAppReq, HVBMS_BMS_RX_CTRL_1_LENGTH);
// can_scheduler_insert_non_periodic_msg(&once_raw_msg, HVBMS_BMS_RX_CTRL_1_FRAME_ID, 0, NULL);
// Como crear un callback? dale el nombre de la funcion pack, pero en vez de pack pon callback (vamos a tener este standard que sino
//es un caos, luego haz el pack y listo, recuerda añadirlo tambien en el .h
#include "can_sched.h"
extern osMutexId_t g_can_scheduler_mutexHandle;
CanSchedulerHeap g_can_scheduler_heap = { 0 };

// CALLBACK DEFINITION
void ter_status_callback(CanMessage_t *msg) {
	ter_ter_status_pack(msg->content, &TeR.status, msg->len);
}

void ter_wheel_info_callback(CanMessage_t *msg) {
	ter_wheel_info_pack(msg->content, &TeR.wheelInfo, msg->len);
}
void ter_inverter_info_callback(CanMessage_t *msg) {
	ter_inverter_info_pack(msg->content, &TeR.invInfo, msg->len);
}
void ter_ang_rate_callback(CanMessage_t *msg) {
	ter_ang_rate_pack(msg->content, &TeR.angRate, msg->len);
}
void ter_accel_callback(CanMessage_t *msg) {
	ter_accel_pack(msg->content, &TeR.accel, msg->len);
}
void ter_gps_lat_callback(CanMessage_t *msg) {
	ter_gps_lat_long_pack(msg->content, &TeR.latlong, msg->len);
}
void ter_ypr_callback(CanMessage_t *msg) {
	ter_ypr_pack(msg->content, &TeR.ypr, msg->len);
}
void ter_vel_body_callback(CanMessage_t *msg) {
	ter_vel_body_pack(msg->content, &TeR.velbody, msg->len);
}
void ter_asb_brake_req_callback(CanMessage_t *msg){
	ter_asb_brake_req_pack(msg->content, &TeR.asb_brake_req, msg->len);
}
void ter_dv_system_status_callback(CanMessage_t *msg){
ter_dv_system_status_pack(msg->content, &TeR.dv_system_status, msg->len);
}
void hvbms_bms_rx_ctrl_1_callback(CanMessage_t *msg) {
	hvbms_bms_rx_ctrl_1_pack(msg->content, &TeR.BmsAppReq, msg->len);
}
void ams_bms_req_callback(CanMessage_t *msg){
	ams_bms_req_pack(msg->content, &TeR.bms_req, msg->len);
}

void ter_tv_debug_callback(CanMessage_t *msg) {
	ter_tv_debug_pack(msg->content, &TeR.tv_debug, msg->len);
}
void ter_dv_config_callback(CanMessage_t *msg){
	ter_dv_config_pack(msg->content, &TeR.dv_config, msg->len);
}
void ter_ebs_state_req_callback(CanMessage_t *msg){
	ter_asb_ebs_state_req_pack(msg->content, &TeR.asb_ebs_state_req, msg->len);
}

static uint32_t can_scheduler_phase_from_id(uint32_t id, uint32_t period_ms) {
	uint32_t h = id * 2654435761u; // 1) mezclar bits (hash multiplicativo de Knuth)
	return (uint32_t) ((((uint64_t) h) * period_ms) >> 32); // 2) escalar a [0, period_ms)
}

static void swap_can_msg(CanMessage_t *a, CanMessage_t *b) {
	CanMessage_t temp = *a;
	*a = *b;
	*b = temp;
}
static void can_scheduler_heapify_up(CanSchedulerHeap *heap, int index) {
	while (index > 0) {
		int parent = (index - 1) / 2;
		if (heap->data[index].next_when < heap->data[parent].next_when) {
			swap_can_msg(&heap->data[index], &heap->data[parent]);
			index = parent;
		} else {
			break;
		}
	}
}

static void can_scheduler_heapify_down(CanSchedulerHeap *heap, int index) {
	while (1) {
		int left = 2 * index + 1;
		int right = 2 * index + 2;
		int smallest = index;

		if (left < heap->size
				&& heap->data[left].next_when < heap->data[smallest].next_when)
			smallest = left;
		if (right < heap->size
				&& heap->data[right].next_when < heap->data[smallest].next_when)
			smallest = right;

		if (smallest != index) {
			swap_can_msg(&heap->data[index], &heap->data[smallest]);
			index = smallest;
		} else {
			break;
		}
	}
}

const CanMessage_t* can_scheduler_peek_next(const CanSchedulerHeap *heap) {
	if (heap->size == 0)
		return NULL;
	return &heap->data[0];
}

bool can_scheduler_get_next(CanSchedulerHeap *heap, CanMessage_t *out) {
	osMutexAcquire(g_can_scheduler_mutexHandle, osWaitForever);
	if (heap->size == 0){
		osMutexRelease(g_can_scheduler_mutexHandle);
		return false;
	}
	*out = heap->data[0];
	heap->size--;
	if (heap->size > 0) {
		heap->data[0] = heap->data[heap->size];
		can_scheduler_heapify_down(heap, 0);
	}
	osMutexRelease(g_can_scheduler_mutexHandle);
	return true;
}

bool can_scheduler_insert_built_msg(CanMessage_t can_msg) {
	osMutexAcquire(g_can_scheduler_mutexHandle, osWaitForever);
	if (g_can_scheduler_heap.size >= CAN_SCHEDULER_HEAP_CAPACITY) {
		osMutexRelease(g_can_scheduler_mutexHandle);
		return false;
	}
	g_can_scheduler_heap.data[g_can_scheduler_heap.size] = can_msg;
	can_scheduler_heapify_up(&g_can_scheduler_heap, g_can_scheduler_heap.size);
	g_can_scheduler_heap.size++;
	osMutexRelease(g_can_scheduler_mutexHandle);
	return true;
}

bool can_scheduler_insert_msg(uint8_t *msg, uint8_t len, uint32_t id,
		uint32_t period_ms, void (*callback)(CanMessage_t*)) {
	CanMessage_t can_msg = { .len = len, .id = id, .next_when =
			osKernelGetTickCount(), .period = period_ms, .callback = callback };
	memcpy(can_msg.content, msg, sizeof(can_msg.content));
	return can_scheduler_insert_built_msg(can_msg);
}

bool can_scheduler_insert_msg_with_timeout(uint8_t *msg, uint8_t len,
		uint32_t id, uint32_t period_ms, int32_t timeout,
		void (*callback)(CanMessage_t*)) {
	CanMessage_t can_msg = { .len = len, .id = id, .next_when = timeout,
			.period = period_ms, .callback = callback };
	memcpy(can_msg.content, msg, sizeof(can_msg.content));
	return can_scheduler_insert_built_msg(can_msg);
}


/*
 * Do not call inside an ISR!!!
 *
 * */
bool can_scheduler_insert_non_periodic_msg(uint8_t *msg, uint8_t len,
		uint32_t id, uint32_t timeout) {
	return can_scheduler_insert_msg_with_timeout(msg, len, id, -1,
			timeout + osKernelGetTickCount(), 0);
}

bool can_scheduler_insert_msg_with_phase(uint8_t *msg, uint8_t len, uint32_t id,
		uint32_t period_ms, void (*callback)(CanMessage_t*)) {
	CanMessage_t can_msg = { .len = len, .id = id, .next_when =
			osKernelGetTickCount() + can_scheduler_phase_from_id(id, period_ms),
			.period = period_ms, .callback = callback };
	memcpy(can_msg.content, msg, sizeof(can_msg.content));
	return can_scheduler_insert_built_msg(can_msg);
}
