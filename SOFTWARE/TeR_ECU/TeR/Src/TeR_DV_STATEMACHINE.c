/*
 * TeR_DV_STATEMACHINE.C
 *
 *  Created on: Jan 31, 2026
 *      Author: pieroebs
 *
 *      Lógica de todo lo relacionado con la obtención y configuración del AS
 *
 *      -> obtención de estado del AS
 *      -> controlar como se cambia de estado en el AS
 *      -> permite o no el uso de actuadores
 *      -> configuración del modo de conduccion del DV
 *      -> request de safety Line (SDC en fsg)
 *
 *
 *
 *
 */
#include "TeR_DV_STATEMACHINE.h"
#include "math.h"
#define MAX_STEER_LEFT_ANGLE 80.0
#define MAX_STEER_RIGHT_ANGLE -80.0
#define DEG2RAD (180.0/PI)
#define ACTUATOR2STEER (3.5)
void set_assi_yellow(uint8_t set);
void set_assi_blue(uint8_t set);
void toggle_assi_yellow();
void toggle_assi_blue();

void permatask();
void dv_stateLoop();
void set_steer_angle(float angle);
void unsafe_set_steer_angle(float angle);
void as_act_statemachine();
dv_act_state_t get_as_act_state();
extern osTimerId_t as_allowed_timerHandle;
extern osTimerId_t as_emergency_beep_timerHandle;
const static uint32_t task_period = 5;
persist_t ready_time; // persistencia que cuenta el tiempo que estamos en AS_READY
persist_t res_k2; // persistencia que cuenta el tiempo que "res k2" ha estado en 1
persist_t ext_ts; // persistencia que cuenta el tiempo que "boton ts dv" ha sido presionado
uint32_t emergency_beep_count; // contador de beeps de emergencia
dv_act_state_t dv_act_state = AS_ACT_OFF; // tengo que crear la señal de can luego lo hago todo

/*
 * This function delays the request for AS_DRIVING
 * Use: To delay any requests from the driverless computer for a period of time after entering AS_DRIVING (required by the rules)
 * Important, DO NOT block (osDelay for example) in any timer callback, read the manual
 *
 */
void as_allowed_timer_callback(void *argument) {
	if (TeR.dv_system_status.as_status == AS_DRIVING) { // imaginate el caso en el que entras en driving, se lanza el timer, y antes de 3 segundos hay fallo, esto previene jittering (5ms coche en driving y luego cae)
		TeR.status.as_allowed = 1;
		TeR.dv_system_status.steering_state =
		TER_DV_SYSTEM_STATUS_STEERING_STATE_AVAILABLE_CHOICE;
	}
}

/*
 * This timer executes the beep the buzzer at a frequency that i forgot for around 8-10 seconds
 * The timer gets called every 200ms, when an internal count value is reached, the timer gets deactivated
 *
 *
 */
void as_emergency_beep_timer_callback(void *argument) {
	HAL_GPIO_TogglePin(DOUT1_GPIO_Port, DOUT1_Pin);
	emergency_beep_count++;
	if (emergency_beep_count >= 40) {
		emergency_beep_count = 0;
		HAL_GPIO_WritePin(DOUT1_GPIO_Port, DOUT1_Pin, GPIO_PIN_RESET); // asegurar que la hemos apagado
		osTimerStop(as_emergency_beep_timerHandle);
	}
}

//K2 o K3 se pueden usar como go signal (fsg 2026)
/*
 * This function gets the state of the DV statemachine, it follows the rule T 14.8
 * fully combinational, as expected by FSG
 * It has to be this way, as we have to implement as the rule says, please check the rules
 *
 * (puede que la haya liado en alguna condicion luego reviso) todo
 *
 */
dv_state_t get_dv_state() { // fsg 2026 T 14.8
	dv_state_t dv_state = AS_OFF;
	if ((TeR.asb_status.asb_ebs_state
			== TER_ASB_STATUS_ASB_EBS_STATE_ACTIVATED_CHOICE)
			|| (TeR.asb_status.asb_redundancy_state
					== TER_ASB_STATUS_ASB_REDUNDANCY_STATE_ACTIVATED_CHOICE)) {
		if ((TeR.dv_info.mission_status
				== TER_DV_INFO_MISSION_STATUS_FINISHED_CHOICE)
				&& (TeR.wheelInfo.speed == 0)) { // supongo que quieren que este en emergency mientras se esta moviendo? ni idea? y si luego llegas a 0 entiendo que no deberia de latchearse? poco sentido la verdad, solo se guarda si el boton del res esta presionado
			if (!TeR.res_pdo_tx.e_stop_1 || !TeR.res_pdo_tx.e_stop_2) { // sl open at res ? (res abierto vamos)
				dv_state = AS_EMERGENCY;
			} else { // sl not open at RES
				dv_state = AS_FINISHED;
			}
		} else {
			dv_state = AS_EMERGENCY;
		}
	} else if ((TeR.dv_system_status.ami_state != 0) // si no es manual
			&& (TeR.status.asms)
			&& (TeR.asb_status.asb_ebs_state
					== TER_ASB_STATUS_ASB_EBS_STATE_INITIAL_CHECK_PASSED_CHOICE)
			&& (TeR.asb_status.asb_redundancy_state
					== TER_ASB_STATUS_ASB_REDUNDANCY_STATE_INITIAL_CHECK_PASSED_CHOICE)
			&& (TeR.status.state >= PRECHARGED)) { // >= precharged ya que a partir de este estado el TS está activo, que es lo que requiere la norma
		if (TeR.status.r2_d == 1) {
			dv_state = AS_DRIVING;
		} else if ((ter_bpps_bpps_decode(TeR.bpps.bpps) >= TeR.config.r2_d_brake)) {
			dv_state = AS_READY;
		}
	} else {
		dv_state = AS_OFF;
	}
	return dv_state;
}

/*
 *
 * Thread that controls the execution of the main DV statemachine
 *
 *
 * */

void dvStateMachine(void *argument) {
	uint32_t currentTick = osKernelGetTickCount();
	for (;;) {

		currentTick += task_period;
		osDelayUntil(currentTick);
		dv_stateLoop();
	}
}
/*
 * Main statemachine of the driverless module
 *
 * -> gets the state
 * -> executes state changes transitions configurations
 * -> executes permanents tasks associated with the current state
 * -> ensures transitions are performed as required by FSG RULES
 * (5s delay as_ready for as_drivind & 3s delay as driving to accept request from dv)
 * -> ensures driverless computer can perform a stable maneuver (o como se escriba) when entering AS EMERGENCY
 * -> CONTROLS THE SL REQUEST OF THE DV STATE MACHINE
 * -> And many other things, check the code
 *
 * */
void dv_stateLoop() {
	 permatask();
	//unsafe_set_steer_angle(20);
	TeR.status.asms = HAL_GPIO_ReadPin(DIN3_GPIO_Port, DIN3_Pin);
	dv_state_t prevState = TeR.dv_system_status.as_status;
	dv_state_t state = get_dv_state();
	uint8_t stateChanged = state != prevState ? 1 : 0;
	if (stateChanged) { // Handles setup conditions for the new state
		switch (state) {
		case AS_OFF:
			TeR.asb_brake_req.brake = TER_ASB_BRAKE_REQ_BRAKE_ENABLED_CHOICE;
			TeR.dv_system_status.steering_state =
			TER_DV_SYSTEM_STATUS_STEERING_STATE_UNAVAILABLE_CHOICE;
			TeR.status.as_allowed = 0;
			break;
		case AS_READY:
			TeR.asb_brake_req.brake = TER_ASB_BRAKE_REQ_BRAKE_ENABLED_CHOICE;
			TeR.dv_system_status.steering_state =
			TER_DV_SYSTEM_STATUS_STEERING_STATE_UNAVAILABLE_CHOICE;
			TeR.status.as_allowed = 0;
			ready_time = 0; // reseteamos ready_time, importantisimo

			// setup dv driving mode & other relevant configurations for the driverless computer
			TeR.config.driving_mode =
			TER_ECU_CONFIG_DRIVING_MODE_DV_TORQUE_REQUEST_CHOICE;
			TeR.config.regen_mode = TER_ECU_CONFIG_REGEN_MODE_FREE_CHOICE;
			TeR.config.regen_enable = TER_ECU_CONFIG_REGEN_ENABLE_ENABLE_CHOICE;
			TeR.config.trq_limit = 20; // todo quitar en un futuro
			break;

		case AS_DRIVING:
			osTimerStart(as_allowed_timerHandle, 3000); // retrasar la asignacion de as_allowed=1 a 3 segundos en el futuro (cosas normativa)
			break;

		case AS_EMERGENCY:
			TeR.asb_brake_req.brake = TER_ASB_BRAKE_REQ_BRAKE_ENABLED_CHOICE;
			TeR.status.as_allowed = 0; // disable AS to make torque requests
			osTimerStart(as_emergency_beep_timerHandle, 200); // start a periodic beep timer that lasts 10 seconds for AS_EMERGENCY
			break;

		default:
			break;
		}
		TeR.dv_system_status.as_status = state;
	}

	switch (state) {

	case AS_OFF:
		// T 14.4 very important this manages the sl relay of the AS
		as_act_statemachine(); // manage requests and SL relay requests at startup
		TeR.dv_config.entry = TER_DV_CONFIG_ENTRY_MISSION_REQ_CHOICE;
		TeR.dv_config.mission_req = TeR.config.dv_mission_req;
		break;

	case AS_READY:
		if (heldFor(&ready_time, 1, 5000)) {                 // 5 s en AS_READY
			if (heldFor(&res_k2, TeR.res_pdo_tx.k3 && TeR.res_pdo_tx.k2, 500)) { // k2 pulsado >= 500 ms
				easyCommand(TER_COMMAND_CMD_READY2_DRIVE_DV_CHOICE);
			}
		}
		break;

	case AS_DRIVING:
		if(TeR.dv_info.mission_status == TER_DV_INFO_MISSION_STATUS_FINISHED_CHOICE){
			// REQUEST DE ACTIVACIÓN DE EBS

		}

		if (TeR.status.as_allowed) { // si la flag as allowed esta puesta, podemos hacer requests al DV

			//bypass request de freno dv -> asb board signal
			//TeR.asb_brake_req.brake = TeR.dv_dynamic_req_2.asb_brake_req;
			TeR.asb_brake_req.brake = 0;
			//bypass request steering dv -> steering motor signal
			set_steer_angle(
					ter_dv_dynamic_req_1_steer_angle_req_decode(
							TeR.dv_dynamic_req_1.steer_angle_req));

			//trq is controlled in the drivingmode, check TeR_TRQMANAGER.c
		}
		break;

	case AS_EMERGENCY:
//		TeR.asb_ebs_state_req = TER_ASB_EBS_STATE_REQ_STATE_REQ_ENABLED_CHOICE;
//		TeR.asb_redundancy_req = TER_ASB_REDUNDANCY_REQ_STATE_REQ_ENABLED_CHOICE;
		TeR.asb_brake_req.brake = TER_ASB_BRAKE_REQ_BRAKE_ENABLED_CHOICE; // no debería de servir para nada pero por si acaso (porque si estamos aqui el ebs ha triggereado)
		if (TeR.wheelInfo.speed > 5) { // permite al DV hacer una parada controlada en caso de entrar en AS_EMERGENCY (le permitimos control de steering hasta un threshold)
			set_steer_angle(
					ter_dv_dynamic_req_1_steer_angle_req_decode(
							TeR.dv_dynamic_req_1.steer_angle_req));
		} else { // cuando la velocidad sea inferior al threshold, quedará desactivado el steering
			TeR.dv_system_status.steering_state =
			TER_DV_SYSTEM_STATUS_STEERING_STATE_UNAVAILABLE_CHOICE;
		}
		break;

	default:
		break;
	}

}
/*
 * Angle should be º * 10000
 * This is to avoid using many encode and decode functions, it is not needed
 * as TeR.dv_dynamic_req_1.steer_angle_req and TeR.steer_actuator_set_position.actuator_position have
 * the same scale factors, so we can avoid enconding and decoding
 * Steering actions will only be performed if the TeR.dv_system_status.steering_state is available, this is done in order
 * to prevent accidental activation of Steering rack in an usafe state
 *
 *
 * */
void set_steer_angle(float angle) {
	// OJO, el ángulo llega en RADIANES
	float max_left = ter_steer_actuator_set_position_actuator_position_encode(
	MAX_STEER_LEFT_ANGLE);
	float max_right = ter_steer_actuator_set_position_actuator_position_encode(
	MAX_STEER_RIGHT_ANGLE);
	angle = ter_steer_actuator_set_position_actuator_position_encode(angle);
	angle = angle*(DEG2RAD*ACTUATOR2STEER); // pasamos a GRADOS
	if (TeR.dv_system_status.steering_state ==
	TER_DV_SYSTEM_STATUS_STEERING_STATE_AVAILABLE_CHOICE) { // steering permitido

		angle = clampf(angle, max_right, max_left); // multiplicado por los factores del DBC todo max y min angle por can configurables
		TeR.steer_actuator_set_position.actuator_position = angle;
		uint8_t TxData[8] = { 0 };
		ter_steer_actuator_set_position_pack(TxData,
				&TeR.steer_actuator_set_position, sizeof(TxData));
		can_scheduler_insert_non_periodic_msg(TxData, sizeof(TxData),
		TER_STEER_ACTUATOR_SET_POSITION_FRAME_ID, 0); // añadir al scheduler (no puede ser un periodico porque sino el motor haria fuerza, solo enviar mensaje cuando queremos que haga fuerza)
	}
}

/*
 * Set the steer angle without caring if the car is in the ready state, useful for testing purposes
 * under your own responsability (this will get you a insta DQ)
 *
 * */
void unsafe_set_steer_angle(float angle) {
	float max_left = ter_steer_actuator_set_position_actuator_position_encode(
	MAX_STEER_LEFT_ANGLE);
	float max_right = ter_steer_actuator_set_position_actuator_position_encode(
	MAX_STEER_RIGHT_ANGLE);
	angle = ter_steer_actuator_set_position_actuator_position_encode(angle);
	angle = angle*DEG2RAD*ACTUATOR2STEER;
	angle = clampf(angle, max_right, max_left); // multiplicado por los factores del DBC todo max y min angle por can configurables en un futuro
	TeR.steer_actuator_set_position.actuator_position = angle;
	uint8_t TxData[8] = { 0 };
	ter_steer_actuator_set_position_pack(TxData,
			&TeR.steer_actuator_set_position, sizeof(TxData));
	can_scheduler_insert_non_periodic_msg(TxData, sizeof(TxData),
	TER_STEER_ACTUATOR_SET_POSITION_FRAME_ID, 0);
}

void permatask() {
	if (TeR.dv_system_status.as_status == AS_DRIVING && TeR.status.as_allowed) {

	}
	// comentado por que de momento tengo que pensar de donde sacarlo
//	TeR.dv_driving_dynamics_1.brake_hydr_actual;
//	TeR.dv_driving_dynamics_1.brake_hydr_target;
//	TeR.dv_driving_dynamics_1.motor_moment_actual = TeR.dv_dynamic_req_1.trq_req;
//	TeR.dv_driving_dynamics_1.motor_moment_target = TeR.dv_dynamic_req_1.trq_req;
//	TeR.dv_driving_dynamics_1.speed_actual = TeR.wheelInfo.speed;
//	TeR.dv_driving_dynamics_1.speed_target = TeR.wheelInfo.speed;
//	TeR.dv_driving_dynamics_1.steering_angle_actual =
//			TeR.steer_actuator_status.position;
//	TeR.dv_driving_dynamics_1.steering_angle_target =
//			TeR.dv_dynamic_req_1.steer_angle_req;

//	TeR.dv_driving_dynamics_2.acceleration_lateral;
//	TeR.dv_driving_dynamics_2.acceleration_longitudinal;
//	TeR.dv_driving_dynamics_2.yaw_rate;
	TeR.dv_system_status.ami_state = TeR.config.dv_mission_req;
	TeR.dv_system_status.as_ebs_state = TeR.asb_status.asb_ebs_state;
	TeR.dv_system_status.asb_redundancy_state = TeR.asb_status.asb_redundancy_state;
	TeR.dv_system_status.cones_count_actual = TeR.dv_info.cones_count_actual;
	TeR.dv_system_status.cones_count_all = TeR.dv_info.cones_count_all;
	TeR.dv_system_status.lap_counter = TeR.dv_info.lap_counter;
}

/*
 * Todavia estoy pensando como hacer esto, se puede hacer sencillo con software timers
 * o todavia creando otro thread, aunque realmente me parece innecesario
 *
 *
 * */
void assiManager(void *argument) {
	for (;;) {
		osDelay(10);
		switch (TeR.dv_system_status.as_status) {
		case AS_OFF:
			set_assi_blue(0);
			set_assi_yellow(0);
			//apagar assi
			break;
		case AS_READY:
			// assi yellow
			set_assi_blue(0);
			set_assi_yellow(1);
			break;
		case AS_DRIVING:
			// assi yellow flashing
			set_assi_blue(0);
			toggle_assi_yellow();
			osDelay(200);
			break;
		case AS_EMERGENCY:
			// blue flashing
			toggle_assi_blue();
			set_assi_yellow(0);
			osDelay(200);
			break;
		case AS_FINISHED:
			//blue continuous
			set_assi_blue(1);
			set_assi_yellow(0);
			osDelay(200);
			break;
		default:
			break;
		}
	}
}

/*
 * Get the activation state of the driverless system
 * -> first, check if the car is in manual mode this is:
 * -ASMS off, MANUAL mission selected, EBS and REDUNDANT deactivated, brake ENERGY unavailable
 * if so, activation state is AS_ACT_OFF
 *
 * -> else, get the activation state of the driverless system
 *
 * Why does this exists?
 * -> While is AS_OFF, the vehicle might be in many different states, such as:
 * -> waiting for brake pressure
 * -> waiting for precharge
 * ->abs energy disconnected
 *
 * It can be done without this "mini" statemachine, but the code is a big mess
 *
 *
 * */
dv_act_state_t get_as_act_state() {
	dv_act_state_t state = AS_ACT_OFF;
	if ((TeR.status.asms == 0)
			&& (TeR.dv_system_status.ami_state == 0)
			&& (TeR.asb_status.asb_energy_status
					== TER_ASB_STATUS_ASB_ENERGY_STATUS_UNAVAILABLE_CHOICE)
			&& (TeR.asb_status.asb_ebs_state
					== TER_ASB_STATUS_ASB_EBS_STATE_DEACTIVATED_CHOICE)
			&& (TeR.asb_status.asb_redundancy_state
					== TER_ASB_STATUS_ASB_REDUNDANCY_STATE_DEACTIVATED_CHOICE)) {
		return state; // AS_ACT_IDLE, no hacemos nada
	} else {
		state = AS_ACT_WAIT_ASMS;
	}
	if (!TeR.status.asms)
		return AS_ACT_WAIT_ASMS;
	if (TeR.dv_system_status.ami_state == 0)
		return AS_ACT_WAIT_MISSION;
//	if (TeR.asb_status.asb_energy_status
//			== TER_ASB_STATUS_ASB_ENERGY_STATUS_UNAVAILABLE_CHOICE)
//		return AS_ACT_WAIT_ENERGY;
	if ((ter_bpps_bpps_decode(TeR.bpps.bpps) < TeR.config.r2_d_brake))
		return AS_ACT_WAIT_BRAKE;
	if (TeR.status.state <= RDY2PRECH) // todo, añadir estado intermedio que sea as_wait_sl por ejemplo, en el que mandemos request de cerrar rele de SL y a la espera de sl cerrada
		return AS_ACT_READY2PRECH;
	if (TeR.status.state == PRECHARGED) {
		if ((TeR.asb_status.asb_ebs_state
				== TER_ASB_STATUS_ASB_EBS_STATE_INITIAL_CHECK_PASSED_CHOICE)
				&& (TeR.asb_status.asb_redundancy_state
						== TER_ASB_STATUS_ASB_REDUNDANCY_STATE_INITIAL_CHECK_PASSED_CHOICE)) {
			return AS_ACT_DONE; // nunca vamos a llegar aqui XD (porque cambiaremos a AS_READY cuando esto sea cierto y esto solo se ejecuta en AS OFF)
		} else {
			return AS_ACT_PRECHARGED; // estamos precargados pero aún falta self-check todo prodriamos añadir timeout (innecesario porque el asb se encargaría de abrir la en caso de fallo supongo)
		}
	}
	return state;
}
/*
 *
 * Executes logic for the activation of the driverless system
 * Controls the safetyline of the DV module
 * how it works:
 * -> gets the state
 * -> executes the requested actions
 * -> controls the SDC, if the state is AS_ACT_OFF the SL is closed
 * -> else, its open util the car is in AS_ACT_OFF or AS_ACT_READY2PRECH
 *
 * */

void as_act_statemachine() {
	dv_act_state_t state = get_as_act_state();
	dv_act_state_t prevState = dv_act_state;
	uint8_t stateChanged = state != prevState ? 1 : 0;
	if (stateChanged) {

		switch (state) {
		case AS_ACT_OFF:
			// cerrar sl, modo manual
			set_sl_request(SL_DV, 1);
			break;

		case AS_ACT_WAIT_ASMS:
			set_sl_request(SL_DV, 0);
			// nada
			break;
		case AS_ACT_WAIT_MISSION:
			set_sl_request(SL_DV, 0);
			//nada
			break;
		case AS_ACT_WAIT_ENERGY:
			set_sl_request(SL_DV, 0);
			// nada
			break;
		case AS_ACT_WAIT_BRAKE:
			set_sl_request(SL_DV, 0);
			//nada
			break;
		case AS_ACT_READY2PRECH: // el sistema esta listo, cerramos safetyline todo quizas un estado anterior de cerrando SL
			set_sl_request(SL_DV, 1);
			break;

		case AS_ACT_PRECHARGED: // coche precargado, pedimos selfcheck de ASB
			TeR.asb_ebs_state_req.state_req =
			TER_ASB_EBS_STATE_REQ_STATE_REQ_SELF_CHECK_CHOICE;
			TeR.asb_redundancy_req.state_req =
			TER_ASB_REDUNDANCY_REQ_STATE_REQ_SELF_CHECK_CHOICE;
			break;
		case AS_ACT_DONE:
			// all check done, automatically will switch to AS_READY
			break;
		}
	}
	dv_act_state = state; // tengo que crear la señal de can luego lo hago todo
	switch (state) { // permanent checking
	case AS_ACT_READY2PRECH:
		if (heldFor(ext_ts, HAL_GPIO_ReadPin(DIN2_GPIO_Port, DIN2_Pin), 500)) { // TODO lectura boton TS externo + SL cerrada
			easyCommand(TER_COMMAND_CMD_PRECHARGE_DV_CHOICE); // enviamos request de precarga DV ( no deberia de haber problema al mantener pulsado, el coche cambia de estado a dirving y listo)
		}
		break;
	default:
		break;
	}
}
void set_assi_yellow(uint8_t set) {
	HAL_GPIO_WritePin(DOUT2_GPIO_Port, DOUT2_Pin, set);

}
void toggle_assi_yellow() {
	HAL_GPIO_TogglePin(DOUT2_GPIO_Port, DOUT2_Pin);
}

void toggle_assi_blue() {
	HAL_GPIO_TogglePin(DOUT3_GPIO_Port, DOUT3_Pin);
}

void set_assi_blue(uint8_t set) {
	HAL_GPIO_WritePin(DOUT3_GPIO_Port, DOUT3_Pin, set);
}

