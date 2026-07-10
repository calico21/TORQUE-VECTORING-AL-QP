/*
 * torqueManager.c
 *
 *  Created on: Mar 30, 2024
 *      Author: Ozuba
 *      Contributors: Piero
 *      Este archivo encapsula la gestión del torque del vehículo
 *      Se trata de un pipeline de ejecución que limita, calcula y valida el torque que se enviará a las ruedas
 *
 * 		Es importante que todo lo que cambies de este archivo sea testeado correctamente, no vale probar y decir "funciona"
 * 		recordemos que esto es algo extremadamente crítico, hay que testear testear y testear en el elevador, lo ultimo que quieres
 * 		es una condición rarisima en la cual la gestión de torque pete
 *
 * 		Siempre testea tus cambios con el coche subido en el elevador, NUNCA, NUNCA cambies algo de aqui y confies en que vaya a funcionar
 * 		recuerda que esto es peligroso
 * */

#include "TeR_TRQMANAGER.h"
#include "tv_mds.h"
#include "gp_interface.h" // NUEVO: Añadimos la interfaz del tv nuevo

const static int task_period = 5; // Task frequency

extern trqMap_t trqDistribution(trq_t limit);

trqPipeline_t DriveConfig = { .drivingMode = &lineal, .limiter = &limitTorque,
		.regenMode = regenModeAPPS, .sanityChecks = trqCheck, .tractionControl =
				tractionControlOFF }; //Configuración en uso (defaulteada por si acaso)
extern osThreadId_t trqManagerTaskHandle; // thread id of trqManager task

void trqManager(void *argument) { // Corre las etapas del pipeline y solicita la comanda
	uint32_t nextTick = osKernelGetTickCount(); // Initialize reference time

	for (;;) {
		nextTick += task_period; //Genera el timestamp de la siguiente ejecucion
		osDelayUntil(nextTick); // esperamos
		//Check if we are driving
		if ((TeR.status.state == DRIVING)) {
			//Execute Pipeline
			trq_t availableTorque = DriveConfig.limiter(); //genera una limitación de torque

			trqMap_t driveTorque = DriveConfig.drivingMode(availableTorque); //Distribuye torque

			driveTorque = DriveConfig.regenMode(driveTorque); //Distribuye torque teniendo en cuenta modo de regeneración (permite / otorga regeneración)

			trqMap_t trqToSanity = DriveConfig.tractionControl(driveTorque); // Limita si se vé que la rueda no puede dar ese grip (Estrategia dependiente del controlador)

			trqMap_t trqToWheels = DriveConfig.sanityChecks(trqToSanity,
					availableTorque); // basicamente que no se vaya para atras y que no pidas 300Nm, entre otras cosas un test que me permite dormir algo mas tranquilo
			//Solicitud de la comanda
			TeR.trqReqLeft.torque_nm_req = trqToWheels.rLeft;
			TeR.trqReqRight.torque_nm_req = trqToWheels.rRight;

		} else {
			// sanity check function pointer
			DriveConfig.sanityChecks = &trqCheck;
			//Config the driving pipeline if not driving
			switch (TeR.config.limiter) {

			case TER_ECU_CONFIG_LIMITER_TORQUE_CHOICE:
				DriveConfig.limiter = &limitTorque;
				break;
			default:
				DriveConfig.limiter = &limitTorque;
				break;
			}
			switch (TeR.config.driving_mode) {

			case TER_ECU_CONFIG_DRIVING_MODE_LINEAL_CHOICE:
				DriveConfig.drivingMode = &lineal;
				break;

			case TER_ECU_CONFIG_DRIVING_MODE_TORQUE_VECTORING_CHOICE:
				DriveConfig.drivingMode = &gp_mode_intermediate; // 1. Asignamos el modo de conducción a la función de Torque Vectoring
				gp_init(); // 2. Reiniciamos integrales y filtros físicos mientras el coche está parado
				break;

			case TER_ECU_CONFIG_DRIVING_MODE_DV_TORQUE_REQUEST_CHOICE:
				DriveConfig.drivingMode = &DVTrqRequest;
				break;

			default:
				DriveConfig.drivingMode = &lineal;
				break;

			}
			switch (TeR.config.traction_control) {

			case TER_ECU_CONFIG_TRACTION_CONTROL_OFF_CHOICE:
				DriveConfig.tractionControl = &tractionControlOFF;
				break;
			default:
				DriveConfig.tractionControl = &tractionControlOFF;
				break;
			}

			switch (TeR.config.regen_mode) {
			case TER_ECU_CONFIG_REGEN_MODE_APPS_CHOICE:
				DriveConfig.regenMode = &regenModeAPPS;
				break;
			case TER_ECU_CONFIG_REGEN_MODE_FREE_CHOICE:
				DriveConfig.regenMode = &regenModeFREE;
				break;
			default:
				DriveConfig.regenMode = &regenModeAPPS;
				break;

			}
			//Torque Zero safestate
			TeR.trqReqLeft.torque_nm_req = 0;
			TeR.trqReqRight.torque_nm_req = 0;
		}
	}
}

//------------------------------------------------[Basic Power Limiters]------------------------------------------------//
// void -> trq_t
//Par Máximo constante
trq_t limitTorque(void) {
	trq_t limit = TeR.config.trq_limit;
	limit = limit > 180 ? 180 : TeR.config.trq_limit; // hard limit
	limit = limit < 10 ? 10 : TeR.config.trq_limit;
	return limit; //Devuelve el valor configurado
}

//------------------------------------------------[Basic Driving Modes]------------------------------------------------//
// trq_t -> trqMap_t
trqMap_t lineal(trq_t limit) { //Entrega lineal de par a las 2 ruedas,se toman valores del APPS como source
	trqMap_t trqMap = { 0 };
	trqMap.rLeft = map(TeR.apps.apps_av, 0, 255, 0, limit * 0.5); // 0.5 porque tenemos 2 ruedas
	trqMap.rRight = map(TeR.apps.apps_av, 0, 255, 0, limit * 0.5);
	return trqMap;
}
// trq_t -> trqMap_t
trqMap_t DVTrqRequest(trq_t limit) { // aceptar request de torque con origen remoto(DV, por ejemplo)
	trqMap_t trqMap = { 0 };
	if (!TeR.status.as_allowed) { // si el dv no esta allowed aun: retornamos 0
		trqMap.rLeft = 0; // innecesario pero para que quede claro
		trqMap.rRight = 0;
		return trqMap;
	}
	float requested_trq = ter_dv_dynamic_req_1_trq_req_decode(
			TeR.dv_dynamic_req_1.trq_req); // importante el decode por factor de escala del DBC
	requested_trq = mapf(requested_trq, -1.0f, 1.0f, -limit, limit); // OJO, no mapees entre max regen y limit, la vas a liar basto!! (0 no sería 0 trq!!!) not funny!!!!
	requested_trq = requested_trq / 2; // 2 wheels
	trqMap.rLeft = (trq_t) requested_trq;
	trqMap.rRight = (trq_t) requested_trq;
	clamp_neg_trq(&trqMap, TeR.config.regen_max_trq);
	return trqMap;
}

//------------------------------------------------[Basic traction Control]------------------------------------------------//
// trqMap_t -> trqMap_t
trqMap_t tractionControlOFF(trqMap_t in) {
	return in;
}
//------------------------------------------------[Basic Regen]------------------------------------------------//
// trqMap_t -> trqMap_t
trqMap_t regenModeAPPS(trqMap_t in) {
	if (!regen_allowed()) {
		in.rLeft = in.rLeft < 0 ? 0 : in.rLeft;
		in.rRight = in.rRight < 0 ? 0 : in.rRight;
		return in; // retornamos, regen no permitida !!!
	}
	if (!((in.rLeft <= TeR.config.regen_max_positive_trq_thr / 2) // threshold de torque pedido a partir del cual consideramos "lift" del pedal /TODO cambiar a valor de apps
	&& (in.rRight <= TeR.config.regen_max_positive_trq_thr / 2)))
		return in; // retornamos, regen no permitida !!!
	int8_t trq = TeR.config.regen_max_trq;
	trq = -abs(trq / 2); // per wheel
	in.rLeft = trq;
	in.rRight = trq;
	return in;
}
//------------------------------------------------[FREE Regen (within limits) (allow DV and other driving modes such as TV to request negative torque within limits)]------------------------------------------------//
// trqMap_t -> trqMap_t
trqMap_t regenModeFREE(trqMap_t in) {
	if (!regen_allowed()) {
		in.rLeft = in.rLeft < 0 ? 0 : in.rLeft;
		in.rRight = in.rRight < 0 ? 0 : in.rRight;
		return in; // Regen is not available !!!
	}
	clamp_neg_trq(&in, TeR.config.regen_max_trq);
	return in; //retornamos pedido
}

/*
 * This function is used as a sanity check in the last stage of the pipeline
 * This function checks if somehow you managed to place a trq request that is impossible to fulfill:
 * 1) You tried to request negative torque with the regen disabled -> you may cause accumulator faults / overvoltages / overtemps / overcurrents/ you did not want to regen (regen disabled)
 * 2) You tried to request negative torque below a threshold speed -> you may cause the wheels to spin backwards -> insta DQ and could be VERY dangerous for the driver
 * 3) You tried to exceed the power limitation of the vehicle (bug in drivingMode or regenMode function and somehow limitation is exceded)
 *
 * All relevant checks should be performed in your DrivingMode and RegenMode functions, do not rely exclusively on sanity checks
 * Its your task as the programmer to ensure that all diving modes are safe, do not rely on sanitychecks
 */
trqMap_t trqCheck(trqMap_t in, trq_t limit) {

// 1) TRQ clamping
	clamp_pos_trq(&in, limit);
	clamp_neg_trq(&in, TeR.config.regen_max_trq);
	scale_max_trq(&in, limit);

// 2) Check if regen is allowed, if not, set negative requests to 0
	if (!regen_allowed()) {
		in.rLeft = in.rLeft < 0 ? 0 : in.rLeft;
		in.rRight = in.rRight < 0 ? 0 : in.rRight;
	}

// 3) Check if negative torque is being requested below activation speed, VERY IMPORTANT (avoids backwards spinning of the wheels)
	if (in.rLeft < 0 || in.rRight < 0) {
		if (TeR.wheelInfo.rl_rpm < TeR.config.regen_thr_rpm) { // responsabilidad tuya si pones los limites de forma incorrecta
			in.rLeft = 0;
		}
		if (TeR.wheelInfo.rr_rpm < TeR.config.regen_thr_rpm) {
			in.rRight = 0;
		}
		if (TeR.wheelInfo.speed < TeR.config.regen_thr_speed) {
			in.rRight = 0;
			in.rLeft = 0;
		}
	}

	return in;
}

uint8_t regen_allowed() { // 1 ok 0 not ok
	if (!isAngleInDeadzone(ter_steer_angle_decode(TeR.steer.angle), 15)) {
		return 0;
	}
	if (!(TeR.config.regen_enable == TER_ECU_CONFIG_REGEN_ENABLE_ENABLE_CHOICE)) // regen activada?
		return 0;
	if (!(ams_cell_voltage_status_cell_max_volt_decode(
			TeR.bms_voltage_status.cell_max_volt) // celdas en rango de tension? (pone min porque el dbc del bms estaba al revés, cuando lo arreglen lo cambio TODO
	< TeR.config.regen_max_cell_volt))
		return 0;
	if (!(ams_cell_temperatures_status_cell_max_temp_decode(
			TeR.bms_temperatures_status.cell_max_temp)
			< TeR.config.regen_max_cell_temp))
		return 0;
	if (!(ams_hv_measurements_status_current_a_decode(
			TeR.bms_hv_measurements_status.current_a) // accu en rango de corriente ?
	> -TeR.config.regen_max_current)) // always set below your max accumulator regen current, currently is 80A so this should be 60A or so
		return 0;
	// si se han coumplido todas las condiciones necesarias para regenerar, retornamos 1
	return 1;
}

/*
 * -> Limit per wheel positive torque based on limit
 *
 * */
void clamp_pos_trq(trqMap_t *in, trq_t limitPos) {
	trq_t maxPosTrqWheel = abs(limitPos) / 2;
	if (in->rLeft > 0) { //if is positive
		in->rLeft = in->rLeft > maxPosTrqWheel ? maxPosTrqWheel : in->rLeft; //if exeeds limit clamp
	}
	if (in->rRight > 0) {
		in->rRight = in->rRight > maxPosTrqWheel ? maxPosTrqWheel : in->rRight; //if exeeds limit clamp
	}
}
/*
 * -> Limit per wheel negative torque based on limit
 *
 * */
void clamp_neg_trq(trqMap_t *in, trq_t limitNeg) {
	trq_t maxNegTrqWheel = -abs(limitNeg) / 2; // sane the input and MAKE IT NEGATIVE, VERY VERY IMPORTANT!!!!  if not catastrophic things could happen (trq stuck to maxNegTrqWheel!!!!!!!!!)
	if (in->rLeft < 0) { //if is negative, check and clamp
		in->rLeft = in->rLeft < maxNegTrqWheel ? maxNegTrqWheel : in->rLeft; //if exeeds limit clamp
	}
	if (in->rRight < 0) { //if is negative, check and clamp
		in->rRight = in->rRight < maxNegTrqWheel ? maxNegTrqWheel : in->rRight; //if exeeds limit clamp
	}
}

/*
 * Check if overall torque exceeds torque limitation
 * this SHOULD be impossible if regentrq < limittrq
 * This function only makes sense if you somehow have a positive torque limitation, a negative torque limitation, and an overall torque limitation
 * */
void scale_max_trq(trqMap_t *in, trq_t limit) {
	limit = abs(limit);
	int32_t total = abs(in->rLeft) + abs(in->rRight);
	if ((total > limit) && (total != 0)) { // prevent stupid and impossible case when a division by 0 could occur
		float scale = (float) limit / total;
		in->rLeft = (trq_t) (in->rLeft * scale); // scale and clamp
		in->rRight = (trq_t) (in->rRight * scale);
	}
}

