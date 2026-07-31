/*
 * tv_mds.c
 *
 *  Created on: Jan 31, 2024
 *      Author: Piero, Telmo Martinez de Salinas, Ozuba
 *      LOOPTIME DEBE DE SER IGUAL A EL EXISTENTE EN TRQMANAGER
 */

#include "tv_mds.h"

// todo: La velocidad hay que sacarla mejor de otro sitio ya que de la ruedas no me mola (torque que se autoafecta a si mismo loop chungo)
PID_t *tvPid; //Est-ructura del PID
float dTorque = 0;
const float looptime = 0.005f; //5ms of looptime (same as TeR_TRQMANAGER task)
static float kp, ki, kd, iMax;
//static float v_sched_ms_ema = 0.0f;
//--------------------------------------------------------[Gain Scheduling]---------------------------------------------------------------//
const int N = sizeof(v_bp_ms) / sizeof(v_bp_ms[0]);

static float interp(const float *bp, const float *tab, int n, float x) {
	if (x <= bp[0])
		return tab[0];
	if (x >= bp[n - 1])
		return tab[n - 1];
	int i = 0;
	while (i < n - 1 && x > bp[i + 1])
		i++;
	float x0 = bp[i], x1 = bp[i + 1];
	float y0 = tab[i], y1 = tab[i + 1];
	if(x1==x0){return 0;}
	float t = (x - x0) / (x1 - x0);
	return y0 + t * (y1 - y0);
}
static float ema(float prev, float x, float alpha) {
	return prev + alpha * (x - prev);
}
static void pidSetGains(PID_t *pid, float kp, float ki, float kd,
		float iMax_new) {
	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;
	pid->antiWindup = iMax_new;
}

//--------------------------------------------------------[Model Functions]---------------------------------------------------------------//
float yawRef(float steer, float vx) { //steer en radianes, vx en m/s
	//girar izq es positivo, realmente es el angulo de giro mediodel modelo bici
	return (steer * vx) / ((L_FRONT + L_REAR) + K_U * (vx * vx)); //unidades rad/seg
}

float mz2DeltaTorque(float alpha) { //Takes PID output (Toca revisar unidades de salida del PID), es una ganancia sin mas, realmente no hace nada
	return I_ZZ * (2 * R_WHEEL) / (GEAR_R * T_WIDTH) * alpha;
}

//--------------------------------------------------------[Model Functions]---------------------------------------------------------------//

trqMap_t trqVectoring(trq_t limit) {
	kp = TeR.config.trq_kp / 10000.0f; // input from CAN
	ki = TeR.config.trq_ki / 10000.0f;
	kd = TeR.config.trq_kd / 10000.0f;
	iMax = IMAX;
	if (!tvPid) { // Init pid if not enabled
		tvPid = initPID(kp, ki, kd, looptime, iMax);
	};

	//Declares a trqMap
	trqMap_t trqMap;
	dTorque = 0;

#ifdef SAFETY // for testing the car in the elevator, comment safety
	//check if car is not at speed, or pedal is not being pressed, if true Reset PID and LINEAR RESPONSE
	//Not in conditions for Torque Vectoring
	if (((TeR.wheelInfo.speed < ACTSPEED) || (TeR.apps.apps_av < ACTAPPS)
			|| (TeR.bpps.bpps > TeR.config.r2_d_brake))) { // if below activation speed or pedal below threshold, or steering not turning, return linear response and clear pid error
		trqMap.rLeft = map(TeR.apps.apps_av, 0, 255, 0, limit * 0.5);
		trqMap.rRight = map(TeR.apps.apps_av, 0, 255, 0, limit * 0.5);
		tvPid->error = 0; //clear P error
		tvPid->errorI = 0; // clear I error
		tvPid->errorD = 0; // clear D error
		tvPid->prevError = 0; //clear prev Error
		dTorque = 0; // set dTorque to 0
		TeR.tv_debug.delta_trq = 0;
		TeR.tv_debug.yaw_ref = 0;
		return trqMap; //return tv output
	}
#endif

	//Torque Vectoring Available lesgooo

	float v_ms = TeR.wheelInfo.speed * KMH2MS;
	v_ms = clampf(v_ms, 0, 22.22f); // clampeamos a 80 kmH
	//v_sched_ms_ema = ema(v_sched_ms_ema, v_ms, 0.1f); // filtro para suavizar cambios de ganancia

	// 2) interpola ganancias

	float kp_mul = interp(v_bp_ms, kp_tab, N, v_ms);
	float ki_mul = interp(v_bp_ms, ki_tab, N, v_ms);
	float kd_mul = interp(v_bp_ms, kd_tab, N, v_ms);
	float iM_mul = interp(v_bp_ms, iMax_tab, N, v_ms);
	kp = clampf(kp * kp_mul, KMIN, KPMAX);
	ki = clampf(ki * ki_mul, KMIN, KIMAX);
	kd = clampf(kd * kd_mul, KMIN, KDMAX);
	iMax = clampf(iMax * iM_mul, KMIN, IMAX);

	// 3) aplicar tabla de ganancias a PID
	pidSetGains(tvPid, kp, ki, kd, iMax);

	//Torque Vectoring Computation
	//1) get yaw, deadzone and calculate yawref
	float steer = ter_steer_angle_decode(TeR.steer.angle) * DEG2RAD;
	steer = isAngleInDeadzone(steer, STEER_DEADZONE) ? 0 : steer;
	float ref = yawRef(steer, TeR.wheelInfo.speed * KMH2MS);

	//2) calculate pid
	float imuYawR = IMU.w_z * DEG2RAD; // Imu yawRate a radianes
	//float imuYawR = ter_ang_rate_yaw_rate_z_decode(TeR.angRate.yaw_rate_z); // output en radianes PUTOS CABRONES DEL AUTONOMO
	imuYawR = isAngleInDeadzone(imuYawR, IMU_DEADZONE) ? 0 : imuYawR;
	float corr = pid(tvPid, ref, imuYawR); //Computa el lazo y devuelve el valor de correccion
	dTorque = mz2DeltaTorque(corr); //es una ganancia sin mas, no aporta al control

	//check if dTorque is in allowable range IF NOT CLAMP DTORQUE TO MAX VALUE
	dTorque = clampf(dTorque, -MAX_DELTA_TORQUE, MAX_DELTA_TORQUE);

	//Fill trqMap structure with dTorque
	trqMap.rRight = map(TeR.apps.apps_av, 0, 255, 0, limit * 0.5) + dTorque / 2;
	trqMap.rLeft = map(TeR.apps.apps_av, 0, 255, 0, limit * 0.5) - dTorque / 2;

	//save tv_debug data
	TeR.tv_debug.delta_trq = ter_tv_debug_delta_trq_encode(dTorque);
	TeR.tv_debug.yaw_ref = ter_tv_debug_yaw_ref_encode(ref / DEG2RAD); // pasamos yawref a centigrados
	return trqMap; //return tv output
}

uint8_t tv_deInitPID(void) {
	if (tvPid) { // is tvPid pointing to something not 0?
		deInitPID(&tvPid); // if yes free and set to NULL
	}
	return 1;
}

//Check if a value is is a deadzone
uint8_t isAngleInDeadzone(float angle, float range) {
	uint8_t result = fabsf(angle) < range ? 1 : 0;
	return result;
}





