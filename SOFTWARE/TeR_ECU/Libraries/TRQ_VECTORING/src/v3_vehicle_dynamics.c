/*
 * v3_vehicle_dynamics.c
 *
 *  Branch: feat/v3-embedded-qp-production
 *  Ver header (v3_vehicle_dynamics.h) para el contrato completo.
 */
#include "v3_vehicle_dynamics.h"
#include <math.h>

/* ------------------------------------------------------------------ */
/* Sanitizacion / regularizacion                                        */
/* ------------------------------------------------------------------ */

float v3_sanitize_f(float x, float fallback) {
	/* isfinite cubre NaN e Inf en un solo check, sin ramas anidadas */
	if (!isfinite(x)) {
		return fallback;
	}
	return x;
}

float v3_softplus(float x, float beta) {
	/*
	 * softplus(x) = ln(1+exp(beta*x)) / beta
	 *
	 * Version numericamente estable (evita overflow de expf cuando beta*x
	 * es grande y positivo): para z = beta*x grande, ln(1+exp(z)) ~= z,
	 * asi que devolvemos x directamente en ese regimen. Para z muy negativo,
	 * exp(z) ~= 0 y el resultado tiende a 0 de forma natural sin overflow.
	 */
	float z = beta * x;
	if (z > 20.0f) {
		return x; /* exp(20) ya satura un float, y ln(1+exp(z))/beta -> x */
	}
	if (z < -20.0f) {
		return 0.0f; /* exp(-20) ~ 2e-9, resultado indistinguible de 0 */
	}
	return log1pf(expf(z)) / beta;
}

/* ------------------------------------------------------------------ */
/* Cargas verticales                                                     */
/* ------------------------------------------------------------------ */

void v3_compute_wheel_loads(const v3_vehicle_params_t *params, float ax_ms2,
		float ay_ms2, v3_wheel_loads_t *loads_out) {

	/* Saneo de entradas: un glitch de IMU nunca debe producir un Fz NaN,
	 * que se propagaria a la elipse de friccion y de ahi al QP entero */
	float ax = v3_sanitize_f(ax_ms2, 0.0f);
	float ay = v3_sanitize_f(ay_ms2, 0.0f);

	float mass = params->mass_kg;
	float h = params->h_cdg_m;
	float track = params->track_width_rear_m;
	float wb = params->wheelbase_m;

	/* --- Reparto estatico + transferencia longitudinal (eje completo) --- */
	float fz_rear_static = mass * V3_GRAVITY_MS2 * params->rear_weight_frac;
	float fz_long_transfer = (wb > 1.0e-3f) ? (mass * ax * h / wb) : 0.0f;
	float fz_rear_axle = fz_rear_static + fz_long_transfer;

	/* --- Transferencia lateral, repartida segun rigidez a balanceo trasera --- */
	float fz_lat_total = (track > 1.0e-3f) ? (mass * ay * h / track) : 0.0f;
	float fz_lat_rear = fz_lat_total * params->rear_roll_stiffness_frac;

	/*
	 * Convencion: ay > 0 (giro a izquierda) transfiere carga hacia la
	 * derecha (RR gana carga, RL la pierde), consistente con el modelo
	 * bicicleta usado en tv_mds.c (yawRef positivo = giro a la izquierda).
	 */
	float fz_rl_raw = (fz_rear_axle * 0.5f) - (fz_lat_rear * 0.5f);
	float fz_rr_raw = (fz_rear_axle * 0.5f) + (fz_lat_rear * 0.5f);

	/*
	 * Regularizacion softplus: garantiza Fz >= V3_MIN_FZ_N de forma suave
	 * (C1-continua) en vez de un clamp duro, para que la elipse de
	 * friccion (y por tanto los limites de par del QP) no den un salto
	 * discontinuo cuando el modelo estatico predice "rueda en el aire".
	 */
	loads_out->fz_n[V3_WHEEL_RL] = v3_softplus(fz_rl_raw - V3_MIN_FZ_N,
	V3_SOFTPLUS_BETA) + V3_MIN_FZ_N;
	loads_out->fz_n[V3_WHEEL_RR] = v3_softplus(fz_rr_raw - V3_MIN_FZ_N,
	V3_SOFTPLUS_BETA) + V3_MIN_FZ_N;
}

/* ------------------------------------------------------------------ */
/* Elipse de friccion de Kamm                                           */
/* ------------------------------------------------------------------ */

float v3_friction_ellipse_fx_max(float fz_n, float fy_n_est, float mu) {

	float fz = v3_sanitize_f(fz_n, V3_MIN_FZ_N);
	float fy = v3_sanitize_f(fy_n_est, 0.0f);
	float mu_s = v3_sanitize_f(mu, 0.0f);

	fz = (fz < V3_MIN_FZ_N) ? V3_MIN_FZ_N : fz; /* invariante del pipeline */
	if (mu_s < 0.0f) {
		mu_s = 0.0f;
	}

	float fmax = mu_s * fz;                 /* radio de la elipse (fuerza total maxima) */
	float discriminant = (fmax * fmax) - (fy * fy);

	/*
	 * Regularizacion del discriminante: si |Fy| > mu*Fz (posible por un
	 * transitorio del observador lateral, o simplemente porque la rueda ya
	 * esta saturada lateralmente), el discriminante matematico es negativo
	 * y sqrt() devolveria NaN. Se acota a V3_ELLIPSE_EPS (>0) para
	 * devolver un Fx_max residual muy pequeño pero SIEMPRE finito y no
	 * negativo, evitando que un NaN se propague al QP (que no tiene
	 * proteccion contra NaN en sus limites de caja).
	 */
	if (discriminant < V3_ELLIPSE_EPS) {
		discriminant = V3_ELLIPSE_EPS;
	}

	return sqrtf(discriminant);
}

/* ------------------------------------------------------------------ */
/* Limites de par por rueda (box constraints del QP)                    */
/* ------------------------------------------------------------------ */

void v3_compute_torque_bounds(const v3_vehicle_params_t *params,
		const v3_wheel_loads_t *loads, const float fy_n_est[V3_N_WHEELS],
		float motor_trq_max_nm, float regen_trq_max_nm,
		v3_wheel_torque_bounds_t *bounds_out) {

	float r = params->wheel_radius_m;
	if (r < 1.0e-3f) {
		r = 1.0e-3f; /* evita division por cero si mal configurado */
	}

	float motor_max = v3_sanitize_f(motor_trq_max_nm, 0.0f);
	float regen_max = v3_sanitize_f(regen_trq_max_nm, 0.0f);
	if (motor_max < 0.0f) {
		motor_max = 0.0f;
	}
	if (regen_max < 0.0f) {
		regen_max = 0.0f;
	}

	for (uint8_t i = 0; i < (uint8_t) V3_N_WHEELS; i++) {
		float fy_i = v3_sanitize_f(fy_n_est[i], 0.0f);
		float fx_ellipse = v3_friction_ellipse_fx_max(loads->fz_n[i], fy_i,
				params->mu);
		float trq_ellipse = fx_ellipse * r; /* N -> Nm en la rueda */

		/* Interseccion (el mas restrictivo) entre elipse de friccion y limite del motor */
		float ub = (trq_ellipse < motor_max) ? trq_ellipse : motor_max;
		float lb = -((trq_ellipse < regen_max) ? trq_ellipse : regen_max);

		bounds_out->ub_nm[i] = ub;
		bounds_out->lb_nm[i] = lb;
	}
}