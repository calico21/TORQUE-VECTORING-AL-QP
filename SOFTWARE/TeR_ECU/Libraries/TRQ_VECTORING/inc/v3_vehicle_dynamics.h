/*
 * v3_vehicle_dynamics.h
 *
 *  Branch: feat/v3-embedded-qp-production
 *  Module: Tire Friction Ellipse & Vehicle Model
 *
 *  Author: Tecnun eRacing - TV Team
 *
 *  Objetivo:
 *  Modelo de vehiculo minimo (quasi-static, no rolling-plane) usado unicamente
 *  para generar los limites de caja (box constraints) por rueda que consume
 *  el solver AL-QP (v3_qp_solver.c). Este modulo es agnostico de HAL/CAN
 *  a proposito: recibe floats desnudos, para poder testearlo bit a bit en
 *  SIL (Software In the Loop) mediante ctypes sin arrastrar stm32f4xx_hal.h
 *  ni las estructuras TeR_t. La integracion con el resto del firmware se
 *  hace en la capa de arriba (llamando a estas funciones desde TeR_TRQMANAGER
 *  o desde tv_mds, pasando los floats ya decodificados del CAN).
 *
 *  Requisitos NO negociables (ver system prompt / normativa interna):
 *  - Cero heap, cero malloc/free, cero recursion.
 *  - O(1) determinista: ninguna funcion contiene bucles de tamano variable.
 *  - Todas las entradas externas (sensores, CAN) se sanean (isfinite) antes
 *    de propagarse a la fisica, para blindar contra sensor glitches
 *    (frames corruptos, floats NaN/Inf por division entre 0 en el decode,
 *    etc.)
 *  - Los limites fisicos (Fz >= 0, sqrt de discriminantes negativos en la
 *    elipse de friccion) se regularizan de forma suave (softplus / máscara
 *    con epsilon), NUNCA con un if/branch duro que introduzca un salto de
 *    torque instantaneo (torque hunting) en el lazo de control.
 */

#ifndef INC_V3_VEHICLE_DYNAMICS_H_
#define INC_V3_VEHICLE_DYNAMICS_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* Constants                                                            */
/* ------------------------------------------------------------------ */
#define V3_GRAVITY_MS2          (9.81f)
#define V3_MIN_FZ_N             (5.0f)   /* Fz mecanico minimo permitido, evita division ~0 en la elipse */
#define V3_SOFTPLUS_BETA        (0.08f)  /* Suavidad del softplus (Fz>=0). Mayor beta -> mas parecido a max(0,x) */
#define V3_ELLIPSE_EPS          (1.0e-3f)/* Regularizacion del discriminante de la elipse de friccion */

/* Indices de rueda (solo tren trasero, RWD para torque vectoring) */
typedef enum {
	V3_WHEEL_RL = 0, /* Rear Left  */
	V3_WHEEL_RR = 1, /* Rear Right */
	V3_N_WHEELS = 2
} v3_wheel_id_t;

/* ------------------------------------------------------------------ */
/* Structures                                                           */
/* ------------------------------------------------------------------ */

/*
 * Parametros estaticos del vehiculo (constantes de calibracion, no cambian
 * en runtime). Analogos a los #define de tv_mds.h (I_ZZ, T_WIDTH, L_FRONT,
 * L_REAR, H_CDG...) pero encapsulados en struct para poder inyectar
 * distintos vehiculos/configs en el test SIL sin recompilar.
 */
typedef struct {
	float mass_kg;             /* Masa total del vehiculo + piloto (kg) */
	float h_cdg_m;              /* Altura del CDG (m) */
	float track_width_rear_m;   /* Via trasera (m) */
	float wheelbase_m;          /* Distancia entre ejes (m) */
	float rear_weight_frac;     /* Fraccion estatica de peso en tren trasero [0..1] */
	float rear_roll_stiffness_frac; /* Fraccion de la transferencia lateral total que absorbe el eje trasero [0..1] */
	float mu;                   /* Coeficiente de friccion neumatico-asfalto efectivo */
	float wheel_radius_m;       /* Radio de rueda (m), para pasar de Fx(N) a Torque(Nm) */
} v3_vehicle_params_t;

/* Salida de cargas verticales instantaneas, una por rueda trasera */
typedef struct {
	float fz_n[V3_N_WHEELS];   /* Carga vertical regularizada (N), siempre >= V3_MIN_FZ_N */
} v3_wheel_loads_t;

/* Limites de par por rueda (Nm), listos para alimentar al QP como box constraints */
typedef struct {
	float lb_nm[V3_N_WHEELS];  /* Limite inferior de par (negativo = regen/frenado motor) */
	float ub_nm[V3_N_WHEELS];  /* Limite superior de par (positivo = traccion) */
} v3_wheel_torque_bounds_t;

/* ------------------------------------------------------------------ */
/* API                                                                   */
/* ------------------------------------------------------------------ */

/*
 * Satura un float de entrada externa (sensor/CAN) a un valor seguro.
 * Si x no es finito (NaN/Inf, tipico de un frame corrupto o un decode con
 * factor de escala erroneo) se sustituye por `fallback`. Usar SIEMPRE al
 * entrar cualquier medida externa (ax, ay, mu estimado...) al modulo.
 */
float v3_sanitize_f(float x, float fallback);

/*
 * Softplus suave: aproxima max(0, x) de forma C1-continua.
 * f(x) = ln(1 + exp(beta*x)) / beta
 * Se usa para evitar Fz negativos (rueda "levantada" en el modelo estatico
 * durante transferencias de carga agresivas) sin introducir un clamp duro
 * que generaria un salto de gradiente y por tanto un salto de torque
 * cuando el QP recalcule sus limites de caja.
 * Implementacion numericamente estable (evita overflow de expf para beta*x grande).
 */
float v3_softplus(float x, float beta);

/*
 * Calcula la carga vertical (Fz, N) de las 2 ruedas traseras a partir de
 * las aceleraciones longitudinal (ax, m/s^2, +accel) y lateral (ay, m/s^2,
 * + = giro a izquierda segun convencion ISO/SAE right-hand con Z arriba)
 * medidas por la IMU, mas la carga estatica del reparto de pesos.
 *
 * Todas las entradas se sanean internamente (isfinite) antes de operar.
 * El resultado esta regularizado con v3_softplus para garantizar
 * fz_n[i] >= V3_MIN_FZ_N siempre (invariante que el resto del pipeline
 * puede asumir sin checks adicionales de division por cero).
 */
void v3_compute_wheel_loads(const v3_vehicle_params_t *params, float ax_ms2,
		float ay_ms2, v3_wheel_loads_t *loads_out);

/*
 * Limite de fuerza longitudinal (Fx, N) disponible en una rueda segun la
 * elipse de friccion de Kamm: Fx^2 + Fy^2 <= (mu*Fz)^2
 * dada una estimacion de la fuerza lateral ya consumida en esa rueda (Fy,
 * N, puede venir de un observador o asumirse 0 en tramos rectos).
 *
 * El discriminante (mu*Fz)^2 - Fy^2 se regulariza con V3_ELLIPSE_EPS para
 * que nunca sea negativo (lo que ocurriria si Fy excede momentaneamente
 * mu*Fz por un glitch del observador lateral), devolviendo un Fx_max >= 0
 * de forma continua en vez de NaN.
 */
float v3_friction_ellipse_fx_max(float fz_n, float fy_n_est, float mu);

/*
 * Calcula los limites de par [lb, ub] (Nm) por rueda trasera, combinando:
 *  - El limite de la elipse de friccion (v3_friction_ellipse_fx_max),
 *    convertido de fuerza (N) a par de rueda (Nm) mediante wheel_radius_m.
 *  - El limite duro del inversor/motor (motor_trq_max_nm), ya con
 *    cualquier derating termico aplicado aguas arriba (por ejemplo por
 *    TeR.invInfo.left_motor_temp / right_motor_temp).
 *  - El limite de regen configurado (regen_trq_max_nm, se espera >= 0,
 *    internamente se usa como -regen_trq_max_nm para el lb).
 *
 * lb/ub finales = intersección (el más restrictivo) de los 3 limites anteriores.
 */
void v3_compute_torque_bounds(const v3_vehicle_params_t *params,
		const v3_wheel_loads_t *loads, const float fy_n_est[V3_N_WHEELS],
		float motor_trq_max_nm, float regen_trq_max_nm,
		v3_wheel_torque_bounds_t *bounds_out);

#ifdef __cplusplus
}
#endif

#endif /* INC_V3_VEHICLE_DYNAMICS_H_ */