/*
 * v3_qp_solver.h
 *
 *  Branch: feat/v3-embedded-qp-production
 *  Module: AL-QP Solver (rear axle torque allocation)
 *
 *  Author: Tecnun eRacing - TV Team
 *
 *  Problema resuelto (por ciclo de control, 2 variables: T_rl, T_rr):
 *
 *    min_x   0.5*wt_i*(x_i - xnom_i)^2 + 0.5*ws_i*(x_i - xprev_i)^2   (sum sobre i=RL,RR)
 *    s.t.    x_RL + x_RR = T_demand                    (igualdad: par total pedido por piloto/DV)
 *            lb_i <= x_i <= ub_i                         (caja: elipse de friccion + limite motor/regen)
 *
 *  - xnom_i:  reparto nominal deseado por rueda (demanda del piloto ya repartida
 *             50/50 + el delta de yaw moment del bicycle-model, ver tv_mds.c/mz2DeltaTorque).
 *             Este termino es el "tracking" del objetivo de alto nivel.
 *  - xprev_i: par realmente demandado a esa rueda en el ciclo anterior. Termino
 *             de "smoothing" que penaliza slew rate para evitar escalones de
 *             par (driveline shock / resonancia de la transmision).
 *  - lb/ub:   ver v3_vehicle_dynamics.h (v3_compute_torque_bounds).
 *
 *  Metodo: Augmented Lagrangian con NUMERO FIJO de iteraciones
 *  (V3_QP_FIXED_ITERS = 16), sin criterio de parada por tolerancia (eso
 *  haria el tiempo de ejecucion dependiente del dato de entrada, lo cual
 *  es inadmisible en un lazo de control a 200Hz con presupuesto de tiempo
 *  fijo). Cada iteracion es:
 *
 *    r      = sum_i(x_i) - T_demand                       (residuo de la igualdad)
 *    x_i    = Proj_[lb_i,ub_i]( c_i - (lambda + rho*r) / W_i )   (paso primal, W_i = wt_i+ws_i precalculado)
 *    lambda = lambda + rho * (sum_i(x_i) - T_demand)        (ascenso dual)
 *
 *  donde c_i = (wt_i*xnom_i + ws_i*xprev_i) / W_i es el centro de la cuadratica
 *  separable de cada rueda, tambien precalculado con la reciproca de W_i para
 *  no dividir dentro del bucle caliente.
 *
 *  El multiplicador lambda (`v3_qp_state_t.lambda`) persiste entre llamadas
 *  (warm start) para acelerar la convergencia ciclo a ciclo, pero se mantiene
 *  DELIBERADAMENTE desacoplado del historial de par de las ruedas (xprev, que
 *  vive en la capa de traccion/slip de TeR_TRQMANAGER). Mezclar ambos estados
 *  produciria "caza" (hunting) entre el optimizador de alto nivel y el control
 *  de traccion de bajo nivel: el lambda representa unicamente la presion de la
 *  restriccion de igualdad, nunca una estimacion de slip.
 *
 *  Garantias de tiempo real:
 *  - v3_qp_solve() no contiene malloc/free, ni recursion, ni bucles de
 *    tamaño variable (todos los bucles son `for` con limite fijo V3_N_WHEELS
 *    o V3_QP_FIXED_ITERS, desenrollables por el compilador).
 *  - Todas las divisiones de la config (1/W_i) estan precalculadas en
 *    v3_qp_config_init(), fuera del lazo de control.
 */

#ifndef INC_V3_QP_SOLVER_H_
#define INC_V3_QP_SOLVER_H_

#include <stdint.h>
#include "v3_vehicle_dynamics.h" /* V3_N_WHEELS, v3_wheel_id_t */

#ifdef __cplusplus
extern "C" {
#endif

#define V3_QP_FIXED_ITERS   (16u)
#define V3_QP_LAMBDA_MAX    (5000.0f) /* anti-windup del multiplicador (Nm), ver nota en v3_qp_solve */

/* ------------------------------------------------------------------ */
/* Config (constante durante una sesion de conduccion, precalculada)    */
/* ------------------------------------------------------------------ */
typedef struct {
	float wt[V3_N_WHEELS];        /* peso de tracking (fidelidad al reparto nominal / yaw moment) */
	float ws[V3_N_WHEELS];        /* peso de smoothing (penaliza slew rate respecto al ciclo anterior) */
	float rho;                     /* penalizacion del Augmented Lagrangian sobre la igualdad */
	float recip_w[V3_N_WHEELS];   /* PRECALCULADO: 1.0f / (wt[i] + ws[i]) */
	uint8_t valid;                 /* 1 si v3_qp_config_init tuvo exito, 0 en caso contrario (fail-safe) */
} v3_qp_config_t;

/*
 * Inicializa/valida la configuracion y precalcula las reciprocas.
 * Devuelve 1 si wt[i]+ws[i] > 0 para toda rueda y rho > 0 (config valida),
 * 0 en caso contrario. Si devuelve 0, cfg->valid queda a 0 y v3_qp_solve()
 * se niega a operar sobre esa config (fail-safe: no se puede dividir por 0
 * silenciosamente en un lazo de control de par).
 */
uint8_t v3_qp_config_init(v3_qp_config_t *cfg, const float wt[V3_N_WHEELS],
		const float ws[V3_N_WHEELS], float rho);

/* ------------------------------------------------------------------ */
/* Estado persistente entre ciclos (warm start del multiplicador)       */
/* ------------------------------------------------------------------ */
typedef struct {
	float lambda; /* multiplicador de la restriccion de igualdad, persiste ciclo a ciclo */
} v3_qp_state_t;

/* Resetea el estado del solver. LLAMAR SIEMPRE al entrar en un estado de
 * conduccion nuevo (p.ej. transicion a DRIVING) o al cambiar de modo de
 * conduccion, para no arrastrar un lambda de un contexto no relacionado */
void v3_qp_state_reset(v3_qp_state_t *state);

/* ------------------------------------------------------------------ */
/* Entradas / Salidas del solver                                        */
/* ------------------------------------------------------------------ */
typedef struct {
	float x_nom[V3_N_WHEELS];  /* reparto nominal deseado (Nm) */
	float x_prev[V3_N_WHEELS]; /* par comandado el ciclo anterior (Nm), referencia de slew */
	float lb[V3_N_WHEELS];     /* limite inferior de par por rueda (Nm) */
	float ub[V3_N_WHEELS];     /* limite superior de par por rueda (Nm) */
	float t_demand;             /* par total demandado (Nm), restriccion de igualdad sum(x)=t_demand */
} v3_qp_input_t;

typedef struct {
	float x[V3_N_WHEELS];   /* solucion: par optimo por rueda (Nm), YA proyectado dentro de [lb,ub] */
	float lambda;            /* multiplicador final (para logging/debug, ya limitado a +-V3_QP_LAMBDA_MAX) */
	float residual;          /* |sum(x) - t_demand| final, ~0 si el problema era factible */
	uint8_t feasible;        /* 0 si sum(lb) > t_demand o sum(ub) < t_demand (igualdad infactible por caja) */
} v3_qp_output_t;

/*
 * Resuelve el QP con V3_QP_FIXED_ITERS iteraciones deterministas.
 *
 * Si cfg->valid == 0 (config invalida), o si las entradas de `in` no son
 * finitas y no se pueden sanear a un valor seguro (por ejemplo lb_i > ub_i
 * tras el saneo, lo que indicaria un fallo aguas arriba en el calculo de
 * limites), la funcion aplica el fail-safe MAS conservador: par 0 en ambas
 * ruedas (out->x = {0,0}), out->feasible = 0. Esto es intencionado: ante
 * duda sobre los limites de seguridad, preferimos parar a arriesgar un
 * torque fuera de los limites fisicos calculados.
 */
void v3_qp_solve(const v3_qp_config_t *cfg, v3_qp_state_t *state,
		const v3_qp_input_t *in, v3_qp_output_t *out);

#ifdef __cplusplus
}
#endif

#endif /* INC_V3_QP_SOLVER_H_ */