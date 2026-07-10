/*
 * gp_vehicle_model.h
 * Parámetros geométricos y estimadores físicos 
 */

#ifndef GP_VEHICLE_MODEL_H
#define GP_VEHICLE_MODEL_H

#include "gp_math.h"

// ── Geometría del Vehículo (Ter27) ──────────────────────────
#define GP_MASS      300.0f   // [kg] Masa total
#define GP_IZ        150.0f   // [kg·m2] Inercia en Z
#define GP_LF        0.8525f  // [m] CG a eje delantero
#define GP_LR        0.6975f  // [m] CG a eje trasero
#define GP_TRACK_F   1.200f   // [m] Ancho de vía delantero
#define GP_TRACK_R   1.180f   // [m] Ancho de vía trasero
#define GP_R_WHEEL   0.2032f  // [m] Radio de rueda
#define GP_H_CG      0.330f   // [m] Altura del CG
#define GP_WB        (GP_LF + GP_LR)

// ── Parámetros del Modelo de Neumático y Aero ─────────
#define GP_C_ALPHA_F 35000.0f
#define GP_C_ALPHA_R 32000.0f
#define GP_MU_NOM    1.5f     
#define GP_P_MAX_WHL 20000.0f 
#define GP_AERO_CL_REAR 1.50f  // Coeficiente de Lift (Downforce) trasero
#define GP_AERO_AREA    1.10f  // Área frontal de referencia [m2]
#define GP_AIR_DENSITY  1.225f // Densidad del aire [kg/m3]

// ── Referencia Subviradora ──────────────────────────────────
#define GP_K_US      0.006f   
#define GP_K_US_FZ   0.0015f  

#define GP_FL 0
#define GP_FR 1
#define GP_RL 2
#define GP_RR 3

// ── Prototipos de funciones ─────────────────────────────────
void gp_estimate_fz(float vx, float ax, float ay, float fz_out[4]);
void gp_estimate_fy(float vx, float vy, float wz, float delta, const float fz[4], float fy_out[4]);
void gp_friction_ellipse_t_ub(const float fz[4], const float fy_est[4], float mu_est, float t_ub_out[4]);
void gp_power_limited_t_ub(const float omega_wheel[4], float t_ub_out[4]);
float gp_adaptive_k_us(const float fz[4]);
void gp_moment_arms(float arms_out[4]);

#endif // GP_VEHICLE_MODEL_H