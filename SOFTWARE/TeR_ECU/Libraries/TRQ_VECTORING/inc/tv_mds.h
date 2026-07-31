/*
 * tv_mds.h
 *
 *  Created on: Jan 31, 2024
 *      Author: Piero,Ozuba Telmo Martinez de Salinas
 */

#ifndef INC_TV_MDS_H_
#define INC_TV_MDS_H_

#include "TeR_TRQMANAGER.h"
#include "TeR_INERTIAL.h"
#include "TeR_CAN.h" //For controlling TeR vehicle
#include <math.h>
//#include "pid.h"
#define SAFETY
#define DEG2RAD (PI/180.0f) //Degs to radians, IMPORTANTE PARENTESIS
#define KMH2MS (1.0f/3.6f)
/////////////////////////////////////////[Constantes del Vehiculo]/////////////////////////////////////////////////////////////

#define I_ZZ 122.0 //Momento de inercia en eje Z (kg*M^4) (Modelo Juan Gastaminza)
#define T_WIDTH 1.185 //Track width REAR (m)
#define L_FRONT 0.806 //A Distance(from front axle to CDG) (m)
#define L_REAR 0.744 //B Distance (from CDG to rear axle)  (m)
#define H_CDG 0.27   //height of the CDG (en estatico)  (m)
#define GEAR_R 5.0     //Indice de Reducccion
#define R_WHEEL 0.2023 //Radio de la rueda (m)
#define K_U 0.0 //Gradiente de subviraje objetivo (rad)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////[Seguridad]/////////////////////////////////////////////////////////////
#define ACTSPEED 20.0 // speed in kmh for threshold
#define ACTAPPS 5.0 // 2 apps (0-255) uint8_t
#define STEER_DEADZONE (8.0*DEG2RAD)
#define IMU_DEADZONE (2.0*DEG2RAD)
#define KPMAX (400000.0f/10000.0f)
#define KIMAX (10.0 /10000.0f)
#define KDMAX (40000.0f/10000.0f)
#define KMIN 0.0f
#define IMAX 1000.0f
#define MAX_DELTA_TORQUE 40.0f
//definimos 8 tramos
static const float v_bp_ms[] = { 0.0f, 5.55f, 8.33f, 11.11f, 13.88f, 16.66f,
		19.44f, 22.22f };    // breakpoints en m/s  de 20 a 80 km/h
static const float kp_tab[] = { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f }; // Kp(v)
static const float ki_tab[] = { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f }; // Ki(v)
static const float kd_tab[] = { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f }; // Kd(v)

// tabla antiwindup
static const float iMax_tab[] =
		{ 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };





float yawRef(float steer, float vx); //Funcion que toma angulo de rueda y velocidad de avance y devuelve referencia de giro yawrate
float mz2DeltaTorque(float alpha);
trqMap_t trqVectoring(trq_t limit);
uint8_t tv_deInitPID(void); // se llama desde otro lado para dealocar el puntero del pid
uint8_t isAngleInDeadzone(float angle, float range);
#endif /* INC_TV_MDS_H_ */
