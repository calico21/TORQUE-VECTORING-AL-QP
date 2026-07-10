/*
 * torqueManager.h
 *
 *  Created on: Mar 30, 2024
 *      Author: Ozuba
 */

/*
                                             STEER
                                             ────────────────────┐
                                             APPS                │
             KWLimit                         ───────────────┐    │
             ──────────────┐                 IMU            │    │                                                      ┌────────┐
                           │                 ──────────┐    │    │                                              ┌──────►│RIGHT   │
                           ▼                           ▼    ▼    ▼                                              │       │INVERTER│
                  ┌──────────────────┐            ┌──────────────────┐ TorqueR   ┌───────────────────┐ TorqueR  │       └────────┘
                  │                  │            │                  ├──────────►│                   ├──────────┘
Speed(Rpm)        │     Limitador    │  Torque    │     Modos de     │           │      Control      │
─────────────────►│        de        ├───────────►│                  │           │        de         │
                  │     Potencia     │ Available  │    Conducción    │ TorqueL   │     Traccion      │ TorqueL
                  │                  │            │                  ├──────────►│                   ├──────────┐
                  └──────────────────┘            └──────────────────┘           └───────────────────┘          │
                                                                                                                │       ┌────────┐
                                                                                                                └──────►│LEFT    │
                                                                                                                        │INVERTER│
                                                                                                                        └────────┘

 The torque manager is the library in charge of managing the dynamic control of the vehicle, it consists on 3 differenciated stages
 which are modular and interchangable with the idea of creating different driving experiences according to the competition. The principal
 interchangeable block is the driving mode


 - Limitador de Potencia: Se establece una limitación de potencia en kw y se calcula un torque máximo desarrollable suponiendo que la potencia
 	 	 	 	 	 se conserva a lo largo del powertrain Pelectrica = Pmecanica*FactorEficiencia

 - Modo de conducción:
 	 * Lineal: El torque se distribuye equitativamente a las 2 ruedas
 	 * BasicTorque: Distrubución del torque basada en una función del steering (Normalmente un polinomio)
 	 * ControlTorque: Distribución del Torque basada en un scheduled gain PID calibrado mediante un modelo bicicleta del vehículo
 	 * Modo Marcha atras: Self-Explainatory (Pitará en modo obra jajaj) Ilegalisimo en competi
	 * Autonomo(Futuro): Permite el control del TeR mediante la librería de comandos TeR_COMMAND
 - Control de tracción:
 	 * Feedforward
 	 * Feedback etc


Para permitir la modularidad se va a utilizar un ciclo de procesado basado en function pointers lo que permite hacer prototipos básicos sobre este esquema y cambiarlos facilmente
- Limitador de potencia void -> trq_t (La estimación es interna al modulo no entra como argumento(Puede hacer estimaciones basadas en el consumo electrico))
- Modo de conducción torque_t -> trqMap_t
- Control de Tracción torqueMap_t -> trqMap_t
 */

#ifndef TORQUE_MANAGER_H
#define TORQUE_MANAGER_H

#include <stdint.h>
#include "TeR_CAN.h" //For controlling TeR vehicle
#include "pid.h" //For torque vectoring

//Dynamic value types
typedef int32_t trq_t; //Mucho ojo va a tener signo por ahora regen/marcha atrás, tiene sentido (Se implementarán sanity checks)
typedef struct { //Si quieres hacer un 4wd añade 2 miembros más y a correr
	trq_t rLeft; //Rear left wheel
	trq_t rRight; //Rear right wheel
} trqMap_t;

//ManagerConfigs
typedef struct { // Contiene configuraciones del pipeline
	trq_t (*limiter)(void); //Toma un valor de limitación de potencia en kw y devuelve el torque desarrollable (trqLimit)
	trqMap_t (*drivingMode)(trq_t in); //Toma un torque limite y lo distribuye según decida el modo en las ruedas
	trqMap_t (*regenMode)(trqMap_t in); //toma la distribución de torque y decide si aplicar/aceptar regeneración
	trqMap_t (*tractionControl)(trqMap_t in); //valida que podamos dar el grip necesario
	trqMap_t (*sanityChecks)(trqMap_t in,trq_t limit); //valida que no estemos haciendo ninguna estupidez
} trqPipeline_t;

extern trqPipeline_t DriveConfig; //Expone al resto de modulos la configuración del pipeline (Solo se puede cambiar fuera de driving mediante el sistema de comandos)



//Main functions
void trqManager(void* argument); //Executes all the torque pipeline
uint8_t loadParams(trqPipeline_t* config);

//Basic limiters
trq_t limitTorque(void);

//Basic modes
trqMap_t lineal(trq_t limit);
trqMap_t DVTrqRequest(trq_t limit);

//Basic traction Control
trqMap_t tractionControlOFF(trqMap_t in);

//Basic regen
trqMap_t regenModeAPPS(trqMap_t in);
//Free regen (with limits)
trqMap_t regenModeFREE(trqMap_t in);


//sanity Checks
trqMap_t trqCheck(trqMap_t in,trq_t limit);
void clamp_pos_trq(trqMap_t *in,trq_t limitPos);
void clamp_neg_trq(trqMap_t *in,trq_t limitNeg);
void scale_max_trq(trqMap_t *in,trq_t limit);

uint8_t regen_allowed();

#endif
