/*
 * TeR_COMMAND.h
 *
 *  Created on: Apr 19, 2024
 *      Author: ozuba
 */
//Modulo que gestiona los comandos del vehiculo, contiene la función command
#include "TeR_CAN.h" //Necesario para la interacción expone los can en uso
#include "TeR_STATEMACHINE.h" //Para los estados
#include "TeR_SL.h"



uint8_t command(struct ter_command_t command); //Función comando
uint8_t easyCommand(uint8_t cmd); //Wrapper para el uso rápido de comandos sin argumentos (Usan solo el campo cmd)
uint8_t switchCommand(uint8_t cmd,uint8_t onOff); //Wrapper para el uso rápido de comandos onOff
