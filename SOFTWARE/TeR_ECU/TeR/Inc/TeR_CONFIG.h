/*
 * TeR_CONFIG.h
 *
 *  Created on: Apr 12, 2025
 *      Author: piero
 */

#ifndef INC_TER_CONFIG_H_
#define INC_TER_CONFIG_H_
#include "TeR_CAN.h"
#include "ee24.h"
typedef struct{
	struct ter_ecu_config_t config;
	uint8_t written;
}eeprom_data_t;
#define NB_ENTRIES 27 //numero de entradas de configuración que hay definidas, CAMBIALO CUANDO AÑADAS UN ENTRY
#define ALL_CONFIGS 0xFFFFFFFF

;
uint8_t send_config(uint32_t frame_id, void *config);
uint8_t init_config();
uint8_t write_config(struct ter_ecu_config_t *config);
void handle_config_entry(struct ter_ecu_config_t *config);
void set_default_config(struct ter_ecu_config_t* config);
void set_init_config(struct ter_ecu_config_t *config);
uint8_t publish_config(struct ter_ecu_config_t *config,uint32_t config_id);
#endif /* INC_TER_CONFIG_H_ */
