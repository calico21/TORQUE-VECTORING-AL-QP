/*
 * TeR_CONFIG.c
 *
 *  Created on: Apr 12, 2025
 *      Author: piero
 *      Contributors:
 *
 *      Sistema de configs.
 *
 *      sendConfig(), envia configuracion para un nodo específico por CAN
 *      Sortea por frame ids de configs y lo envia.
 *      tienes que castear el puntero generico a la config que quieras, de momento solo refri
 *
 *      initConfig(), lee la eeprom, verifica estado y actualiza vector de configuraciones, si algo va mal se setea una config default, se llama cada vez que se inicializa la maquina de estados
 *      writeConfig(), escribe TeR.config en la eeprom, se llama cada vez que se recibe el mensaje de TeR Config
 *
 *
 *
 */
#include "TeR_CONFIG.h"
extern I2C_HandleTypeDef hi2c2; // Que lo pille de donde sea
EE24_HandleTypeDef eeprom; // handle de la eeprom
eeprom_data_t data; // estructura de datos de la eeprom
uint8_t config_ready; // indicate that config has been loaded

/*
 * Enviar un struct de configuración particular de una placa externa
 * IMPORTANTE: es tu responsabilidad saber lo que estás haciendo y asegurarte que en el uso de la función,
 * el tipo que deseas usar esta considerado como caso, y que el frame_id esté relacionado con ese tipo,
 * sino, vas a hacer una corrupción de memoria buena. Nunca me ha pasado pero tienes que estar pendiente de lo que estás haciendo
 */
uint8_t send_config(uint32_t frame_id, void *config) {
	//Buffers volatiles para el envío
	uint8_t TxData[8] = { 0 }; //Buffer para datos de envio
	switch (frame_id) {
	case TER_REFRI_CONFIG_FRAME_ID: //configurar refri
		struct ter_refri_config_t refri_config =
				*(struct ter_refri_config_t*) config; // interpretamos el puntero como un struct de config de refri
		ter_refri_config_pack(TxData, &refri_config, sizeof(TxData));
		break;
	default:
		return 1;
	}
	while (!can_scheduler_insert_non_periodic_msg(TxData, sizeof(TxData),
			frame_id, 0)) {
		osDelay(5);
	}
	return 0;
}
/*
 * Arrancar la EEPROM y copiar los datos al struct del vehículo
 * En caso de fallo en la lectura, se configurarán los valores por defecto ellamando a set_default_config
 * */
uint8_t init_config() {
	EE24_Init(&eeprom, &hi2c2, EE24_ADDRESS_DEFAULT);
	EE24_Read(&eeprom, 0, (uint8_t*) &data, sizeof(data), 500); // load config struct
	if (data.written == 1) { // Si la eeprom ha sido leida
		TeR.config = data.config; // copiamos datos de eeprom al vehículo
		set_init_config(&TeR.config); // ponemos valores a su valor de init por defecto (por si la lias)
		config_ready = 1;
		return 1;
	} // if eeprom was not written or anything when bad (data.written is defaulted 0), default config should be loaded
	set_default_config(&TeR.config);
	set_init_config(&TeR.config);
	write_config(&TeR.config);
	config_ready = 1;
	return 0;
}
/*
 *	Escribir en la EEPROM un struct de config
 */
uint8_t write_config(struct ter_ecu_config_t *config) {
	//memcpy(&TeR.config,config,sizeof(struct ter_ecu_config_t));
	TeR.config = *config; // copiamos config a la RAM
	data.config = TeR.config; // copiamos config a la eeprom
	data.written = 1; // establecemos byte para indicar escritura
	return EE24_Write(&eeprom, 0, (uint8_t*) &data, sizeof(data), 500); //ojo, podría llegar a bloquear, como máximo durante 500ms
}

/*
 * Setear un struct de config a valores por defecto, EN CASO DE QUE LA EEPROM NO SE HAYA CARGADOOJO, debes de configurar tu manualmente aqui cuales son los valores que se
 * considerarán por defecto.
 * */
void set_default_config(struct ter_ecu_config_t *config) { //set car internal config struct to default
	memset(config, 0, sizeof(struct ter_ecu_config_t)); // seteamos el struct a 0 por seguridad (intentar siempre que safestate sea 0 en lo que sea que añadas, asi si te olvidas no es un desastre)
	config->entry = TER_ECU_CONFIG_ENTRY_SCS_ENABLE_CHOICE; // para evitar bucle de reset de eeprom, seteamos entry a un valor por defecto (por definir en .dbc)
	config->driving_mode =
	TER_ECU_CONFIG_DRIVING_MODE_LINEAL_CHOICE;
	config->limiter = TER_ECU_CONFIG_LIMITER_TORQUE_CHOICE;
	config->r2_d_brake = 5;
	config->scs_enable = TER_ECU_CONFIG_SCS_ENABLE_ENABLE_CHOICE;
	config->traction_control =
	TER_ECU_CONFIG_TRACTION_CONTROL_OFF_CHOICE;
	config->trq_kp = 0;
	config->trq_ki = 0;
	config->trq_kd = 0;
	config->trq_limit = 100;
	config->flap_enable = TER_ECU_CONFIG_FLAP_ENABLE_OFF_CHOICE;
	config->flap_l_offset = -8;
	config->flap_r_offset = 65;
	config->flap_l_reverse = TER_ECU_CONFIG_FLAP_L_REVERSE_NORMAL_CHOICE;
	config->flap_r_reverse = TER_ECU_CONFIG_FLAP_R_REVERSE_REVERSE_CHOICE;
	config->flap_pedal_setpoint = 90;
	config->regen_enable = TER_ECU_CONFIG_REGEN_ENABLE_DISABLE_CHOICE; //defaulted off
	config->regen_max_cell_temp = 45;
	config->regen_max_cell_volt = 3900;
	config->regen_max_trq = 10;
	config->regen_thr_speed = 10;
	config->regen_max_current = 40;
	config->regen_thr_rpm = 5;
	config->regen_trq_slope = 1;
	config->regen_mode = TER_ECU_CONFIG_REGEN_MODE_APPS_CHOICE;
	config->regen_max_positive_trq_thr = 5;
	config->dv_mission_req = TER_ECU_CONFIG_DV_MISSION_REQ_MANUAL_CHOICE;
	return;
}

/* Setear valores de config de INIT del vehículo
 * overridea lo que este escrito en la eeprom
 *
 * Existe por si dejas configurado algo por defecto que no puede ser válido
 *
 * */
void set_init_config(struct ter_ecu_config_t *config){ // TODO dv mission req
//	config->driving_mode =
//	TER_ECU_CONFIG_DRIVING_MODE_LINEAL_CHOICE;
	config->regen_mode = TER_ECU_CONFIG_REGEN_MODE_APPS_CHOICE;
}

/*
 * Esta funcion permite dumpear un struct de config por CAN
 * Permite seleccionar cual config deseas enviar en particular, o publicar todas las configuraciones cargadas
 * del sistema, con el define ALL_CONFIGS
 * */
uint8_t publish_config(struct ter_ecu_config_t *config, uint32_t config_id) {
	if (config_ready != 1) { // si no se ha cargado la eeprom, no publicamos la configuracion (esto es una tremenda estupidez pero porsiaka)
		return 1;
	}
//Buffers volatiles para el envío
	uint8_t TxData[8] = { 0 }; //Buffer para datos de envio
	uint32_t size = TER_ECU_CONFIG_LENGTH;
	uint32_t id = TER_ECU_CONFIG_FRAME_ID;
	struct ter_ecu_config_t ecu_config = *(struct ter_ecu_config_t*) config; // castear el puntero void al tipo que yo espero que sea (OJO, es tu responsabilidad handelear esto bien)
	if ((config_id != ALL_CONFIGS) && (config_id <= NB_ENTRIES)) { // if user is requesting a specific config, and the config is valid, send specific config
		ecu_config.entry = config_id;
		ter_ecu_config_pack(TxData, &ecu_config, size);
		while (!can_scheduler_insert_non_periodic_msg(TxData, size, id, 0)) {
			osDelay(5);
		}

	} else { // if user requested all configs or requested config is not valid, send all configs
		for (uint8_t i = 0; i < NB_ENTRIES; i++) {
			if (i == TER_ECU_CONFIG_ENTRY_EEPROM_CHOICE) {
				continue;
			}
			ecu_config.entry = i;
			ter_ecu_config_pack(TxData, &ecu_config, size);
			while (!can_scheduler_insert_non_periodic_msg(TxData, size, id, 0)) {
				osDelay(5);
			}
		}
	}
	return 0;
}

/*
 * Esta función toma un puntero a un struct de configuración de la ecu y analiza el request
 * Tomará las acciones pertinentes según sea el request pedido
 *
 *   En el caso en el que sea un request de EEPROM:
 * 	- se analizará los 3 tipos de request de eeprom y se realizaran las acciones pertinentes
 * 	- si el campo de mensaje no es ninguno de los casos, significa que se está haciendo un request específico de lectura de datos guardados en RAM
 *
 * 	Ejemplo:
 *
 * 	recibimos TER_ECU_CONFIG_ENTRY_EEPROM_CHOICE, con TER_ECU_CONFIG_EEPROM_SAVE_ALL_CHOICE
 * 	-> La eeprom escribirá la config en la ram y en la eeprom
 *
 * 	recibimos TER_ECU_CONFIG_ENTRY_EEPROM_CHOICE pero ninguno de los casos de config-> eeprom coinciden:
 * 	-> asumimos que el transmisor nos está haciendo un request de lo que sea que valga config->eeprom, por ejemplo TER_ECU_CONFIG_ENTRY_LIMITER_CHOICE
 * 	siempre, enviaremos en config-> eeprom el valor de entry que queremos consultar
 *   -> enviaremos la configuración correspondiente a ese registro para que el que la haya pedido la pueda ver
 *   La idea es permitir a nodos externos, como por ejemplo la pantalla, consultar estados de configuración del vehículo
 *
 *   recibimos configuración de misión:
 *   -> si el request es manual actualizamos estado interno
 *   -> si el request es algo dv hacemos request al dv de mision
 *
 * */

void handle_config_entry(struct ter_ecu_config_t *config) {
	if (config->entry == TER_ECU_CONFIG_ENTRY_EEPROM_CHOICE) {
		switch (config->eeprom) {
		case TER_ECU_CONFIG_EEPROM_CLEAR_AND_DEFAULT_ALL_CHOICE:
			set_default_config(config);
			write_config(config);
			break;

		case TER_ECU_CONFIG_EEPROM_READ_ALL_CHOICE:
			publish_config(config, ALL_CONFIGS);
			break;

		case TER_ECU_CONFIG_EEPROM_SAVE_ALL_CHOICE:
			write_config(config);
			break;

		default: // si no es ninguno de los anteriores, signifíca que se nos está intentando hacer un request de lectura de una config específica
			publish_config(config, config->eeprom);
			break;
		}
	}
}


