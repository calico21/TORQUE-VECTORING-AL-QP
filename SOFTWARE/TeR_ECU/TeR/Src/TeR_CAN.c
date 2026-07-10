/*
 * TeR_CAN.c
 *
 *  Created on: Feb 2, 2024
 *      Author: Ozuba
 *      Contributors: Piero, Asempere
 *
 * ████████╗███████╗██████╗          ██████╗ █████╗ ███╗   ██╗
 * ╚══██╔══╝██╔════╝██╔══██╗        ██╔════╝██╔══██╗████╗  ██║
 *    ██║   █████╗  ██████╔╝        ██║     ███████║██╔██╗ ██║
 *    ██║   ██╔══╝  ██╔══██╗        ██║     ██╔══██║██║╚██╗██║
 *    ██║   ███████╗██║  ██║███████╗╚██████╗██║  ██║██║ ╚████║
 *    ╚═╝   ╚══════╝╚═╝  ╚═╝╚══════╝ ╚═════╝╚═╝  ╚═╝╚═╝  ╚═══╝
 */

/*
 *  Este Fichero tiene como Objetivo almacenar las funciones de decodificación
 *  y envío de todos los mensajes de una placa, incluye como librerías aquellas
 *  autogeneradas mediante cantools y ofrece una interfáz de cara al micro con dos
 *  Funciones:
 *  - decodeMSG -> Decodifica las estructuras pertinentes
 *  - sendCAN -> Envía los mensajes pertinentes (Esto no va a depender del estado, ya que los inverters siempre estarán a 0)
 *  - command -> Función que se llama cuando se recibe el mensaje de comando para que cada placa lo interprete como corresponde
 *  A su vez están creados aqui todas las estructuras de memoria del can
 *
 */

/*Implementacion FreeRTOS
 *
 * - El envio de CAN de inverters, main CAN y decodificación son tareas diferentes, con prioridades diferentes, siendo la de decodificación superior a las anteriores.
 * - En los envios se utiliza vTaskDelayUntil (en nuestro caso osDelayUntil), y para la recepción desbloqueo basado en colas.
 *
 *
 * - Se utiliza un semaforo para comprobar el estado de las mailboxes de una manera non blocking, de esta manera evitamos busy waiting antes de enviar el mensaje
 *  (revisar task del scheduler de mensajes)
 *
 * - Se utilizan colas para comunicar la interrupcion de recepcion (y su mensaje) con la decodificación, es la manera mas optima cuando utilizamos un sistema
 * 		operativo en tiempo real
 *
 *
 */

#include "TeR_CAN.h"
volatile uint32_t boot_flag __attribute__((section(".no_init")));
/* ---------------------------[Estructuras del CAN]-------------------------- */
//Pointer to timer and can peripheral being used
CAN_HandleTypeDef *invCAN;
CAN_HandleTypeDef *mainCAN;

//Index for can senders
uint8_t invIndex;
uint8_t mainIndex;

/* -------------------------------------------------------------------------- */
struct TeR_t TeR;

//FreeRTOS Dependencies
extern osMessageQueueId_t rxMsgHandle; //handle de la cola de recepcion
osSemaphoreId_t g_can_tx_mailbox_handle; //counting semaphore of main can MAILBOX
/* ---------------------------[Inicialización + Interrupts]-------------------------- */

uint8_t initCAN(CAN_HandleTypeDef *invCan, CAN_HandleTypeDef *mainCan) {
	//Inicializacion de los perifericos can
	invCAN = invCan;
	mainCAN = mainCan;
	//Arranque del periferico y la interrupcion
	configFilter(invCan, mainCan); //Configura los filtros
	//Registramos los 2 callbacks de recepcion a la función conjunta de decodificación
	HAL_CAN_RegisterCallback(invCAN, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID,
			canRxCallback);
	HAL_CAN_RegisterCallback(mainCAN, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID,
			canRxCallback);

	//Arranque del modulo
	HAL_CAN_Start(invCAN); //Activamos el can
	HAL_CAN_Start(mainCAN); //Activamos el can

	//Arrancamos las interrupts
	HAL_CAN_ActivateNotification(invCAN, CAN_IT_RX_FIFO0_MSG_PENDING); //Activamos notificación de mensaje pendiente a lectura
	HAL_CAN_ActivateNotification(mainCAN,
	CAN_IT_RX_FIFO0_MSG_PENDING); //hay mensaje, mailbox libre, rror + busoff
	return 1;
}

/*----------------------------------[Funcion de Callback FIFO0]--------------------------------*/

void canRxCallback(CAN_HandleTypeDef *hcan) {
	CAN_RxHeaderTypeDef rxHeader; //Header temporal
	canMsg_t msg = { 0 }; //Bufer temporal
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, msg.data); //Recoge el mensaje
	msg.id = (rxHeader.IDE == CAN_ID_STD) ? rxHeader.StdId : rxHeader.ExtId; // si es es standard pillo id standard, sino pillo ext
	msg.DLC = rxHeader.DLC;
	osMessageQueuePut(rxMsgHandle, &msg, 0U, 0U); //ponemos el mensaje en una cola, que será atendido cuando sea posible
if(hcan == invCAN){
	//Bridge inverter output to our can
	CAN_TxHeaderTypeDef TxHeader; //Header de transmisión
	uint32_t mailbox; //Variable para guardar provisionalmente el slot donde se coloca el mensaje
	TxHeader.DLC = rxHeader.DLC;
	TxHeader.StdId = rxHeader.StdId;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	if (HAL_CAN_GetTxMailboxesFreeLevel(mainCAN) > 0) { // si hay slot para envio
		HAL_CAN_AddTxMessage(mainCAN, &TxHeader, msg.data, &mailbox); //Envía el mensaje procesado
	}
}

}
/*----------------------------------[Configuración de filtros]--------------------------------*/

void configFilter(CAN_HandleTypeDef *invCan, CAN_HandleTypeDef *mainCan) {
	CAN_FilterTypeDef filter;
	//Inverter Filter (CAN1 MASter)
	filter.FilterActivation = CAN_FILTER_ENABLE;
	filter.FilterBank = 0; // which filter bank to use from the assigned ones
	filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	filter.FilterIdHigh = 0;
	filter.FilterIdLow = 0;
	filter.FilterMaskIdHigh = 0;
	filter.FilterMaskIdLow = 0;
	filter.FilterMode = CAN_FILTERMODE_IDMASK;
	filter.FilterScale = CAN_FILTERSCALE_32BIT;
	filter.SlaveStartFilterBank = 14; // Los filtros son compartidos
	HAL_CAN_ConfigFilter(invCan, &filter);

	//Main Filter (Slave)
	filter.FilterActivation = CAN_FILTER_ENABLE;
	filter.FilterBank = 15; // which filter bank to use from the assigned ones
	filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	filter.FilterIdHigh = 0;
	filter.FilterIdLow = 0;
	filter.FilterMaskIdHigh = 0;
	filter.FilterMaskIdLow = 0;
	filter.FilterMode = CAN_FILTERMODE_IDMASK;
	filter.FilterScale = CAN_FILTERSCALE_32BIT;
	filter.SlaveStartFilterBank = 14; // Cursor de división de filtros
	HAL_CAN_ConfigFilter(mainCan, &filter);

}

/* ----------------------------------[Envío]---------------------------------------- */

/* ---------------------------[INVERTER CAN]-------------------------- */

void invCanTx(void *argument) {
	//Tarea con ejecucion temoporizada FreeRTOS (echar ojo a reference manual)
	uint32_t currentTick = osKernelGetTickCount(); // sincronizamos nuestra variable de tick con el valor actual del tick del kernel
	//Buffers volatiles para el envío
	uint8_t TxData[8]; //Buffer para datos de envio
	CAN_TxHeaderTypeDef TxHeader; //Header de transmisión
	uint32_t mailbox; //Variable para guardar provisionalmente el slot donde se coloca el mensaje
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	//Van los 3 mensajes de golpe pq justo nos caben en la fifo a la vez y el inverter los requiere
	for (;;) {
		currentTick += 2; //añadimos 2 tick a el valor actual del tick del kernel
		osDelayUntil(currentTick); //cuando el kernel consiga llegar a el valor actual de currentTick, el kernel desbloqueará la tarea
		if (HAL_CAN_GetTxMailboxesFreeLevel(invCAN) > 0) { // Hay un slot para nuestro mensaje
			switch (invIndex++) {

			/* ---------------------------[DERECHO]-------------------------- */

			case 0: //Inverter Derecho
				//SETPOINT_1
				TxHeader.StdId = INVERTER_EMCU_SETPOINT_1_RIGHT_FRAME_ID;
				TxHeader.DLC = INVERTER_EMCU_SETPOINT_1_RIGHT_LENGTH;
				inverter_emcu_setpoint_1_right_pack(TxData, &TeR.appReqRight,
						TxHeader.DLC);
				HAL_CAN_AddTxMessage(invCAN, &TxHeader, TxData, &mailbox); //Envía el mensaje procesado

				//SETPOINT_2
				TxHeader.StdId = INVERTER_EMCU_SETPOINT_2_RIGHT_FRAME_ID;
				TxHeader.DLC = INVERTER_EMCU_SETPOINT_2_RIGHT_LENGTH;
				inverter_emcu_setpoint_2_right_pack(TxData,
						&TeR.currentReqRight, TxHeader.DLC);
				HAL_CAN_AddTxMessage(invCAN, &TxHeader, TxData, &mailbox); //Envía el mensaje procesado

				//SETPOINT_3
				TxHeader.StdId = INVERTER_EMCU_SETPOINT_3_RIGHT_FRAME_ID;
				TxHeader.DLC = INVERTER_EMCU_SETPOINT_3_RIGHT_LENGTH;
				inverter_emcu_setpoint_3_right_pack(TxData, &TeR.trqReqRight,
						TxHeader.DLC);
				HAL_CAN_AddTxMessage(invCAN, &TxHeader, TxData, &mailbox); //Envía el mensaje procesado

				break;

				/* ---------------------------[IZQUIERDO]-------------------------- */

			case 1: //Torque Setpoint L
				//SETPOINT_1
				TxHeader.StdId = INVERTER_EMCU_SETPOINT_1_LEFT_FRAME_ID;
				TxHeader.DLC = INVERTER_EMCU_SETPOINT_1_LEFT_LENGTH;
				inverter_emcu_setpoint_1_left_pack(TxData, &TeR.appReqLeft,
						TxHeader.DLC);
				HAL_CAN_AddTxMessage(invCAN, &TxHeader, TxData, &mailbox); //Envía el mensaje procesado

				//SETPOINT_2
				TxHeader.StdId = INVERTER_EMCU_SETPOINT_2_LEFT_FRAME_ID;
				TxHeader.DLC = INVERTER_EMCU_SETPOINT_2_LEFT_LENGTH;
				inverter_emcu_setpoint_2_left_pack(TxData, &TeR.currentReqLeft,
						TxHeader.DLC);
				HAL_CAN_AddTxMessage(invCAN, &TxHeader, TxData, &mailbox); //Envía el mensaje procesado

				//SETPOINT_3
				TxHeader.StdId = INVERTER_EMCU_SETPOINT_3_LEFT_FRAME_ID;
				TxHeader.DLC = INVERTER_EMCU_SETPOINT_3_LEFT_LENGTH;
				inverter_emcu_setpoint_3_left_pack(TxData, &TeR.trqReqLeft,
						TxHeader.DLC);
				HAL_CAN_AddTxMessage(invCAN, &TxHeader, TxData, &mailbox); //Envía el mensaje procesado

				invIndex = 0; //Evita un ciclo muerto
				break;
				/* ---------------------------[Default]-------------------------- */

			default: //Por si algo wtf pasa
				invIndex = 0; //cualquier otro valor retorna al ultimo mensaje
				break;
			}
		}
	}
}

void CanSchedulerTask(void *argument) {
	uint32_t mailbox;
	CanMessage_t next_msg;
	// Periodic Messages insertion to queue, we use a function that generates a dephase between messages in order to reduce puntual loads
	uint8_t TxData[8] = { 0 };

	can_scheduler_insert_msg_with_phase(TxData, TER_TER_STATUS_LENGTH, // TER STATUS
			TER_TER_STATUS_FRAME_ID, 100, ter_status_callback);

	can_scheduler_insert_msg_with_phase(TxData, TER_WHEEL_INFO_LENGTH, // WHEELINFO
			TER_WHEEL_INFO_FRAME_ID, 10, ter_wheel_info_callback);

	can_scheduler_insert_msg_with_phase(TxData, TER_INVERTER_INFO_LENGTH, // INVERTER INFO
			TER_INVERTER_INFO_FRAME_ID, 10, ter_inverter_info_callback);

	can_scheduler_insert_msg_with_phase(TxData, TER_TV_DEBUG_LENGTH, // TV DEBUG
			TER_TV_DEBUG_FRAME_ID, 5, ter_tv_debug_callback);

//	can_scheduler_insert_msg(TxData, HVBMS_BMS_RX_CTRL_1_LENGTH, // BMS CONTROL
//			HVBMS_BMS_RX_CTRL_1_FRAME_ID, 10, hvbms_bms_rx_ctrl_1_callback);

	can_scheduler_insert_msg_with_phase(TxData, AMS_BMS_REQ_LENGTH,AMS_BMS_REQ_FRAME_ID,10,ams_bms_req_callback); // MIPUTOBMS CONTROL

	can_scheduler_insert_msg_with_phase(TxData, TER_ANG_RATE_LENGTH,
	TER_ANG_RATE_FRAME_ID, 5, ter_ang_rate_callback);

	can_scheduler_insert_msg_with_phase(TxData, TER_ACCEL_LENGTH,
	TER_ACCEL_FRAME_ID, 5, ter_accel_callback);

	can_scheduler_insert_msg_with_phase(TxData, TER_GPS_LAT_LONG_LENGTH,
	TER_GPS_LAT_LONG_FRAME_ID, 5, ter_gps_lat_callback);

	can_scheduler_insert_msg_with_phase(TxData, TER_YPR_LENGTH,
	TER_YPR_FRAME_ID, 5, ter_ypr_callback);

	can_scheduler_insert_msg_with_phase(TxData, TER_VEL_BODY_LENGTH,
	TER_VEL_BODY_FRAME_ID, 5, ter_vel_body_callback);

	can_scheduler_insert_msg_with_phase(TxData, TER_ASB_BRAKE_REQ_LENGTH, TER_ASB_BRAKE_REQ_FRAME_ID, 10, ter_asb_brake_req_callback);
	can_scheduler_insert_msg_with_phase(TxData, TER_DV_SYSTEM_STATUS_LENGTH, TER_DV_SYSTEM_STATUS_FRAME_ID, 100, ter_dv_system_status_callback);
	can_scheduler_insert_msg_with_phase(TxData, TER_DV_CONFIG_LENGTH, TER_DV_CONFIG_FRAME_ID, 100, ter_dv_config_callback);
	//can_scheduler_insert_msg_with_phase(TxData, TER_ASB_EBS_STATE_REQ_LENGTH, TER_ASB_EBS_STATE_REQ_FRAME_ID, 100, ter_ebs_state_req_callback);

	for(;;) {
		CAN_TxHeaderTypeDef TxHeader = { .IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA };
		while (!can_scheduler_get_next(&g_can_scheduler_heap, &next_msg))
			osDelay(10);
		osDelayUntil(next_msg.next_when); // esperamos el tiempo necesario al proximo mensaje
		if (next_msg.callback)
			next_msg.callback(&next_msg); // si el callback es 0 (no se ha definido) no se llama el callback
		TxHeader.StdId = next_msg.id;
		TxHeader.DLC = next_msg.len;
		// puto motorcito de los cojones, no voy a reescribir todo por tu culpa pedazo de cabron
		if(next_msg.id == TER_STEER_ACTUATOR_SET_POSITION_FRAME_ID){
			TxHeader.ExtId = next_msg.id;
			TxHeader.IDE = CAN_ID_EXT;
		}
		if ((HAL_CAN_GetTxMailboxesFreeLevel(mainCAN) > 0) // recuerda, se evalua de izquierda a derecha !!!
				&& (HAL_CAN_AddTxMessage(mainCAN, &TxHeader, next_msg.content,
						&mailbox) == HAL_OK)) {
			// se ha enviado correctamente el mensaje, fino
			if (next_msg.period != -1) { // si el mensaje es periódico, lo reañadimos
				next_msg.next_when += next_msg.period;
				if (!can_scheduler_insert_built_msg(next_msg)) { // añadir mensaje otra vez (porque es periodico) los no periodicos agur
					// handle this somehow (confia)
				}
			}

		} else { // algo ha fallado, reenviamos el mensaje
			next_msg.next_when = osKernelGetTickCount(); // rescheduled to be first
			if (!can_scheduler_insert_built_msg(next_msg)) { // reañadir a la cola
				// handle this somehow (confia)
			}
		}
	}
}

//Función de decodificación del CAN, recive un mensaje de un bus y lo coloca en la estructura global
void canRx(void *argument) {
	canMsg_t msg; // tipo de variable que almacena id, datos y DLC del mensaje recibido en la interrupcion
	for (;;) {
		osMessageQueueGet(rxMsgHandle, &msg, 0U, osWaitForever); // la tarea se desbloquea cuando hay algo en cola
		logSCS(msg.id); //System Critical signal Timestamp, solo cuando podamos ejecutar recepcion
		switch (msg.id) {
		//Attend the command
		case TER_COMMAND_FRAME_ID: //Sistema de comandos
			struct ter_command_t cmdMsg;
			ter_command_init(&cmdMsg); //Por si se usan variables indebidamente inicializadas
			ter_command_unpack(&cmdMsg, msg.data,
			TER_COMMAND_LENGTH);
			command(cmdMsg); //Llama a la interpretación del comando (Se lo pasa por copia)
			break;

			/* ---------------------------[TER]-------------------------- */

			//Mesage Decoding
		case TER_APPS_FRAME_ID:
			ter_apps_unpack(&TeR.apps, msg.data, msg.DLC);
			break;

		case TER_BPPS_FRAME_ID:
			ter_bpps_unpack(&TeR.bpps, msg.data, msg.DLC);
			break;

		case TER_STEER_FRAME_ID:
			ter_steer_unpack(&TeR.steer, msg.data, msg.DLC);
			break;

		case TER_FRONT_V_FRAME_ID:
			ter_front_v_unpack(&TeR.speed, msg.data, msg.DLC);
			break;

		case TER_LV_STATUS_FRAME_ID:
			ter_lv_status_unpack(&TeR.lvbms, msg.data, msg.DLC);
			break;

		case TER_ECU_CONFIG_FRAME_ID:
			ter_ecu_config_unpack(&TeR.config, msg.data, msg.DLC);
			handle_config_entry(&TeR.config);
			break;

		case BOOTER_BOOT_TX_FRAME_ID: // request del bootloader, verifica comando init y request a esta placa de entrar en bootloader
			struct booter_boot_tx_t boot;
			booter_boot_tx_init(&boot);
			booter_boot_tx_unpack(&boot, msg.data, msg.DLC);
			if ((boot.boot_cmd == BOOTER_BOOT_TX_BOOT_CMD_BOOT_INIT_CHOICE)
					&& (boot.node_id == BOOTER_BOOT_TX_NODE_ID_ECU_CHOICE)) {
				boot_flag = 1; // variable ubicada en sección específica en flash
				HAL_NVIC_SystemReset(); // reset del NVIC, el bootloader tomará el control (si lo has flasheado)
			}
			break;

		case TER_ANG_RATE_FRAME_ID:
			ter_ang_rate_unpack(&TeR.angRate, msg.data, msg.DLC);
			break;

		case TER_BTN_FRAME_ID:
			ter_btn_unpack(&TeR.buttons, msg.data, msg.DLC);
			break;

		case TER_STEER_ACTUATOR_STATUS_FRAME_ID:
			ter_steer_actuator_status_unpack(&TeR.steer_actuator_status,
					msg.data, msg.DLC);
			break;

		case TER_RES_PDO_TX_FRAME_ID:
			ter_res_pdo_tx_unpack(&TeR.res_pdo_tx, msg.data, msg.DLC);
			break;

		case TER_DV_DYNAMIC_REQ_1_FRAME_ID:
			ter_dv_dynamic_req_1_unpack(&TeR.dv_dynamic_req_1, msg.data, msg.DLC);
			break;

		case TER_DV_DYNAMIC_REQ_2_FRAME_ID:
			ter_dv_dynamic_req_2_unpack(&TeR.dv_dynamic_req_2, msg.data, msg.DLC);
			break;
		case TER_ASB_STATUS_FRAME_ID:
			ter_asb_status_unpack(&TeR.asb_status, msg.data, msg.DLC);
			break;

		case TER_DV_INFO_FRAME_ID:
			ter_dv_info_unpack(&TeR.dv_info, msg.data, msg.DLC);
			break;



			/* ---------------------------[INVERTER]-------------------------- */

		case INVERTER_EMCU_STATE_2_RIGHT_FRAME_ID:
			inverter_emcu_state_2_right_unpack(&TeR.appStateRight, msg.data,
					msg.DLC);
			break;

		case INVERTER_EMCU_STATE_2_LEFT_FRAME_ID:
			inverter_emcu_state_2_left_unpack(&TeR.appStateLeft, msg.data,
					msg.DLC);
			break;

		case INVERTER_EMCU_STATE_3_RIGHT_FRAME_ID:
			inverter_emcu_state_3_right_unpack(&TeR.dqErpmRight, msg.data,
					msg.DLC);
			break;

		case INVERTER_EMCU_STATE_3_LEFT_FRAME_ID:
			inverter_emcu_state_3_left_unpack(&TeR.dqErpmLeft, msg.data,
					msg.DLC);
			break;

		case INVERTER_EMCU_STATE_4_RIGHT_FRAME_ID:
			inverter_emcu_state_4_right_unpack(&TeR.tempsRight, msg.data,
					msg.DLC);
			break;

		case INVERTER_EMCU_STATE_4_LEFT_FRAME_ID:
			inverter_emcu_state_4_left_unpack(&TeR.tempsLeft, msg.data,
					msg.DLC);
			break;

		case INVERTER_EMCU_STATE_7_LEFT_FRAME_ID:
			inverter_emcu_state_7_left_unpack(&TeR.demLeft, msg.data, msg.DLC);
			break;

		case INVERTER_EMCU_STATE_7_RIGHT_FRAME_ID:
			inverter_emcu_state_7_right_unpack(&TeR.demRight, msg.data,
					msg.DLC);
			break;

		case INVERTER_EMCU_STATE_9_LEFT_FRAME_ID:
			inverter_emcu_state_9_left_unpack(&TeR.trqEstLeft, msg.data,
					msg.DLC);
			break;

		case INVERTER_EMCU_STATE_9_RIGHT_FRAME_ID:
			inverter_emcu_state_9_right_unpack(&TeR.trqEstRight, msg.data,
					msg.DLC);
			break;

			/* ---------------------------[HVBMS]-------------------------- */

		case HVBMS_BMS_TX_STATE_3_FRAME_ID:
			hvbms_bms_tx_state_3_unpack(&TeR.BmsAppState, msg.data, msg.DLC);
			break;

		case HVBMS_BMS_TX_STATE_6_FRAME_ID:
			hvbms_bms_tx_state_6_unpack(&TeR.BmsCellsVolt, msg.data, msg.DLC);
			break;

		case HVBMS_BMS_TX_STATE_9_FRAME_ID:
			hvbms_bms_tx_state_9_unpack(&TeR.BmsCellsTemp, msg.data, msg.DLC);
			break;

		case HVBMS_BMS_TX_STATE_4_FRAME_ID:
			hvbms_bms_tx_state_4_unpack(&TeR.BmsCurrent, msg.data,
					sizeof(msg.data)); // si el dbc estuviera bien hecho, esto no tendriamos que hacer
			break;

		case HVBMS_BMS_TX_STATE_5_FRAME_ID:
			hvbms_bms_tx_state_5_unpack(&TeR.BmsBatVolt,msg.data,sizeof(msg.data));
			break;
			/* ---------------------------[MIPUTOBMS]-------------------------- */
		case AMS_BMS_STATUS_FRAME_ID:
			ams_bms_status_unpack(&TeR.bms_status, msg.data, sizeof(msg.data));
			break;
		case AMS_CELL_TEMPERATURES_STATUS_FRAME_ID:
			ams_cell_temperatures_status_unpack(&TeR.bms_temperatures_status, msg.data, sizeof(msg.data));
			break;
		case AMS_CELL_VOLTAGE_STATUS_FRAME_ID:
			ams_cell_voltage_status_unpack(&TeR.bms_voltage_status, msg.data, sizeof(msg.data));
			break;
		case AMS_HV_MEASUREMENTS_STATUS_FRAME_ID:
			ams_hv_measurements_status_unpack(&TeR.bms_hv_measurements_status, msg.data, sizeof(msg.data));
			break;

		default:
			break;

		}
	}
}
