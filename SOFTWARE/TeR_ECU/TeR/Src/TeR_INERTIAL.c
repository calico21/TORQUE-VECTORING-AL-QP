/*
 * TeR_IMU.c
 *
 *  Created on: Jan 9, 2025
 *      Author: eracing
 */
#include <TeR_INERTIAL.h>
#include "TeR_CAN.h"

//Sensor Interface Wrappers
static int32_t imu_write(void *handle, uint8_t reg, const uint8_t *bufp,
		uint16_t len);
static int32_t imu_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static int32_t mag_write(void *handle, uint8_t reg, const uint8_t *bufp,
		uint16_t len);
static int32_t mag_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);

//FreeRtos task
static const int task_period = 10;
//Global IMU
imu_t IMU;

//MARG devices
stmdev_ctx_t imu;
stmdev_ctx_t mag;

static uint8_t whoamI, rst; //Aux variables for operation

/* IMU variables ---------------------------------------------------------*/
static int16_t acc_raw[3];
static int16_t gy_raw[3];
static float_t acc_xyz[3]; //In m/s
static float_t a_rate_rpy[3]; //In deg/s
float a_rate_offset[3];

/* MAG variables ---------------------------------------------------------*/
static int16_t mag_raw[3];
static float_t mag_xyz[3];

/* Combined attitude ---------------------------------------------------------*/

void inertial(void *argument) {
	uint32_t lastTick = osKernelGetTickCount(); // Initialize reference time
	configIMU();
	configMAG();

	for (;;) {
		lastTick += task_period;
		osDelayUntil(lastTick);

		uint8_t reg;
		/* Read output only if new xl value is available */
		asm330lhh_xl_flag_data_ready_get(&imu, &reg);

		if (reg) {
			/* Read acceleration field data */
			memset(acc_raw, 0x00, 3 * sizeof(int16_t));
			asm330lhh_acceleration_raw_get(&imu, acc_raw);
			acc_xyz[0] = asm330lhh_from_fs2g_to_mg(acc_raw[0]) / 1000.0;
			acc_xyz[1] = asm330lhh_from_fs2g_to_mg(acc_raw[1]) / 1000.0;
			acc_xyz[2] = asm330lhh_from_fs2g_to_mg(acc_raw[2] / 1000.0);

		}

		asm330lhh_gy_flag_data_ready_get(&imu, &reg);

		if (reg) {
			//logSCS(0xDEADBEEF); // log scs imu (influences on torque, dangerous)
			/* Read angular rate field data */
			memset(gy_raw, 0x00, 3 * sizeof(int16_t));
			asm330lhh_angular_rate_raw_get(&imu, gy_raw);
			a_rate_rpy[0] = asm330lhh_from_fs500dps_to_mdps(gy_raw[0]) / 1000.0
					- a_rate_offset[0];
			a_rate_rpy[1] = asm330lhh_from_fs500dps_to_mdps(gy_raw[1]) / 1000.0
					- a_rate_offset[1];
			a_rate_rpy[2] = asm330lhh_from_fs500dps_to_mdps(gy_raw[2]) / 1000.0
					- a_rate_offset[2];
		}

		//-----------------------------------------------------------------------------------------------------//
		/* Read magnetic field data */
		lis3mdl_mag_data_ready_get(&mag, &reg);

		if (reg) {
			memset(mag_raw, 0x00, 3 * sizeof(int16_t));
			lis3mdl_magnetic_raw_get(&mag, mag_raw);
			mag_xyz[0] = 1000 * lis3mdl_from_fs16_to_gauss(mag_raw[0]);
			mag_xyz[1] = 1000 * lis3mdl_from_fs16_to_gauss(mag_raw[1]);
			mag_xyz[2] = 1000 * lis3mdl_from_fs16_to_gauss(mag_raw[2]);
		}
		//Fill up the IMU struct
		IMU.a_x = acc_xyz[0];
		IMU.a_y = acc_xyz[1];
		IMU.a_z = acc_xyz[2];
		// Angular Rates
		IMU.w_x = a_rate_rpy[0];
		IMU.w_y = a_rate_rpy[1];
		IMU.w_z = a_rate_rpy[2];

		//Process attitude in euler angles
		compFilter(IMU.w_x, IMU.w_y, IMU.w_z, IMU.a_x, IMU.a_y, IMU.a_z,
				mag_xyz[0], mag_xyz[1], mag_xyz[2], &IMU.roll, &IMU.pitch,
				&IMU.yaw);

		//For viewer usage
		//printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", IMU.roll,  IMU.pitch,
		//		 IMU.yaw, a_rate_rpy[0], a_rate_rpy[1], a_rate_rpy[2], acc_xyz[0],
		//		acc_xyz[1], acc_xyz[2]);
		//Dump to canbus
		//YPR

		TeR.ypr.yaw = ter_ypr_yaw_encode(IMU.yaw);
		TeR.ypr.pitch = ter_ypr_pitch_encode(IMU.pitch);
		TeR.ypr.roll = ter_ypr_roll_encode(IMU.roll);
		//Accelerations
		TeR.accel.a_x = ter_accel_a_x_encode(IMU.a_x);
		TeR.accel.a_y = ter_accel_a_x_encode(IMU.a_y);
		TeR.accel.a_z = ter_accel_a_x_encode(IMU.a_z);

		//Angular Rate
		TeR.angRate.yaw_rate_z = ter_ang_rate_yaw_rate_z_encode(IMU.w_z);
		TeR.angRate.pitch_rate_y = ter_ang_rate_pitch_rate_y_encode(IMU.w_y);
		TeR.angRate.roll_rate_x = ter_ang_rate_roll_rate_x_encode(IMU.w_x);

	}
}

void configIMU(void) {

	imu.write_reg = &imu_write;
	imu.read_reg = &imu_read;
	imu.handle = &hi2c1;
	/* Wait sensor boot time */

	osDelay(50);
	/* Restore default configuration */
	//Reset device
	asm330lhh_reset_set(&imu, PROPERTY_ENABLE);
	do {
		asm330lhh_reset_get(&imu, &rst);
		osDelay(100);
	} while (rst);

	asm330lhh_device_id_get(&imu, &whoamI);
	while (whoamI != ASM330LHH_ID) {
		asm330lhh_device_id_get(&imu, &whoamI);
		osDelay(100);
	}

	//Turn on light to indicate IMU is running
	HAL_GPIO_WritePin(IMU_LED_GPIO_Port, IMU_LED_Pin, 1);

	asm330lhh_device_conf_set(&imu, PROPERTY_ENABLE);
	/* Enable Block Data Update */
	asm330lhh_block_data_update_set(&imu, PROPERTY_ENABLE);
	asm330lhh_auto_increment_set(&imu, 1); //nuevo
	/* Set Output Data Rate*/
	asm330lhh_xl_data_rate_set(&imu, ASM330LHH_XL_ODR_104Hz);
	asm330lhh_gy_data_rate_set(&imu, ASM330LHH_GY_ODR_104Hz);
	/* Set full scale */
	asm330lhh_xl_full_scale_set(&imu, ASM330LHH_2g);
	asm330lhh_gy_full_scale_set(&imu, ASM330LHH_500dps);
	/* Configure filtering chain(No aux interface)
	 * Accelerometer - LPF1 + LPF2 path
	 */
	asm330lhh_xl_hp_path_on_out_set(&imu, ASM330LHH_LP_ODR_DIV_100);
	asm330lhh_xl_filter_lp2_set(&imu, PROPERTY_ENABLE);
	// Filtros Giroscopio
	asm330lhh_gy_filter_lp1_set(&imu, PROPERTY_ENABLE);  // LPF1 ON
	asm330lhh_gy_lp1_bandwidth_set(&imu, ASM330LHH_AGGRESSIVE);
	asm330lhh_gy_hp_path_internal_set(&imu, ASM330LHH_HP_FILTER_NONE);

	// Evitar lecturas mientras asienta el filtro
	asm330lhh_filter_settling_mask_set(&imu, 1);
	osDelay(200);

	const uint32_t N = 256;
	uint8_t reg = 0;
	for (uint32_t i = 0; i < N; i++) {
		do {
			asm330lhh_gy_flag_data_ready_get(&imu, &reg);
			osDelay(2);
		} while (!reg);
		/* Read angular rate field data */
		memset(gy_raw, 0x00, 3 * sizeof(int16_t));
		asm330lhh_angular_rate_raw_get(&imu, gy_raw);
		a_rate_offset[0] += asm330lhh_from_fs500dps_to_mdps(gy_raw[0]) / 1000.0;
		a_rate_offset[1] += asm330lhh_from_fs500dps_to_mdps(gy_raw[1]) / 1000.0;
		a_rate_offset[2] += asm330lhh_from_fs500dps_to_mdps(gy_raw[2]) / 1000.0;

	}
	a_rate_offset[0] /= (float)N;
	a_rate_offset[1] /= (float)N;
	a_rate_offset[2] /= (float)N;
}

void configMAG() {
	/* Initialize mems driver interface */
	mag.write_reg = &mag_write;
	mag.read_reg = &mag_read;
	mag.handle = &hi2c1;
	/* Check device ID */
	lis3mdl_device_id_get(&mag, &whoamI);

	if (whoamI != LIS3MDL_ID)
		while (1)
			osDelay(100); /*manage here device not found */

	/* Restore default configuration */
	lis3mdl_reset_set(&mag, PROPERTY_ENABLE);

	do {
		lis3mdl_reset_get(&mag, &rst);
		osDelay(100);
	} while (rst);

	/* Enable Block Data Update */
	lis3mdl_block_data_update_set(&mag, PROPERTY_ENABLE);
	/* Set Output Data Rate */
	lis3mdl_data_rate_set(&mag, LIS3MDL_HP_20Hz);
	/* Set full scale */
	lis3mdl_full_scale_set(&mag, LIS3MDL_16_GAUSS);
	/* Enable temperature sensor */
	lis3mdl_temperature_meas_set(&mag, PROPERTY_ENABLE);
	/* Set device in continuous mode */
	lis3mdl_operating_mode_set(&mag, LIS3MDL_CONTINUOUS_MODE);

}

static int32_t imu_write(void *handle, uint8_t reg, const uint8_t *bufp,
		uint16_t len) {
	return HAL_I2C_Mem_Write(handle, ASM330LHH_I2C_ADD_L, reg,
	I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 100);
	return 0;
}

static int32_t imu_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
	return HAL_I2C_Mem_Read(handle, ASM330LHH_I2C_ADD_L, reg,
	I2C_MEMADD_SIZE_8BIT, bufp, len, 100);
}

static int32_t mag_write(void *handle, uint8_t reg, const uint8_t *bufp,
		uint16_t len) {
	return HAL_I2C_Mem_Write(handle, LIS3MDL_I2C_ADD_L, reg,
	I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 100);
	return 0;
}

static int32_t mag_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
	return HAL_I2C_Mem_Read(handle, LIS3MDL_I2C_ADD_L, reg,
	I2C_MEMADD_SIZE_8BIT, bufp, len, 100);
}

//-------------------------------------------------[Filtering Functions]------------------------------------------------//

// Function to update roll, pitch, yaw using complementary filter
void compFilter(float gx, float gy, float gz, float ax, float ay, float az,
		float mx, float my, float mz, float *roll, float *pitch, float *yaw) {
	// Convert gyroscope degrees/sec to radians/sec
	gx *= M_PI / 180.0f;
	gy *= M_PI / 180.0f;
	gz *= M_PI / 180.0f;

	// Compute roll and pitch from accelerometer (gravity vector)
	float accelRoll = atan2f(ay, az) * 180.0f / M_PI;
	float accelPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / M_PI;

	// Integrate gyroscope data
	*roll = ALPHA * (*roll + gx * DT) + (1 - ALPHA) * accelRoll;
	*pitch = ALPHA * (*pitch + gy * DT) + (1 - ALPHA) * accelPitch;

	// Compute yaw from magnetometer (only needed if magnetometer is used)
	float magYaw = atan2f(-my, mx) * 180.0f / M_PI;
	*yaw = ALPHA * (*yaw + gz * DT) + (1 - ALPHA) * magYaw;
}

