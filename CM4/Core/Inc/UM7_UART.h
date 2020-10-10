/*
 * UM7_UART.h
 *
 *  Created on: Mar 12, 2020
 *      Author: kpiek
 */

#ifndef INC_UM7_UART_H_
#define INC_UM7_UART_H_

#include "main.h"
#include "usart.h"

/* Length of received packet */
#define PROC_ACCLE_DATA_LENGTH					19
#define COMMAND_PACKET_LENGHT					7

/* Structure for holding received packet information */
typedef struct{
	uint8_t Address;
	uint8_t PT;
	uint16_t Checksum;

	uint8_t data_length;
	uint8_t data[30];
}UM7_packet_struct_typedef;

/* Status of parse serial data */
#define	NOT_ENOUGH_DATA_FOR_FULL_PACKET			1
#define NO_PACKET_HEADER_WAS_FOUND				2
#define INSUFFICIENT_DATA_TO_PARSE				3
#define CHECKSUM_WAS_BAD						4
#define GOOD_PACKET_WAS_RECEIVED				0

/* Status of send command */
enum{
	COMMAND_SEND_ERROR,
	COMMAND_SEND_SUCCESFUL
};

/* Status of read operation */
enum{
	READ_OPERATION_ERROR,
	READ_OPERATION_SUCCESFUL
};

/* Pocket type */
enum{
	DATA_PACKET,
	COMMAND_COMPLETE_PACKET,
	COMMAND_FAILED_PACKET,
	GET_FW_VERSION_PACKET,
	ERROR_PACKET
};

/* Sensor orientation data read */
#define PROC_ACCEL_DATA_BATCH_READ	 		0x65	// dreg accel proc x
#define PROC_GYRO_DATA_BATCH_READ			0x61	// dreg gyro proc x

/**
 * @brief Function to convert ieee value to float value
 * @param
 * @param
 * @retval uint8_t - status of parse serial data
 */
void convert_to_ieee(uint8_t ieee_value[], uint8_t ieee_value_count, float *float_vlaue_table);

/**
 * @brief Function to construct packet type byte in packet to be send
 * @param
 * @param
 * @retval uint8_t - status of parse serial data
 */
uint8_t packet_type_constructor(uint8_t has_data, uint8_t is_batch, uint8_t batch_length, uint8_t hidden, uint8_t command_filed);


/**
 * @brief Function to parse serial data
 * @param
 * @param
 * @retval uint8_t - status of parse serial data
 */
uint8_t parse_serial_data(uint8_t *rx_data, uint8_t rx_length, UM7_packet_struct_typedef *packet);

/**
 * @brief Function for constructing and sending the command
 * @param
 * @param
 * @retval uint8_t ststus of send command
 */
uint8_t UM7_send_command(UART_HandleTypeDef *huartx, uint8_t command);

/**
 * @brief Function for requesting read operation
 * @param
 * @param
 * @retval uint8_t ststus of send command
 */
uint8_t UM7_read_operation(UART_HandleTypeDef *huartx, uint8_t reg_address, uint8_t batch_length, uint8_t *rx_data, uint8_t *rx_length);

/**
 * @brief Function to process command complete packet
 * @param
 * @param
 * @retval
 */
void UM7_process_command_complete_packet(UM7_packet_struct_typedef *packet);

/**
 * @brief Function to process command failed packet
 * @param
 * @param
 * @retval
 */
void UM7_process_command_failed_packet(UM7_packet_struct_typedef *packet);


/**********************************************************************************/

/* UM7 communication mode */
#define READ_MODE		0x00
#define WRITE_MODE		0x01


/* Register overview */
/* Configuration register */
#define CREG_COM_SETTINGS		0x00	/* General communication settings */
#define CREG_COM_RATES1			0x01	/* Broadcast rate settings */
#define CREG_COM_RATES2			0x02	/* Broadcast rate settings */
#define CREG_COM_RATES3			0x03	/* Broadcast rate settings */
#define CREG_COM_RATES4			0x04	/* Broadcast rate settings */
#define CREG_COM_RATES5			0x05	/* Broadcast rate settings */
#define CREG_COM_RATES6			0x06	/* Broadcast rate settings */
#define CREG_COM_RATES7			0x07	/* Broadcast rate settings */
#define CREG_MISC_SETTING		0x08	/* Miscellaneous filter and sensor control options */
#define CREG_HOME_NORTH			0x09	/* GPS */
#define CREG_HOME_EAST			0x0A	/* GPS */
#define CREG_HOME_UP			0x0B	/* GPS */
#define CREG_GYRO_TRIM_X		0x0C	/* Bias trim for x-axis rate gyro */
#define CREG_GYRO_TRIM_Y		0x0D	/* Bias trim for y-axis rate gyro */
#define CREG_GYRO_TRIM_Z		0x0E	/* Bias trim for z-axis rate gyro */
#define CREG_MAG_CAL1_1			0x0F	/* Row 1, Column 1 of magnetometer calibration matrix */
#define CREG_MAG_CAL1_2			0x10	/* Row 1, Column 2 of magnetometer calibration matrix */
#define CREG_MAG_CAL1_3			0x11	/* Row 1, Column 3 of magnetometer calibration matrix */
#define CREG_MAG_CAL2_1			0x12	/* Row 2, Column 1 of magnetometer calibration matrix */
#define CREG_MAG_CAL2_2			0x13	/* Row 2, Column 2 of magnetometer calibration matrix */
#define CREG_MAG_CAL2_3			0x14	/* Row 2, Column 3 of magnetometer calibration matrix */
#define CREG_MAG_CAL3_1			0x15	/* Row 3, Column 1 of magnetometer calibration matrix */
#define CREG_MAG_CAL3_2			0x16	/* Row 3, Column 2 of magnetometer calibration matrix */
#define CREG_MAG_CAL3_3			0x17	/* Row 3, Column 3 of magnetometer calibration matrix */
#define CREG_MAG_BIAS_X			0x18	/* Magnetometer X-axis bias */
#define CREG_MAG_BIAS_Y			0x19	/* Magnetometer Y-axis bias */
#define CREG_MAG_BIAS_Z			0x1A	/* Magnetometer Z-axis bias */
#define CREG_ACCEL_CAL1_1		0x1B	/* Row 1, Column 1 of accelerometer calibration matrix */
#define CREG_ACCEL_CAL1_2		0x1C	/* Row 1, Column 2 of accelerometer calibration matrix */
#define CREG_ACCEL_CAL1_3		0x1D	/* Row 1, Column 3 of accelerometer calibration matrix */
#define CREG_ACCEL_CAL2_1		0x1E	/* Row 2, Column 1 of accelerometer calibration matrix */
#define CREG_ACCEL_CAL2_2		0x1F	/* Row 2, Column 2 of accelerometer calibration matrix */
#define CREG_ACCEL_CAL2_3		0x20	/* Row 2, Column 3 of accelerometer calibration matrix */
#define CREG_ACCEL_CAL3_1		0x21	/* Row 3, Column 1 of accelerometer calibration matrix */
#define CREG_ACCEL_CAL3_2		0x22	/* Row 3, Column 2 of accelerometer calibration matrix */
#define CREG_ACCEL_CAL3_3		0x23	/* Row 3, Column 3 of accelerometer calibration matrix */
#define CREG_ACCEL_BIAS_X		0x24	/* Accelerometer X-axis bias */
#define CREG_ACCEL_BIAS_Y		0x25	/* Accelerometer Y-axis bias */
#define CREG_ACCEL_BIAS_Z		0x26	/* Accelerometer Z-axis bias */

/* Data Register */
#define DREG_HEALTH				0x55	/* Contains information about the health and status of the UM7 */
#define DREG_GYRO_RAW_XY		0x56	/* Raw X and Y rate gyro data */
#define DREG_GYRO_RAW_Z			0x57	/* Raw Z rate gyro data */
#define DREG_GYRO_TIME			0x58	/* Time at which rate gyro data was acquired */
#define DREG_ACCEL_RAW_XY		0x59	/* Raw X and Y rate accelerometer data */
#define DREG_ACCEL_RAW_Z		0x5A	/* Raw Z rate accelerometer data */
#define DREG_ACCEL_TIME			0x5B	/* Time at which rate accelerometer data was acquired */
#define DREG_MAG_RAW_XY			0x5C	/* Raw X and Y rate magnetometer data */
#define DREG_MAG_RAW_Z			0x5D	/* Raw Z rate magnetometer data */
#define DREG_MAG_TIME			0x5E	/* Time at which rate magnetometer data was acquired */
#define DREG_TEMPERATURE 		0x5F	/* Temperature data */
#define DREG_TEMPERATURE_TIME	0x60	/* Time at which temperature data was acquired */
#define DREG_GYRO_PROC_X		0x61	/* Processed x-axis rate gyro data */
#define DREG_GYRO_PROC_Y		0x62	/* Processed y-axis rate gyro data */
#define DREG_GYRO_PROC_Z		0x63	/* Processed z-axis rate gyro data */
#define DREG_GYRO_PROC_TIME		0x64	/* Time at which gryoscope data was acquired */
#define DREG_ACCEL_PROC_X		0x65	/* Processed x-axis accel data */
#define DREG_ACCEL_PROC_Y		0x66	/* Processed y-axis accel data */
#define DREG_ACCEL_PROC_Z		0x67	/* Processed z-axis accel data */
#define DREG_ACCEL_PROC_TIME	0x68	/* Time at which accelerometer data was acquired */
#define DREG_MAG_PROC_X			0x69	/* Processed x-axis magnetometer data */
#define DREG_MAG_PROC_Y			0x6A	/* Processed y-axis magnetometer data */
#define DREG_MAG_PROC_Z			0x6B	/* Processed z-axis magnetometer data */
#define DREG_MAG_PROC_TIME		0x6C	/* Time at which magnetometer data was acquired */
#define DREG_QUAT_AB			0x6D	/* Quaternion elements A and B */
#define DREG_QUAT_CD			0x6E	/* Quaternion elements C and D */
#define DREG_QUAT_TIME			0x6F	/* Time at which the sensor was at the specified quaternion rotation */
#define DREG_EULER_PHI_THETA	0x70	/* Roll and pitch angles */
#define DREG_EULER_PSI			0x71 	/* Yaw angle */
#define DREG_EULER_PHI_THETA_DOT	0x72	/* Roll and Pitch angle rates */
#define DREG_EULER_PSI_DOT		0x73	/* Yaw rate */
#define DREG_EULER_TIME 		0x74	/* Time of compted Euler attitude and rates */
#define DREG_POSITION_NORTH		0x75	/* GPS */
#define DREG_POSITION_EAST		0x76	/* GPS */
#define DREG_POSITION_UP		0x77	/* GPS */
#define DREG_POSITION_TIME		0x78	/* GPS */
#define DREG_VELOCITY_NORTH		0x79	/* GPS */
#define DREG_VELOCITY_EAST		0x7A	/* GPS */
#define DREG_VELOCITY_UP		0x7B	/* GPS */
#define DREG_VELOCITY_TIME		0x7C	/* GPS */
#define DREG_GPS_LATITUDE		0x7D	/* GPS */
#define DREG_GPS_LONGITUDE		0x7E	/* GPS */
#define DREG_GPS_ALTITUDE 		0x7F	/* GPS */
#define DREG_GPS_COURSE			0x80	/* GPS */
#define DREG_GPS_SPEED			0x81	/* GPS */
#define DREG_GPS_TIME			0x82	/* GPS */
#define DREG_GPS_SAT_1_2		0x83	/* GPS */
#define DREG_GPS_SAT_3_4		0x84	/* GPS */
#define DREG_GPS_SAT_5_6		0x85	/* GPS */
#define DREG_GPS_SAT_7_8		0x86	/* GPS */
#define DREG_GPS_SAT_9_10		0x87	/* GPS */
#define DREG_GPS_SAT_11_12		0x88	/* GPS */
#define DREG_GYRO_BIAS_X		0x89	/* Gyro x-axis bias estimate */
#define DREG_GYRO_BIAS_Y		0x8A	/* Gyro y-axis bias estimate */
#define DREG_GYRO_BIAS_Z		0x8B	/* Gyro z-axis bias estimate */

/* Commands */
#define GET_FW_VERSION			0xAA	/* Causes the autopilot to respond with a packet containing the current firmware revision */
#define FLASH_COMMIT 			0xAB	/* Writes all current configuration settings to flash */
#define RESET_TO_FACTORY		0xAC	/* Reset all setting to factory defaults */
#define ZERO_GYROS				0xAD	/* Causes the rate gyro biases to be calibrated */
#define SET_HOME_POSITION		0xAE	/* GPS */
#define SET_MAG_REFERENCE		0xB0	/* Sets the magnetometer reference vector */
#define CALIBRATE_ACCELEROMETER 0xB1	/* Calibrates the accelerometer biases */
#define RESET_EKF				0xB3	/* Resets the EKF */

/**********************************************************************************/

#endif /* INC_UM7_UART_H_ */
