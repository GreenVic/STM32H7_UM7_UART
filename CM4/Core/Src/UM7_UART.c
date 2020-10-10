/*
 * UM7_UART.c
 *
 *  Created on: Mar 12, 2020
 *      Author: kpiek
 */


#include "UM7_UART.h"

/* union to convert ieee value to float */
typedef union {

    float f;
    struct
    {

        // Order is important.
        // Here the members of the union data structure
        // use the same memory (32 bits).
        // The ordering is taken
        // from the LSB to the MSB.

        uint32_t mantissa : 23;
        uint32_t exponent : 8;
        uint32_t sign : 1;

    } raw;
} myfloat;

/* Generally to convert ieee value to float value */
// Function to convert a binary array
// to the corresponding integer
uint32_t convertToInt(uint32_t* arr, uint32_t low, uint32_t high)
{
    uint32_t f = 0, i;
    for (i = high; i >= low; i--) {
        f = f + arr[i] * pow(2, high - i);
    }
    return f;
}


/**
 * @brief Function to convert ieee value to float value
 * @param
 * @param
 * @retval uint8_t - status of parse serial data
 */
void convert_to_ieee(uint8_t ieee_value[], uint8_t ieee_value_count, float *float_vlaue_table){

	uint32_t ieee_bin_value = 0;

	uint8_t r;
	for(r=0; r<ieee_value_count; r++){

		/* copying exact value from ieee_value to ieee_table */
		uint8_t ieee_table[4];
		int j;
		for (j = 0; j < 4; j++) {
			ieee_table[3 - j] = ieee_value[(r*4)+j];
		}

		ieee_bin_value |= ieee_table[3];
		ieee_bin_value <<= 8;
		ieee_bin_value |= ieee_table[2];
		ieee_bin_value <<= 8;
		ieee_bin_value |= ieee_table[1];
		ieee_bin_value <<= 8;
		ieee_bin_value |= ieee_table[0];

		/* ieee table to store bits from ieee_bin_value */
		uint32_t ieee[32];
		int c;
		for (c = 0; c < 32; c++)
			ieee[c] = 0;
		c = 31;
		while (ieee_bin_value > 0) {

			ieee[c--] = (ieee_bin_value & 0x01);
			ieee_bin_value >>= 1;

		}

		/* union to store float ieee value */
		myfloat var;

		// Convert the least significant
		// mantissa part (23 bits)
		// to corresponding decimal integer
		int f = convertToInt(ieee, 9, 31);

		// Assign integer representation of mantissa
		var.raw.mantissa = f;

		// Convert the exponent part (8 bits)
		// to a corresponding decimal integer
		f = convertToInt(ieee, 1, 8);

		// Assign integer representation
		// of the exponent
		var.raw.exponent = f;

		// Assign sign bit
		var.raw.sign = ieee[0];

		float_vlaue_table[r] = var.f;

	}

}

/**
 * @brief Function to parse serial data
 * @param
 * @param
 * @retval uint8_t - status of parse serial data
 */
uint8_t packet_type_constructor(uint8_t has_data, uint8_t is_batch, uint8_t batch_length, uint8_t hidden, uint8_t command_filed){

	uint8_t packet_type = 0;

	packet_type |= (has_data<<7);
	packet_type |= (is_batch<<6);
	packet_type |= (batch_length<<2);
	packet_type |= (hidden<<1);
	packet_type |= (command_filed);

	return packet_type;
}

/**
 * @brief Function to parse serial data
 * @param
 * @param
 * @retval uint8_t
 */
uint8_t parse_serial_data(uint8_t *rx_data, uint8_t rx_length, UM7_packet_struct_typedef *packet) {

	uint8_t index_serial;

	// Make sure that the data buffer provided is long enough to contain a full packet
	// The minimum packet length is 7 bytes
	if (rx_length < 7) {
		return NOT_ENOUGH_DATA_FOR_FULL_PACKET;
	}

	// Try to find the ‘snp’ start sequence for the packet
	for(index_serial = 0; index_serial < (rx_length - 2); index_serial++)
	{
		// Check for ‘snp’. If found, immediately exit the loop
		if( rx_data[index_serial] == 's' && rx_data[index_serial+1] == 'n' && rx_data[index_serial+2] == 'p' ){
			break;
		}
	}

	uint8_t packet_index = index_serial;

	// Check to see if the variable ‘packet_index’ is equal to (rx_length - 2). If it is, then the above
	// loop executed to completion and never found a packet header.
	if( packet_index == (rx_length - 2)){
		return NO_PACKET_HEADER_WAS_FOUND;
	}

	// If we get here, a packet header was found. Now check to see if we have enough room
	// left in the buffer to contain a full packet. Note that at this point, the variable ‘packet_index’
	// contains the location of the ‘s’ character in the buffer (the first byte in the header)
	if ((rx_length - packet_index) < 7) {
		return INSUFFICIENT_DATA_TO_PARSE;
	}

	// We’ve found a packet header, and there is enough space left in the buffer for at least
	// the smallest allowable packet length (7 bytes). Pull out the packet type byte to determine
	// the actual length of this packet
	uint8_t PT = rx_data[packet_index + 3];

	// Do some bit-level manipulation to determine if the packet contains data and if it is a batch
	// We have to do this because the individual bits in the PT byte specify the contents of the
	// packet.
	uint8_t packet_has_data = (PT >> 7) & 0x01; // Check bit 7 (HAS_DATA)
	uint8_t packet_is_batch = (PT >> 6) & 0x01; // Check bit 6 (IS_BATCH)
	uint8_t batch_length = (PT >> 2) & 0x0F; // Extract the batch length (bits 2 through 5)

	// Now finally figure out the actual packet length
	uint8_t data_length = 0;
	if(packet_has_data){

		if(packet_is_batch){

			// Packet has data and is a batch. This means it contains ‘batch_length' registers, each
			// of which has a length of 4 bytes
			data_length = 4 * batch_length;

		}else{

			// Packet has data but is not a batch. This means it contains one register (4 bytes)
			data_length = 4;

		}

	}else{

		// Packet has no data
		data_length = 0;

	}

	// At this point, we know exactly how long the packet is. Now we can check to make sure
	// we have enough data for the full packet.
	if((rx_length - packet_index) < (data_length + 5)){
		return INSUFFICIENT_DATA_TO_PARSE;
	}

	// If we get here, we know that we have a full packet in the buffer. All that remains is to pull
	// out the data and make sure the checksum is good.
	// Start by extracting all the data
	packet->Address = rx_data[packet_index + 4];
	packet->PT = PT;

	// Get the data bytes and compute the checksum all in one step
	packet->data_length = data_length;
	uint16_t computed_checksum = 's' + 'n' + 'p' + packet->PT + packet->Address;

	for(index_serial = 0; index_serial < data_length; index_serial++){

		// Copy the data into the packet structure’s data array
		packet->data[index_serial] = rx_data[packet_index + 5 + index_serial];
	    // Add the new byte to the checksum
		computed_checksum += packet->data[index_serial];

	}

	// Now see if our computed checksum matches the received checksum
	// First extract the checksum from the packet
	uint16_t received_checksum = (rx_data[packet_index + 5 + data_length] << 8);
	received_checksum |= rx_data[packet_index + 6 + data_length];

	// Now check to see if they don’t match
	if (received_checksum != computed_checksum) {
		return CHECKSUM_WAS_BAD;
	}

	// At this point, we’ve received a full packet with a good checksum. It is already
	// fully parsed and copied to the ‘packet’ structure, so return 0 to indicate that a packet was
	// processed.
	return GOOD_PACKET_WAS_RECEIVED;

}

/**
 * @brief Function
 * @param
 * @param
 * @retval uint8_t ststus of send command
 */
static uint8_t check_command_status(uint8_t PT_byte){

	if((PT_byte&0x01) == 0x01){
		return COMMAND_SEND_ERROR;
	}else{
		return COMMAND_SEND_SUCCESFUL;
	}

}

/**
 * @brief Function for constructing and sending the command
 * @param
 * @param
 * @retval uint8_t ststus of send command
 */
uint8_t UM7_send_command(UART_HandleTypeDef *huartx, uint8_t command){

	uint8_t tx_data[20];
	uint8_t rx_data[20];

	tx_data[0] = 's';
	tx_data[1] = 'n';
	tx_data[2] = 'p';

	tx_data[3] = 0x00; // Packet Type byte

	tx_data[4] = command; // Address of command register

	/* Count checksum of packet  */
	uint16_t checksum = 0;
	uint8_t i;
	for(i = 0; i < 5; i++){
		checksum += tx_data[i];
	}

	tx_data[5] = (checksum & 0xFF00)>>8; // Checksum high byte
	tx_data[6] = (checksum & 0x00FF); // Checksum low byte

	//Send command
	HAL_UART_Transmit(huartx, tx_data, COMMAND_PACKET_LENGHT, HAL_MAX_DELAY);

	/* garbage */
	HAL_UART_Receive(huartx, rx_data, 7, HAL_MAX_DELAY);
	HAL_UART_Receive(huartx, rx_data, 19, HAL_MAX_DELAY);
	/* garbage */

	HAL_UART_Receive(huartx, rx_data, COMMAND_PACKET_LENGHT, HAL_MAX_DELAY);

    UM7_packet_struct_typedef packet;
	parse_serial_data(rx_data, COMMAND_PACKET_LENGHT, &packet);

	return check_command_status(packet.PT);

}

/**
 * @brief Function for requesting read operation
 * @param
 * @param
 * @retval uint8_t ststus of send command
 */
uint8_t UM7_read_operation(UART_HandleTypeDef *huartx, uint8_t reg_address, uint8_t batch_length, uint8_t rx_data[], uint8_t *rx_length){

	uint8_t tx_data[20];

	tx_data[0] = 's';
	tx_data[1] = 'n';
	tx_data[2] = 'p';

	tx_data[3] = packet_type_constructor(0, 1, batch_length, 0, 0); // Packet Type byte

	tx_data[4] = reg_address; // Address of command register

	/* Count checksum of packet  */
	uint16_t checksum = 0;
	uint8_t i;
	for (i = 0; i < 5; i++) {
		checksum += tx_data[i];
	}

	tx_data[5] = (checksum & 0xFF00) >> 8; // Checksum high byte
	tx_data[6] = (checksum & 0x00FF); // Checksum low byte

	//Send read request
	HAL_UART_Transmit(huartx, tx_data, 7, HAL_MAX_DELAY);

	(*rx_length) = 7+(4*batch_length);
	HAL_UART_Receive(huartx, rx_data, (*rx_length), HAL_MAX_DELAY);


	return READ_OPERATION_SUCCESFUL;

}

/**
 * @brief Function to fw version packet
 * @param
 * @param
 * @retval
 */
void UM7_process_get_fw_version_pocket(UM7_packet_struct_typedef *packet){



}

/**
 * @brief Function to process command complete packet
 * @param
 * @param
 * @retval
 */
void UM7_process_command_complete_packet(UM7_packet_struct_typedef *packet){



}

/**
 * @brief Function to process command failed packet
 * @param
 * @param
 * @retval
 */
void UM7_process_command_failed_packet(UM7_packet_struct_typedef *packet){



}

















