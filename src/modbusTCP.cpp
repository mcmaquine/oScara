//============================================================================
// Name        : modbusTCP.cpp
// Author      : Maquine
// Version     :
// Copyright   : A FPF TECH project
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <modbus/modbus-tcp.h>
#include <MR_JE.h>
#include <thread>

using namespace std;

modbus_t *servo1, *servo2;
uint8_t bits[16];//read buffer
uint8_t coil[16];//write buffer
uint16_t w_reg[4], r_reg[4]; //write register and read register respectivelly

uint16_t convert_raw_data_to_word( int nb, uint8_t *bits)
{
	uint16_t data = 0;

	for( int i = 0; i < nb; i++)
	{
		data += bits[i] << i;
	}

	return data;
}

//Return 1 when finish
int convert_word_to_raw_data(int nb, uint16_t data, uint8_t *bits)
{
	for (int i = 0; i < nb; ++i)
	{
		bits[i] = data & 0x1;
		data = data >> 1;
	}

	return 1;
}

int main() {

	thread read;
	char mode;

	servo1 = modbus_new_tcp("10.8.0.201", 502);
	servo2 = modbus_new_tcp("10.8.0.202", 502);

	//Verifica se o objeto modbus foi criado
	if (servo2 == NULL) {
	    printf("Unable to allocate libmodbus context\n");
	    return -1;
	}

	//Verifica se a comunicacao foi realizada
	if( modbus_connect( servo2 ) == -1 )
	{
		printf("Connection failed %s\n", modbus_strerror(errno));
		return -1;
	}
	modbus_flush( servo2 );

	//set_home_method(servo2, MR_METHOD_35);
	servo_off( servo2 );

	set_mode(servo2, MR_HOME_MODE);
	//this_thread::sleep_for(500ms); //wait half second before continue
	get_mode(servo2, &mode);
	this_thread::sleep_for(500ms); //wait half second before continue

	switch (mode) {
		case MR_INDEXER_MODE:
			printf("Mode: Indexer mode\n");
			break;
		case MR_POINT_TABLE:
			printf("Mode: Point table mode (pt)\n");
			break;
		case MR_JOG_MODE:
			printf("Mode: Jog mode\n");
			break;
		case MR_PROFILE_POSTION:
			printf("Mode: Profile position mode\n");
			break;
		case MR_HOME_MODE:
			printf("Home mode\n");
			break;
		default:
			break;
	}

	if( modbus_read_registers(servo2, MR_STATUS_WORD , 1 , r_reg) == -1 )
		printf("Note read %s", modbus_strerror(errno) );
	else
		printf("Inputs INT: %d Hex: 0x%X\n", r_reg[0], r_reg[0]);

	modbus_close( servo2 );
	modbus_free( servo2 );
	modbus_free( servo1 );

	//this_thread::sleep_for( chrono::milliseconds( 980 ) );
	cout	<<	"Exit"	<<	endl;
	return 0;
}
/**
 * cout << "Modbus Test" << endl; // prints "Hello Modbus"
	read_status =  modbus_read_input_bits(mb, 0, 8, bits);
	input = convert_raw_data_to_word(8, bits);

	cout << "Modbus read inputs: " << read_status << endl;

	printf("Inputs Hex: 0x%X\n", input);

	for (int var = 0; var < 8; ++var) {
		printf("Input[%d]: %d\n", var, bits[var]);
	}

	int program()
{
	//if enters here, Factory I/O is running
	modbus_read_input_bits(servo1, 0, 16, bits);
	coil[0] = 1;
	modbus_write_bits(servo1, 16, 16, coil);

	do
	{
		if( bits[1] == 1 && bits[2] == 0 ) //sensor B esta detectando
		{
			coil[0] = 0;
			coil[1] = 1;
		}

		if( bits[1] == 0 && bits[2] == 1) //sensor A esta detectando
		{
			coil[0] = 1;
			coil[1] = 0;
		}

		modbus_write_bits(servo1, 16, 16, coil);
		modbus_read_input_bits(servo1, 0, 16, bits);

		if( bits[0] == 0 ) {
			coil[0] = 0;
			coil[1] = 0;
			modbus_write_bits(servo1, 16, 16, coil);
			printf("factory I/O paused\n");
		}
	}while( bits[0] );

	return 1;
}
 */
