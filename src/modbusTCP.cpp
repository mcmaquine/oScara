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
#include "robot.h"
#include "cinematics.h"

using namespace std;

modbus_t *servo1, *servo2;
uint16_t w_reg[16], r_reg[16]; //write register and read register respectivelly

int connectAll( modbus_t *servo1, modbus_t *servo2 );
int enableAll( modbus_t *servo1, modbus_t *servo2 );
int setMode( modbus_t *servo1, modbus_t *servo2, char mode);
int offAll( modbus_t *servo1, modbus_t *servo2 );
int positioning_profile( modbus_t *servo1, modbus_t *servo2 );
int positioning_pt( modbus_t *servo1, modbus_t *servo2, int point );

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
	pt *teste;

	servo1 = modbus_new_tcp("10.8.0.201", 502);
	servo2 = modbus_new_tcp("10.8.0.202", 502);

	connectAll(servo1, servo2);
	enableAll(servo1, servo2);
	setMode(servo1, servo2, MR_POSITION_CONTROL_MODE);

	modbus_read_registers(servo1, MR_CONTROL_WORD, 1, r_reg);
	//resetBit(&r_reg[0], BIT_7 );
	//write_register(servo2, MR_CONTROL_WORD, r_reg[0]);

	printf("ControlWord 0x%X\n", r_reg[0]);

	if( modbus_read_registers(servo1, MR_STATUS_WORD , 1 , r_reg) == -1 )
		printf("Note read %s", modbus_strerror(errno) );
	else
		printf("Inputs INT: %d Hex: 0x%X\n", r_reg[0], r_reg[0]);

	if( get_data_from_pt(servo2, 2, teste) == -1)
	{
		printf("No read\n");
	}
	else
	{
		//print some data
		printf("%d\n", teste->point_data);
	}

	modbus_close( servo2 );
	modbus_close( servo1 );
	modbus_free( servo2 );
	modbus_free( servo1 );
	free(teste);

	//this_thread::sleep_for( chrono::milliseconds( 980 ) );
	cout	<<	"Exit"	<<	endl;
	return 0;
}

int connectAll( modbus_t *sevo1, modbus_t *servo2 )
{
	//Verifica se o objeto modbus foi criado
	if (servo1 == NULL && servo2 == NULL) {
	    printf("Unable to allocate libmodbus context\n");
	    return -1;
	}

	//Verifica se a comunicacao foi realizada
	if( modbus_connect( servo1 ) == -1 )
	{
		printf( "Connection failed %s\n", modbus_strerror(errno) );
		return -1;
	}
	if( modbus_connect( servo2 ) == -1 )
	{
		printf( "Connection failed %s\n", modbus_strerror(errno) );
		return -1;
	}
	modbus_flush( servo1 );
	modbus_flush( servo2 );

	return 1;
}

int enableAll( modbus_t *servo1, modbus_t *servo2 )
{
	//set_home_method(servo1, MR_METHOD_35);
	servo_on( servo1 );
	servo_on( servo2 );

	return 1;
}

int offAll( modbus_t *servo1, modbus_t *servo2 )
{
	servo_off( servo1 );
	servo_off( servo2 );

	return 1;
}

int setMode( modbus_t *servo1, modbus_t *servo2, char mode)
{
	set_mode(servo1, MR_POINT_TABLE_MODE);
	set_mode(servo2, MR_POINT_TABLE_MODE);
	//this_thread::sleep_for(500ms); //wait half second before continue
	get_mode(servo1, &mode);
	//home(servo1);
	this_thread::sleep_for(500ms); //wait half second before continue

	switch (mode) {
		case MR_INDEXER_MODE:
			printf("Mode: Indexer mode\n");
			break;
		case MR_POINT_TABLE_MODE:
			printf("Mode: Point table mode (pt)\n");
			break;
		case MR_JOG_MODE:
			printf("Mode: Jog mode\n");
			break;
		case MR_PROFILE_POSITION_MODE:
			printf("Mode: Profile position mode\n");
			break;
		case MR_HOME_MODE:
			printf("Home mode\n");
			break;
		default:
			break;
	}

	return 1;
}

int positioning_pt(modbus_t *servo1, modbus_t *servo2, int point )
{
	if( pt_move(servo1, point) == 1)
	{
		printf("Movendo servo 1...\n");
	}
	if( pt_move(servo2, point) == 1 )
	{
		printf("Movendo servo 2...\n");
	}

	return 1;
}

/**
 * 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
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

int stat = home(servo2);
		switch( stat )
		{
			case -1:
				printf("Not communicate\n");
				break;
			case 1:
				printf("Home completed\n");
				break;
			case 0:
				printf("Home not Done\n");
				break;
			default:
				break;
		}

	set_mode(servo2, MR_POINT_TABLE);
 */
