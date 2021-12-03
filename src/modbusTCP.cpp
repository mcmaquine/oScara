//============================================================================
// Name        : modbusTCP.cpp
// Author      : Maquine
// Version     :
// Copyright   : A FPF TECH project
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <stdlib.h>
#include <modbus/modbus-tcp.h>
#include <thread>
#include <readline/readline.h>
#include <readline/history.h>
#include <MR_JE.h>

using namespace std;

modbus_t *J1, *J2;
uint16_t wr_reg[16], r_reg[16]; //write register and read register respectivelly

int parse (char *comm);

int connectAll( modbus_t *J1, modbus_t *J2 );
int enableAll( modbus_t *J1, modbus_t *J2 );
int setMode( modbus_t *J1, modbus_t *J2, char mode);
int offAll( modbus_t *J1, modbus_t *J2 );
int positioning_profile( modbus_t *J1, modbus_t *J2 );
int positioning_pt( modbus_t *J1, modbus_t *J2, int point );

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
	pt *teste;
	int ps;
	char *comm;

	J1 = modbus_new_tcp("10.8.0.201", 502);
	J2 = modbus_new_tcp("10.8.0.202", 502);

	do {
		do {
			comm = readline( "oScara> " );
		} while ( comm == NULL );

		//add_history( comm );
		ps = parse( comm );
		free( comm );
	} while ( ps != 0 );

	modbus_read_registers(J1, MR_CONTROL_WORD, 1, r_reg);
	//resetBit(&r_reg[0], BIT_7 );
	//write_register(J2, MR_CONTROL_WORD, r_reg[0]);

	printf("ControlWord 0x%X\n", r_reg[0]);

	if( modbus_read_registers(J1, MR_STATUS_WORD , 1 , r_reg) == -1 )
		printf("Note read %s", modbus_strerror(errno) );
	else
		printf("Inputs INT: %d ex: 0x%X\n", r_reg[0], r_reg[0]);

	if( get_pt_data(J2, 3, &teste) == -1)
	{
		printf("No read\n");
	}

	modbus_close( J2 );
	modbus_close( J1 );
	modbus_free( J2 );
	modbus_free( J1 );
	free(teste);

	//this_thread::sleep_for( chrono::milliseconds( 980 ) );
	cout	<<	"Exit"	<<	endl;
	return 0;
}

/**
 * 	Aqui tem todas as funcoes que
 */
int parse( char *comm)
{
	int status;

	if(!strcmp(comm, "exit")) //sair
	{
		printf("Exit\n");
		return 0;
	}
	else if( !strcmp( comm, "on")) //servo on
	{
		printf("Servo On\n");
		status = enableAll(J1, J2);
		return 1;
	}
	else if( !strcmp( comm, "connect")) //conectar no modbus
	{
		printf("Connect servos\n");
		status = connectAll(J1, J2);
		return status;
	}
	else if( !strcmp(comm, "off")) //realiza servo off
	{
		status = offAll(J1, J2);
		printf("Servo off\n");
		return status;
	}
	else
	{
		printf("Not a command\n");
		return 1;
	}
}

int connectAll( modbus_t *sevo1, modbus_t *J2 )
{
	//Verifica se o objeto modbus foi criado
	if (J1 == NULL && J2 == NULL) {
	    printf("Unable to allocate libmodbus context\n");
	    return -1;
	}

	//Verifica se a comunicacao foi realizada
	if( modbus_connect( J1 ) == -1 )
	{
		printf( "Connection failed %s\n", modbus_strerror(errno) );
		return -1;
	}
	if( modbus_connect( J2 ) == -1 )
	{
		printf( "Connection failed %s\n", modbus_strerror(errno) );
		return -1;
	}
	modbus_flush( J1 );
	modbus_flush( J2 );

	return 1;
}

int enableAll( modbus_t *J1, modbus_t *J2 )
{
	//set_home_method(J1, MR_METHOD_35);
	servo_on( J1 );
	servo_on( J2 );

	return 1;
}

int offAll( modbus_t *J1, modbus_t *J2 )
{
	servo_off( J1 );
	servo_off( J2 );

	return 1;
}

int setMode( modbus_t *J1, modbus_t *J2, char mode)
{
	set_mode(J1, mode);
	//this_thread::sleep_for(500ms); //wait half second before continue
	get_mode(J1, &mode);
	//home(J1);
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

int positioning_pt(modbus_t *J1, modbus_t *J2, int point )
{
	if( pt_move(J1, point) == 1)
	{
		printf("Movendo servo 1...\n");
	}
	if( pt_move(J2, point) == 1 )
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

int stat = home(J2);
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

	set_mode(J2, MR_POINT_TABLE);
 */
