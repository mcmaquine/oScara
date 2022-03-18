//============================================================================
// Name        : modbusTCP.cpp
// Author      : Maquine
// Version     :
// Copyright   : A FPF TECH project
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <robot.h>
#include <iostream>
#include <stdlib.h>
#include <modbus/modbus-tcp.h>
#include <thread>
#include <readline/readline.h>
#include <readline/history.h>
#include <MR_JE.h>
#include <mov.h>

using namespace std;

modbus_t *J1, *J2;
//modbus_t *J3, *J4; servos ainda nao existentes
//uint16_t wr_reg[16], r_reg[16]; //write register and read register respectivelly

int PARSE (char *comm);

int connectAll( modbus_t *J1, modbus_t *J2 );
int enableAll( modbus_t *J1, modbus_t *J2 );
int setMode( modbus_t *J1, modbus_t *J2, char mode);
int offAll( modbus_t *J1, modbus_t *J2 );
//int positioning_profile( modbus_t *J1, modbus_t *J2 );
int positioning_pt( modbus_t *J1, modbus_t *J2, int point );

/**
 * Separa a string em tokens. A primeira string é o comando e as demais os parametros
 * cada funcao trata do seu parametro
 */
int tokenString	( char comm[NUM_STRINGS][MAX_LENGTH], char *parse );

/**
 *
 */
int home_servo	( char comm[NUM_STRINGS][MAX_LENGTH], int size);
int set_mode	( char comm[NUM_STRINGS][MAX_LENGTH], int size);
int movept	( char comm[NUM_STRINGS][MAX_LENGTH], int size);
int servo_status	( char comm[NUM_STRINGS][MAX_LENGTH], int size);

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
	int ps;
	char *comm;

	J1 = modbus_new_tcp("10.8.0.201", 502);
	J2 = modbus_new_tcp("10.8.0.202", 502);

	do {
		do {
			comm = readline( "oScara> " );
		} while ( comm == NULL );

		//add_history( comm );
		ps = PARSE( comm );
		free( comm );
	} while ( ps != 0 );

	modbus_close( J2 );
	modbus_close( J1 );
	modbus_free( J2 );
	modbus_free( J1 );

	return 0;
}

/**
 * 	Aqui tem todas os comandos do CMD
 */
int PARSE( char *comm)
{
	int status;
	char arr[NUM_STRINGS][MAX_LENGTH] = {""};

	int size = tokenString( arr, comm );

	if(!strcmp(arr[0], "exit")) //sair
	{
		printf("Exit\n");
		return 0;
	}
	else if( !strcmp( arr[0], "on")) //servo on
	{
		printf("Servo On\n");
		status = enableAll(J1, J2);
		return 1;
	}
	else if( !strcmp( arr[0], "connect")) //conectar no modbus
	{
		printf("Connect servos\n");
		status = connectAll(J1, J2);
		return status;
	}
	else if( !strcmp( arr[0], "off")) //realiza servo off
	{
		status = offAll(J1, J2);
		printf("Servo off\n");
		return status;
	}
	else if( !strcmp( arr[0], "pshow"))
	{
		show_points( arr, size);
		return (1);
	}
	else if( !strcmp( arr[0], "psave") )
	{
		save_point( arr, size);
		return ( 1 );
	}
	else if( !strcmp( arr[0], "home"))
	{
		status = home_servo(arr , size);
		return status;
	}
	else if( !strcmp( arr[0], "mode"))
	{
		status = set_mode( arr, size);
		return status;
	}
	else if( !strcmp( arr[0], "pt"))
	{
		status = movept( arr, size);
		return status;
	}
	else if( !strcmp( arr[0], "status"))
	{
		status = servo_status( arr, size);
		return status;
	}
	else if( !strcmp( arr[0], "mov"))
	{
		status = mov( arr, size );
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
	int status = 0;
	status = servo_on( J1 );
	status = status << 1;
	status = servo_on( J2 );

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
	set_mode(J2, mode);
	//set_mode(J3, mode);
	//set_mode(J4, mode);
	this_thread::sleep_for(500ms); //wait half second before continue
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
	/*if( pt_move(J1, point) == 1)
	{
		printf("Movendo servo 1...\n");
	}*/
	if( pt_move(J2, point) == 1 )
	{
		printf("Movendo servo 2...\n");
	}

	return 1;
}

int tokenString(char comm[NUM_STRINGS][MAX_LENGTH], char * parse)
{
	int index = 0;

	char * token = strtok(parse, " ");

	while( token != NULL )
	{
		strcpy( comm[index], token );
        token = strtok(NULL, " ");
        index++;
    }

	return index;
}

int home_servo	( char comm[NUM_STRINGS][MAX_LENGTH], int size)
{
	//Verifica a junta do robo, nao pode ser vazio pela variavel size. Nao pode ser menor que 2
	int status = 0;
	if( size < 2)
	{
		printf("E necessario especificar a junta. Ex: J1, J2\n");
		return 1;
	}

	if( !strcmp( "J1", comm[1] ) )
	{
		status = home(J1);
		if( status == -1 ) printf("Home nao realziado em %s\n", comm[1]);
		else if( status == 1 ) printf("Home done\n");
		return 1;
	}else if( !strcmp( "J2", comm[1] ))
	{
		status = home(J2);
		if( status == -1 ) printf("Home nao realizado em %s\n", comm[1]);
		else if( status == 1) printf("Home done\n");
		return 1;
	}else if( !strcmp( "J3", comm[1] ))
	{
		printf("J3 não presente no momento\n");
		return 1;
	}else if( !strcmp( "J4", comm[1] ))
	{
		printf("J4 não presente no momento\n");
		return 1;
	}else
	{
		printf("E necessario especificar a junta. Ex: J1, J2\n");
		return 1;
	}
}

int set_mode	( char comm[NUM_STRINGS][MAX_LENGTH], int size)
{
	int status = 0;
	if( size < 2)
	{
		printf("Parametros insuficientes\n");
		return 1;
	}

	if( !strcmp( comm[1], "pt")){
		status = setMode(J1, J2, MR_POINT_TABLE_MODE);
		return 1;
	}else if( !strcmp( comm[1], "hm"))
	{
		status = setMode( J1, J2, MR_HOME_MODE);
		return 1;
	}else if( !strcmp( comm[1], "pm"))
	{
		status = setMode(J1, J2, MR_PROFILE_POSITION_MODE);
		return 1;
	}else
		printf("Opcao de modo invalido\n");

	return 1;
}

int movept	( char comm[NUM_STRINGS][MAX_LENGTH], int size)
{
	int status = 0;
	long point;
	char *ptr;
	char stringNumber[20];

	strcpy(stringNumber, comm[1]);

	if( size < 2)
	{
		printf("Parametros insuficientes\n");
		return 1;
	}

	point = strtol(stringNumber, &ptr, 10);

	if( (point < 1) || (point > 250) ){
		printf("Parametro precisa estar entre 0 e 250\n");
		return 1;
	}

	positioning_pt(J1, J2, (int)point );

	return 1;
}

int servo_status	( char comm[NUM_STRINGS][MAX_LENGTH], int size)
{
	uint16_t status;
	char mode;

	if( modbus_read_registers(J1, MR_STATUS_WORD, 1, &status) == -1)
		printf("Nao foi pessivel ler estado de J1\n");
	else printf("J1 status: 0x%.X\n", status);

	if( modbus_read_registers(J2, MR_STATUS_WORD, 1, &status) == -1)
		printf("Nao foi pessivel ler estado de J2\n");
	else
	{
		printf("J2 status: 0x%.X\n", status);
		get_mode(J2, &mode);
		printf("Mode %d\n", mode);
	}

	return 1;
}
/**
 *
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
