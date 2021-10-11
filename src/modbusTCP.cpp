//============================================================================
// Name        : modbusTCP.cpp
// Author      : Maquine
// Version     :
// Copyright   : A FPF TECH project
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <modbus/modbus-tcp.h>
#include <thread>

using namespace std;

modbus_t *mb;
uint8_t bits[16];//read buffer
uint8_t coil[16];//write buffer
//uint16_t tab_reg[32];


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

int program()
{
	//if enters here, Factory I/O is running
	modbus_read_input_bits(mb, 0, 16, bits);
	coil[0] = 1;
	modbus_write_bits(mb, 16, 16, coil);

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

		modbus_write_bits(mb, 16, 16, coil);
		modbus_read_input_bits(mb, 0, 16, bits);

		if( bits[0] == 0 ) {
			coil[0] = 0;
			coil[1] = 0;
			modbus_write_bits(mb, 16, 16, coil);
			printf("factory I/O paused\n");
		}
	}while( bits[0] );

	return 1;
}

int main() {

	thread read;

	mb = modbus_new_tcp("10.2.0.27", 502);

	//Verifica se o objeto modbus foi criado
	if (mb == NULL) {
	    printf("Unable to allocate libmodbus context\n");
	    return -1;
	}

	//Verifica se a comunicacao foi realizada
	if( modbus_connect( mb ) == -1 )
	{
		printf("Connection failed %s\n", modbus_strerror(errno));
		return -1;
	}

	//Read First input to check if FACTIRY I/O is Running
	modbus_read_input_bits(mb, 0, 1, bits);

	if( bits[0] )
	{
		//Running lock in while
		printf("Factory I/O running\n");
		program();
	}else
		printf("Factory I/O not running\n");

	modbus_close( mb );
	modbus_free( mb );

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
 */
