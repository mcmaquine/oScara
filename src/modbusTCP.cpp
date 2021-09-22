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
uint8_t bits[8];
uint16_t input;
//uint16_t tab_reg[32];


uint16_t convert_raw_data( int nb, uint8_t *bits)
{
	uint16_t data = 0;

	for( int i = 0; i < nb; i++)
	{
		data += bits[i] << i;
	}

	return data;
}

int main() {

	thread read;

	int read_status;

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

	/*Read 1 bit*/

	cout << "Modbus Test" << endl; // prints "Hello Modbus"
	read_status =  modbus_read_input_bits(mb, 0, 8, bits);
	input = convert_raw_data(8, bits);

	cout << "Modbus read inputs: " << read_status << endl;

	printf("Inputs Hex: 0x%X\n", input);

	for (int var = 0; var < 8; ++var) {
		printf("Input[%d]: %d\n", var, bits[var]);
	}

	modbus_close( mb );
	modbus_free( mb );

	//this_thread::sleep_for( chrono::milliseconds( 980 ) );
	cout	<<	"Exit"	<<	endl;
	return 0;
}
