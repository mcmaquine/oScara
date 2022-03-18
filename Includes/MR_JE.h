
#ifndef MR_JE_H
#define MR_JE_H

#include <modbus/modbus-tcp.h>

//Indexes (modbus addresses)
#define MR_POINT_TABLE_OFFSET		0x2801	//point 1 address
#define MR_TARGET_POINT_TABLE		0x2D60
#define MR_ERROR_CODE				0x603F
#define MR_CONTROL_WORD				0x6040
#define	MR_STATUS_WORD				0x6041
#define MR_MODE_OF_OPERATION		0x6060
#define MR_MODES_OPERATION_DISPLAY	0x6061
#define MR_POSITION_ACTUAL_VALUE	0x6064
#define MR_HOMING_METHOD			0x6098

//words
#define	MR_SWITCHED_ON				0x7
#define MR_READY_TO_SWITCH_ON		0x1
#define MR_WARNING					0x80

///homming method
#define MR_NO_HOMMING	 0
#define MR_METHOD_3		 3
#define MR_METHOD_35	35
#define MR_METHOD_37	37

//modes of operation
#define MR_NO_MODE					0
#define MR_PROFILE_POSITION_MODE	1
#define	MR_PROFILE_VELOCITY_MODE	2
#define MR_PROFILE_TORQUE_MODE		3
#define MR_HOME_MODE				6
#define	MR_POSITION_CONTROL_MODE	-20
#define MR_SPEED_CONTROL_MODE		-21
#define	MR_TORQUE_CONTROL_MODE		-22
#define MR_JOG_MODE					-100
#define MR_POINT_TABLE_MODE			-101
#define MR_INDEXER_MODE				-103

//homing states
#define MR_HOME_INTERRUPTED		0x400
#define MR_ILA					0x800
#define MR_HOME_COMPLETED		0x1400 //bit 10 e 12
#define MR_HOME_ERROR_SPEED 	0x2000 //bit 13
#define MR_HOME_ERROR_SPEED_0 	0x2400 //bit 10 e 13

//bits definition
#define BIT_0	0x0001
#define BIT_1	0x0002
#define	BIT_2	0x0004
#define BIT_3	0x0008
#define BIT_4	0x0010
#define BIT_5	0x0020
#define BIT_6   0x0040
#define BIT_7   0x0080
#define BIT_8   0x0100
#define BIT_9   0x0200
#define BIT_10  0x0400
#define BIT_11  0x0800
#define BIT_12  0x1000
#define BIT_13  0x2000
#define BIT_14  0x4000
#define BIT_15  0x8000
#define NYBLE_0   0x000F
#define NYBLE_1   0x00F0
#define NYBLE_2   0x0F00
#define NYBLE_3   0xF000

int servo_on		( modbus_t *J );
int servo_off		( modbus_t *J );
int is_servo_on		( modbus_t *J ); //return 1 is it is in SERVO ON, o SERVO OFF, -1 not possible to determine.
int home_servo		( modbus_t *J );
int set_home_method	( modbus_t *J, int8_t mode );
int get_home_method	( modbus_t *J, char *mode );
int start_home		( modbus_t *J ); //start homing
int is_EM2_on		( modbus_t *J ); //check if servo is EM2 activated (emergency button)
int home			( modbus_t *J );
int set_mode		( modbus_t *J, char mode);
int get_mode		( modbus_t *J, char *mode );
int position_actual_value( modbus_t *J);
int pt_move			( modbus_t *J, uint16_t point );
int get_pt_data		( modbus_t *J, uint16_t point, pt *data);
int set_pt_data		( modbus_t *J, uint16_t point, pt *data);
int set_pp_data		( modbus_t *J, pt *data);


void setBit(uint16_t *word, uint16_t bits);
void resetBit(uint16_t *word, uint16_t bits);

//Utiliza a função de escrita de multiplos regisradores para escrever 1 somente
int write_register( modbus_t *J, int addr, uint16_t data );

int servo_on( modbus_t *J )
{
	uint16_t w_reg;

	setBit(&w_reg, NYBLE_0 );
	return write_register(J, MR_CONTROL_WORD, w_reg);
}

int servo_off( modbus_t *J )
{
	uint16_t w_reg;

	resetBit(&w_reg, NYBLE_0);
	return write_register(J, MR_CONTROL_WORD, w_reg);
}

int is_servo_on( modbus_t *J )
{
	uint16_t r_reg;
	int status = modbus_read_registers(J, MR_STATUS_WORD, 1, &r_reg);
	if ( status == -1 )
	{
		return status;
	}
	else
	{
		if( ( r_reg & MR_SWITCHED_ON ) == MR_SWITCHED_ON )
			return 1;
		else
			return 0;
	}
}

int set_home_method( modbus_t *J, int8_t mode )
{
	int status;
	uint16_t w_reg = mode;

	status = write_register( J , MR_HOMING_METHOD, w_reg);

	return status;
}

int get_home_method( modbus_t *J, char *mode )
{
	int status;
	uint16_t r_reg;

	status = modbus_read_registers(J, MR_HOMING_METHOD, 1, &r_reg);

	if( status != -1)
		*mode = MODBUS_GET_LOW_BYTE( r_reg );

	return status;
}
/**
 * Verify if servo is in warning, if so, disable servo on.
 * @param servo modbus_t structure pointer of objects comunication
 * @return -1 if not possible to read status, 0 if not on warning, 1 if its in warning
 */
int is_EM2_on( modbus_t *J )
{
	int status;
	uint16_t r_reg;

	status = modbus_read_registers( J , MR_STATUS_WORD, 1, &r_reg);

	if( status == -1 )
		return status;
	else
	{
		//verify if warning bit is set
		if( ( r_reg & MR_WARNING) == MR_WARNING )
		{
			//now verify if it is on servo off
			if( (r_reg & MR_READY_TO_SWITCH_ON ) == MR_READY_TO_SWITCH_ON )
			{
				//put in servo off
				write_register(J, MR_CONTROL_WORD, 0x0);
				return 1;
			}else
				return 0;
		}
		else
			return 0;
	}
}

/**
 * Realiza home do servo
 */
int home( modbus_t *J)
{
	int status = 0, homing = 0, retorno = 0;
	uint16_t r_reg;

	//set home mode

	status = set_mode(J, MR_HOME_MODE);

	if( status == -1 )
		return status;

	//issue home
	status = modbus_read_registers(J, MR_CONTROL_WORD, 1, &r_reg);

	setBit(&r_reg, BIT_4 ); //bir 4 start homing

	status = write_register(J, MR_CONTROL_WORD, r_reg );

	if( status == -1)
		return status;

	if( modbus_read_registers( J, MR_STATUS_WORD, 1, &r_reg) == -1)
		return -1;

	homing = 1;
	//Verifica se finalizou ok ou com erro.
	while(  homing )
	{
		//Verifica se terminou o home
		if( ( r_reg & MR_HOME_COMPLETED ) == MR_HOME_COMPLETED )
		{
			homing = 0;
			retorno = 1;
		}
		else if ( ( ( r_reg & MR_HOME_ERROR_SPEED_0) == MR_HOME_ERROR_SPEED_0 ) ||
				( ( r_reg & MR_HOME_ERROR_SPEED) == MR_HOME_ERROR_SPEED ) 		||
				( ( r_reg & MR_HOME_INTERRUPTED) == MR_HOME_INTERRUPTED ) )
		{
			homing = 0;
			retorno = 0; //home nao realizado
		}

		status = modbus_read_registers( J, MR_STATUS_WORD, 1, &r_reg);
		if( status == -1 )
			break;
		printf("STATUS WORD %X\n", r_reg);
	}
	status = modbus_read_registers(J, MR_CONTROL_WORD, 1, &r_reg);
	resetBit(&r_reg, BIT_4); //reset bit 4
	status = write_register(J, MR_CONTROL_WORD, r_reg );
	if (status == -1) return -1;

	return retorno;
}

/**
 * Set mode of operation, if return -1 means that mode of operation was no set
 * @param J modbus_t struct object
 * @param mode	mode of operation (Eg. MR_HOME_MODE)
 * @return mode operation set, if -1 mode was note set
 */
int set_mode( modbus_t *J, char mode)
{
	int status;
	uint16_t w_reg = mode;

	status = write_register(J, MR_MODE_OF_OPERATION, w_reg);

	return status;
}

/*
 * Get servo current mode
 */
int get_mode( modbus_t *J, char *mode)
{
	int status;
	uint16_t r_reg;

	status = modbus_read_registers(J, MR_MODES_OPERATION_DISPLAY, 1, &r_reg);

	if( status == -1 )
	{
		return status;
	}
	else
	{
		*mode = MODBUS_GET_LOW_BYTE( r_reg );
		return 1;
	}
}

/**
 * utiliza a função write_registers para escrever apenas 1 word
 */
int write_register( modbus_t *J, int addr, uint16_t data)
{
	int status = 0;

	status = modbus_write_registers(J, addr, 1, &data );

	return status;
}

/**
 * Return the actula servo position
 * @param modbus_t	Communication struct
 */
int position_actual_value( modbus_t *J )
{
	uint16_t r_reg[3];
	int stat, position;

	stat = modbus_read_registers(J, MR_POSITION_ACTUAL_VALUE, 2, r_reg);
	if( stat == -1) return -1;

	position = r_reg[1];
	position = position << 16;
	position = position | r_reg[0];
	return position;
}
/*
 * Do a move registered on a Point_Table
 */
int pt_move( modbus_t *J, uint16_t point)
{
	uint16_t r_reg;
	char mode;

	//point can not be zero
	if( point == 0 ) return 0;

	//check if servo is Enable
	if( is_servo_on(J) != 1 ) return 0;

	//check if servo is point table mode
	if( get_mode(J, &mode) == -1) return -1;
	if( mode != MR_POINT_TABLE_MODE ) return 0;

	int stat = modbus_read_registers(J, MR_CONTROL_WORD, 1, &r_reg);

	if( stat == -1 ) return -1; //if not possible to read, return -1

	resetBit( &r_reg, BIT_4 ); //reset bit 4
	stat = write_register(J, MR_CONTROL_WORD, r_reg ); //write word with reseted bit 4

	stat = write_register(J, MR_TARGET_POINT_TABLE, point); //write position table number
	if( stat == -1) return -1;	//if not possible to write, return -1

	setBit(&r_reg, BIT_4 );		//set bit 4
	stat = write_register(J, MR_CONTROL_WORD, r_reg); //write word with reseted bit 4
	printf("ControlWord 0x%X\n", r_reg );

	if( stat == -1) return -1;

	return 1;
}

/**
 * Word which bits will be set. For more than one bit use bitwise | (OR), Ex. BIT_0 | BIT_1 | BIT_2 to set bits from 0 to 2.
 * @param *word word whom birs will be set
 * @param bits	bits to set in word
 */
void setBit(uint16_t *word, uint16_t bits)
{
	//just a single bitwise OR
	*word = *word | bits;
}

/**
 * Word which bits will be reset. For more than one bit use bitwise | (OR), Ex. BIT_0 | BIT_1 | BIT_2 to set bits from 0 to 2.
 * @param *word word whom birs will be reset
 * @param bits	bits to reset in word
 */
void resetBit( uint16_t *word, uint16_t bits )
{
	//bitwise bit inversion
    uint16_t notBits = ~bits;
    
    //bitwise AND to reset bits
    *word = *word & notBits;
}

/*
 * Get data from a point table previously recorded alloc a pointer pt
 */
int get_pt_data( modbus_t *J, uint16_t point, pt **data)
{
	const int size = 15;
	int status;
	uint16_t r_reg[size];

	if( point < 1 || point > 255 ) return 0; // only numbers between 1 and 255 (inclusive)

	status = modbus_read_registers(J, MR_POINT_TABLE_OFFSET + point - 1, size, r_reg);
	if( status == -1 ) return -1;

	*data = (pt *)malloc( sizeof (pt) );

	(*data)->n_entries = r_reg[0];

	(*data)->point_data = r_reg[2]; //most sgnificant word
	(*data)->point_data = (*data)->point_data << 16;
	(*data)->point_data = (*data)->point_data | r_reg[1]; //least significant word

	(*data)->speed = r_reg[4];
	(*data)->speed = (*data)->speed << 16;
	(*data)->speed = (*data)->speed | r_reg[3];

	(*data)->acceleration = r_reg[6];
	(*data)->acceleration = (*data)->acceleration << 16;
	(*data)->acceleration = (*data)->acceleration | r_reg[5];

	(*data)->deceleration = r_reg[8];
	(*data)->deceleration = (*data)->deceleration << 16;
	(*data)->deceleration = (*data)->deceleration | r_reg[7];

	(*data)->dwell = r_reg[10];
	(*data)->dwell = (*data)->dwell << 16;
	(*data)->dwell = (*data)->dwell | r_reg[9];

	(*data)->aux = r_reg[12];
	(*data)->aux = (*data)->aux << 16;
	(*data)->aux = (*data)->aux | r_reg[11];

	(*data)->mcode = r_reg[14];
	(*data)->mcode = (*data)->mcode << 16;
	(*data)->mcode = (*data)->mcode | r_reg[13];

	return 1;
}

int set_pt_data( modbus_t *J, uint16_t point, pt *data )
{
	const int size = 15;
	int status;
	uint16_t reg[size];

	reg[1] = data->point_data & 0x0000FFFF;
	reg[2] = data->point_data >> 16;

	reg[3] = data->speed & 0x0000FFFF;
	reg[4] = data->speed >> 16;

	reg[5] = data->acceleration & 0x0000FFFF;
	reg[6] = data->acceleration >> 16;

	reg[7] = data->deceleration & 0x0000FFFF;
	reg[8] = data->deceleration >> 16;

	reg[9] = data->dwell & 0x0000FFFF;
	reg[10]= data->dwell >> 16;

	reg[11]= data->aux & 0x0000FFFF;
	reg[12]= data->aux >> 16;

	reg[13]= data->mcode & 0x0000FFFF;
	reg[14]= data->mcode >> 16;

	status = modbus_write_registers(J, MR_POINT_TABLE_OFFSET + point - 1, size, reg);

	if( status == -1) return status;

	return 1;
}

#endif
