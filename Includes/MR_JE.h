
#ifndef MR_JE_H
#define MR_JE_H

#include <modbus/modbus-tcp.h>

//Indexes (modbus addresses)
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
#define MR_NO_MODE			0
#define MR_PROFILE_POSTION	1
#define	MR_PROFILE_VELOCITY 2
#define MR_PROFILE_TORQUE	3
#define MR_HOME_MODE		6
#define	MR_POSITION_CONTROL -20
#define MR_SPEED_CONTROL	-21
#define	MR_TORQUE_CONTROL	-22
#define MR_JOG_MODE			-100
#define MR_POINT_TABLE		-101
#define MR_INDEXER_MODE		-103

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

int servo_on		( modbus_t *servo );
int servo_off		( modbus_t *servo );
int is_servo_on		( modbus_t *servo ); //return 1 is it is in SERVO ON, o SERVO OFF, -1 not possible to determine.
int home_servo		( modbus_t *servo );
int set_home_method	( modbus_t *servo, int8_t mode );
int get_home_method	( modbus_t *servo, char *mode );
int start_home		( modbus_t *servo ); //start homing
int is_EM2_on		( modbus_t *servo ); //check if servo is EM2 activated (emergency button)
int home			( modbus_t *servo );
int set_mode		( modbus_t *servo, char mode);
int get_mode		( modbus_t *servo, char *mode );
int position_actual_value( modbus_t *servo);

void setBit(uint16_t *word, uint16_t bits);
void resetBit(uint16_t *word, uint16_t bits);

//Utiliza a função de escrita de multiplos regisradores para escrever 1 somente
int write_register( modbus_t *servo, int addr, uint16_t data );

int servo_on( modbus_t *servo )
{
	return write_register(servo, MR_CONTROL_WORD, 0x0F);
}

int servo_off( modbus_t *servo )
{
	return write_register(servo, MR_CONTROL_WORD, 0x00);
}

int is_servo_on( modbus_t *servo )
{
	uint16_t r_reg;
	int status = modbus_read_registers(servo, MR_STATUS_WORD, 1, &r_reg);
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

int set_home_method( modbus_t *servo, int8_t mode )
{
	int status;
	uint16_t w_reg = mode;

	status = write_register( servo , MR_HOMING_METHOD, w_reg);

	return status;
}

int get_home_method( modbus_t *servo, char *mode )
{
	int status;
	uint16_t r_reg;

	status = modbus_read_registers(servo, MR_HOMING_METHOD, 1, &r_reg);

	if( status != -1)
		*mode = MODBUS_GET_LOW_BYTE( r_reg );

	return status;
}
/**
 * Verify if servo is in warning, if so, disable servo on.
 * @param servo modbus_t structure pointer of objects comunication
 * @return -1 if not possible to read status, 0 if not on warning, 1 if its in warning
 */
int is_EM2_on( modbus_t *servo )
{
	int status;
	uint16_t r_reg;

	status = modbus_read_registers( servo , MR_STATUS_WORD, 1, &r_reg);

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
				write_register(servo, MR_CONTROL_WORD, 0x0);
				return 1;
			}else
				return 0;
		}
		else
			return 0;
	}
}

/**
 * Metdo esta incompleto
 */
int home( modbus_t *servo)
{
	int status, homing, retorno;
	uint16_t r_reg;

	//set home mode

	status = set_mode(servo, MR_HOME_MODE);

	if( status == -1 )
		return status;

	//issue home
	status = modbus_read_registers(servo, MR_CONTROL_WORD, 1, &r_reg);

	setBit(&r_reg, BIT_4 ); //bir 4 start homing

	status = write_register(servo, MR_CONTROL_WORD, r_reg );

	if( status == -1)
		return status;

	if( modbus_read_registers( servo, MR_STATUS_WORD, 1, &r_reg) == -1)
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

		status = modbus_read_registers( servo, MR_STATUS_WORD, 1, &r_reg);
		if( status == -1 )
			break;
		printf("STATUS WORD %X\n", r_reg);
	}
	status = modbus_read_registers(servo, MR_CONTROL_WORD, 1, &r_reg);
	resetBit(&r_reg, BIT_4); //reset bit 4
	status = write_register(servo, MR_CONTROL_WORD, r_reg );
	if (status == -1) return -1;

	return retorno;
}

/**
 * Set mode of operation, if return -1 means that mode of operation was no set
 * @param servo modbus_t struct object
 * @param mode	mode of operation (Eg. MR_HOME_MODE)
 * @return mode operation set, if -1 mode was note set
 */
int set_mode( modbus_t *servo, char mode)
{
	int status;
	uint16_t w_reg = mode;

	status = write_register(servo, MR_MODE_OF_OPERATION, w_reg);

	return status;
}

int get_mode( modbus_t *servo, char *mode)
{
	int status;
	uint16_t r_reg;

	status = modbus_read_registers(servo, MR_MODES_OPERATION_DISPLAY, 1, &r_reg);

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
int write_register( modbus_t *servo, int addr, uint16_t data)
{
	int status = 0;

	status = modbus_write_registers(servo, addr, 1, &data );

	return status;
}

/**
 * Return the actula servo position
 * @param modbus_t	Communication struct
 */
int position_actual_value( modbus_t *servo )
{
	uint16_t r_reg[3];
	int stat, position;

	stat = modbus_read_registers(servo, MR_POSITION_ACTUAL_VALUE, 2, r_reg);
	if( stat == -1) return -1;

	position = r_reg[1];
	position = position << 16;
	position = position | r_reg[0];
	return position;
}

int move_point( modbus_t *servo, uint16_t point)
{
	uint16_t r_reg;

	if( is_servo_on(servo) != 1 ) return 0;

	int stat = modbus_read_registers(servo, MR_CONTROL_WORD, 1, &r_reg);

	if( stat == -1 ) return -1; //if not possible to read, return -1

	resetBit( &r_reg, BIT_4 ); //reset bit 4
	stat = write_register(servo, MR_CONTROL_WORD, r_reg ); //write word with reseted bit 4

	stat = write_register(servo, MR_TARGET_POINT_TABLE, point); //write position table number
	if( stat == -1) return -1;	//if not possible to write, return -1

	setBit(&r_reg, BIT_4 );		//set bit 4
	stat = write_register(servo, MR_CONTROL_WORD, r_reg); //write word with reseted bit 4
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

#endif
