
#ifndef MR_JE_H
#define MR_JE_H

#include <modbus/modbus-tcp.h>

//Indexes (modbus addresses)
#define MR_ERROR_CODE				0x603F
#define MR_CONTROL_WORD				0x6040
#define	MR_STATUS_WORD				0x6041
#define MR_MODE_OF_OPERATION		0x6060
#define MR_MODES_OPERATION_DISPLAY	0x6061
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
#define MR_HOME_COMPLETED		0x1400
#define MR_HOME_ERROR_SPEED 	0x2000
#define MR_HOME_ERROR_SPEED_0 	0x2400

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
	int status;
	uint16_t r_reg;

	//set home mode
	status = set_mode(servo, MR_HOME_MODE);

	if( status == -1 )
		return status;

	//issue home
	status = write_register(servo, MR_CONTROL_WORD, 0x1F);

	if( status == -1)
		return status;

	if( modbus_read_registers( servo, MR_CONTROL_WORD, 1, &r_reg) == -1)
		return -1;

	//Verifica se finalizou ok ou com erro.
	while(  ( ( r_reg & MR_HOME_COMPLETED ) == MR_HOME_COMPLETED ) ||
			( ( r_reg & MR_HOME_ERROR_SPEED) == MR_HOME_ERROR_SPEED ) ||
			( ( r_reg & MR_HOME_ERROR_SPEED_0) == MR_HOME_ERROR_SPEED_0 ) ||
			( ( r_reg & MR_HOME_INTERRUPTED) == MR_HOME_INTERRUPTED ) )
	{
		status = modbus_read_registers( servo, MR_CONTROL_WORD, 1, &r_reg);

		if( status == -1 )
			break;
	}

	if (status == -1) return -1;

	if ( (r_reg & MR_HOME_COMPLETED ) == MR_HOME_COMPLETED )
		return 1;
	else if( ((r_reg & MR_HOME_ERROR_SPEED ) == MR_HOME_ERROR_SPEED) ||
			((r_reg & MR_HOME_ERROR_SPEED_0 ) == MR_HOME_ERROR_SPEED_0) ||
			((r_reg & MR_HOME_INTERRUPTED ) == MR_HOME_INTERRUPTED )) return 0;
	return 0;
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

#endif
