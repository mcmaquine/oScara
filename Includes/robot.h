/**
 * All structs are here, even those used on MR_JE.h header
 */

#ifndef ROBOT_H
#define ROBOT_H

#include <stdlib.h>
#include <stdint.h>

#define L1 255
#define L2 195

#define MAX_LENGTH 20
#define NUM_STRINGS 5


/*
 * see MR-JE-_C Network manual p. 84 (7 - 5)
 */
typedef struct pt_data
{
	uint16_t n_entries;	//sub index 0 Number of entries
	int point_data;		//sub index 1 Position data
	int speed;			//sub index 2
	int acceleration;	//sub index 3
	int deceleration;	//sub index 4
	int dwell;			//sub index 5
	int aux;			//sub index 6
	int mcode;			//sub index 7
}pt;

typedef struct pp_data //profile mode positioning data
{
	int target_position;
	uint32_t speed;
	uint32_t accel;
	uint32_t decel;
	int16_t profile = -1; //only mode supported on MR-JE-20C
}pp;

typedef struct pv_data //profile mode velocity data
{
	int16_t torque;
	uint16_t slope;
	int16_t profile;
}pv;

typedef struct tq_data //profile mode torque data
{

}tq;

typedef struct points
{
	char name[15];
	int J1;
	int J2;
	int J3;
	int J4;
}point;


#endif
