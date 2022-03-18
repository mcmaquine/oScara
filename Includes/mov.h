/*
 * mov.h
 *
 *  Created on: 18 de mar de 2022
 *      Author: maquine
 */

#ifndef INCLUDES_MOV_H_
#define INCLUDES_MOV_H_
#include <stdlib.h>
#include <MR_JE.h>
#include <robot.h>
#include <modbus/modbus-tcp.h>

extern modbus_t *J1, *J2;
/**
 * 1 - Ler o ponto do arquivo e armazenar na estrutura
 * 2 - Mudar modo do servo para gravar no point table
 * 3 - Escrever no point table 1 com opcao de nao continuacao para o point table seguinte
 */
int save_point	(	char comm[NUM_STRINGS][MAX_LENGTH], int size	);
int show_points ( 	char comm[NUM_STRINGS][MAX_LENGTH], int size	);
int mov	( char comm[NUM_STRINGS][MAX_LENGTH], int size);
int write_point ();
int verify_point_on_file();
int parse_point();
int isPoint( char *str, point *joints );

int save_point	(	char comm[NUM_STRINGS][MAX_LENGTH], int size	)
{
	FILE *fptr;
	point p;

	//Verifica o nome do ponto, nao pode ser vazio pela variavel size. Nao pode ser menor que 2
	if( size < 2)
	{
		printf("E necessario nomear o ponto. Ex: p1, ponto1, pick, place\n");
		return 0;
	}

	//ler as posições dos servos
	strcpy( p.name, comm[1] );
	p.J1 = position_actual_value( J1 );
	p.J2 = position_actual_value( J2 );
	p.J3 = 0;
	p.J4 = 0;

	//abrir o arquivo de pontos.
	if ((fptr = fopen("points.bin","ab")) == NULL){
		printf("Error! opening file\n");
		// Program exits if the file pointer returns NULL.
		return(0);
	}

	//antes verificar se um ponto ja existe
	fseek( fptr, -sizeof(point), SEEK_END );
	fwrite( &p, sizeof(point), 1, fptr);
	fclose( fptr );

	return 1;
}

int show_points (char comm[NUM_STRINGS][MAX_LENGTH], int size)
{
	FILE *fptr;
	point p;
	int seek = 0;
	int control = 0;

	if(( fptr = fopen("points.bin", "rb")) == NULL ){
		printf("Nao foi possivel abrir tabela de pontos\n");
		return(0);
	}

	seek = fread(&p, sizeof(point), 1, fptr);
	while( seek != 0 || control > 10)
	{
		printf("Point name %s\t J1: %d\t J2: %d\t J3: %d\t J4: %d\n", p.name, p.J1, p.J2, p.J3, p.J4);
		control++;
		seek = fread(&p, sizeof(point), 1, fptr);
	}

	fclose( fptr );
	return 1;
}

int mov	( char comm[NUM_STRINGS][MAX_LENGTH], int size)
{

	int ispoint = 0;
	char point_name[MAX_LENGTH];
	point point_data;

	if( size < 2 )
	{
		printf("Precisa nomear um Ponto\n");
		return 0;
	}

	// verifica se o ponto existe
	ispoint = isPoint( comm[1], &point_data  );

	if( !ispoint )
	{
		printf("Ponto nao existe!");
		return 0;
	}

	//
}
/**
 * Verifica se o ponto existe no arquivo
 */
int isPoint( char *str, point *joints )
{
	FILE *fptr;
	point p;
	int seek = 0;
	int control = 0;
	int ispoint = 0;

	if(( fptr = fopen("points.bin", "rb")) == NULL ){
		printf("Nao foi possivel abrir tabela de pontos\n");
		return(0);
	}

	seek = fread(&p, sizeof(point), 1, fptr);
	while( seek != 0 || control > 10)
	{
		if( !strcmp( str, p.name) )
		{
			joints->J1 = p.J1;
			joints->J2 = p.J2;
			joints->J3 = p.J3;
			joints->J4 = p.J4;

			ispoint = 1;
		}
		control++;
		seek = fread(&p, sizeof(point), 1, fptr);
	}

	fclose( fptr );
	return ispoint;
}

#endif /* INCLUDES_MOV_H_ */
