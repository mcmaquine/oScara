#ifndef CINEMATICS_H
#define CINEMATICS_H

#include <math.h>

double isZero( double _zero, double _TOL);
double x_direct_cinematic( double _L1, double _L2, double _theta1, double _theta2);
double y_direct_cinematic( double _L1, double _L2, double _theta1, double _theta2);
double inverse_cinematic( double _L1, double _L2, double _x, double _y);
double inverse_theta2( double _L1, double _L2, double _x, double _y);
double inverse_theta1( double _L1, double _L2, double _x, double _y);

double isZero( double _zero, double _TOL)
{
	double  n = fabs( _zero - _TOL );

	if ( n <= _TOL ) return 0.0;
	else return _zero;
}

double x_direct_cinematic( double _L1, double _L2, double _theta1, double _theta2)
{
	return _L1*cos( _theta1 * M_PI/180.0) + _L2*sin( (_theta1 + _theta2)/M_PI/180.0 );
}

double y_direct_cinematic( double _L1, double _L2, double _theta1, double _theta2)
{
	return _L1*sin(_theta1*M_PI/180.0) + _L2*sin( (_theta1 + _theta2) * M_PI/180.0 );
}

double inverse_theta2( double _L1, double _L2, double _x, double _y)
{
	return acosf( (_x*_x + _y*_y - (_L1*_L1 + _L2*_L2) ) / (2 * _L1 * _L2 ));
}

double inverse_theta1( double _L1, double _L2, double _x, double _y)
{
	double theta2 = inverse_theta2( _L1, _L2, _x, _y);
	double theta1;

	if( isZero(_x, 0.001) == 0.0 ) return 0.0;

	theta1 = atan2( _y, _x) - atan2 ( _L2 * sin (theta2), _L2 + _L2 * cos (theta2 ) );

	return isZero( theta1, 1e-3);
}

#endif
