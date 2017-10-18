#ifndef LIBHPF_H
#define LIBHPF_H

#include "utils.h"
#include "lib_be.hh"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <sys/time.h>

#ifdef UTILS_FLOAT_DATATYPE
	#define MAX_POTENTIAL 1.0f
	#define MIN_POTENTIAL 0.0f
	#define START_POTENTIAL 0.995f
#else 
	#define MAX_POTENTIAL 1.0
	#define MIN_POTENTIAL 0.0
	#define START_POTENTIAL 0.995
#endif

#define FREE_CELL 0
#define OCCUPIED_CELL 1

//#define HPFE_CLEAR_WITH_START_POTENTIAL



// {[GS,SOR][C, UP]}
// {[Gauss-Seidel, Successive Over-Relaxation][Centered, UP-Wind]}
// to future: parallel, other bvps, ...
enum HPFE_IterateModes{ HPFE_gs, HPFE_sor};
enum HPFE_IterateFlags{ HPFE_normal=0x00, HPFE_printErrors=0x01, HPFE_printTimes=0x02, HPFE_printTimeAtEnd=0x04 };
//enum HPFE_printGridModes{ HPFE_column, HPFE_row }; //to future use

typedef struct{
	BE *be__;

#ifdef UTILS_FLOAT_DATATYPE
	float *potentialGrid__;
#else 
	double *potentialGrid__;
#endif

	double wSor__;
	int *occupancyGrid__;
	int gridWidth__, gridHeight__, gridAllocatedSize__;
	
	//to future use
//	void *cudaStuff__; //handle something
//	unsigned int sizeOfCudaStuff__;// size of something handled
	
	} HPFE; //Harmonic Potential Field Environment


/*
 _     ___ ____    _____                 _   _                 
| |   |_ _| __ )  |  ___|   _ _ __   ___| |_(_) ___  _ __  ___ 
| |    | ||  _ \  | |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __|
| |___ | || |_) | |  _|| |_| | | | | (__| |_| | (_) | | | \__ \
|_____|___|____/  |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
*/



HPFE* HPFE_new(int maxNumberOfObstacle, int maxNumberOfEnemyRobot, int maxNumberOfOwnRobot, int maxNumberOfGoal, int width, int height);

void HPFE_delete( HPFE *e );

int HPFE_clearAllLists( HPFE *e );



int HPFE_getGridWidth( HPFE *e );

int HPFE_setGridWidth( HPFE *e, int width );

int HPFE_getGridHeight( HPFE *e );

int HPFE_setGridHeight( HPFE *e, int height );


//int HPFE_setCudaStuff( HPFE *e, void *cudaStuff, unsigned int sizeOfCudaStuff );

//void* HPFE_getCudaStuff( HPFE *e );

//int HPFE_getCudaStuffSize( HPFE *e );



#ifdef UTILS_FLOAT_DATATYPE
//Ainda nao implementadas
//int HPFE_getPotentialGrid( HPFE *e, int *width, int *height, float *dest);
//int HPFE_setPotentialGrid( HPFE *e, int *width, int *height, float *source);

int HPFE_setPotential( HPFE *e, int x, int y, float potential);

float HPFE_getPotential( HPFE *e, int x, int y);

int HPFE_setAllPotential( HPFE *e, float potential );

#else 
//Ainda nao implementadas
//int HPFE_getPotentialGrid( HPFE *e, int *width, int *height, double *dest); 
//int HPFE_setPotentialGrid( HPFE *e, int *width, int *height, double *source); 

int HPFE_setPotential( HPFE *e, int x, int y, double potential);

double HPFE_getPotential( HPFE *e, int x, int y);

int HPFE_setAllPotential( HPFE *e, double potential );

#endif

double HPFE_getWSor( HPFE *e );

int HPFE_setWSor( HPFE *e, double wSor);

//Ainda nao implementadas
//int HPFE_setOccupancyGrid( HPFE *e, int *width, int *height, int *source);
//int HPFE_getOccupancyGrid( HPFE *e, int *width, int *height, int *dest);

int HPFE_setOccupancy( HPFE *e, int x, int y, int occupancy);

int HPFE_getOccupancy( HPFE *e, int x, int y);

int HPFE_getGradient( HPFE *e, Pos *position, Pos *result);

int HPFE_getGradientDescent( HPFE *e, Pos *position, Pos *result);



int HPFE_setMaxNumberOfObstacle(HPFE *e,  int maxNumberOfObstacle, int list=0);

int HPFE_getMaxNumberOfObstacle(HPFE *e,  int list=0);

int HPFE_getNumberOfObstacle(HPFE *e, int list=0);

int HPFE_addObstacle( HPFE *e, Pos *newObstacle, int list=0 );

int HPFE_getObstacleAtIndex( HPFE *e, int index, Pos *result, int list=0 );

int HPFE_clearListOfObstacle( HPFE *e, int list=0 );



int HPFE_setMaxNumberOfEnemyRobot(HPFE *e,  int maxNumberOfEnemyRobot );

int HPFE_getMaxNumberOfEnemyRobot(HPFE *e);

int HPFE_getNumberOfEnemyRobot(HPFE *e);

int HPFE_addEnemyRobot( HPFE *e, Pos *newEnemyRobot);

int HPFE_getEnemyRobotAtIndex( HPFE *e, int index, Pos *result );

int HPFE_clearListOfEnemyRobot( HPFE *e );



int HPFE_setMaxNumberOfOwnRobot(HPFE *e,  int maxNumberOfOwnRobot );

int HPFE_getMaxNumberOfOwnRobot(HPFE *e);

int HPFE_getNumberOfOwnRobot(HPFE *e);

int HPFE_addOwnRobot( HPFE *e, Pos *newOwnRobot);

int HPFE_setMe( HPFE *e, Pos *newOwnRobot);

int HPFE_getMe( HPFE *e, Pos *result );

int HPFE_getOwnRobotAtIndex( HPFE *e, int index, Pos *result );

int HPFE_clearListOfOwnRobot( HPFE *e );



int HPFE_setMaxNumberOfGoal(HPFE *e,  int maxNumberGoal );

int HPFE_getMaxNumberOfGoal(HPFE *e);

int HPFE_getNumberOfGoal(HPFE *e);

int HPFE_addGoal( HPFE *e, Pos *newGoal);

int HPFE_getGoalAtIndex( HPFE *e, int index, Pos *result  );

int HPFE_clearListOfGoal( HPFE *e );


/*
 ___ _                 _                 
|_ _| |_ ___ _ __ __ _| |_ ___  _ __ ___ 
 | || __/ _ \ '__/ _` | __/ _ \| '__/ __|
 | || ||  __/ | | (_| | || (_) | |  \__ \
|___|\__\___|_|  \__,_|\__\___/|_|  |___/
*/


inline double HPFE_iteratorGS(HPFE *e );
inline double HPFE_iteratorSOR(HPFE *e );


/*
 ____        _                    
/ ___|  ___ | |_   _____ _ __ ___ 
\___ \ / _ \| \ \ / / _ \ '__/ __|
 ___) | (_) | |\ V /  __/ |  \__ \
|____/ \___/|_| \_/ \___|_|  |___/
*/

//[I, T, E]
//Limited by: {Iterations, Time, Error}
long int HPFE_solveI( HPFE *e, HPFE_IterateModes it, int itMax, HPFE_IterateFlags itFlags = HPFE_normal);
//long int HPFE_solveIT( HPFE *e, HPFE_IterateModes it, int itMax, int usecMax, HPFE_IterateFlags itFlags = HPFE_normal);
long int HPFE_solveIE( HPFE *e, HPFE_IterateModes it, int itMax, double errorMax, HPFE_IterateFlags itFlags = HPFE_normal);
long int HPFE_solveE( HPFE *e, HPFE_IterateModes it, double errorMax, HPFE_IterateFlags itFlags = HPFE_normal);
//long int HPFE_solveET( HPFE *e, HPFE_IterateModes it, double errorMax, int usecMax, int itMax, HPFE_IterateFlags itFlags = HPFE_normal);
//long int HPFE_solveT( HPFE *e, HPFE_IterateModes it, int usecMax, HPFE_IterateFlags itFlags = HPFE_normal);







/*
 ___       _ _              _____                 _   _                 
|_ _|_ __ | (_)_ __   ___  |  ___|   _ _ __   ___| |_(_) ___  _ __  ___ 
 | || '_ \| | | '_ \ / _ \ | |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __|
 | || | | | | | | | |  __/ |  _|| |_| | | | | (__| |_| | (_) | | | \__ \
|___|_| |_|_|_|_| |_|\___| |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
*/
//These are internal functions. Just for purposes of acceleration, they are inline and without safe check.
//They just wrap some command.
#ifdef UTILS_FLOAT_DATATYPE
inline void HPFE_setPotential_UNSAFE( HPFE *e, int x, int y, float potential){
	e->potentialGrid__[ (y*(e->gridWidth__)) + x  ] = potential;
	return;
}

inline float HPFE_getPotential_UNSAFE( HPFE *e, int x, int y){
	return( e->potentialGrid__[ (y*e->gridWidth__) + x  ] );
}

inline int HPFE_setPotentialP( HPFE *e, Pos *p, float potential){
	return( HPFE_setPotential( e, (int)(floor(p->x__+0.5)), (int)(floor(p->y__+0.5)), potential) );
}

inline float HPFE_getPotentialP( HPFE *e, Pos *p ){
	return( HPFE_getPotential( e, (int)(floor(p->x__+0.5)), (int)(floor(p->y__+0.5))) );
}

#else 
inline void HPFE_setPotential_UNSAFE( HPFE *e, int x, int y, double potential){
	e->potentialGrid__[ (y*(e->gridWidth__)) + x  ] = potential;
	return;
}

inline double HPFE_getPotential_UNSAFE( HPFE *e, int x, int y){
	return( e->potentialGrid__[ (y*e->gridWidth__) + x  ] );
}

inline int HPFE_setPotentialP( HPFE *e, Pos *p, double potential){
	return( HPFE_setPotential( e, (int)(floor(p->x__+0.5)), (int)(floor(p->y__+0.5)), potential) );
}

inline double HPFE_getPotentialP( HPFE *e, Pos *p ){
	return( HPFE_getPotential( e, (int)(floor(p->x__+0.5)), (int)(floor(p->y__+0.5))) );
}
#endif

inline int HPFE_setOccupancyP( HPFE *e, Pos *p, int occupancy){
	return( HPFE_setOccupancy( e, (int)(floor(p->x__+0.5)), (int)(floor(p->y__+0.5)), occupancy) );
}

inline int HPFE_getOccupancyP( HPFE *e, Pos *p ){
	return( HPFE_getOccupancy( e, (int)(floor(p->x__+0.5)), (int)(floor(p->y__+0.5))) );
}


inline Booleano HPFE_isValidPosition(HPFE *e, Pos *p){
	return(  ((p->x__<0) || (p->x__>=(e->gridWidth__)) || (p->y__<0) || (p->y__>=(e->gridHeight__))) ? u_false : u_true   );
}

inline void HPFE_posToInt(HPFE *e, Pos *p, int *x, int *y){
	*x = ( (p->x__) < 0 ? 0 : (int) floor( p->x__ + 0.5) );
	*x = ( (*x) >= e->gridWidth__ ? e->gridWidth__ -1 : (*x) );
	*y = ( (p->x__) < 0 ? 0 : (int) floor( p->y__ + 0.5) );
	*y = ( (*y) > e->gridHeight__ ? e->gridHeight__ -1 : (*y) );
}

inline void HPFE_setOccupancy_UNSAFE( HPFE *e, int x, int y, int occupancy){
	e->occupancyGrid__[ (y*e->gridWidth__) + x  ] = occupancy;
	return;	
}

inline int HPFE_getOccupancy_UNSAFE( HPFE *e, int x, int y){
	return( e->occupancyGrid__[ (y*e->gridWidth__) + x  ] );	
}


inline void HPFE_getVectorOut_UNSAFE(HPFE *e, Pos *start, Pos *vectorOut){
	vectorOut->x__ =  	HPFE_getPotential_UNSAFE(e, (int)(start->x__ + 1), (int)(start->y__)) -\
						HPFE_getPotential_UNSAFE(e, (int)(start->x__ - 1), (int)(start->y__));
	vectorOut->y__ =  	HPFE_getPotential_UNSAFE(e, (int)(start->x__), (int)(start->y__ + 1)) -\
						HPFE_getPotential_UNSAFE(e, (int)(start->x__), (int)(start->y__ - 1));
	return;
}

inline int HPFE_nextCellFrom(HPFE *e, Pos *start, Pos *next){

	if( HPFE_isValidPosition(e, start) == u_false ){
		return( -1 );
	}


	HPFE_getVectorOut_UNSAFE(e, start, next);
	double ang = atan2( -next->y__, -next->x__ ); //gradiente descendent
	ang = ( ang < 0 ? 2*M_PI + ang : ang );

	if( (ang<1.17809725) || (ang>=5.1050880) ){
		next->x__ = start->x__ + 1;
	}
	else{
		if( (ang>=1.9634954) && (ang<4.31968990) ){
			next->x__ = start->x__ - 1;
		}
		else{
			next->x__ = start->x__;
		}
	}

	
	if ( (ang >= 0.39269908 ) && ( ang < 2.74889357 ) ){
		next->y__ = start->y__ +1;
	}
	else{
		if ( (ang >= 3.53429174 ) && ( ang < 5.89048623 ) ){
			next->y__ = start->y__ -1;
		}
		else{
			next->y__ = start->y__ ;
		}
	}

	if( (next->y__ == start->y__) && (next->x__ == start->x__) ){
		return( 1 );
	}
	else{
		return( 0 );
	}
		
}

inline int HPFE_getPathFrom(HPFE *e, int maxSteps, Pos *start, Pos *path){
	int i=1;
	path[0].x__ = start->x__;
	path[0].y__ = start->y__;
	int stop=0, x, y;
	while( (i<maxSteps) && (!stop) ){
		if( HPFE_nextCellFrom(e, &path[i-1], &path[i] ) != 0 ){//if it is the same cell, stops
			stop=1;
		}
		x = (int)floor( path[i].x__ + 0.5);
		y = (int)floor( path[i].y__ + 0.5);
		if( (HPFE_getPotential_UNSAFE(e, x, y ) < 10e-15) && (HPFE_getOccupancy_UNSAFE(e, x, y) == OCCUPIED_CELL )) { //goal reached
			stop=1;
		}

		i++;
	}

	return( i );

}


inline int HPFE_getPath(HPFE *e, int maxSteps, Pos *path){
	if( HPFE_getMe(e, &path[0] ) != 0 ){
		fprintf(stderr, "HPFE_getPath: Unknown error at \"HPFE_getMe\"\n");
		return(-1);
	}
	return( HPFE_getPathFrom( e, maxSteps, &path[0], path ) );
}


inline int HPFE_makePathFrom(HPFE *e, int maxSteps, Pos *start, Pos *destiny){
	int i=0, stop, x, y;
	Pos old;
	old.x__ = start->x__;
	old.y__ = start->y__;

	while( (i<maxSteps) &&  (!stop) ){
		if( HPFE_nextCellFrom(e, &old, destiny ) != 0 ){//if it is the same cell, stops
			stop=1;
		}
		
		x = (int)floor( destiny->x__ + 0.5);
		y = (int)floor( destiny->y__ + 0.5);

		if( (HPFE_getPotential_UNSAFE(e, x, y ) < 10e-15) && (HPFE_getOccupancy_UNSAFE(e, x, y) == OCCUPIED_CELL )) { //goal reached
			stop=1;
		}
		
		old.x__ = destiny->x__;
		old.y__ = destiny->y__;		
		i++;
	}
	
	return( i );

}


inline int HPFE_makePath(HPFE *e, int maxSteps, Pos *destiny){
	if( HPFE_getMe(e, destiny ) != 0 ){
		fprintf(stderr, "HPFE_getPath: Unknown error at \"HPFE_getMe\"\n");
		return(-1);
	}
	return( HPFE_getPathFrom( e, maxSteps, destiny, destiny ) );
}


inline void HPFE_clearGridWithStartPotential_UNSAFE( HPFE *e ){
	for( int i=0; i<(e->gridAllocatedSize__); i++){
		e->potentialGrid__[i] = START_POTENTIAL;
	}
	return;
}

inline void HPFE_fprintfGrid(HPFE *e, FILE *out){
	int i,j;
	for(i=0; i<(e->gridHeight__); i++){
		for(j=0; j<(e->gridWidth__); j++){
			fprintf(out, "%02.010lf\n", e->potentialGrid__[ (i*(e->gridWidth__)) + j ] );
		}
		fprintf(out, "\n");
	}
	return;
}

#endif
