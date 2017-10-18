#ifndef OPF_H
#define OPF_H

#include "utils.h"
#include "lib_be.hh"
#include "lib_hpf.hh"
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <sys/time.h>


// {[GS,SOR][C, UP]}
// {[Gauss-Seidel, Successive Over-Relaxation][Centered, UP-Wind]}
// to future: parallel, other bvps, ...
enum OPFE_IterateModes{ OPFE_gsc, OPFE_sorc, OPFE_gsup, OPFE_sorup};
enum OPFE_IterateFlags{ OPFE_normal=0x00, OPFE_printErrors=0x01, OPFE_printTimes=0x02, OPFE_printTimeAtEnd=0x04 };
//enum HPFE_printGridModes{ OPFE_column, OPFE_row }; //to future use

typedef struct{
	HPFE *hpfe__;
	
	double v__, epsilon__;
	double sinv__, cosv__;

	} OPFE; //Oriented Potential Field Environment
	
/*
 _     ___ ____    _____                 _   _                 
| |   |_ _| __ )  |  ___|   _ _ __   ___| |_(_) ___  _ __  ___ 
| |    | ||  _ \  | |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __|
| |___ | || |_) | |  _|| |_| | | | | (__| |_| | (_) | | | \__ \
|_____|___|____/  |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
*/



OPFE* OPFE_new(int maxNumberOfObstacle, int maxNumberOfEnemyRobot, int maxNumberOfOwnRobot, int maxNumberOfGoal, int width, int height, double v=0, double epsilon=0);

void OPFE_delete( OPFE *e );

int OPFE_clearAllLists( OPFE *e );


double OPFE_getWSor( OPFE *e );

int OPFE_setWSor( OPFE *e, double wSor);

double OPFE_getVector( OPFE *e );

int OPFE_setVector( OPFE *e, double v);

double OPFE_getEpsilon( OPFE *e );

int OPFE_setEpsilon( OPFE *e, double epsilon);



int OPFE_getGridWidth( OPFE *e );

int OPFE_setGridWidth( OPFE *e, int width );

int OPFE_getGridHeight( OPFE *e );

int OPFE_setGridHeight( OPFE *e, int height );


#ifdef UTILS_FLOAT_DATATYPE

//int OPFE_getPotentialGrid( OPFE *e, int *width, int *height, float *dest);//Ainda nao implementada

//int OPFE_setPotentialGrid( OPFE *e, int *width, int *height, float *source);//Ainda nao implementada

int OPFE_setPotential( OPFE *e, int x, int y, float potential);

float OPFE_getPotential( OPFE *e, int x, int y);

int OPFE_setAllPotential( OPFE *e, float potential );

#else 
//int OPFE_getPotentialGrid( OPFE *e, int *width, int *height, double *dest); //Ainda nao implementada

//int OPFE_setPotentialGrid( OPFE *e, int *width, int *height, double *source); //Ainda nao implementada

int OPFE_setPotential( OPFE *e, int x, int y, double potential);

double OPFE_getPotential( OPFE *e, int x, int y);

int OPFE_setAllPotential( OPFE *e, double potential );

#endif

//int HPFE_setOccupancyGrid( HPFE *e, int *width, int *height, int *source);

//int HPFE_getOccupancyGrid( HPFE *e, int *width, int *height, int *dest);

int OPFE_setOccupancy( OPFE *e, int x, int y, int occupancy);

int OPFE_getOccupancy( OPFE *e, int x, int y);

int OPFE_getGradient( OPFE *e, Pos *position, Pos *result);

int OPFE_getGradientDescent( OPFE *e, Pos *position, Pos *result);



int OPFE_setMaxNumberOfObstacle(OPFE *e,  int maxNumberOfObstacle, int list=0);

int OPFE_getMaxNumberOfObstacle(OPFE *e,  int list=0);

int OPFE_addObstacle( OPFE *e, Pos *newObstacle, int list=0 );

int OPFE_getObstacleAtIndex( OPFE *e, int index, Pos *result, int list=0 );

int OPFE_clearListOfObstacle( OPFE *e, int list=0 );



int OPFE_setMaxNumberOfEnemyRobot(OPFE *e,  int maxNumberOfEnemyRobot );

int OPFE_getMaxNumberOfEnemyRobot(OPFE *e);

int OPFE_addEnemyRobot( OPFE *e, Pos *newEnemyRobot);

int OPFE_getEnemyRobotAtIndex( OPFE *e, int index, Pos *result );

int OPFE_clearListOfEnemyRobot( OPFE *e );



int OPFE_setMaxNumberOfOwnRobot(OPFE *e,  int maxNumberOfOwnRobot );

int OPFE_getMaxNumberOfOwnRobot(OPFE *e);

int OPFE_addOwnRobot( OPFE *e, Pos *newOwnRobot);

int OPFE_setMe( OPFE *e, Pos *newOwnRobot);

int OPFE_getMe( OPFE *e, Pos *result );

int OPFE_getOwnRobotAtIndex( OPFE *e, int index, Pos *result );

int OPFE_clearListOfOwnRobot( OPFE *e );



int OPFE_setMaxNumberOfGoal(OPFE *e,  int maxNumberGoal );

int OPFE_getMaxNumberOfGoal(OPFE *e);

int OPFE_addGoal(OPFE *e, Pos *newGoal);

int OPFE_getGoalAtIndex( OPFE *e, int index, Pos *result );

int OPFE_clearListOfGoal( OPFE *e );




/*
 ____        _                    
/ ___|  ___ | |_   _____ _ __ ___ 
\___ \ / _ \| \ \ / / _ \ '__/ __|
 ___) | (_) | |\ V /  __/ |  \__ \
|____/ \___/|_| \_/ \___|_|  |___/
*/

//[I, T, E]
//Limited by: {Iterations, Time, Error}
long int OPFE_solveI( OPFE *e, OPFE_IterateModes it, int itMax, OPFE_IterateFlags itFlags = OPFE_normal);
//double OPFE_solveIT( OPFE *e, OPFE_IterateModes it, int itMax, int usecMax, OPFE_IterateFlags itFlags = OPFE_normal);
long int OPFE_solveIE( OPFE *e, OPFE_IterateModes it, int itMax, double errorMax, OPFE_IterateFlags itFlags = OPFE_normal);
long int OPFE_solveE( OPFE *e, OPFE_IterateModes it, double errorMax, OPFE_IterateFlags itFlags = OPFE_normal);
//double OPFE_solveET( OPFE *e, OPFE_IterateModes it, double errorMax, int usecMax, int itMax, OPFE_IterateFlags itFlags = OPFE_normal);
//double OPFE_solveT( OPFE *e, OPFE_IterateModes it, int usecMax, OPFE_IterateFlags itFlags = OPFE_normal);




/*
 ___ _                 _                 
|_ _| |_ ___ _ __ __ _| |_ ___  _ __ ___ 
 | || __/ _ \ '__/ _` | __/ _ \| '__/ __|
 | || ||  __/ | | (_| | || (_) | |  \__ \
|___|\__\___|_|  \__,_|\__\___/|_|  |___/
*/


double OPFE_iteratorGSC(OPFE *e );

double OPFE_iteratorGSUP(OPFE *e );
double OPFE_iteratorGSUP_PP(OPFE *e );
double OPFE_iteratorGSUP_PN(OPFE *e );
double OPFE_iteratorGSUP_NP(OPFE *e );
double OPFE_iteratorGSUP_NN(OPFE *e );

double OPFE_iteratorSORC(OPFE *e );

double OPFE_iteratorSORUP(OPFE *e );
double OPFE_iteratorSORUP_PP(OPFE *e );
double OPFE_iteratorSORUP_PN(OPFE *e );
double OPFE_iteratorSORUP_NP(OPFE *e );
double OPFE_iteratorSORUP_NN(OPFE *e );


/*
 ___       _ _              _____                 _   _                 
|_ _|_ __ | (_)_ __   ___  |  ___|   _ _ __   ___| |_(_) ___  _ __  ___ 
 | || '_ \| | | '_ \ / _ \ | |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __|
 | || | | | | | | | |  __/ |  _|| |_| | | | | (__| |_| | (_) | | | \__ \
|___|_| |_|_|_|_| |_|\___| |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
*/
//These are internal functions. Just for purposes of acceleration, they are inline and (some of then) without safe check.
//They just wrap some command.
#ifdef UTILS_FLOAT_DATATYPE
inline void OPFE_setPotential_UNSAFE( OPFE *e, int x, int y, float potential){
	HPFE_setPotential_UNSAFE( e->hpfe__, x, y, potential);
	return;
}

inline float OPFE_getPotential_UNSAFE( OPFE *e, int x, int y){
	return( HPFE_getPotential_UNSAFE( e->hpfe__, x, y) );
}

inline int OPFE_setPotentialP( OPFE *e, Pos *p, float potential){
	return( OPFE_setPotential( e, (int)(floor(p->x__+0.5)), (int)(floor(p->y__+0.5)), potential) );
}

inline float OPFE_getPotentialP( OPFE *e, Pos *p ){
	return( OPFE_getPotential( e, (int)(floor(p->x__+0.5)), (int)(floor(p->y__+0.5))) );
}

#else 
inline void OPFE_setPotential_UNSAFE( OPFE *e, int x, int y, double potential){
	HPFE_setPotential_UNSAFE( e->hpfe__, x, y, potential);
	return;
}

inline double OPFE_getPotential_UNSAFE( OPFE *e, int x, int y){
	return( HPFE_getPotential_UNSAFE( e->hpfe__, x, y) );
}

inline int OPFE_setPotentialP( OPFE *e, Pos *p, double potential){
	return( OPFE_setPotential( e, (int)(floor(p->x__+0.5)), (int)(floor(p->y__+0.5)), potential) );
}

inline double OPFE_getPotentialP( OPFE *e, Pos *p ){
	return( OPFE_getPotential( e, (int)(floor(p->x__+0.5)), (int)(floor(p->y__+0.5))) );
}
#endif

inline int OPFE_setOccupancyP( OPFE *e, Pos *p, int occupancy){
	return( OPFE_setOccupancy( e, (int)(floor(p->x__+0.5)), (int)(floor(p->y__+0.5)), occupancy ) );
}

inline int OPFE_getOccupancyP( OPFE *e, Pos *p ){
	return( OPFE_getOccupancy( e, (int)(floor(p->x__+0.5)), (int)(floor(p->y__+0.5))) );
}

inline Booleano OPFE_isValidPosition(OPFE *e, Pos *p){
    return( (Booleano)HPFE_isValidPosition(e->hpfe__, p) );
}

int HPFE_setMe( HPFE *e, Pos *newOwnRobot);

int HPFE_getMe( HPFE *e, Pos *result );
inline void OPFE_posToInt(OPFE *e, Pos *p, int *x, int *y){
	HPFE_posToInt(e->hpfe__, p, x, y);
	return;
}

inline void OPFE_setOccupancy_UNSAFE( OPFE *e, int x, int y, int occupancy){
	HPFE_setOccupancy_UNSAFE( e->hpfe__, x, y, occupancy);
	return;	
}

inline int OPFE_getOccupancy_UNSAFE( OPFE *e, int x, int y){
	return( HPFE_getOccupancy_UNSAFE( e->hpfe__, x, y) );	
}double OPFE_iteratorGSUP(OPFE *e );

inline void OPFE_getVectorOut_UNSAFE(OPFE *e, Pos *start, Pos *vectorOut){
	HPFE_getVectorOut_UNSAFE(e->hpfe__, start, vectorOut);
	return;
}

inline int OPFE_nextCellFrom(OPFE *e, Pos *start, Pos *next){
	return( HPFE_nextCellFrom(e->hpfe__, start, next) );
}

inline int OPFE_getPathFrom(OPFE *e, int maxSteps, Pos *start, Pos *path){
	return( HPFE_getPathFrom(e->hpfe__, maxSteps, start, path) );
}

inline int OPFE_makePathFrom(OPFE *e, int maxSteps, Pos *start, Pos *destiny){
	return( HPFE_makePathFrom(e->hpfe__, maxSteps, start, destiny) );
}

inline void OPFE_clearGridWithStartPotential_UNSAFE( OPFE *e){
	HPFE_clearGridWithStartPotential_UNSAFE( e->hpfe__ );
	return;
}

inline void OPFE_fprintfGrid(OPFE *e, FILE *out){
	HPFE_fprintfGrid(e->hpfe__, out);
	return;
}




#endif

