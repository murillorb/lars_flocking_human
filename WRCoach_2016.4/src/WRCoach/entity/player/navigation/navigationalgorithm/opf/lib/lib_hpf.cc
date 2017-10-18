#include "lib_hpf.hh"

/*
 _     ___ ____    _____                 _   _                 
| |   |_ _| __ )  |  ___|   _ _ __   ___| |_(_) ___  _ __  ___ 
| |    | ||  _ \  | |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __|
| |___ | || |_) | |  _|| |_| | | | | (__| |_| | (_) | | | \__ \
|_____|___|____/  |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
*/


HPFE *HPFE_new(int maxNumberOfObstacle, int maxNumberOfEnemyRobot, int maxNumberOfOwnRobot, int maxNumberOfGoal, int width, int height){
	
	//allocating mem to HPFE struct, validating and cleaning
	HPFE *e = (HPFE *) malloc( sizeof(HPFE) );
	if( e == NULL ){
		fprintf(stderr, "ERROR: HPFE_new couldn\'t allocate memory to HPFE Struct!\n");
		return( NULL );
	}
	memset( e, 0, sizeof(HPFE) );
	
	
	//checking if parameters are valid
	if( (width<=0) || (height<=0) ){
		fprintf(stderr, "ERROR: HPFE_new couldn\'t allocate memory to Grids! Invalid values of width( %d ) and/or height( %d )!!\n", width, height);
		free( e );//if couldnt allocate, release mem occupied by "HPFE *e"
		return( NULL );
	}
	
	//allocating mem to HPFE struct and validating
	e->be__ = BE_new( maxNumberOfObstacle, 0, maxNumberOfEnemyRobot, maxNumberOfOwnRobot, maxNumberOfGoal);
	if (e->be__==NULL ){
		fprintf(stderr, "ERROR: HPFE_new couldn\'t allocate memory to BE Struct!\n");
		free( e );//if couldnt allocate, release mem occupied by "HPFE *e"
		return( NULL );
	}


	e->gridWidth__ = width;
	e->gridHeight__ = height;
	e->gridAllocatedSize__ = width * height;
	
	e->wSor__=1;
//	e->cudaStuff__ = NULL;
//	e->sizeOfCudaStuff__ = 0;


	//allocating mem to potentialGrid and validating
#ifdef UTILS_FLOAT_DATATYPE
	e->potentialGrid__ = (float *)malloc( sizeof(float)*width*height );
	if(e->potentialGrid__==NULL ){
		fprintf(stderr, "ERROR: HPFE_new couldn\'t allocate (float type) memory to potentialGrid!\n");
		BE_delete( e->be__ );//if couldnt allocate, release mem occupied by "BE *(e->be__)"
		free( e );//if couldnt allocate, release mem occupied by "HPFE *e"
		return( NULL );
	}

	memset( e->potentialGrid__, 0, sizeof(float)*width*height);
	HPFE_clearGridWithStartPotential_UNSAFE( e );

#else 
	e->potentialGrid__ = (double *)malloc( sizeof(double)*width*height );
	if(e->potentialGrid__==NULL ){
		fprintf(stderr, "ERROR: HPFE_new couldn\'t allocate (double type) memory to potentialGrid!\n");
		BE_delete( e->be__ );//if couldnt allocate, release mem occupied by "BE *(e->be__)"
		free( e );//if couldnt allocate, release mem occupied by "HPFE *e"
		return( NULL );
	}
	memset( e->potentialGrid__, 0, sizeof(double)*width*height);
	HPFE_clearGridWithStartPotential_UNSAFE( e );
#endif	


	//allocating mem to occupancyGrid and validating
	e->occupancyGrid__ = (int *)malloc( sizeof(int)*width*height );
	if (e->occupancyGrid__==NULL ){
		fprintf(stderr, "ERROR: HPFE_new couldn\'t allocate memory to occupancyGrid!\n");
		BE_delete( e->be__ );//if couldnt allocate, release mem occupied by "BE *(e->be__)"
		free( e->potentialGrid__ );//if couldnt allocate, release mem occupied by "potentialGrid"
		free( e );//if couldnt allocate, release mem occupied by "HPFE *e"
		return( NULL );
	}
	memset( e->occupancyGrid__, 0, sizeof(int)*width*height);
	
	
	

	
	return( e );
}


void HPFE_delete( HPFE *e ){

	BE_delete( e->be__ );
	free( e->potentialGrid__ );
	free( e->occupancyGrid__ );
	free( e );
	
	return;
}


int HPFE_clearAllLists( HPFE *e ){

	HPFE_clearListOfObstacle( e );
	HPFE_clearListOfEnemyRobot( e );
	HPFE_clearListOfOwnRobot( e );
	HPFE_clearListOfGoal( e );

#ifdef HPFE_CLEAR_WITH_START_POTENTIAL
	HPFE_clearGridWithStartPotential_UNSAFE( e );
#endif

	return( 0 );
}



int HPFE_getGridWidth( HPFE *e ){
	
	return( e->gridWidth__ );
}

int HPFE_setGridWidth( HPFE *e, int width ){
	//checking if parameter is valid
	if( width<=0) {
		fprintf(stderr, "ERROR: HPFE_setGridWidth couldn\'t allocate memory to Grids! Invalid value of width( %d )!!\n", width);
		return( -1 );
	}

	e->gridWidth__ = width;

	if( ((e->gridWidth__) * (e->gridHeight__)) < (e->gridAllocatedSize__) ){
#ifdef UTILS_FLOAT_DATATYPE
		memset( e->potentialGrid__, 0, sizeof(float)*(e->gridAllocatedSize__) );
#else
		memset( e->potentialGrid__, 0, sizeof(double)*(e->gridAllocatedSize__) );

#endif
		memset( e->occupancyGrid__, 0, sizeof(int)*(e->gridAllocatedSize__) );
	}
	else {
		free( e->potentialGrid__ );
		free( e->occupancyGrid__ );
		
		e->gridAllocatedSize__ = e->gridHeight__ * e->gridWidth__;

#ifdef UTILS_FLOAT_DATATYPE
		e->potentialGrid__ = (float *)malloc( sizeof(float)*(e->gridAllocatedSize__) );
		if(e->potentialGrid__==NULL ){
			fprintf(stderr, "ERROR: HPFE_setGridWidth couldn\'t allocate (float type) memory to potentialGrid!\n");
			e->gridWidth__ = 0;
			e->gridHeight__ = 0;
			e->gridAllocatedSize__ = e->gridHeight__ * e->gridWidth__;
			return( -2 );
		}
		memset( e->potentialGrid__, 0, sizeof(float)*(e->gridAllocatedSize__));
		HPFE_clearGridWithStartPotential_UNSAFE( e );
#else 
		e->potentialGrid__ = (double *)malloc( sizeof(double)*(e->gridAllocatedSize__) );
		if(e->potentialGrid__==NULL ){
			fprintf(stderr, "ERROR: HPFE_setGridWidth couldn\'t allocate (double type) memory to potentialGrid!\n");
			e->gridWidth__ = 0;
			e->gridHeight__ = 0;
			e->gridAllocatedSize__ = e->gridHeight__ * e->gridWidth__;
			return( -2 );
		}
		memset( e->potentialGrid__, 0, sizeof(double)*(e->gridAllocatedSize__));
		HPFE_clearGridWithStartPotential_UNSAFE( e );
#endif	

	
		//allocating mem to occupancyGrid and validating
		e->occupancyGrid__ = (int *)malloc( sizeof(int)*(e->gridAllocatedSize__) );
		if (e->occupancyGrid__==NULL ){
			fprintf(stderr, "ERROR: HPFE_setGridWidth couldn\'t allocate memory to occupancyGrid!\n");
			e->gridWidth__ = 0;
			e->gridHeight__ = 0;
			e->gridAllocatedSize__ = 0;
			free( e->potentialGrid__ );
			return( -3 );
		}
		memset( e->occupancyGrid__, 0, sizeof(int)*(e->gridAllocatedSize__));

	}
	
	return( e->gridWidth__ );
}

int HPFE_getGridHeight( HPFE *e ){
	
	return( e->gridHeight__ );
}

int HPFE_setGridHeight( HPFE *e, int height ){
	//checking if parameter is valid
	if( height<=0) {
		fprintf(stderr, "ERROR: HPFE_setGridHeight couldn\'t allocate memory to Grids! Invalid value of height( %d )!!\n", height);
		return( -1 );
	}

	e->gridHeight__ = height;

	if( (e->gridHeight__ * (e->gridWidth__)) < (e->gridAllocatedSize__) ){
#ifdef UTILS_FLOAT_DATATYPE
		memset( e->potentialGrid__, 0, sizeof(float)*(e->gridAllocatedSize__) );
#else
		memset( e->potentialGrid__, 0, sizeof(double)*(e->gridAllocatedSize__) );
#endif
		memset( e->occupancyGrid__, 0, sizeof(int)*(e->gridAllocatedSize__) );
	}
	else {
		free( e->potentialGrid__ );
		free( e->occupancyGrid__ );

		e->gridAllocatedSize__ = e->gridWidth__ * e->gridHeight__;

#ifdef UTILS_FLOAT_DATATYPE
		e->potentialGrid__ = (float *)malloc( sizeof(float)*e->gridAllocatedSize__ );
		if(e->potentialGrid__==NULL ){
			fprintf(stderr, "ERROR: HPFE_setGridHeight couldn\'t allocate (float type) memory to potentialGrid!\n");
			e->gridHeight__ = 0;
			e->gridWidth__ = 0;
			e->gridAllocatedSize__ = 0;
			return( -2 );
		}
		memset( e->potentialGrid__, 0, sizeof(float)*e->gridAllocatedSize__);
		HPFE_clearGridWithStartPotential_UNSAFE( e );
#else 
		e->potentialGrid__ = (double *)malloc( sizeof(double)*e->gridAllocatedSize__ );
		if(e->potentialGrid__==NULL ){
			fprintf(stderr, "ERROR: HPFE_setGridHeight couldn\'t allocate (double type) memory to potentialGrid!\n");
			e->gridHeight__ = 0;
			e->gridWidth__ = 0;
			e->gridAllocatedSize__ = 0;
			return( -2 );
		}
		memset( e->potentialGrid__, 0, sizeof(double)*e->gridAllocatedSize__);
		HPFE_clearGridWithStartPotential_UNSAFE( e );
#endif	
		//allocating mem to occupancyGrid and validating
		e->occupancyGrid__ = (int *)malloc( sizeof(int)*e->gridAllocatedSize__ );
		if (e->occupancyGrid__==NULL ){
			fprintf(stderr, "ERROR: HPFE_setGridHeight couldn\'t allocate memory to occupancyGrid!\n");
			e->gridHeight__ = 0;
			e->gridWidth__ = 0;
			e->gridAllocatedSize__ = 0;
			free( e->potentialGrid__ );
			return( -3 );
		}
		memset( e->occupancyGrid__, 0, sizeof(int)*e->gridAllocatedSize__);
	}
	return( e->gridHeight__ );
}






//int HPFE_setCudaStuff( HPFE *e, void *cudaStuff, unsigned int sizeOfCudaStuff ){
//	if( sizeOfCudaStuff < 0 ){
//		fprintf(stderr, "ERROR: HPFE_setCudaStuff couldn\'t set new cudaStuff! Invalid value of sizeOfCudaStuff( %d )!!\n", sizeOfCudaStuff);
//		return( -1 );
//	}		
//	e->cudaStuff__ = cudaStuff;
//	e->sizeOfCudaStuff__ = sizeOfCudaStuff;
//	return( 0 );
//}

//void* HPFE_getCudaStuff( HPFE *e ){
//	return( e->cudaStuff__ );
//}

//int HPFE_getCudaStuffSize( HPFE *e ){
//	return( e->sizeOfCudaStuff__ );
//}

#ifdef UTILS_FLOAT_DATATYPE

int HPFE_setPotential( HPFE *e, int x, int y, float potential){
	if( (x<0) || (x>=(e->gridWidth__)) || (y<0) || (y>=(e->gridHeight__))  ){
		fprintf(stderr, "ERROR: HPFE_setPotential (float) couldn\'t set new potential! Invalid value of x( %d ) and/or y( %d )!!\n", x, y);
		return( -1 );
	}
	
	e->potentialGrid__[ (y*(e->gridWidth__)) + x  ] = potential;

	return( 0 );	
}

float HPFE_getPotential( HPFE *e, int x, int y){
	if( (x<0) || (x>=(e->gridWidth__)) || (y<0) || (y>=(e->gridHeight__)) ){
		fprintf(stderr, "ERROR: HPFE_getPotential (float) couldn\'t set new potential! Invalid value of x( %d ) and/or y( %d )!!\n", x, y);
		return( -1 );
	}
	
	return( e->potentialGrid__[ (y*e->gridWidth__) + x  ] );
}

int HPFE_setAllPotential( HPFE *e, float potential ){
	int i, j;

	for(i=0; i<(e->gridHeight__); i++){
		for(j=0; j<(e->gridWidth__); j++){
			if( HPFE_getOccupancy_UNSAFE(e, j, i) != FREE_CELL ){
				continue;
			}
			HPFE_setPotential_UNSAFE(e, j, i, potential);
		}
	}

	return( 0 );

}

#else 

int HPFE_setPotential( HPFE *e, int x, int y, double potential){
	if( (x<0) || (x>=(e->gridWidth__)) || (y<0) || (y>=(e->gridHeight__)) ){
		fprintf(stderr, "ERROR: HPFE_setPotential (double) couldn\'t set new potential! Invalid value of x( %d ) and/or y( %d )!!\n", x, y);
		return( -1 );
	}
	
	e->potentialGrid__[ (y*e->gridWidth__) + x  ] = potential;
	return( 0 );	
}

double HPFE_getPotential( HPFE *e, int x, int y){
	if( (x<0) || (x>=(e->gridWidth__)) || (y<0) || (y>=(e->gridHeight__)) ){
		fprintf(stderr, "ERROR: HPFE_getPotential (double) couldn\'t set new potential! Invalid value of x( %d ) and/or y( %d )!!\n", x, y);
		return( -1 );
	}
	
	return( e->potentialGrid__[ (y*e->gridWidth__) + x  ] );	
}

int HPFE_setAllPotential( HPFE *e, double potential ){
	int i, j;

	for(i=0; i<(e->gridHeight__); i++){
		for(j=0; j<(e->gridWidth__); j++){
			if( HPFE_getOccupancy_UNSAFE(e, j, i) != FREE_CELL ){
				continue;
			}
			HPFE_setPotential_UNSAFE(e, j, i, potential);
		}
	}

	return( 0 );

}
#endif

double HPFE_getWSor( HPFE *e ){

	return( e->wSor__ );
	
}

int HPFE_setWSor( HPFE *e, double wSor){
	if( (wSor < 0) || (wSor>2) ){
		fprintf(stderr, "ERROR: HPFE_setWSor: Invalid value of wSor( %lf )!!\n", wSor);
		return(-1);
	}

	e->wSor__ = wSor;
	
	return( 0 );
	
}


int HPFE_setOccupancy( HPFE *e, int x, int y, int occupancy){
	if( (x<0) || (x>=(e->gridWidth__)) || (y<0) || (y>=(e->gridHeight__)) ){
		fprintf(stderr, "ERROR: HPFE_setOccupancy couldn\'t set new occupancy! Invalid value of x( %d ) and/or y( %d )!!\n", x, y);
		return( -1 );
	}
	
	e->occupancyGrid__[ (y*e->gridWidth__) + x  ] = occupancy;
	return( 0 );	
}

int HPFE_getOccupancy( HPFE *e, int x, int y){
	if( (x<0) || (x>=(e->gridWidth__)) || (y<0) || (y>=(e->gridHeight__)) ){
		fprintf(stderr, "ERROR: HPFE_getOccupancy couldn\'t get new occupancy! Invalid value of x( %d ) and/or y( %d )!!\n", x, y);
		return( -1 );
	}
	return( e->occupancyGrid__[ (y*e->gridWidth__) + x  ] );	
}

///////////////////////////////////////////////////////////PAREI AQUI!!!!!! Daqui para baixo não é confiável!
int HPFE_getGradient( HPFE *e, Pos *position, Pos *result){
	int x = (int) floor(position->x__ + 0.5);
	int y = (int) floor(position->y__ + 0.5);
	
	if( (x<0) || x>=(e->gridWidth__) || y<0 || y>=(e->gridHeight__) ){
#ifdef UTILS_FLOAT_DATATYPE
		fprintf(stderr, "ERROR: HPFE_getGradient couldn\'t get Gradient! Invalid value of x( %f ) and/or y( %f )!!\n", position->x__, position->y__);
#else
		fprintf(stderr, "ERROR: HPFE_getGradient couldn\'t get Gradient! Invalid value of x( %lf ) and/or y( %lf )!!\n", position->x__, position->y__);
#endif
		return( -1 );
	}	

	if( x == 0 ){
		(result->x__) = e->potentialGrid__[ (y*(e->gridWidth__)) + (x + 1)  ] - MAX_POTENTIAL;
	}
	else{
		if( x == ((e->gridWidth__)-1) ){
			(result->x__) = MAX_POTENTIAL - e->potentialGrid__[ (y*(e->gridWidth__)) + (x - 1)  ];
		}
		else{
			(result->x__) = e->potentialGrid__[ (y*(e->gridWidth__)) + (x + 1)  ] - e->potentialGrid__[ (y*(e->gridWidth__)) + (x - 1)  ];
		}
	}

	if( y == 0 ){
		(result->y__) = e->potentialGrid__[ ((y+1)*(e->gridWidth__)) + x ] - MAX_POTENTIAL;
	}
	else{
		if( y == ((e->gridHeight__)-1) ){
			(result->y__) = MAX_POTENTIAL - e->potentialGrid__[ ((y-1)*(e->gridWidth__)) + x ];
		}
		else{
			(result->y__) = e->potentialGrid__[ (y*(e->gridWidth__)) + (x + 1)  ] - e->potentialGrid__[ (y*(e->gridWidth__)) + (x - 1)  ];
		}
	}
	
	
	

	return( 0 );
}

int HPFE_getGradientDescent( HPFE *e, Pos *position, Pos *result){

	if( HPFE_getGradient( e, position, result) < 0 ){
#ifdef UTILS_FLOAT_DATATYPE
		fprintf(stderr, "ERROR: HPFE_getGradientDescent couldn\'t get Gradient! Invalid value of x( %f ) and/or y( %f )!!\n", position->x__, position->y__);
#else
		fprintf(stderr, "ERROR: HPFE_getGradientDescent couldn\'t get Gradient! Invalid value of x( %lf ) and/or y( %lf )!!\n", position->x__, position->y__);
#endif

		return( -1 );
	}	

	(result->x__) *= -1;
	(result->y__) *= -1;

	return( 0 );
}


int HPFE_setMaxNumberOfObstacle(HPFE *e,  int maxNumberOfObstacle, int list){
	if( BE_setMaxNumberOfObstacle(e->be__, maxNumberOfObstacle, list) < 0 ){
		fprintf(stderr, "ERROR: HPFE_setMaxNumberOfObstacle couldn\'t set Maximum number of Obstacle!!!\n");
		return( -1 );
	}
	return( 0 );

}

int HPFE_getMaxNumberOfObstacle(HPFE *e,  int list ){

	return( BE_getMaxNumberOfObstacle(e->be__, list) );	
}

int HPFE_getNumberOfObstacle(HPFE *e, int list){
   return( BE_getNumberOfObstacle(e->be__, list) );
}

int HPFE_addObstacle( HPFE *e, Pos *newObstacle, int list ){
	int x = (int) floor(newObstacle->x__ + 0.5);
	int y = (int) floor(newObstacle->y__ + 0.5);

	if( (x<0) || (x>=(e->gridWidth__)) || (y<0) || (y>=(e->gridHeight__)) ){
#ifdef UTILS_FLOAT_DATATYPE
		fprintf(stderr, "ERROR: HPFE_addObstacle  couldn\'t add new Obstacle! Invalid value of x( %f ) and/or y( %f )!!\n", newObstacle->x__, newObstacle->y__);
#else
		fprintf(stderr, "ERROR: HPFE_addObstacle  couldn\'t add new Obstacle! Invalid value of x( %lf ) and/or y( %lf )!!\n", newObstacle->x__, newObstacle->y__);
#endif
		return( -1 );
	}
	
	if( BE_addObstacle( e->be__, newObstacle, list ) < 0 ){
		fprintf(stderr, "ERROR: HPFE_addObstacle couldn\'t add new Obstacle! BE ERROR! \n");
		return( -2 );
	}
	
	e->occupancyGrid__[ (y*(e->gridWidth__)) + x ] = OCCUPIED_CELL;
	e->potentialGrid__[ (y*(e->gridWidth__)) + x ] = MAX_POTENTIAL;

	return( 0 );
}

int HPFE_getObstacleAtIndex( HPFE *e, int index, Pos *result, int list ){

	if( BE_getObstacleAtIndex( e->be__, index, result, list ) < 0 ){
		fprintf(stderr, "ERROR: HPFE_getObstacleAtIndex  couldn\'t get Obstacle! Invalid value of index( %d )!!\n", index);
		return( -1 );
	}

	return( 0 );	
}


int HPFE_clearListOfObstacle( HPFE *e, int list ){
	int nObstacle = BE_getNumberOfObstacle(e->be__, list);
	Pos aux;
	int x, y;

	if( nObstacle < 0 ){
		fprintf(stderr, "ERROR: HPFE_clearListOfObstacle  couldn\'t clear obstacle list!\n");
		return( -1 );
	}	
	
	for( int i=0; i< nObstacle; i++){
		BE_getObstacleAtIndex(e->be__, i, &aux, list);
		x = (int) floor(aux.x__ + 0.5);
		y = (int) floor(aux.y__ + 0.5);
		e->occupancyGrid__[(y*(e->gridWidth__)) + x ] = FREE_CELL;
#ifdef HPFE_CLEAR_WITH_START_POTENTIAL
		e->potentialGrid__[(y*(e->gridWidth__)) + x ] = START_POTENTIAL;	
#endif
	}
	BE_clearListOfObstacle(e->be__, list );

	return( 0 );	
}



int HPFE_setMaxNumberOfEnemyRobot(HPFE *e,  int maxNumberOfEnemyRobot ){
	if( BE_setMaxNumberOfEnemyRobot(e->be__, maxNumberOfEnemyRobot) < 0 ){
		fprintf(stderr, "ERROR: HPFE_setMaxNumberOfEnemyRobot couldn\'t set Maximum number of Enemy Robot!!!\n");
		return( -1 );
	}
	return( 0 );
}

int HPFE_getNumberOfEnemyRobot(HPFE *e){

	return( BE_getNumberOfEnemyRobot(e->be__) );	
}

int HPFE_getMaxNumberOfEnemyRobot(HPFE *e){

	return( BE_getMaxNumberOfEnemyRobot(e->be__) );	
}

int HPFE_addEnemyRobot( HPFE *e, Pos *newEnemyRobot){
	int x = (int) floor(newEnemyRobot->x__ + 0.5);
	int y = (int) floor(newEnemyRobot->y__ + 0.5);

	if( (x<0) || (x>=(e->gridWidth__)) || (y<0) || (y>=(e->gridHeight__)) ){
#ifdef UTILS_FLOAT_DATATYPE
		fprintf(stderr, "ERROR: HPFE_addEnemyRobot  couldn\'t add new Enemy Robot! Invalid value of x( %f ) and/or y( %f )!!\n", newEnemyRobot->x__, newEnemyRobot->y__);
#else
		fprintf(stderr, "ERROR: HPFE_addEnemyRobot  couldn\'t add new Enemy Robot! Invalid value of x( %lf ) and/or y( %lf )!!\n", newEnemyRobot->x__, newEnemyRobot->y__);
#endif
		return( -1 );
	}
	
	if( BE_addEnemyRobot( e->be__, newEnemyRobot) < 0 ){
		fprintf(stderr, "ERROR: HPFE_addEnemyRobot couldn\'t add new Enemy Robot! BE ERROR! \n");
		return( -2 );
	}
	
	e->occupancyGrid__[ (y*(e->gridWidth__)) + x ] = OCCUPIED_CELL;
	e->potentialGrid__[ (y*(e->gridWidth__)) + x ] = MAX_POTENTIAL;
	
	return( 0 );	
}

int HPFE_getEnemyRobotAtIndex( HPFE *e, int index, Pos *result ){

	if( BE_getEnemyRobotAtIndex( e->be__, index, result) < 0 ){
		fprintf(stderr, "ERROR: HPFE_getEnemyRobotAtIndex  couldn\'t get Enemy Robot! Invalid value of index( %d )!!\n", index);
		return( -1 );
	}
	return( 0 );	
}

int HPFE_clearListOfEnemyRobot( HPFE *e ){
	int nEnemyRobot = BE_getNumberOfEnemyRobot(e->be__);
	Pos aux;
	int x, y;

	if( nEnemyRobot < 0 ){
		fprintf(stderr, "ERROR: HPFE_clearListOfEnemyRobot  couldn\'t clear Enemy Robot list!\n");
		return( -1 );
	}	
	
	for( int i=0; i< nEnemyRobot; i++){
		BE_getEnemyRobotAtIndex(e->be__, i, &aux);
		x = (int) floor(aux.x__ + 0.5);
		y = (int) floor(aux.y__ + 0.5);
		e->occupancyGrid__[(y*(e->gridWidth__)) + x ] = FREE_CELL;
#ifdef HPFE_CLEAR_WITH_START_POTENTIAL
		e->potentialGrid__[(y*(e->gridWidth__)) + x ] = START_POTENTIAL;	
#endif
	}
	BE_clearListOfEnemyRobot(e->be__);
	return( 0 );	
}



int HPFE_setMaxNumberOfOwnRobot(HPFE *e,  int maxNumberOfOwnRobot ){
	if( BE_setMaxNumberOfOwnRobot(e->be__, maxNumberOfOwnRobot) < 0 ){
		fprintf(stderr, "ERROR: HPFE_setMaxNumberOfOwnRobot couldn\'t set Maximum number of Own Robot!!!\n");
		return( -1 );
	}
	return( 0 );
}

int HPFE_getNumberOfOwnRobot(HPFE *e){

	return( BE_getNumberOfOwnRobot(e->be__) );	
}

int HPFE_getMaxNumberOfOwnRobot(HPFE *e){

	return( BE_getMaxNumberOfOwnRobot(e->be__) );	
}

int HPFE_addOwnRobot( HPFE *e, Pos *newOwnRobot){
	int x = (int) floor(newOwnRobot->x__ + 0.5);
	int y = (int) floor(newOwnRobot->y__ + 0.5);

	if( (x<0) || (x>=(e->gridWidth__)) || (y<0) || (y>=(e->gridHeight__)) ){
#ifdef UTILS_FLOAT_DATATYPE
		fprintf(stderr, "ERROR: HPFE_addOwnRobot  couldn\'t add new Own Robot! Invalid value of x( %f ) and/or y( %f )!!\n", newOwnRobot->x__, newOwnRobot->y__);
#else
		fprintf(stderr, "ERROR: HPFE_addOwnRobot  couldn\'t add new Own Robot! Invalid value of x( %lf ) and/or y( %lf )!!\n", newOwnRobot->x__, newOwnRobot->y__);
#endif
		return( -1 );
	}
	
	if( BE_addOwnRobot( e->be__, newOwnRobot) < 0 ){
		fprintf(stderr, "ERROR: HPFE_addOwnRobot couldn\'t add new Own Robot! BE ERROR! \n");
		return( -2 );
	}
	
	e->occupancyGrid__[ (y*(e->gridWidth__)) + x ] = OCCUPIED_CELL;
	e->potentialGrid__[ (y*(e->gridWidth__)) + x ] = MAX_POTENTIAL;
	
	return( 0 );	
}

int HPFE_setMe( HPFE *e, Pos *newOwnRobot){
	if(BE_setMe( e->be__, newOwnRobot) != 0 ){
		fprintf(stderr, "HPFE_setMe: Unknown Error at \"BE_setMe\"\n");
		return(-1);
	}
	return(0);
}

int HPFE_getMe( HPFE *e, Pos *result ){
	if( BE_getMe( e->be__, result ) != 0 ){
		fprintf(stderr, "HPFE_getMe: Unknown Error at \"BE_getMe\"\n");
		return(-1);
	}
	return(0);
}


int HPFE_getOwnRobotAtIndex( HPFE *e, int index, Pos *result ){

	if( BE_getOwnRobotAtIndex( e->be__, index, result) < 0 ){
		fprintf(stderr, "ERROR: HPFE_getOwnRobotAtIndex  couldn\'t get Own Robot! Invalid value of index( %d )!!\n", index);
		return( -1 );
	}
	return( 0 );	
}

int HPFE_clearListOfOwnRobot( HPFE *e ){
	int nOwnRobot = BE_getNumberOfOwnRobot(e->be__);
	Pos aux;
	int x, y;

	if( nOwnRobot < 0 ){
		fprintf(stderr, "ERROR: HPFE_clearListOfOwnRobot  couldn\'t clear Own Robot list!\n");
		return( -1 );
	}	
	
	for( int i=0; i< nOwnRobot; i++){
		BE_getOwnRobotAtIndex(e->be__, i, &aux);
		x = (int) floor(aux.x__ + 0.5);
		y = (int) floor(aux.y__ + 0.5);
		e->occupancyGrid__[(y*(e->gridWidth__)) + x ] = FREE_CELL;
#ifdef HPFE_CLEAR_WITH_START_POTENTIAL
		e->potentialGrid__[(y*(e->gridWidth__)) + x ] = START_POTENTIAL;	
#endif
	}
	BE_clearListOfOwnRobot(e->be__);
	return( 0 );	
}

int HPFE_setMaxNumberOfGoal(HPFE *e,  int maxNumberOfGoal ){
	if( BE_setMaxNumberOfGoal(e->be__, maxNumberOfGoal) < 0 ){
		fprintf(stderr, "ERROR: HPFE_setMaxNumberOfGoal couldn\'t set Maximum number of Goal!!!\n");
		return( -1 );
	}
	return( 0 );
}

int HPFE_getNumberOfGoal(HPFE *e){

	return( BE_getNumberOfGoal(e->be__) );	
}

int HPFE_getMaxNumberOfGoal(HPFE *e){

	return( BE_getMaxNumberOfGoal(e->be__) );	
}

int HPFE_addGoal( HPFE *e, Pos *newGoal){
	int x = (int) floor(newGoal->x__ + 0.5);
	int y = (int) floor(newGoal->y__ + 0.5);

	if( (x<0) || (x>=(e->gridWidth__)) || (y<0) || (y>=(e->gridHeight__)) ){
#ifdef UTILS_FLOAT_DATATYPE
		fprintf(stderr, "ERROR: HPFE_addGoal  couldn\'t add new Goal! Invalid value of x( %f ) and/or y( %f )!!\n", newGoal->x__, newGoal->y__);
#else
		fprintf(stderr, "ERROR: HPFE_addGoal  couldn\'t add new Goal! Invalid value of x( %lf ) and/or y( %lf )!!\n", newGoal->x__, newGoal->y__);
#endif
		return( -1 );
	}
	
	if( BE_addGoal( e->be__, newGoal) < 0 ){
		fprintf(stderr, "ERROR: HPFE_addGoal couldn\'t add new Goal! BE ERROR! \n");
		return( -2 );
	}
	
	e->occupancyGrid__[ (y*(e->gridWidth__)) + x ] = OCCUPIED_CELL;
	e->potentialGrid__[ (y*(e->gridWidth__)) + x ] = MIN_POTENTIAL;
	
	return( 0 );	
}

int HPFE_getGoalAtIndex( HPFE *e, int index, Pos *result ){

	if( BE_getGoalAtIndex( e->be__, index, result) < 0 ){
		fprintf(stderr, "ERROR: HPFE_getGoalAtIndex  couldn\'t get Goal! Invalid value of index( %d )!!\n", index);
		return( -1 );
	}
	return( 0 );	
}

int HPFE_clearListOfGoal( HPFE *e ){
	int nGoal = BE_getNumberOfGoal(e->be__);
	Pos aux;
	int x, y;

	if( nGoal < 0 ){
		fprintf(stderr, "ERROR: HPFE_clearListOfGoal  couldn\'t clear Goal list!\n");
		return( -1 );
	}	
	
	for( int i=0; i< nGoal; i++){
		BE_getGoalAtIndex(e->be__, i, &aux);
		x = (int) floor(aux.x__ + 0.5);
		y = (int) floor(aux.y__ + 0.5);
		e->occupancyGrid__[(y*(e->gridWidth__)) + x ] = FREE_CELL;
#ifdef HPFE_CLEAR_WITH_START_POTENTIAL
		e->potentialGrid__[(y*(e->gridWidth__)) + x ] = START_POTENTIAL;	
#endif
	}
	BE_clearListOfGoal(e->be__);
	return( 0 );	
}







/*
 ___ _                 _                 
|_ _| |_ ___ _ __ __ _| |_ ___  _ __ ___ 
 | || __/ _ \ '__/ _` | __/ _ \| '__/ __|
 | || ||  __/ | | (_| | || (_) | |  \__ \
|___|\__\___|_|  \__,_|\__\___/|_|  |___/
*/

double HPFE_iteratorGS(HPFE *e ){

	int i,j;
	double error=0.0;
#ifdef UTILS_FLOAT_DATATYPE
	float oldPotential, newPotential, left, right, top, bottom;
#else
	double oldPotential, newPotential, left, right, top, bottom;
#endif

	for(i=0; i<(e->gridHeight__); i++){
		for(j=0; j<(e->gridWidth__); j++){
			if( HPFE_getOccupancy_UNSAFE(e, j, i) != FREE_CELL ){
				continue;
			}
			oldPotential = HPFE_getPotential_UNSAFE(e, j, i);
			
			if( 0==i ){
				bottom = MAX_POTENTIAL;
				top = HPFE_getPotential_UNSAFE(e, j, i+1);
			}
			else {
				if( (e->gridHeight__-1) == i ){
					bottom=HPFE_getPotential_UNSAFE(e, j, i-1);
					top = MAX_POTENTIAL;
				}
				else{
					bottom=HPFE_getPotential_UNSAFE(e, j, i-1);
					top = HPFE_getPotential_UNSAFE(e, j, i+1);
				}
			}

			if( 0==j ){
				left = MAX_POTENTIAL;
				right = HPFE_getPotential_UNSAFE(e, j+1, i);
			}
			else {
				if( (e->gridWidth__-1) == j ){
					left = HPFE_getPotential_UNSAFE(e, j-1, i);
					right = MAX_POTENTIAL;
				}
				else{
					left=HPFE_getPotential_UNSAFE(e, j-1, i);
					right = HPFE_getPotential_UNSAFE(e, j+1, i);
				}
			}

			newPotential = 0.25*(top + bottom + left + right);
			error+= ((newPotential-oldPotential)*(newPotential-oldPotential));
			HPFE_setPotential_UNSAFE(e, j, i, newPotential);
		}
	}
	return(error);
}


inline double HPFE_iteratorSOR(HPFE *e ){

	int i,j;
	double error=0.0;
#ifdef UTILS_FLOAT_DATATYPE
	float oldPotential, newPotential, left, right, top, bottom;
#else
	double oldPotential, newPotential, left, right, top, bottom;
#endif
	for(i=0; i<(e->gridHeight__); i++){
		for(j=0; j<(e->gridWidth__); j++){
			if( HPFE_getOccupancy_UNSAFE(e, j, i) != FREE_CELL ){
				continue;
			}
			oldPotential = HPFE_getPotential_UNSAFE(e, j, i);
			
			if( 0==i ){
				bottom = MAX_POTENTIAL;
				top = HPFE_getPotential_UNSAFE(e, j, i+1);
			}
			else {
				if( (e->gridHeight__-1) == i ){
					bottom=HPFE_getPotential_UNSAFE(e, j, i-1);
					top = MAX_POTENTIAL;
				}
				else{
					bottom=HPFE_getPotential_UNSAFE(e, j, i-1);
					top = HPFE_getPotential_UNSAFE(e, j, i+1);
				}
			}

			if( 0==j ){
				left = MAX_POTENTIAL;
				right = HPFE_getPotential_UNSAFE(e, j+1, i);
			}
			else {
				if( (e->gridWidth__-1) == j ){
					left = HPFE_getPotential_UNSAFE(e, j-1, i);
					right = MAX_POTENTIAL;
				}
				else{
					left=HPFE_getPotential_UNSAFE(e, j-1, i);
					right = HPFE_getPotential_UNSAFE(e, j+1, i);
				}
			}

			newPotential = 0.25*(top + bottom + left + right) ;
			newPotential =  ((1-e->wSor__)*(oldPotential) ) +  ((e->wSor__)*(newPotential) );//SORing

			error+= ((newPotential-oldPotential)*(newPotential-oldPotential));
			HPFE_setPotential_UNSAFE(e, j, i, newPotential);
		}
	}
	return(error);
}



/*
 ____        _                    
/ ___|  ___ | |_   _____ _ __ ___ 
\___ \ / _ \| \ \ / / _ \ '__/ __|
 ___) | (_) | |\ V /  __/ |  \__ \
|____/ \___/|_| \_/ \___|_|  |___/
*/
long int HPFE_solveI( HPFE *e, HPFE_IterateModes it, int itMax, HPFE_IterateFlags itFlags){

	double error;
	struct timeval ti, tf;
	gettimeofday( &ti, NULL);
	switch( it ){
		case HPFE_gs:
			if( (HPFE_printErrors & itFlags) != 0){
				for(int i=0; i<itMax; i++){
					error = HPFE_iteratorGS(e);
					printf("%04d: %02.020lf\n", i, error);
				}
			}
			else{
				for(int i=0; i<itMax; i++){
					error = HPFE_iteratorGS(e);
				}
			}
		break;


		case HPFE_sor:
			if( (HPFE_printErrors & itFlags) != 0){
				for(int i=0; i<itMax; i++){
					error = HPFE_iteratorSOR(e);
					printf("%04d: %02.020lf\n", i, error);
				}
			}
			else{
				for(int i=0; i<itMax; i++){
					error = HPFE_iteratorSOR(e);
				}
			}
		break;
		default: 
#ifdef UTILS_WITH_DEBUG
		fprintf(stderr, "HPFE_SolveI: Iterator not found!\n");
#endif
		break;

	}
	gettimeofday( &tf, NULL);

	return( (1000000*(tf.tv_sec - ti.tv_sec) ) + (tf.tv_usec - ti.tv_usec) );
}




long int HPFE_solveIE( HPFE *e, HPFE_IterateModes it, int itMax, double errorMax, HPFE_IterateFlags itFlags){

	double error;
	struct timeval ti, tf;
	gettimeofday( &ti, NULL);
	errorMax = (errorMax > 0 ? errorMax : 1e-6 );
	error = 10*errorMax;

	switch( it ){
		case HPFE_gs:
			if( (HPFE_printErrors & itFlags) != 0){
				for(int i=0; (i<itMax) && ( error>=errorMax ); i++){
					error = HPFE_iteratorGS(e);
					printf("%04d: %02.020lf\n", i, error);
				}
			}
			else{
				for(int i=0; (i<itMax) && ( error>=errorMax ); i++){
					error = HPFE_iteratorGS(e);
				}
			}
		break;


		case HPFE_sor:
			if( (HPFE_printErrors & itFlags) != 0){
				for(int i=0; (i<itMax) && ( error>=errorMax ); i++){
					error = HPFE_iteratorSOR(e);
					printf("%04d: %02.020lf\n", i, error);
				}
			}
			else{
				for(int i=0; (i<itMax) && ( error>=errorMax ); i++){
					error = HPFE_iteratorSOR(e);
				}
			}
		break;
		default: 
#ifdef UTILS_WITH_DEBUG
		fprintf(stderr, "HPFE_SolveI: Iterator not found!\n");
#endif
		break;

	}

	gettimeofday( &tf, NULL);

	return( (1000000*(tf.tv_sec - ti.tv_sec) ) + (tf.tv_usec - ti.tv_usec) );
}


long int HPFE_solveE( HPFE *e, HPFE_IterateModes it, double errorMax, HPFE_IterateFlags itFlags){

	double error;
	struct timeval ti, tf;
	gettimeofday( &ti, NULL);
	errorMax = (errorMax > 0 ? errorMax : 1e-6 );
	error = 10*errorMax;


	switch( it ){
		case HPFE_gs:
			if( (HPFE_printErrors & itFlags) != 0){
				for(int i=0;  error>=errorMax ; i++){
					error = HPFE_iteratorGS(e);
					printf("%04d: %02.020lf\n", i, error);
				}
			}
			else{
				for(int i=0; error>=errorMax ; i++){
					error = HPFE_iteratorGS(e);
				}
			}
		break;


		case HPFE_sor:
			if( (HPFE_printErrors & itFlags) != 0){
				for(int i=0;  error>=errorMax ; i++){
					error = HPFE_iteratorSOR(e);
					printf("%04d: %02.020lf\n", i, error);
				}
			}
			else{
				for(int i=0;error>=errorMax ; i++){
					error = HPFE_iteratorSOR(e);
				}
			}
		break;
		default: 
#ifdef UTILS_WITH_DEBUG
		fprintf(stderr, "HPFE_SolveI: Iterator not found!\n");
#endif
		break;

	}

	gettimeofday( &tf, NULL);

	return( (1000000*(tf.tv_sec - ti.tv_sec) ) + (tf.tv_usec - ti.tv_usec) );
}


