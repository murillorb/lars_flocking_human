#include "lib_opf.hh"

#pragma GCC diagnostic ignored "-Wunused-but-set-variable"


OPFE* OPFE_new(int maxNumberOfObstacle, int maxNumberOfEnemyRobot, int maxNumberOfOwnRobot, int maxNumberOfGoal, int width, int height, double v, double epsilon){
	//allocating mem to OPFE struct, validating and cleaning
	OPFE *e = (OPFE *) malloc( sizeof(OPFE) );
	if( e == NULL ){
		fprintf(stderr, "ERROR: OPFE_new couldn\'t allocate memory to OPFE Struct!\n");
		return( NULL );
	}
	memset( e, 0, sizeof(OPFE) );
	
	//allocating mem to HPFE struct and validating
	e->hpfe__ = HPFE_new( maxNumberOfObstacle, maxNumberOfEnemyRobot, maxNumberOfOwnRobot, maxNumberOfGoal, width, height);
	if (e->hpfe__==NULL ){
		fprintf(stderr, "ERROR: OPFE_new couldn\'t allocate memory to HPFE Struct!\n");
		free( e );//if couldnt allocate, release mem occupied by "HPFE *e"
		return( NULL );
	}

	e->v__ = v;
	e->sinv__ = sin(v);
	e->cosv__ = cos(v);
	e->epsilon__ = epsilon;
	
	return( e );	
}

void OPFE_delete( OPFE *e ){
	HPFE_delete( e->hpfe__ );
	free( e );
}

int OPFE_clearAllLists( OPFE *e ){
	if( 0 != HPFE_clearAllLists( e->hpfe__ ) ){
		fprintf(stderr, "OPFE_clearAllLists: Unknown Error at HPFE_clearAllLists\n");
		return( -1 );
	}

	return( 0 );

}


double OPFE_getWSor( OPFE *e ){

	return( HPFE_getWSor(e->hpfe__) );
}

int OPFE_setWSor( OPFE *e, double wSor){
	if( 0 != HPFE_setWSor( e->hpfe__, wSor ) ){
		fprintf(stderr, "OPFE_setWSor: Unknown Error at HPFE_setWSor\n");
		return( -1 );
	}

	return( 0 );
}

double OPFE_getVector( OPFE *e ){
	return( e->v__ );
}

int OPFE_setVector( OPFE *e, double v){
	e->v__ = v;
	e->sinv__ = sin(v);
	e->cosv__ = cos(v);
	return( 0 );
}

double OPFE_getEpsilon( OPFE *e ){
	return( e->epsilon__ );
}

int OPFE_setEpsilon( OPFE *e, double epsilon){
	e->epsilon__ = epsilon;
	return( 0 );
}


int OPFE_getGridWidth( OPFE *e ){
	return( HPFE_getGridWidth( e->hpfe__ ) );
}

int OPFE_setGridWidth( OPFE *e, int width ){
	if( 0 != HPFE_setGridWidth( e->hpfe__, width ) ){
		fprintf(stderr, "OPFE_setGridWidth: Unknown Error at HPFE_setGridWidth\n");
		return(-1);
	}
	return(0);
}

int OPFE_getGridHeight( OPFE *e ){
	return( HPFE_getGridHeight( e->hpfe__ ) );
}

int OPFE_setGridHeight( OPFE *e, int height ){
	if( 0 != HPFE_setGridHeight( e->hpfe__, height ) ){
		fprintf(stderr, "OPFE_setGridHeight: Unknown Error at HPFE_setGridHeight\n");
		return(-1);
	}
	return(0);
}

#ifdef UTILS_FLOAT_DATATYPE

int OPFE_setPotential( OPFE *e, int x, int y, float potential){
	if( 0 != HPFE_setPotential( e->hpfe__, x, y, potential) ) {
		fprintf(stderr, "OPFE_setPotential: Unknown Error at HPFE_setPotential\n");
		return(-1);
	}
	return(0);
}

float OPFE_getPotential( OPFE *e, int x, int y){
	return( HPFE_getPotential( e->hpfe__, x, y) );
}

int OPFE_setAllPotential( OPFE *e, float potential ){
	if( 0 != HPFE_setAllPotential( e->hpfe__, potential) ) {
		fprintf(stderr, "OPFE_setAllPotential: Unknown Error at HPFE_setAllPotential\n");
		return(-1);
	}
	return(0);
}

#else 

int OPFE_setPotential( OPFE *e, int x, int y, double potential){
	if( 0 != HPFE_setPotential( e->hpfe__, x, y, potential) ) {
		fprintf(stderr, "OPFE_setPotential: Unknown Error at HPFE_setPotential\n");
		return(-1);
	}
	return(0);
}

double OPFE_getPotential( OPFE *e, int x, int y){
	return( HPFE_getPotential( e->hpfe__, x, y) );
}

int OPFE_setAllPotential( OPFE *e, double potential ){
	if( 0 != HPFE_setAllPotential( e->hpfe__, potential) ) {
		fprintf(stderr, "OPFE_setAllPotential: Unknown Error at HPFE_setAllPotential\n");
		return(-1);
	}
	return(0);
}

#endif

int OPFE_setOccupancy( OPFE *e, int x, int y, int occupancy){
	if( 0 != HPFE_setOccupancy( e->hpfe__, x, y, occupancy) ){
		fprintf(stderr, "OPFE_setOccupancy: Unknown Error at HPFE_setOccupancy\n");
		return(-1);
	}
	return(0);
}

int OPFE_getOccupancy( OPFE *e, int x, int y){
	return( HPFE_getOccupancy( e->hpfe__, x, y) );
}

int OPFE_getGradient( OPFE *e, Pos *position, Pos *result){
	if( 0 != HPFE_getGradient( e->hpfe__, position, result) ){
		fprintf(stderr, "OPFE_getGradient: Unknown Error at HPFE_getGradient\n");
		return(-1);
	}
	return(0);
}

int OPFE_getGradientDescent( OPFE *e, Pos *position, Pos *result){
	if( 0 != HPFE_getGradientDescent( e->hpfe__, position, result) ){
		fprintf(stderr, "OPFE_getGradientDescent: Unknown Error at HPFE_getGradientDescent\n");
		return(-1);
	}
	return(0);
}

int OPFE_setMaxNumberOfObstacle(OPFE *e,  int maxNumberOfObstacle, int list){
	if( 0 != HPFE_setMaxNumberOfObstacle(e->hpfe__, maxNumberOfObstacle, list) ){
		fprintf(stderr, "OPFE_setMaxNumberOfObstacle: Unknown Error at HPFE_setMaxNumberOfObstacle\n");
		return(-1);
	}
	return(0);
}

int OPFE_getMaxNumberOfObstacle(OPFE *e,  int list){
	return( HPFE_getMaxNumberOfObstacle(e->hpfe__, list) );
}

int OPFE_addObstacle( OPFE *e, Pos *newObstacle, int list ){
	return( HPFE_addObstacle( e->hpfe__, newObstacle, list ) );
}

int OPFE_getObstacleAtIndex( OPFE *e, int index, Pos *result, int list ){
	return( HPFE_getObstacleAtIndex( e->hpfe__, index, result, list ) );
}

int OPFE_clearListOfObstacle( OPFE *e, int list ){
	return( HPFE_clearListOfObstacle( e->hpfe__, list ) );
}

int OPFE_setMaxNumberOfEnemyRobot(OPFE *e,  int maxNumberOfEnemyRobot ){
	return( HPFE_setMaxNumberOfEnemyRobot(e->hpfe__, maxNumberOfEnemyRobot ));
}

int OPFE_getMaxNumberOfEnemyRobot(OPFE *e){
	return( HPFE_getMaxNumberOfEnemyRobot(e->hpfe__) );
}

int OPFE_addEnemyRobot( OPFE *e, Pos *newEnemyRobot){
	return( HPFE_addEnemyRobot( e->hpfe__, newEnemyRobot) );
}

int OPFE_getEnemyRobotAtIndex( OPFE *e, int index, Pos *result ){
	return( HPFE_getEnemyRobotAtIndex( e->hpfe__, index, result ) );
}

int OPFE_clearListOfEnemyRobot( OPFE *e ){
	return( HPFE_clearListOfEnemyRobot(e->hpfe__) );
}

int OPFE_setMaxNumberOfOwnRobot(OPFE *e,  int maxNumberOfOwnRobot ){
	return( HPFE_setMaxNumberOfOwnRobot(e->hpfe__, maxNumberOfOwnRobot ) );
}

int OPFE_getMaxNumberOfOwnRobot(OPFE *e){
	return( HPFE_getMaxNumberOfOwnRobot(e->hpfe__) );
}

int OPFE_addOwnRobot( OPFE *e, Pos *newOwnRobot){
	return( HPFE_addOwnRobot( e->hpfe__, newOwnRobot) );
}

int OPFE_setMe( OPFE *e, Pos *newOwnRobot){
	return( HPFE_setMe( e->hpfe__, newOwnRobot) );
}

int OPFE_getMe( OPFE *e, Pos *result ){
	return( HPFE_getMe( e->hpfe__, result) );
}

int OPFE_getOwnRobotAtIndex( OPFE *e, int index, Pos *result  ){
	return( HPFE_getOwnRobotAtIndex( e->hpfe__, index, result ) );
}

int OPFE_clearListOfOwnRobot( OPFE *e ){
	return( HPFE_clearListOfOwnRobot( e->hpfe__ ) );
}

int OPFE_setMaxNumberOfGoal(OPFE *e,  int maxNumberGoal ){
	return( HPFE_setMaxNumberOfGoal(e->hpfe__,  maxNumberGoal ) );
}

int OPFE_getMaxNumberOfGoal(OPFE *e){
	return( HPFE_getMaxNumberOfGoal(e->hpfe__) );
}

int OPFE_addGoal( OPFE *e, Pos *newGoal){
	return( HPFE_addGoal( e->hpfe__, newGoal) );
}

int OPFE_getGoalAtIndex( OPFE *e, int index, Pos *result  ){
	return( HPFE_getGoalAtIndex( e->hpfe__, index, result ) );
}

int OPFE_clearListOfGoal( OPFE *e ){
	return( HPFE_clearListOfGoal( e->hpfe__ ) );
}


/*
 ____        _                    
/ ___|  ___ | |_   _____ _ __ ___ 
\___ \ / _ \| \ \ / / _ \ '__/ __|
 ___) | (_) | |\ V /  __/ |  \__ \
|____/ \___/|_| \_/ \___|_|  |___/
*/

long int OPFE_solveI( OPFE *e, OPFE_IterateModes it, int itMax, OPFE_IterateFlags itFlags){

	double error;
	struct timeval ti, tf;
	gettimeofday( &ti, NULL);
	
	switch( it ){
		case OPFE_gsc:
			if( (OPFE_printErrors & itFlags) != 0){
				for(int i=0; i<itMax; i++){
					error = OPFE_iteratorGSC(e);
					printf("%04d: %02.020lf\n", i, error);
				}
			}
			else{
				for(int i=0; i<itMax; i++){
					error = OPFE_iteratorGSC(e);
				}
			}
		break;

		case OPFE_gsup:
			if( (OPFE_printErrors & itFlags) != 0){
				for(int i=0; i<itMax; i++){
					error = OPFE_iteratorGSUP(e);
					printf("%04d: %02.020lf\n", i, error);
				}
			}
			else{
				for(int i=0; i<itMax; i++){
					error = OPFE_iteratorGSUP(e);
				}
			}
		break;
		
		case OPFE_sorc:
			if( (OPFE_printErrors & itFlags) != 0){
				for(int i=0; i<itMax; i++){
					error = OPFE_iteratorSORC(e);
					printf("%04d: %02.020lf\n", i, error);
				}
			}
			else{
				for(int i=0; i<itMax; i++){
					error = OPFE_iteratorSORC(e);
				}
			}
		break;


		case OPFE_sorup:
			if( (OPFE_printErrors & itFlags) != 0){
				for(int i=0; i<itMax; i++){
					error = OPFE_iteratorSORUP(e);
					printf("%04d: %02.020lf\n", i, error);
				}
			}
			else{
				for(int i=0; i<itMax; i++){
					error = OPFE_iteratorSORUP(e);
				}
			}
		break;

		default: 
#ifdef UTILS_WITH_DEBUG
		fprintf(stderr, "OPFE_SolveI: Iterator not found!\n");
#endif
		break;

	}
	gettimeofday( &tf, NULL);

	return( (1000000*(tf.tv_sec - ti.tv_sec) ) + (tf.tv_usec - ti.tv_usec) );
}

long int  OPFE_solveIE( OPFE *e, OPFE_IterateModes it, int itMax, double errorMax, OPFE_IterateFlags itFlags){

	double error;
	
	struct timeval ti, tf;
	gettimeofday( &ti, NULL);
	
	errorMax = (errorMax > 0 ? errorMax : 1e-6 );
	error = 10*errorMax;
	switch( it ){
		case OPFE_gsc:
			if( (OPFE_printErrors & itFlags) != 0){
				for(int i=0; (i<itMax) && ( error>=errorMax ); i++){
					error = OPFE_iteratorGSC(e);
					printf("%04d: %02.020lf\n", i, error);
				}
			}
			else{
				for(int i=0; (i<itMax) && ( error>=errorMax ); i++){
					error = OPFE_iteratorGSC(e);
				}
			}
		break;

		case OPFE_gsup:
			if( (OPFE_printErrors & itFlags) != 0){
				for(int i=0; (i<itMax) && ( error>=errorMax ); i++){
					error = OPFE_iteratorGSUP(e);
					printf("%04d: %02.020lf\n", i, error);
				}
			}
			else{
				for(int i=0; (i<itMax) && ( error>=errorMax ); i++){
					error = OPFE_iteratorGSUP(e);
				}
			}
		break;
		
		case OPFE_sorc:
			if( (OPFE_printErrors & itFlags) != 0){
				for(int i=0; (i<itMax) && ( error>=errorMax ); i++){
					error = OPFE_iteratorSORC(e);
					printf("%04d: %02.020lf\n", i, error);
				}
			}
			else{
				for(int i=0; (i<itMax) && ( error>=errorMax ); i++){
					error = OPFE_iteratorSORC(e);
				}
			}
		break;


		case OPFE_sorup:
			if( (OPFE_printErrors & itFlags) != 0){
				for(int i=0; (i<itMax) && ( error>=errorMax ); i++){
					error = OPFE_iteratorSORUP(e);
					printf("%04d: %02.020lf\n", i, error);
				}
			}
			else{
				for(int i=0; (i<itMax) && ( error>=errorMax ); i++){
					error = OPFE_iteratorSORUP(e);
				}
			}
		break;

		default: 
#ifdef UTILS_WITH_DEBUG
		fprintf(stderr, "OPFE_SolveIE: Iterator not found!\n");
#endif
		break;

	}
	gettimeofday( &tf, NULL);

	return( (1000000*(tf.tv_sec - ti.tv_sec) ) + (tf.tv_usec - ti.tv_usec) );
}

long int OPFE_solveE( OPFE *e, OPFE_IterateModes it, double errorMax, OPFE_IterateFlags itFlags){

	double error;
	struct timeval ti, tf;
	gettimeofday( &ti, NULL);
	
	errorMax = (errorMax > 0 ? errorMax : 1e-6 );
	error = 10*errorMax;
	switch( it ){
		case OPFE_gsc:
			if( (OPFE_printErrors & itFlags) != 0){
				for(int i=0; error>=errorMax ; i++){
					error = OPFE_iteratorGSC(e);
					printf("%04d: %02.020lf\n", i, error);
				}
			}
			else{
				for(int i=0; error>=errorMax ; i++){
					error = OPFE_iteratorGSC(e);
				}
			}
		break;

		case OPFE_gsup:
			if( (OPFE_printErrors & itFlags) != 0){
				for(int i=0; error>=errorMax ; i++){
					error = OPFE_iteratorGSUP(e);
					printf("%04d: %02.020lf\n", i, error);
				}
			}
			else{
				for(int i=0; error>=errorMax ; i++){
					error = OPFE_iteratorGSUP(e);
				}
			}
		break;
		
		case OPFE_sorc:
			if( (OPFE_printErrors & itFlags) != 0){
				for(int i=0; error>=errorMax ; i++){
					error = OPFE_iteratorSORC(e);
					printf("%04d: %02.020lf\n", i, error);
				}
			}
			else{
				for(int i=0; error>=errorMax; i++){
					error = OPFE_iteratorSORC(e);
				}
			}
		break;


		case OPFE_sorup:
			if( (OPFE_printErrors & itFlags) != 0){
				for(int i=0; error>=errorMax; i++){
					error = OPFE_iteratorSORUP(e);
					printf("%04d: %02.020lf\n", i, error);
				}
			}
			else{
				for(int i=0; error>=errorMax; i++){
					error = OPFE_iteratorSORUP(e);
				}
			}
		break;

		default: 
#ifdef UTILS_WITH_DEBUG
		fprintf(stderr, "OPFE_SolveI: Iterator not found!\n");
#endif
		break;

	}
	gettimeofday( &tf, NULL);

	return( (1000000*(tf.tv_sec - ti.tv_sec) ) + (tf.tv_usec - ti.tv_usec) );
}






/*
 ___ _                 _                 
|_ _| |_ ___ _ __ __ _| |_ ___  _ __ ___ 
 | || __/ _ \ '__/ _` | __/ _ \| '__/ __|
 | || ||  __/ | | (_| | || (_) | |  \__ \
|___|\__\___|_|  \__,_|\__\___/|_|  |___/
*/


double OPFE_iteratorGSC(OPFE *e ){
	int i, j, gridWidth, gridHeight;
	double error=0.0;
#ifdef UTILS_FLOAT_DATATYPE
	float oldPotential, newPotential, left, right, top, bottom, epsAux, lambda_vx, lambda_vy;
#else
	double oldPotential, newPotential, left, right, top, bottom, epsAux, lambda_vx, lambda_vy;
#endif

	error=0.0;

	gridWidth = OPFE_getGridWidth( e );
	gridHeight = OPFE_getGridHeight( e );
	epsAux = e->epsilon__  / 8;

	lambda_vx = (e->epsilon__ * e->cosv__) / 2;
	lambda_vy = (e->epsilon__ * e->sinv__) / 2;

	for(i=0; i<(gridHeight); i++){
		for(j=0; j<(gridWidth); j++){
			if( OPFE_getOccupancy_UNSAFE(e, j, i) != FREE_CELL ){
				continue;
			}
			oldPotential = OPFE_getPotential_UNSAFE(e, j, i);
			
			if( 0==i ){
				bottom = MAX_POTENTIAL;
				top = OPFE_getPotential_UNSAFE(e, j, i+1);
			}
			else {
				if( (gridHeight-1) == i ){
					bottom=OPFE_getPotential_UNSAFE(e, j, i-1);
					top = MAX_POTENTIAL;
				}
				else{
					bottom=OPFE_getPotential_UNSAFE(e, j, i-1);
					top = OPFE_getPotential_UNSAFE(e, j, i+1);
				}
			}

			if( 0==j ){
				left = MAX_POTENTIAL;
				right = OPFE_getPotential_UNSAFE(e, j+1, i);
			}
			else {
				if( (gridWidth-1) == j ){
					left = OPFE_getPotential_UNSAFE(e, j-1, i);
					right = MAX_POTENTIAL;
				}
				else{
					left=OPFE_getPotential_UNSAFE(e, j-1, i);
					right = OPFE_getPotential_UNSAFE(e, j+1, i);
				}
			}

			//Old way
			newPotential = (top + bottom + left + right)/4;
			newPotential += epsAux*((right - left)*(e->cosv__) + (top - bottom)*(e->sinv__)) ;

			//New way
//			newPotential = 0.25 * ( (1+lambda_vx)*right + (1-lambda_vx)*left + (1+lambda_vy)*top + (1-lambda_vy)*bottom  );
			
			error+= ((newPotential-oldPotential)*(newPotential-oldPotential));
			OPFE_setPotential_UNSAFE(e, j, i, newPotential);
		}
	}
	return(error);

}


double OPFE_iteratorGSUP(OPFE *e ){
	if( e->cosv__ >=0 ){
		if( e->sinv__>=0 ){
			return( OPFE_iteratorGSUP_PP( e ) );
		}
		else{
			return( OPFE_iteratorGSUP_PN( e ) );
		}
	}
	else{
		if( e->sinv__>=0 ){
			return( OPFE_iteratorGSUP_NP( e ) );
		}
		else{
			return( OPFE_iteratorGSUP_NN( e ) );
		}
	}
}

double OPFE_iteratorGSUP_PP(OPFE *e ){
	int i, j, gridWidth, gridHeight;
	double error=0.0;
#ifdef UTILS_FLOAT_DATATYPE
	float oldPotential, newPotential, left, right, top, bottom, ro_x, ro_y;
#else
	double oldPotential, newPotential, left, right, top, bottom, ro_x, ro_y;
#endif

	error=0.0;
	
	gridWidth = OPFE_getGridWidth( e );
	gridHeight = OPFE_getGridHeight( e );


	ro_x = 1 + (e->epsilon__ * fabs(e->cosv__)) ;
	ro_y = 1 + (e->epsilon__ * fabs(e->sinv__));

	for(i=0; i<(gridHeight); i++){
		for(j=0; j<(gridWidth); j++){
			if( OPFE_getOccupancy_UNSAFE(e, j, i) != FREE_CELL ){
				continue;
			}
			oldPotential = OPFE_getPotential_UNSAFE(e, j, i);
			
			if( 0==i ){
				bottom = MAX_POTENTIAL;
				top = OPFE_getPotential_UNSAFE(e, j, i+1);
			}
			else {
				if( (gridHeight-1) == i ){
					bottom=OPFE_getPotential_UNSAFE(e, j, i-1);
					top = MAX_POTENTIAL;
				}
				else{
					bottom=OPFE_getPotential_UNSAFE(e, j, i-1);
					top = OPFE_getPotential_UNSAFE(e, j, i+1);
				}
			}

			if( 0==j ){
				left = MAX_POTENTIAL;
				right = OPFE_getPotential_UNSAFE(e, j+1, i);
			}
			else {
				if( (gridWidth-1) == j ){
					left = OPFE_getPotential_UNSAFE(e, j-1, i);
					right = MAX_POTENTIAL;
				}
				else{
					left=OPFE_getPotential_UNSAFE(e, j-1, i);
					right = OPFE_getPotential_UNSAFE(e, j+1, i);
				}
			}

			newPotential = ( ro_x*right + left + ro_y*top + bottom  ) / ( 2 + ro_x + ro_y );
			
			error+= ((newPotential-oldPotential)*(newPotential-oldPotential));
			OPFE_setPotential_UNSAFE(e, j, i, newPotential);
		}
	}
	return(error);

}


double OPFE_iteratorGSUP_PN(OPFE *e ){
	int i, j, gridWidth, gridHeight;
	double error=0.0;
#ifdef UTILS_FLOAT_DATATYPE
	float oldPotential, newPotential, left, right, top, bottom, ro_x, ro_y;
#else
	double oldPotential, newPotential, left, right, top, bottom, ro_x, ro_y;
#endif

	error=0.0;
	
	gridWidth = OPFE_getGridWidth( e );
	gridHeight = OPFE_getGridHeight( e );


	ro_x = 1 + (e->epsilon__ * fabs(e->cosv__)) ;
	ro_y = 1 + (e->epsilon__ * fabs(e->sinv__));

	for(i=0; i<(gridHeight); i++){
		for(j=0; j<(gridWidth); j++){
			if( OPFE_getOccupancy_UNSAFE(e, j, i) != FREE_CELL ){
				continue;
			}
			oldPotential = OPFE_getPotential_UNSAFE(e, j, i);
			
			if( 0==i ){
				bottom = MAX_POTENTIAL;
				top = OPFE_getPotential_UNSAFE(e, j, i+1);
			}
			else {
				if( (gridHeight-1) == i ){
					bottom=OPFE_getPotential_UNSAFE(e, j, i-1);
					top = MAX_POTENTIAL;
				}
				else{
					bottom=OPFE_getPotential_UNSAFE(e, j, i-1);
					top = OPFE_getPotential_UNSAFE(e, j, i+1);
				}
			}

			if( 0==j ){
				left = MAX_POTENTIAL;
				right = OPFE_getPotential_UNSAFE(e, j+1, i);
			}
			else {
				if( (gridWidth-1) == j ){
					left = OPFE_getPotential_UNSAFE(e, j-1, i);
					right = MAX_POTENTIAL;
				}
				else{
					left=OPFE_getPotential_UNSAFE(e, j-1, i);
					right = OPFE_getPotential_UNSAFE(e, j+1, i);
				}
			}

			newPotential = ( ro_x*right + left + top + ro_y*bottom  ) / ( 2 + ro_x + ro_y );
			
			error+= ((newPotential-oldPotential)*(newPotential-oldPotential));
			OPFE_setPotential_UNSAFE(e, j, i, newPotential);
		}
	}
	return(error);
}


double OPFE_iteratorGSUP_NP(OPFE *e ){
	int i, j, gridWidth, gridHeight;
	double error=0.0;
#ifdef UTILS_FLOAT_DATATYPE
	float oldPotential, newPotential, left, right, top, bottom, ro_x, ro_y;
#else
	double oldPotential, newPotential, left, right, top, bottom, ro_x, ro_y;
#endif

	error=0.0;
	
	gridWidth = OPFE_getGridWidth( e );
	gridHeight = OPFE_getGridHeight( e );


	ro_x = 1 + (e->epsilon__ * fabs(e->cosv__)) ;
	ro_y = 1 + (e->epsilon__ * fabs(e->sinv__));

	for(i=0; i<(gridHeight); i++){
		for(j=0; j<(gridWidth); j++){
			if( OPFE_getOccupancy_UNSAFE(e, j, i) != FREE_CELL ){
				continue;
			}
			oldPotential = OPFE_getPotential_UNSAFE(e, j, i);
			
			if( 0==i ){
				bottom = MAX_POTENTIAL;
				top = OPFE_getPotential_UNSAFE(e, j, i+1);
			}
			else {
				if( (gridHeight-1) == i ){
					bottom=OPFE_getPotential_UNSAFE(e, j, i-1);
					top = MAX_POTENTIAL;
				}
				else{
					bottom=OPFE_getPotential_UNSAFE(e, j, i-1);
					top = OPFE_getPotential_UNSAFE(e, j, i+1);
				}
			}

			if( 0==j ){
				left = MAX_POTENTIAL;
				right = OPFE_getPotential_UNSAFE(e, j+1, i);
			}
			else {
				if( (gridWidth-1) == j ){
					left = OPFE_getPotential_UNSAFE(e, j-1, i);
					right = MAX_POTENTIAL;
				}
				else{
					left=OPFE_getPotential_UNSAFE(e, j-1, i);
					right = OPFE_getPotential_UNSAFE(e, j+1, i);
				}
			}

			newPotential = ( right + ro_x*left + ro_y*top + bottom  ) / ( 2 + ro_x + ro_y );
			
			error+= ((newPotential-oldPotential)*(newPotential-oldPotential));
			OPFE_setPotential_UNSAFE(e, j, i, newPotential);
		}
	}
	return(error);

}


double OPFE_iteratorGSUP_NN(OPFE *e ){
	int i, j, gridWidth, gridHeight;
	double error=0.0;
#ifdef UTILS_FLOAT_DATATYPE
	float oldPotential, newPotential, left, right, top, bottom, ro_x, ro_y;
#else
	double oldPotential, newPotential, left, right, top, bottom, ro_x, ro_y;
#endif

	error=0.0;
	
	gridWidth = OPFE_getGridWidth( e );
	gridHeight = OPFE_getGridHeight( e );


	ro_x = 1 + (e->epsilon__ * fabs(e->cosv__)) ;
	ro_y = 1 + (e->epsilon__ * fabs(e->sinv__));

	for(i=0; i<(gridHeight); i++){
		for(j=0; j<(gridWidth); j++){
			if( OPFE_getOccupancy_UNSAFE(e, j, i) != FREE_CELL ){
				continue;
			}
			oldPotential = OPFE_getPotential_UNSAFE(e, j, i);
			
			if( 0==i ){
				bottom = MAX_POTENTIAL;
				top = OPFE_getPotential_UNSAFE(e, j, i+1);
			}
			else {
				if( (gridHeight-1) == i ){
					bottom=OPFE_getPotential_UNSAFE(e, j, i-1);
					top = MAX_POTENTIAL;
				}
				else{
					bottom=OPFE_getPotential_UNSAFE(e, j, i-1);
					top = OPFE_getPotential_UNSAFE(e, j, i+1);
				}
			}

			if( 0==j ){
				left = MAX_POTENTIAL;
				right = OPFE_getPotential_UNSAFE(e, j+1, i);
			}
			else {
				if( (gridWidth-1) == j ){
					left = OPFE_getPotential_UNSAFE(e, j-1, i);
					right = MAX_POTENTIAL;
				}
				else{
					left=OPFE_getPotential_UNSAFE(e, j-1, i);
					right = OPFE_getPotential_UNSAFE(e, j+1, i);
				}
			}

			newPotential = ( right + ro_x*left + top + ro_y*bottom  ) / ( 2 + ro_x + ro_y );
			
			error+= ((newPotential-oldPotential)*(newPotential-oldPotential));
			OPFE_setPotential_UNSAFE(e, j, i, newPotential);
		}
	}
	return(error);

}

double OPFE_iteratorSORC(OPFE *e ){
	int i, j, gridWidth, gridHeight;
	double error=0.0;
#ifdef UTILS_FLOAT_DATATYPE
	float oldPotential, newPotential, left, right, top, bottom, epsAux, lambda_vx, lambda_vy, wsor;
#else
	double oldPotential, newPotential, left, right, top, bottom, epsAux, lambda_vx, lambda_vy, wsor;
#endif

	error=0.0;

	gridWidth = OPFE_getGridWidth( e );
	gridHeight = OPFE_getGridHeight( e );
	epsAux = e->epsilon__  / 8.0f;

	lambda_vx = (e->epsilon__ * e->cosv__) / 2.0f;
	lambda_vy = (e->epsilon__ * e->sinv__) / 2.0f;

	wsor = OPFE_getWSor( e );

	for(i=0; i<(gridHeight); i++){
		for(j=0; j<(gridWidth); j++){
			if( OPFE_getOccupancy_UNSAFE(e, j, i) != FREE_CELL ){
				continue;
			}
			oldPotential = OPFE_getPotential_UNSAFE(e, j, i);
			
			if( 0==i ){
				bottom = MAX_POTENTIAL;
				top = OPFE_getPotential_UNSAFE(e, j, i+1);
			}
			else {
				if( (gridHeight-1) == i ){
					bottom=OPFE_getPotential_UNSAFE(e, j, i-1);
					top = MAX_POTENTIAL;
				}
				else{
					bottom=OPFE_getPotential_UNSAFE(e, j, i-1);
					top = OPFE_getPotential_UNSAFE(e, j, i+1);
				}
			}

			if( 0==j ){
				left = MAX_POTENTIAL;
				right = OPFE_getPotential_UNSAFE(e, j+1, i);
			}
			else {
				if( (gridWidth-1) == j ){
					left = OPFE_getPotential_UNSAFE(e, j-1, i);
					right = MAX_POTENTIAL;
				}
				else{
					left=OPFE_getPotential_UNSAFE(e, j-1, i);
					right = OPFE_getPotential_UNSAFE(e, j+1, i);
				}
			}

			//Old way
			newPotential = (top + bottom + left + right)/4.0;
			newPotential += epsAux*((right - left)*(e->cosv__) + (top - bottom)*(e->sinv__)) ;

			//New way
//			newPotential = 0.25 * ( (1+lambda_vx)*right + (1-lambda_vx)*left + (1+lambda_vy)*top + (1-lambda_vy)*bottom  );


			newPotential =  ((1-wsor)*oldPotential ) +  (wsor*newPotential );//SORing

			error+= ((newPotential-oldPotential)*(newPotential-oldPotential));
			OPFE_setPotential_UNSAFE(e, j, i, newPotential);
		}
	}
	return(error);

}


double OPFE_iteratorSORUP(OPFE *e ){
	if( e->cosv__ >=0 ){
		if( e->sinv__>=0 ){
			return( OPFE_iteratorSORUP_PP( e ) );
		}
		else{
			return( OPFE_iteratorSORUP_PN( e ) );
		}
	}
	else{
		if( e->sinv__>=0 ){
			return( OPFE_iteratorSORUP_NP( e ) );
		}
		else{
			return( OPFE_iteratorSORUP_NN( e ) );
		}
	}
}


double OPFE_iteratorSORUP_PP(OPFE *e ){
	int i, j, gridWidth, gridHeight;
	double error=0.0;
#ifdef UTILS_FLOAT_DATATYPE
	float oldPotential, newPotential, left, right, top, bottom, ro_x, ro_y, wsor;
#else
	double oldPotential, newPotential, left, right, top, bottom, ro_x, ro_y, wsor;
#endif

	error=0.0;

	gridWidth = OPFE_getGridWidth( e );
	gridHeight = OPFE_getGridHeight( e );


	ro_x = 1 + (e->epsilon__ * fabs(e->cosv__)) ;
	ro_y = 1 + (e->epsilon__ * fabs(e->sinv__));

	wsor = OPFE_getWSor( e );

	for(i=0; i<(gridHeight); i++){
		for(j=0; j<(gridWidth); j++){
			if( OPFE_getOccupancy_UNSAFE(e, j, i) != FREE_CELL ){
				continue;
			}
			oldPotential = OPFE_getPotential_UNSAFE(e, j, i);
			
			if( 0==i ){
				bottom = MAX_POTENTIAL;
				top = OPFE_getPotential_UNSAFE(e, j, i+1);
			}
			else {
				if( (gridHeight-1) == i ){
					bottom=OPFE_getPotential_UNSAFE(e, j, i-1);
					top = MAX_POTENTIAL;
				}
				else{
					bottom=OPFE_getPotential_UNSAFE(e, j, i-1);
					top = OPFE_getPotential_UNSAFE(e, j, i+1);
				}
			}

			if( 0==j ){
				left = MAX_POTENTIAL;
				right = OPFE_getPotential_UNSAFE(e, j+1, i);
			}
			else {
				if( (gridWidth-1) == j ){
					left = OPFE_getPotential_UNSAFE(e, j-1, i);
					right = MAX_POTENTIAL;
				}
				else{
					left=OPFE_getPotential_UNSAFE(e, j-1, i);
					right = OPFE_getPotential_UNSAFE(e, j+1, i);
				}
			}

			newPotential = ( ro_x*right + left + ro_y*top + bottom  ) / ( 2 + ro_x + ro_y );

			newPotential =  ((1-wsor)*oldPotential ) +  (wsor*newPotential );//SORing
			
			error+= ((newPotential-oldPotential)*(newPotential-oldPotential));
			OPFE_setPotential_UNSAFE(e, j, i, newPotential);
		}
	}
	return(error);

}


double OPFE_iteratorSORUP_PN(OPFE *e ){
	int i, j, gridWidth, gridHeight;
	double error=0.0;
#ifdef UTILS_FLOAT_DATATYPE
	float oldPotential, newPotential, left, right, top, bottom, ro_x, ro_y, wsor;
#else
	double oldPotential, newPotential, left, right, top, bottom, ro_x, ro_y, wsor;
#endif

	error=0.0;

	gridWidth = OPFE_getGridWidth( e );
	gridHeight = OPFE_getGridHeight( e );


	ro_x = 1 + (e->epsilon__ * fabs(e->cosv__)) ;
	ro_y = 1 + (e->epsilon__ * fabs(e->sinv__));

	wsor = OPFE_getWSor( e );

	for(i=0; i<(gridHeight); i++){
		for(j=0; j<(gridWidth); j++){
			if( OPFE_getOccupancy_UNSAFE(e, j, i) != FREE_CELL ){
				continue;
			}
			oldPotential = OPFE_getPotential_UNSAFE(e, j, i);
			
			if( 0==i ){
				bottom = MAX_POTENTIAL;
				top = OPFE_getPotential_UNSAFE(e, j, i+1);
			}
			else {
				if( (gridHeight-1) == i ){
					bottom=OPFE_getPotential_UNSAFE(e, j, i-1);
					top = MAX_POTENTIAL;
				}
				else{
					bottom=OPFE_getPotential_UNSAFE(e, j, i-1);
					top = OPFE_getPotential_UNSAFE(e, j, i+1);
				}
			}

			if( 0==j ){
				left = MAX_POTENTIAL;
				right = OPFE_getPotential_UNSAFE(e, j+1, i);
			}
			else {
				if( (gridWidth-1) == j ){
					left = OPFE_getPotential_UNSAFE(e, j-1, i);
					right = MAX_POTENTIAL;
				}
				else{
					left=OPFE_getPotential_UNSAFE(e, j-1, i);
					right = OPFE_getPotential_UNSAFE(e, j+1, i);
				}
			}

			newPotential = ( ro_x*right + left + top + ro_y*bottom  ) / ( 2 + ro_x + ro_y );

			newPotential =  ((1-wsor)*oldPotential ) +  (wsor*newPotential );//SORing
			
			error+= ((newPotential-oldPotential)*(newPotential-oldPotential));
			OPFE_setPotential_UNSAFE(e, j, i, newPotential);
		}
	}
	return(error);

}

double OPFE_iteratorSORUP_NP(OPFE *e ){
	int i, j, gridWidth, gridHeight;
	double error=0.0;
#ifdef UTILS_FLOAT_DATATYPE
	float oldPotential, newPotential, left, right, top, bottom, ro_x, ro_y, wsor;
#else
	double oldPotential, newPotential, left, right, top, bottom, ro_x, ro_y, wsor;
#endif

	error=0.0;

	gridWidth = OPFE_getGridWidth( e );
	gridHeight = OPFE_getGridHeight( e );


	ro_x = 1 + (e->epsilon__ * fabs(e->cosv__)) ;
	ro_y = 1 + (e->epsilon__ * fabs(e->sinv__));

	wsor = OPFE_getWSor( e );

	for(i=0; i<(gridHeight); i++){
		for(j=0; j<(gridWidth); j++){
			if( OPFE_getOccupancy_UNSAFE(e, j, i) != FREE_CELL ){
				continue;
			}
			oldPotential = OPFE_getPotential_UNSAFE(e, j, i);
			
			if( 0==i ){
				bottom = MAX_POTENTIAL;
				top = OPFE_getPotential_UNSAFE(e, j, i+1);
			}
			else {
				if( (gridHeight-1) == i ){
					bottom=OPFE_getPotential_UNSAFE(e, j, i-1);
					top = MAX_POTENTIAL;
				}
				else{
					bottom=OPFE_getPotential_UNSAFE(e, j, i-1);
					top = OPFE_getPotential_UNSAFE(e, j, i+1);
				}
			}

			if( 0==j ){
				left = MAX_POTENTIAL;
				right = OPFE_getPotential_UNSAFE(e, j+1, i);
			}
			else {
				if( (gridWidth-1) == j ){
					left = OPFE_getPotential_UNSAFE(e, j-1, i);
					right = MAX_POTENTIAL;
				}
				else{
					left=OPFE_getPotential_UNSAFE(e, j-1, i);
					right = OPFE_getPotential_UNSAFE(e, j+1, i);
				}
			}

			newPotential = ( right + ro_x*left + ro_y*top + bottom  ) / ( 2 + ro_x + ro_y );

			newPotential =  ((1-wsor)*oldPotential ) +  (wsor*newPotential );//SORing
			
			error+= ((newPotential-oldPotential)*(newPotential-oldPotential));
			OPFE_setPotential_UNSAFE(e, j, i, newPotential);
		}
	}
	return(error);

}

double OPFE_iteratorSORUP_NN(OPFE *e ){
	int i, j, gridWidth, gridHeight;
	double error=0.0;
#ifdef UTILS_FLOAT_DATATYPE
	float oldPotential, newPotential, left, right, top, bottom, ro_x, ro_y, wsor;
#else
	double oldPotential, newPotential, left, right, top, bottom, ro_x, ro_y, wsor;
#endif

	error=0.0;

	gridWidth = OPFE_getGridWidth( e );
	gridHeight = OPFE_getGridHeight( e );


	ro_x = 1 + (e->epsilon__ * fabs(e->cosv__)) ;
	ro_y = 1 + (e->epsilon__ * fabs(e->sinv__));

	wsor = OPFE_getWSor( e );

	for(i=0; i<(gridHeight); i++){
		for(j=0; j<(gridWidth); j++){
			if( OPFE_getOccupancy_UNSAFE(e, j, i) != FREE_CELL ){
				continue;
			}
			oldPotential = OPFE_getPotential_UNSAFE(e, j, i);
			
			if( 0==i ){
				bottom = MAX_POTENTIAL;
				top = OPFE_getPotential_UNSAFE(e, j, i+1);
			}
			else {
				if( (gridHeight-1) == i ){
					bottom=OPFE_getPotential_UNSAFE(e, j, i-1);
					top = MAX_POTENTIAL;
				}
				else{
					bottom=OPFE_getPotential_UNSAFE(e, j, i-1);
					top = OPFE_getPotential_UNSAFE(e, j, i+1);
				}
			}

			if( 0==j ){
				left = MAX_POTENTIAL;
				right = OPFE_getPotential_UNSAFE(e, j+1, i);
			}
			else {
				if( (gridWidth-1) == j ){
					left = OPFE_getPotential_UNSAFE(e, j-1, i);
					right = MAX_POTENTIAL;
				}
				else{
					left=OPFE_getPotential_UNSAFE(e, j-1, i);
					right = OPFE_getPotential_UNSAFE(e, j+1, i);
				}
			}

			newPotential = ( right + ro_x*left + top + ro_y*bottom  ) / ( 2 + ro_x + ro_y );

			newPotential =  ((1-wsor)*oldPotential ) +  (wsor*newPotential );//SORing
			
			error+= ((newPotential-oldPotential)*(newPotential-oldPotential));
			OPFE_setPotential_UNSAFE(e, j, i, newPotential);
		}
	}
	return(error);

}
