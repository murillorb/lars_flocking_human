#ifndef BASE_ENVIRONMENT_H
#define BASE_ENVIRONMENT_H
#include "utils.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <time.h>


//uncoment just once!
//#define BE_CLEAR_WITH_ZEROING_OCCUPIED_MEMORY
//#define BE_CLEAR_WITH_ZEROING_ALLOCATED_MEMORY



//BE_ to identify baseEnvironment routines
//follow the naming pattern... example: BE__setMaxNumberOf{Obstacle, EnemyRobot, OwnRobot, Goal}
//pay attention to the fact that there're two obstacles lists, so functions that deals with obstacles has an extra argument that specifies what list is being used on that moment

typedef struct{
	Pos *obstacle__[2]; //two kinds of obstacles
	Pos *enemyRobot__, *ownRobot__; 
	Pos *goal__;

	int nObstacle__[2], maxNumberOfObstacle__[2], \
	    nEnemyRobot__, maxNumberOfEnemyRobot__,\
	    nOwnRobot__, maxNumberOfOwnRobot__,\
	    nGoal__, maxNumberOfGoal__;

	unsigned int flags__;

	}BE; //BaseEnvironment
	


BE* BE_new( int maxNumberOfObstacle1, int maxNumberOfObstacle2, int maxNumberOfEnemyRobot, int maxNumberOfOwnRobot, int maxNumberOfGoal);//allocate mem and set default values to the base environment
void BE_delete( BE *e );//delete a BE type.
int BE_clearAllLists( BE *e );
int BE_test( FILE *out );

//Serialization functions
inline int BE_getStreamSizeObstacle( BE *e, int list){
	if( e == NULL ){
	   fprintf(stderr, "\n[ERROR]BE_getStreamSizeObstacle: BE struct is NULL!\n");
		return(-1);
	}
   return( (2*(e->nObstacle__[list])) + 2 );
}

inline int BE_getStreamSizeEnemyRobot( BE *e ){
	if( e == NULL ){
	   fprintf(stderr, "\n[ERROR]BE_getStreamSizeEnemyRobot: BE struct is NULL!\n");
		return(-1);
	}
   return( (2*(e->nEnemyRobot__)) + 2 );
}

inline int BE_getStreamSizeOwnRobot( BE *e ){
	if( e == NULL ){
	   fprintf(stderr, "\n[ERROR]BE_getStreamSizeOwnRobot: BE struct is NULL!\n");
		return(-1);
	}
   return( (2*(e->nOwnRobot__ +1)) + 4 );
}

inline int BE_getStreamSizeGoal( BE *e ){
	if( e == NULL ){
	   fprintf(stderr, "\n[ERROR]BE_getStreamSizeGoal: BE struct is NULL!\n");
		return(-1);
	}
   return( (2*(e->nGoal__)) + 2 );
}

inline int BE_getStreamSize( BE *e ){
	if( e == NULL ){
	   fprintf(stderr, "\n[ERROR]BE_getStreamSize: BE struct is NULL!\n");
		return(-1);
	}
   return( 6 + BE_getStreamSizeObstacle(e,0) + BE_getStreamSizeObstacle(e,1) + BE_getStreamSizeEnemyRobot(e) + BE_getStreamSizeOwnRobot(e) + BE_getStreamSizeGoal(e) );
}


int BE_toDoubleStreamObstacle( BE *e, double *str, int list);
int BE_toDoubleStreamEnemyRobot( BE *e, double *str);
int BE_toDoubleStreamOwnRobot( BE *e, double *str);
int BE_toDoubleStreamGoal( BE *e, double *str);
int BE_toDoubleStream( BE *e, double *stream);


int BE_getFromDoubleStream( BE *e, double *str );
int BE_getFromDoubleStreamObstacle( BE *e, double *str, int list);
int BE_getFromDoubleStreamEnemyRobot( BE *e, double *str);
int BE_getFromDoubleStreamOwnRobot( BE *e, double *str);
int BE_getFromDoubleStreamGoal( BE *e, double *str);


int BE_toFloatStreamObstacle( BE *e, float *str, int list);
int BE_toFloatStreamEnemyRobot( BE *e, float *str);
int BE_toFloatStreamOwnRobot( BE *e, float *str);
int BE_toFloatStreamGoal( BE *e, float *str);
int BE_toFloatStream( BE *e, float *stream);


int BE_getFromFloatStream( BE *e, float *str );
int BE_getFromFloatStreamObstacle( BE *e, float *str, int list);
int BE_getFromFloatStreamEnemyRobot( BE *e, float *str);
int BE_getFromFloatStreamOwnRobot( BE *e, float *str);
int BE_getFromFloatStreamGoal( BE *e, float *str);




int BE_setMaxNumberOfObstacle(BE *e,  int maxNumberOfObstacle, int list);
int BE_getMaxNumberOfObstacle(BE *e,  int list);
int BE_getNumberOfObstacle(BE *e,  int list);
int BE_addObstacle( BE *e, Pos *newObstacle, int list );
int BE_getObstacleAtIndex( BE *e, int index, Pos *result, int list );
int BE_clearListOfObstacle( BE *e, int list );


int BE_setMaxNumberOfEnemyRobot(BE *e,  int maxNumberOfEnemyRobot );
int BE_getMaxNumberOfEnemyRobot(BE *e);
int BE_getNumberOfEnemyRobot(BE *e);
int BE_addEnemyRobot( BE *e, Pos *newEnemyRobot);
int BE_getEnemyRobotAtIndex( BE *e, int index, Pos *result );
int BE_clearListOfEnemyRobot( BE *e );


int BE_setMaxNumberOfOwnRobot(BE *e,  int maxNumberOfOwnRobot );
int BE_getMaxNumberOfOwnRobot(BE *e);
int BE_getNumberOfOwnRobot(BE *e);
int BE_addOwnRobot( BE *e, Pos *newOwnRobot);
int BE_setMe( BE *e, Pos *newOwnRobot);
int BE_getMe( BE *e, Pos *result );
int BE_getOwnRobotAtIndex( BE *e, int index, Pos *result  );
int BE_clearListOfOwnRobot( BE *e );


int BE_setMaxNumberOfGoal(BE *e,  int maxNumberGoal );
int BE_getMaxNumberOfGoal(BE *e);
int BE_getNumberOfGoal(BE *e);
int BE_addGoal( BE *e, Pos *newGoal);
int BE_getGoalAtIndex( BE *e, int index, Pos *result  );
int BE_clearListOfGoal( BE *e );


#endif

