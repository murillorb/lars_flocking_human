#include "lib_be.hh"

//allocate mem and set default values to the base environment
BE* BE_new( int maxNumberOfObstacle1, int maxNumberOfObstacle2, int maxNumberOfEnemyRobot, int maxNumberOfOwnRobot, int maxNumberOfGoal){
	BE *e = (BE *) myAllocator( 1, sizeof(BE) ); //malloc( sizeof(BE) );
	if( e == NULL ){
		fprintf(stderr, "ERROR: BE_new couldn\'t allocate memory to BE Struct!\n");
		return( NULL );
	}

	e->flags__ = 0;
	
	e->obstacle__[0]  = NULL;
	e->obstacle__[1]  = NULL;
	e->enemyRobot__   = NULL;
	e->ownRobot__     = NULL;
	e->goal__         = NULL;
	
	//Allocating memory to first obstacle list
	if( BE_setMaxNumberOfObstacle(e, maxNumberOfObstacle1, 0) < 0 ){
		fprintf(stderr, "ERROR: BE_new couldn\'t allocate memory to the FIRST obstacleList!\n");
	   e->obstacle__[0]  = NULL;
		BE_delete(e);//if couldnt allocate, release all mem
		return( NULL );//if one is impossible, then nothing
	}

	//Allocating memory to second obstacle list
	if( BE_setMaxNumberOfObstacle(e, maxNumberOfObstacle2, 1) < 0 ){
		fprintf(stderr, "ERROR: BE_new couldn\'t allocate memory to the SECOND obstacleList!\n");
	   e->obstacle__[1]  = NULL;
		BE_delete(e);//if couldnt allocate, release all mem
		return( NULL );//if one is impossible, then nothing
	}	
	
	//allocate mem to enemy robot list
	if( BE_setMaxNumberOfEnemyRobot(e, maxNumberOfEnemyRobot) < 0 ){
		fprintf(stderr, "ERROR: BE_new couldn\'t allocate memory to the enemyRobotList!\n");
	   e->enemyRobot__  = NULL;
		BE_delete(e);//if couldnt allocate, release all mem
		return( NULL );
	}
	
	//allocat mem to own robots list
	if( BE_setMaxNumberOfOwnRobot(e, maxNumberOfOwnRobot) < 0 ){
		fprintf(stderr, "ERROR: BE_new couldn\'t allocate memory to the ownRobotList!\n");
	   e->ownRobot__  = NULL;
		BE_delete(e);//if couldnt allocate, release all mem
		return( NULL );
	}
	
	//allocat mem to goals  list
	if( BE_setMaxNumberOfGoal(e, maxNumberOfGoal) < 0 ) {
		fprintf(stderr, "ERROR: BE_new couldn\'t allocate memory to the goalList!\n");
	   e->goal__  = NULL;
		BE_delete(e);//if couldnt allocate, release all mem
		return( NULL );
	}	
	
	
	return( e );
}

int BE_clearAllLists( BE *e ){
	int result=0;
	if ( BE_clearListOfObstacle(e, 0) != 0)
		result-=1;
	if( BE_clearListOfObstacle(e, 1) != 0 )
		result-=2;
	if( BE_clearListOfEnemyRobot(e)  != 0 )
		result-=4;
	if( BE_clearListOfOwnRobot(e)  != 0 )
		result-=8;
	if( BE_clearListOfGoal(e) != 0 )
		result-=16;
	return( result );
	};


//delete a BE type.
void BE_delete( BE *e ){
	if( e == NULL ){
		return;
	}

	if( NULL != e->obstacle__[0])    free( e->obstacle__[0] );
	if( NULL != e->obstacle__[1] )   free( e->obstacle__[1] );
	if( NULL != e->enemyRobot__ )    free( e->enemyRobot__ );
	if( NULL != e->ownRobot__ )      free( e->ownRobot__ );
	if( NULL != e->goal__ )          free( e->goal__ );
		
	free( e );
	return;
}


int BE_toDoubleStreamObstacle( BE *e, double *str, int list){
   int      k; //Pos iterator
   Pos      aux;
   
	if( e == NULL ){
	   fprintf(stderr, "\n[ERROR]BE_toDoubleStreamObstacle: BE struct is NULL!\n");
		return(-1);
	}   

   str[0] = (e->maxNumberOfObstacle__[list]);
   str[1] = (e->nObstacle__[list]);
   
   for( k=0; k<(e->nObstacle__[list]); k++) {
      BE_getObstacleAtIndex( e, k, &aux, list);
      str[2*(k+1)]         = aux.x__;
      str[(2*(k+1)) + 1]   = aux.y__;
   }
   
   return( 0 );
}

int BE_toDoubleStreamEnemyRobot( BE *e, double *str){
   int      k; //Pos iterator
   Pos      aux;
   
	if( e == NULL ){
	   fprintf(stderr, "\n[ERROR]BE_toDoubleStreamEnemyRobot: BE struct is NULL!\n");
		return(-1);
	}   

   str[0] = (e->maxNumberOfEnemyRobot__);
   str[1] = (e->nEnemyRobot__);
   
   for( k=0; k<(e->nEnemyRobot__); k++) {
      BE_getEnemyRobotAtIndex( e, k, &aux);
      str[2*(k+1)]         = aux.x__;
      str[(2*(k+1)) + 1]   = aux.y__;
   }
   
   return( 0 );
}

int BE_toDoubleStreamOwnRobot( BE *e, double *str){
   int      k; //Pos iterator
   Pos      aux;
   
	if( e == NULL ){
	   fprintf(stderr, "\n[ERROR]BE_toDoubleStreamOwnRobot: BE struct is NULL!\n");
		return(-1);
	}   

   str[0] = (e->maxNumberOfOwnRobot__);
   str[1] = (e->nOwnRobot__);

   BE_getMe( e, &aux);
   str[2] = aux.x__;
   str[3] = aux.y__;
   
   for( k=0; k<(e->nOwnRobot__); k++) {
      BE_getOwnRobotAtIndex( e, k, &aux);
      str[2*(k+2)]         = aux.x__;
      str[(2*(k+2)) + 1]   = aux.y__;
   }
     
   return( 0 );
}

int BE_toDoubleStreamGoal( BE *e, double *str){
   int      k; //Pos iterator
   Pos      aux;
   
	if( e == NULL ){
	   fprintf(stderr, "\n[ERROR]BE_toDoubleStreamOwnRobot: BE struct is NULL!\n");
		return(-1);
	}   

   str[0] = (e->maxNumberOfGoal__);
   str[1] = (e->nGoal__);
   
   for( k=0; k<(e->nGoal__); k++) {
      BE_getGoalAtIndex( e, k, &aux);
      str[2*(k+1)]         = aux.x__;
      str[(2*(k+1)) + 1]   = aux.y__;
   }
   
   return( 0 );
}



int BE_toDoubleStream( BE *e, double *stream){
   int      nTotal, //total of elements
            nStrSizeObstacle[2], nStrSizeEnemyRobot, nStrSizeOwnRobot, nStrSizeGoal, nStrHeader, //size of stream parts
            offset, 
            k; //Pos iterator
   double   *auxStream;// Just an alias to the double stream that will contain a BE struct
   
	if( e == NULL ){
	   fprintf(stderr, "\n[ERROR]BE_toDoubleStream: BE struct is NULL!\n");
		return(-1);
	}
	
   nStrSizeObstacle[0] = (2*(e->nObstacle__[0])) + 2; //obtacle 0
   nStrSizeObstacle[1] = (2*(e->nObstacle__[1])) + 2; //obtacle 1
   nStrSizeEnemyRobot  = (2*(e->nEnemyRobot__)) + 2; //enemy robot
   nStrSizeOwnRobot    = (2*(e->nOwnRobot__ +1)) + 2; //own robot; pay attention to "+1", that came from "me own robot"
   nStrSizeGoal        = (2*(e->nGoal__)) + 2; //goal
   nStrHeader          = 6;
   
   nTotal = nStrHeader + nStrSizeObstacle[0] + nStrSizeObstacle[1] + nStrSizeEnemyRobot + nStrSizeOwnRobot + nStrSizeGoal;

   auxStream = (double*) myAllocator( UTILS_MAX5( nStrSizeObstacle[0], nStrSizeObstacle[1], nStrSizeEnemyRobot, nStrSizeOwnRobot, nStrSizeGoal), sizeof(double) );


	if( NULL == auxStream ){
	   fprintf(stderr, "\n[ERROR]BE_toDoubleStream: Couldn\'t allocate memory to auxStream!\n");
		return(-1);
	}

   //header
   stream[0] = nTotal;
   stream[1] = nStrHeader;//offset where obstacle 0 starts
   stream[2] = stream[1] + nStrSizeObstacle[0];//offset where obstacle 1 starts
   stream[3] = stream[2] + nStrSizeObstacle[1];//offset where enemy robot starts
   stream[4] = stream[3] + nStrSizeEnemyRobot;//offset where own robot starts
   stream[5] = stream[4] + nStrSizeOwnRobot;//offset where goal starts

  //obstacle 0
   offset = (int)(stream[1]);
   BE_toDoubleStreamObstacle( e, auxStream, 0);
   for( k=0; k<((2*auxStream[1]) +2); k++) {
      stream[offset + k] = auxStream[k];
   }
   
   //obstacle 1
   offset = (int)(stream[2]);
   BE_toDoubleStreamObstacle( e, auxStream, 1);
   for( k=0; k<((2*auxStream[1]) +2); k++) {
      stream[offset + k] = auxStream[k];
   }
    
   //enemy robot
   offset = (int)(stream[3]);
   BE_toDoubleStreamEnemyRobot( e, auxStream);
   for( k=0; k<((2*auxStream[1]) +2); k++) {
      stream[offset + k] = auxStream[k];
   }
 
   //own robot
   offset = (int)(stream[4]);
   BE_toDoubleStreamOwnRobot( e, auxStream);
   for( k=0; k<((2*auxStream[1]) +4); k++) {
      stream[offset + k] = auxStream[k];
   }
  
   //goal
   offset = (int)(stream[5]);
   BE_toDoubleStreamGoal( e, auxStream);
   for( k=0; k<((2*auxStream[1]) +2); k++) {
      stream[offset + k] = auxStream[k];
   }

   free( auxStream );

   return( 0 );
}


int BE_getFromDoubleStreamObstacle( BE *e, double *str, int list){
   int      k; //Pos iterator
   Pos      aux;

   //checking arguments
	if( list<0 || list>1 ){
		fprintf(stderr, "\n[ERROR]BE_getFromDoubleStreamObstacle[%d]: Invalid list [%d]!\n", list, list);
		return( -1 );
	}
	if( e == NULL ){
	   fprintf(stderr, "\n[ERROR]BE_getFromDoubleStreamObstacle[%d]: BE struct is NULL!\n", list);
		return(-2);
	}
	
	if( NULL == str ){
	   fprintf(stderr, "\n[ERROR]BE_getFromDoubleStreamObstacle[%d]: stream is NULL!\n", list);
		return(-3);	
	}
	
	//checking integrity
   if( (str[0]<0) || (str[1]<0) ){
	   fprintf(stderr, "\n[ERROR]BE_getFromDoubleStreamObstacle[%d]: MaxNumberOfObstacle[%lf] and/or nObstacle[%lf] are negative \n", list, str[0], str[1]);
		return(-4);   
   }

   //setting maximum number of obstacles at list [list]
   if( BE_setMaxNumberOfObstacle( e, (int) str[0], list ) < 0 ){
	   fprintf(stderr, "\n[ERROR]BE_getFromDoubleStreamObstacle[%d]: Couldn\'t set maxNumberOfObstacle[%lf]!\n", list, str[0]);
		return(-5);   
   }

   for( k=0; k<str[1]; k++) {
      aux.x__ = str[2*(k+1)];
      aux.y__ = str[(2*(k+1)) + 1];
      //try and check if objects are added
      if( BE_addObstacle( e, &aux, list) < 0 ){
         fprintf(stderr, "\n[ERROR]BE_getFromDoubleStreamObstacle[%d]: Couldn\'t add obstacle(%lf, %lf)!\n", list, aux.x__, aux.y__);
		   return(-6);
      }
      
   }
    
   return( 0 );
}


int BE_getFromDoubleStreamEnemyRobot( BE *e, double *str){
   int      k; //Pos iterator
   Pos      aux;

	if( e == NULL ){
	   fprintf(stderr, "\n[ERROR]BE_getFromDoubleStreamEnemyRobot: BE struct is NULL!\n");
		return(-1);
	}
	
	if( NULL == str ){
	   fprintf(stderr, "\n[ERROR]BE_getFromDoubleStreamEnemyRobot: stream is NULL!\n");
		return(-2);	
	}
	
	
	//checking integrity
   if( (str[0]<0) || (str[1]<0) ){
	   fprintf(stderr, "\n[ERROR]BE_getFromDoubleStreamEnemyRobot: MaxNumberOfEnemyRobot[%lf] and/or nEnemyRobot[%lf] are negative \n", str[0], str[1] );
		return(-3);   
   }

   //setting maximum number of enemyRobot at list [list]
   if( BE_setMaxNumberOfEnemyRobot( e, (int) str[0] ) < 0 ){
	   fprintf(stderr, "\n[ERROR]BE_getFromDoubleStreamEnemyRobot: Couldn\'t set maxNumberOfEnemyRobot[%lf]!\n", str[0]);
		return(-4);   
   }

   for( k=0; k<str[1]; k++) {
      aux.x__ = str[2*(k+1)];
      aux.y__ = str[(2*(k+1)) + 1];
      //try and check if objects are added
      if( BE_addEnemyRobot( e, &aux) < 0 ){
         fprintf(stderr, "\n[ERROR]BE_getFromDoubleStreamEnemyRobot: Couldn\'t add enemyRobot (%lf, %lf)!\n", aux.x__, aux.y__);
		   return(-5);
      }
      
   }
    
   return( 0 );
}


int BE_getFromDoubleStreamOwnRobot( BE *e, double *str){
   int      k; //Pos iterator
   Pos      aux;

	if( e == NULL ){
	   fprintf(stderr, "\n[ERROR]BE_getFromDoubleStreamOwnRobot: BE struct is NULL!\n");
		return(-1);
	}
	
	if( NULL == str ){
	   fprintf(stderr, "\n[ERROR]BE_getFromDoubleStreamOwnRobot: stream is NULL!\n");
		return(-2);	
	}
	
	//checking integrity
   if( (str[0]<0) || (str[1]<0) ){
	   fprintf(stderr, "\n[ERROR]BE_getFromDoubleStreamOwnRobot: MaxNumberOfOwnRobot[%lf] and/or nOwnRobot[%lf] are negative \n", str[0], str[1] );
		return(-3);   
   }

   //setting maximum number of ownRobot at list [list]
   if( BE_setMaxNumberOfOwnRobot( e, (int) str[0] ) < 0 ){
	   fprintf(stderr, "\n[ERROR]BE_getFromDoubleStreamOwnRobot: Couldn\'t set maxNumberOfOwnRobot[%lf]!\n", str[0]);
		return(-4);   
   }
   

   aux.x__ = str[2];
   aux.y__ = str[3];
   BE_setMe( e, &aux);

   for( k=0; k<str[1]; k++) {
      aux.x__ = str[2*(k+2)];
      aux.y__ = str[(2*(k+2)) + 1];
      //try and check if objects are added
      if( BE_addOwnRobot( e, &aux) < 0 ){
         fprintf(stderr, "\n[ERROR]BE_getFromDoubleStreamOwnRobot: Couldn\'t add ownRobot (%lf, %lf)!\n", aux.x__, aux.y__);
		   return(-5);
      }
      
   }
    
   return( 0 );
}



int BE_getFromDoubleStreamGoal( BE *e, double *str){
   int      k; //Pos iterator
   Pos      aux;

	if( e == NULL ){
	   fprintf(stderr, "\n[ERROR]BE_getFromDoubleStreamGoal: BE struct is NULL!\n");
		return(-1);
	}
	
	if( NULL == str ){
	   fprintf(stderr, "\n[ERROR]BE_getFromDoubleStreamGoal: stream is NULL!\n");
		return(-2);	
	}
	
	//checking integrity
   if( (str[0]<0) || (str[1]<0) ){
	   fprintf(stderr, "\n[ERROR]BE_getFromDoubleStreamGoal: MaxNumberOfGoal[%lf] and/or nGoal[%lf] are negative \n", str[0], str[1] );
		return(-3);   
   }

   //setting maximum number of goal at list [list]
   if( BE_setMaxNumberOfGoal( e, (int) str[0] ) < 0 ){
	   fprintf(stderr, "\n[ERROR]BE_getFromDoubleStreamGoal: Couldn\'t set maxNumberOfGoal[%lf]!\n", str[0]);
		return(-4);   
   }

   for( k=0; k<str[1]; k++) {
      aux.x__ = str[2*(k+1)];
      aux.y__ = str[(2*(k+1)) + 1];
      //try and check if objects are added
      if( BE_addGoal( e, &aux) < 0 ){
         fprintf(stderr, "\n[ERROR]BE_getFromDoubleStreamGoal: Couldn\'t add goal (%lf, %lf)!\n", aux.x__, aux.y__);
		   return(-5);
      }
      
   }
    
   return( 0 );
}




int BE_getFromDoubleStream( BE *e, double *stream){
   int offsetObstacle[2], offsetEnemyRobot, offsetOwnRobot, offsetGoal; //offsets of stream
   
	if( NULL == e ){
	   fprintf(stderr, "\n[ERROR]BE_getFromDoubleStream: BE struct is NULL!\n");
		return(-1);
	}
	if( NULL == stream ){
	   fprintf(stderr, "\n[ERROR]BE_getFromDoubleStream: stream is NULL!\n");
		return(-2);	
	}
	
	
	offsetObstacle[0] = (int) stream[1];
	offsetObstacle[1] = (int) stream[2];
	offsetEnemyRobot  = (int) stream[3];
	offsetOwnRobot    = (int) stream[4];
	offsetGoal        = (int) stream[5];
	
	if( (offsetObstacle[0]<0) || (offsetObstacle[1]<0) || (offsetEnemyRobot<0)|| (offsetOwnRobot<0)|| (offsetGoal<0)){
	   fprintf(stderr, "\n[ERROR]BE_getFromDoubleStream: Some data offset are negative!\
	                     \noffsetObstacle[0]=[%d]\
	                     \noffsetObstacle[1]=[%d]\
	                     \noffsetEnemyRobot=[%d]\
	                     \noffsetOwnRobot=[%d]\
	                     \noffsetGoal=[%d]\n",\
	                     offsetObstacle[0],\
	                     offsetObstacle[1],\
	                     offsetEnemyRobot,\
	                     offsetOwnRobot,\
	                     offsetGoal);
		return(-3);	
	}
	
   if( BE_getFromDoubleStreamObstacle( e, &stream[offsetObstacle[0]], 0) < 0 ){
      fprintf(stderr, "\n[ERROR]BE_getFromDoubleStream: Couldn\'t get FIRST obstacleList from stream\n");
      return(-4);
   }

   if( BE_getFromDoubleStreamObstacle( e, &stream[offsetObstacle[1]], 1) < 0 ){
      fprintf(stderr, "\n[ERROR]BE_getFromDoubleStream: Couldn\'t get SECOND obstacleList from stream\n");
      return(-5);
   }
	
   if( BE_getFromDoubleStreamEnemyRobot( e, &stream[offsetEnemyRobot]) < 0 ){
      fprintf(stderr, "\n[ERROR]BE_getFromDoubleStream: Couldn\'t get enemyRobot from stream\n");
      return(-6);
   }

   if( BE_getFromDoubleStreamOwnRobot( e, &stream[offsetOwnRobot]) < 0 ){
      fprintf(stderr, "\n[ERROR]BE_getFromDoubleStream: Couldn\'t get ownRobot from stream\n");
      return(-7);
   }
   
   if( BE_getFromDoubleStreamGoal( e, &stream[offsetGoal]) < 0 ){
      fprintf(stderr, "\n[ERROR]BE_getFromDoubleStream: Couldn\'t get goal from stream\n");
      return(-8);
   }
   
   return( 0 );
}



int BE_toFloatStreamObstacle( BE *e, float *str, int list){
   int      k; //Pos iterator
   Pos      aux;
   
	if( e == NULL ){
	   fprintf(stderr, "\n[ERROR]BE_toFloatStreamObstacle: BE struct is NULL!\n");
		return(-1);
	}   

   str[0] = (e->maxNumberOfObstacle__[list]);
   str[1] = (e->nObstacle__[list]);
   
   for( k=0; k<(e->nObstacle__[list]); k++) {
      BE_getObstacleAtIndex( e, k, &aux, list);
      str[2*(k+1)]         = (float) aux.x__;
      str[(2*(k+1)) + 1]   = (float) aux.y__;
   }
   
   return( 0 );
}

int BE_toFloatStreamEnemyRobot( BE *e, float *str){
   int      k; //Pos iterator
   Pos      aux;
   
	if( e == NULL ){
	   fprintf(stderr, "\n[ERROR]BE_toFloatStreamEnemyRobot: BE struct is NULL!\n");
		return(-1);
	}   

   str[0] = (e->maxNumberOfEnemyRobot__);
   str[1] = (e->nEnemyRobot__);
   
   for( k=0; k<(e->nEnemyRobot__); k++) {
      BE_getEnemyRobotAtIndex( e, k, &aux);
      str[2*(k+1)]         = (float) aux.x__;
      str[(2*(k+1)) + 1]   = (float) aux.y__;
   }
   
   return( 0 );
}

int BE_toFloatStreamOwnRobot( BE *e, float *str){
   int      k; //Pos iterator
   Pos      aux;
   
	if( e == NULL ){
	   fprintf(stderr, "\n[ERROR]BE_toFloatStreamOwnRobot: BE struct is NULL!\n");
		return(-1);
	}   

   str[0] = (e->maxNumberOfOwnRobot__);
   str[1] = (e->nOwnRobot__);

   BE_getMe( e, &aux);
   str[2] = (float) aux.x__;
   str[3] = (float) aux.y__;
   
   for( k=0; k<(e->nOwnRobot__); k++) {
      BE_getOwnRobotAtIndex( e, k, &aux);
      str[2*(k+2)]         = (float) aux.x__;
      str[(2*(k+2)) + 1]   = (float) aux.y__;
   }
     
   return( 0 );
}

int BE_toFloatStreamGoal( BE *e, float *str){
   int      k; //Pos iterator
   Pos      aux;
   
	if( e == NULL ){
	   fprintf(stderr, "\n[ERROR]BE_toFloatStreamOwnRobot: BE struct is NULL!\n");
		return(-1);
	}   

   str[0] = (e->maxNumberOfGoal__);
   str[1] = (e->nGoal__);
   
   for( k=0; k<(e->nGoal__); k++) {
      BE_getGoalAtIndex( e, k, &aux);
      str[2*(k+1)]         = (float) aux.x__;
      str[(2*(k+1)) + 1]   = (float) aux.y__;
   }
   
   return( 0 );
}



int BE_toFloatStream( BE *e, float *stream){
   int      nTotal, //total of elements
            nStrSizeObstacle[2], nStrSizeEnemyRobot, nStrSizeOwnRobot, nStrSizeGoal, nStrHeader, //size of stream parts
            offset, 
            k; //Pos iterator
   float   *auxStream;// Just an alias to the float stream that will contain a BE struct
   
	if( e == NULL ){
	   fprintf(stderr, "\n[ERROR]BE_toFloatStream: BE struct is NULL!\n");
		return(-1);
	}
	
   nStrSizeObstacle[0] = (2*(e->nObstacle__[0])) + 2; //obtacle 0
   nStrSizeObstacle[1] = (2*(e->nObstacle__[1])) + 2; //obtacle 1
   nStrSizeEnemyRobot  = (2*(e->nEnemyRobot__)) + 2; //enemy robot
   nStrSizeOwnRobot    = (2*(e->nOwnRobot__ +1)) + 2; //own robot; pay attention to "+1", that came from "me own robot"
   nStrSizeGoal        = (2*(e->nGoal__)) + 2; //goal
   nStrHeader          = 6;
   
   nTotal = nStrHeader + nStrSizeObstacle[0] + nStrSizeObstacle[1] + nStrSizeEnemyRobot + nStrSizeOwnRobot + nStrSizeGoal;

   auxStream = (float*) myAllocator( UTILS_MAX5( nStrSizeObstacle[0], nStrSizeObstacle[1], nStrSizeEnemyRobot, nStrSizeOwnRobot, nStrSizeGoal), sizeof(float) );


	if( NULL == auxStream ){
	   fprintf(stderr, "\n[ERROR]BE_toFloatStream: Couldn\'t allocate memory to auxStream!\n");
		return(-1);
	}

   //header
   stream[0] = nTotal;
   stream[1] = nStrHeader;//offset where obstacle 0 starts
   stream[2] = stream[1] + nStrSizeObstacle[0];//offset where obstacle 1 starts
   stream[3] = stream[2] + nStrSizeObstacle[1];//offset where enemy robot starts
   stream[4] = stream[3] + nStrSizeEnemyRobot;//offset where own robot starts
   stream[5] = stream[4] + nStrSizeOwnRobot;//offset where goal starts

  //obstacle 0
   offset = (int)(stream[1]);
   BE_toFloatStreamObstacle( e, auxStream, 0);
   for( k=0; k<((2*auxStream[1]) +2); k++) {
      stream[offset + k] = auxStream[k];
   }
   
   //obstacle 1
   offset = (int)(stream[2]);
   BE_toFloatStreamObstacle( e, auxStream, 1);
   for( k=0; k<((2*auxStream[1]) +2); k++) {
      stream[offset + k] = auxStream[k];
   }
    
   //enemy robot
   offset = (int)(stream[3]);
   BE_toFloatStreamEnemyRobot( e, auxStream);
   for( k=0; k<((2*auxStream[1]) +2); k++) {
      stream[offset + k] = auxStream[k];
   }
 
   //own robot
   offset = (int)(stream[4]);
   BE_toFloatStreamOwnRobot( e, auxStream);
   for( k=0; k<((2*auxStream[1]) +4); k++) {
      stream[offset + k] = auxStream[k];
   }
  
   //goal
   offset = (int)(stream[5]);
   BE_toFloatStreamGoal( e, auxStream);
   for( k=0; k<((2*auxStream[1]) +2); k++) {
      stream[offset + k] = auxStream[k];
   }

   free( auxStream );

   return( 0 );
}


int BE_getFromFloatStreamObstacle( BE *e, float *str, int list){
   int      k; //Pos iterator
   Pos      aux;

   //checking arguments
	if( list<0 || list>1 ){
		fprintf(stderr, "\n[ERROR]BE_getFromFloatStreamObstacle[%d]: Invalid list [%d]!\n", list, list);
		return( -1 );
	}
	if( e == NULL ){
	   fprintf(stderr, "\n[ERROR]BE_getFromFloatStreamObstacle[%d]: BE struct is NULL!\n", list);
		return(-2);
	}
	
	if( NULL == str ){
	   fprintf(stderr, "\n[ERROR]BE_getFromFloatStreamObstacle[%d]: stream is NULL!\n", list);
		return(-3);	
	}
	
	//checking integrity
   if( (str[0]<0) || (str[1]<0) ){
	   fprintf(stderr, "\n[ERROR]BE_getFromFloatStreamObstacle[%d]: MaxNumberOfObstacle[%lf] and/or nObstacle[%lf] are negative \n", list, str[0], str[1]);
		return(-4);   
   }

   //setting maximum number of obstacles at list [list]
   if( BE_setMaxNumberOfObstacle( e, (int) str[0], list ) < 0 ){
	   fprintf(stderr, "\n[ERROR]BE_getFromFloatStreamObstacle[%d]: Couldn\'t set maxNumberOfObstacle[%lf]!\n", list, str[0]);
		return(-5);   
   }

   for( k=0; k<str[1]; k++) {
      aux.x__ = str[2*(k+1)];
      aux.y__ = str[(2*(k+1)) + 1];
      //try and check if objects are added
      if( BE_addObstacle( e, &aux, list) < 0 ){
         fprintf(stderr, "\n[ERROR]BE_getFromFloatStreamObstacle[%d]: Couldn\'t add obstacle(%lf, %lf)!\n", list, aux.x__, aux.y__);
		   return(-6);
      }
      
   }
    
   return( 0 );
}


int BE_getFromFloatStreamEnemyRobot( BE *e, float *str){
   int      k; //Pos iterator
   Pos      aux;

	if( e == NULL ){
	   fprintf(stderr, "\n[ERROR]BE_getFromFloatStreamEnemyRobot: BE struct is NULL!\n");
		return(-1);
	}
	
	if( NULL == str ){
	   fprintf(stderr, "\n[ERROR]BE_getFromFloatStreamEnemyRobot: stream is NULL!\n");
		return(-2);	
	}
	
	
	//checking integrity
   if( (str[0]<0) || (str[1]<0) ){
	   fprintf(stderr, "\n[ERROR]BE_getFromFloatStreamEnemyRobot: MaxNumberOfEnemyRobot[%lf] and/or nEnemyRobot[%lf] are negative \n", str[0], str[1] );
		return(-3);   
   }

   //setting maximum number of enemyRobot at list [list]
   if( BE_setMaxNumberOfEnemyRobot( e, (int) str[0] ) < 0 ){
	   fprintf(stderr, "\n[ERROR]BE_getFromFloatStreamEnemyRobot: Couldn\'t set maxNumberOfEnemyRobot[%lf]!\n", str[0]);
		return(-4);   
   }

   for( k=0; k<str[1]; k++) {
      aux.x__ = str[2*(k+1)];
      aux.y__ = str[(2*(k+1)) + 1];
      //try and check if objects are added
      if( BE_addEnemyRobot( e, &aux) < 0 ){
         fprintf(stderr, "\n[ERROR]BE_getFromFloatStreamEnemyRobot: Couldn\'t add enemyRobot (%lf, %lf)!\n", aux.x__, aux.y__);
		   return(-5);
      }
      
   }
    
   return( 0 );
}


int BE_getFromFloatStreamOwnRobot( BE *e, float *str){
   int      k; //Pos iterator
   Pos      aux;

	if( e == NULL ){
	   fprintf(stderr, "\n[ERROR]BE_getFromFloatStreamOwnRobot: BE struct is NULL!\n");
		return(-1);
	}
	
	if( NULL == str ){
	   fprintf(stderr, "\n[ERROR]BE_getFromFloatStreamOwnRobot: stream is NULL!\n");
		return(-2);	
	}
	
	//checking integrity
   if( (str[0]<0) || (str[1]<0) ){
	   fprintf(stderr, "\n[ERROR]BE_getFromFloatStreamOwnRobot: MaxNumberOfOwnRobot[%lf] and/or nOwnRobot[%lf] are negative \n", str[0], str[1] );
		return(-3);   
   }

   //setting maximum number of ownRobot at list [list]
   if( BE_setMaxNumberOfOwnRobot( e, (int) str[0] ) < 0 ){
	   fprintf(stderr, "\n[ERROR]BE_getFromFloatStreamOwnRobot: Couldn\'t set maxNumberOfOwnRobot[%lf]!\n", str[0]);
		return(-4);   
   }
   

   aux.x__ = str[2];
   aux.y__ = str[3];
   BE_setMe( e, &aux);

   for( k=0; k<str[1]; k++) {
      aux.x__ = str[2*(k+2)];
      aux.y__ = str[(2*(k+2)) + 1];
      //try and check if objects are added
      if( BE_addOwnRobot( e, &aux) < 0 ){
         fprintf(stderr, "\n[ERROR]BE_getFromFloatStreamOwnRobot: Couldn\'t add ownRobot (%lf, %lf)!\n", aux.x__, aux.y__);
		   return(-5);
      }
      
   }
    
   return( 0 );
}



int BE_getFromFloatStreamGoal( BE *e, float *str){
   int      k; //Pos iterator
   Pos      aux;

	if( e == NULL ){
	   fprintf(stderr, "\n[ERROR]BE_getFromFloatStreamGoal: BE struct is NULL!\n");
		return(-1);
	}
	
	if( NULL == str ){
	   fprintf(stderr, "\n[ERROR]BE_getFromFloatStreamGoal: stream is NULL!\n");
		return(-2);	
	}
	
	//checking integrity
   if( (str[0]<0) || (str[1]<0) ){
	   fprintf(stderr, "\n[ERROR]BE_getFromFloatStreamGoal: MaxNumberOfGoal[%lf] and/or nGoal[%lf] are negative \n", str[0], str[1] );
		return(-3);   
   }

   //setting maximum number of goal at list [list]
   if( BE_setMaxNumberOfGoal( e, (int) str[0] ) < 0 ){
	   fprintf(stderr, "\n[ERROR]BE_getFromFloatStreamGoal: Couldn\'t set maxNumberOfGoal[%lf]!\n", str[0]);
		return(-4);   
   }

   for( k=0; k<str[1]; k++) {
      aux.x__ = str[2*(k+1)];
      aux.y__ = str[(2*(k+1)) + 1];
      //try and check if objects are added
      if( BE_addGoal( e, &aux) < 0 ){
         fprintf(stderr, "\n[ERROR]BE_getFromFloatStreamGoal: Couldn\'t add goal (%lf, %lf)!\n", aux.x__, aux.y__);
		   return(-5);
      }
      
   }
    
   return( 0 );
}




int BE_getFromFloatStream( BE *e, float *stream){
   int offsetObstacle[2], offsetEnemyRobot, offsetOwnRobot, offsetGoal; //offsets of stream
   
	if( NULL == e ){
	   fprintf(stderr, "\n[ERROR]BE_getFromFloatStream: BE struct is NULL!\n");
		return(-1);
	}
	if( NULL == stream ){
	   fprintf(stderr, "\n[ERROR]BE_getFromFloatStream: stream is NULL!\n");
		return(-2);	
	}
	
	
	offsetObstacle[0] = (int) stream[1];
	offsetObstacle[1] = (int) stream[2];
	offsetEnemyRobot  = (int) stream[3];
	offsetOwnRobot    = (int) stream[4];
	offsetGoal        = (int) stream[5];
	
	if( (offsetObstacle[0]<0) || (offsetObstacle[1]<0) || (offsetEnemyRobot<0)|| (offsetOwnRobot<0)|| (offsetGoal<0)){
	   fprintf(stderr, "\n[ERROR]BE_getFromFloatStream: Some data offset are negative!\
	                     \noffsetObstacle[0]=[%d]\
	                     \noffsetObstacle[1]=[%d]\
	                     \noffsetEnemyRobot=[%d]\
	                     \noffsetOwnRobot=[%d]\
	                     \noffsetGoal=[%d]\n",\
	                     offsetObstacle[0],\
	                     offsetObstacle[1],\
	                     offsetEnemyRobot,\
	                     offsetOwnRobot,\
	                     offsetGoal);
		return(-3);	
	}
	
   if( BE_getFromFloatStreamObstacle( e, &stream[offsetObstacle[0]], 0) < 0 ){
      fprintf(stderr, "\n[ERROR]BE_getFromFloatStream: Couldn\'t get FIRST obstacleList from stream\n");
      return(-4);
   }

   if( BE_getFromFloatStreamObstacle( e, &stream[offsetObstacle[1]], 1) < 0 ){
      fprintf(stderr, "\n[ERROR]BE_getFromFloatStream: Couldn\'t get SECOND obstacleList from stream\n");
      return(-5);
   }
	
   if( BE_getFromFloatStreamEnemyRobot( e, &stream[offsetEnemyRobot]) < 0 ){
      fprintf(stderr, "\n[ERROR]BE_getFromFloatStream: Couldn\'t get enemyRobot from stream\n");
      return(-6);
   }

   if( BE_getFromFloatStreamOwnRobot( e, &stream[offsetOwnRobot]) < 0 ){
      fprintf(stderr, "\n[ERROR]BE_getFromFloatStream: Couldn\'t get ownRobot from stream\n");
      return(-7);
   }
   
   if( BE_getFromFloatStreamGoal( e, &stream[offsetGoal]) < 0 ){
      fprintf(stderr, "\n[ERROR]BE_getFromFloatStream: Couldn\'t get goal from stream\n");
      return(-8);
   }
   
   return( 0 );
}










int BE_setMaxNumberOfObstacle(BE *e,  int maxNumberOfObstacle, int list){
	if( (maxNumberOfObstacle < 0) ||  list<0 || list>1  ){
		fprintf(stderr, "ERROR: BE_setMaxNumberObstacle couldn\'t set \"%d\" as a new maximum number of obstacles at list \"%d\"! Invalid values!!\n", maxNumberOfObstacle, list);
		return( -1 );
	}
	//cleaning, setting new values and allocating memory
	e->maxNumberOfObstacle__[list] = maxNumberOfObstacle;
	if( NULL != e->obstacle__[list])  free( e->obstacle__[list] );
	if( 0 == maxNumberOfObstacle ){
	   e->obstacle__[list]= NULL;
	}
	else{
   	e->obstacle__[list] = (Pos *) myAllocator(e->maxNumberOfObstacle__[list], sizeof(Pos)); 
   	
      //checking
	   if( NULL == e->obstacle__[list] ){
		   fprintf(stderr, "ERROR: BE_setMaxNumberObstacle couldn\'t allocate memory to the %s obstacleList!\n", (list==0 ? "FIRST" : "SECOND") );
		   e->maxNumberOfObstacle__[list] = 0;
		   return( -1 );
	   }
   }
   
	e->nObstacle__[list]=0;

	return( e->maxNumberOfObstacle__[list] );
	
}



int BE_getMaxNumberOfObstacle(BE *e,  int list){
	if( list<0 || list>1  ){
		fprintf(stderr, "ERROR: BE_getMaxNumberObstacle couldn\'t get maximum number of obstacles of list \"%d\"! Invalid values!!\n", list);
		return( -1 );
	}
	
	return( e->maxNumberOfObstacle__[list] );
}


int BE_getNumberOfObstacle(BE *e,  int list){
	if( list<0 || list>1  ){
		fprintf(stderr, "ERROR: BE_getNumberOfObstacle couldn\'t get number of obstacles of list \"%d\"! Invalid values!!\n", list);
		return( -1 );
	}
	
	return( e->nObstacle__[list] );
}

int BE_addObstacle( BE *e, Pos *newObstacle, int list ){

	if( list<0 || list>1  ){
		fprintf(stderr, "ERROR: BE_addObstacle couldn\'t add new obstacle at list \"%d\"! Invalid values!!\n", list);
		return(-1);
	}

	if(e->nObstacle__[ list ] < e->maxNumberOfObstacle__[ list ] ){
		e->obstacle__[ list ][ (e->nObstacle__[ list ]) ].x__ = newObstacle->x__;
		e->obstacle__[ list ][ (e->nObstacle__[ list ]) ].y__ = newObstacle->y__;

		e->nObstacle__[ list ]+=1;
	}
	else {
#ifdef UTILS_WITH_ERROR_MESSAGE
		fprintf(stderr, "ERROR: BE_addObstacle couldn\'t add new obstacle at list \"%d\"! Maximum number of obstacles reached!!\n", list);
#endif
		return(-2); 
	}

	return(0);
}



int BE_getObstacleAtIndex( BE *e, int index, Pos *result, int list ){
	if( list<0 || list>1 || (index >= e->nObstacle__[ list ])  ){
#ifdef UTILS_WITH_ERROR_MESSAGE
		fprintf(stderr, "ERROR: BE_getObstacleAtIndex couldn\'t get obstacle from list \"%d\" at index \"%d\"! Invalid values!!\n", list, index);
#endif
		return(-1);
	}
	
	result->x__ = e->obstacle__[list][index].x__;
	result->y__ = e->obstacle__[list][index].y__;
	
	return( 0 );
}
	

int BE_clearListOfObstacle( BE *e, int list ){
	if( list<0 || list>1  ){
		fprintf(stderr, "ERROR: BE_clearListOfObstacle couldn\'t clear obstacle list \"%d\"! Invalid values!!\n", list);
		return(-1);
	}
	memset( e->obstacle__[list] , 0,  sizeof(Pos)*(e->maxNumberOfObstacle__[ list ]) );
	e->nObstacle__[ list ] = 0;
	return( 0 );
	
}



int BE_setMaxNumberOfEnemyRobot(BE *e,  int maxNumberOfEnemyRobot ){
	if( (maxNumberOfEnemyRobot < 0)   ){
		fprintf(stderr, "ERROR: BE_setMaxNumberEnemyRobot couldn\'t set \"%d\" as a new maximum number of enemyRobot list! Invalid values!!\n", maxNumberOfEnemyRobot);
		return( -1 );
	}
	
	//cleaning, setting new values and allocating memory
	e->maxNumberOfEnemyRobot__ = maxNumberOfEnemyRobot;
	if( NULL != e->enemyRobot__) free( e->enemyRobot__ );
	if( 0 == maxNumberOfEnemyRobot ){
      e->enemyRobot__ = NULL;
	}
	else{
		e->enemyRobot__ = (Pos *) myAllocator(e->maxNumberOfEnemyRobot__, sizeof(Pos) );// malloc(sizeof(Pos) * e->maxNumberOfEnemyRobot__ );
	   //checking
	   if( e->enemyRobot__ == NULL ){
		   fprintf(stderr, "ERROR: BE_setMaxNumberEnemyRobot couldn\'t allocate memory to the enemyRobotList!\n" );
		   e->maxNumberOfEnemyRobot__ = 0;
		   return( -1 );
	   }	
	}
	e->nEnemyRobot__=0;


	
	return( e->maxNumberOfEnemyRobot__ );
	
}



int BE_getMaxNumberOfEnemyRobot(BE *e){
	return( e->maxNumberOfEnemyRobot__ );
}


int BE_getNumberOfEnemyRobot(BE *e){
	return( e->nEnemyRobot__ );
}


int BE_addEnemyRobot( BE *e, Pos *newEnemyRobot){

	if(e->nEnemyRobot__ < e->maxNumberOfEnemyRobot__ ){
		e->enemyRobot__[ (e->nEnemyRobot__) ].x__ = newEnemyRobot->x__;
		e->enemyRobot__[ (e->nEnemyRobot__) ].y__ = newEnemyRobot->y__;

		e->nEnemyRobot__+=1;
	}
	else {
		fprintf(stderr, "ERROR: BE_addEnemyRobot couldn\'t add new enemyRobot! Maximum number of obstacles reached!!\n");
		return( -1 );
	}
	
	
	return( 0 );
	
}

int BE_getEnemyRobotAtIndex( BE *e, int index, Pos *result ){
	if( index >= e->nEnemyRobot__  ){
		fprintf(stderr, "ERROR: BE_getEnemyRobotAtIndex couldn\'t get enemyRobot at index \"%d\"! Invalid values!!\n",  index);
		return(-1);
	}
	
	result->x__ = e->enemyRobot__[index].x__;
	result->y__ = e->enemyRobot__[index].y__;
	
	return( 0 );
}

int BE_clearListOfEnemyRobot( BE *e ){

	memset( e->enemyRobot__ , 0,  sizeof(Pos)*(e->maxNumberOfEnemyRobot__) );
	e->nEnemyRobot__ = 0;

	return( 0 );
}


int BE_setMaxNumberOfOwnRobot(BE *e,  int maxNumberOfOwnRobot ){
	if( (maxNumberOfOwnRobot < 0)   ){
		fprintf(stderr, "ERROR: BE_setMaxNumberOwnRobot couldn\'t set \"%d\" as a new maximum number of ownRobot list! Invalid values!!\n", maxNumberOfOwnRobot);
		return( -1 );
	}

   //cleaning, setting new values and allocating memory
	e->maxNumberOfOwnRobot__ = maxNumberOfOwnRobot;
	if( NULL != e->ownRobot__) free( e->ownRobot__ );
	if( 0 == maxNumberOfOwnRobot ){
	   e->ownRobot__ = NULL;
	}
	else{
	   e->ownRobot__ = (Pos *) myAllocator( e->maxNumberOfOwnRobot__ +1, sizeof(Pos) );
   	//checking
	   if( e->ownRobot__ == NULL ){
		   fprintf(stderr, "ERROR: BE_maxNumberOfOwnRobot couldn\'t allocate memory to the ownRobotList!\n" );
		   e->maxNumberOfOwnRobot__ = 0;
		   return( -2 );
	   }
	}

	e->nOwnRobot__ = 0;
		
	return( e->maxNumberOfOwnRobot__ );
	
}



int BE_getMaxNumberOfOwnRobot(BE *e){
	return( e->maxNumberOfOwnRobot__ );
}

int BE_getNumberOfOwnRobot(BE *e){
	return( e->nOwnRobot__ );
}

int BE_addOwnRobot( BE *e, Pos *newOwnRobot){

	if(e->nOwnRobot__ < e->maxNumberOfOwnRobot__ ){
		e->ownRobot__[ (e->nOwnRobot__+1) ].x__ = newOwnRobot->x__;
		e->ownRobot__[ (e->nOwnRobot__+1) ].y__ = newOwnRobot->y__;

		e->nOwnRobot__+=1;
	}
	else {
		fprintf(stderr, "ERROR: BE_addOwnRobot couldn\'t add new ownRobot! Maximum number of obstacles reached!!\n");
		return(-1);
	}	
	
	return( 0 );
}

int BE_setMe( BE *e, Pos *newOwnRobot){
	if( e->ownRobot__ == NULL ){
		fprintf(stderr, "BE_setMe: Own Robot List is NULL\n");
		return(-1);
	}
	e->ownRobot__[ 0 ].x__ = newOwnRobot->x__;
	e->ownRobot__[ 0 ].y__ = newOwnRobot->y__;
	return( 0 );
	
}

int BE_getMe( BE *e, Pos *result ){
	if( e->ownRobot__ == NULL ){
		fprintf(stderr, "BE_getMe: Own Robot List is NULL\n");
		return(-1);
	}
	result->x__ = e->ownRobot__[ 0 ].x__;
	result->y__ = e->ownRobot__[ 0 ].y__;
	return( 0 );
}


int BE_getOwnRobotAtIndex( BE *e, int index, Pos *result ){
	if( index >= e->nOwnRobot__  ){
		fprintf(stderr, "ERROR: BE_getOwnRobotAtIndex couldn\'t get ownRobot at index \"%d\"! Invalid values!!\n",  index);
		return(-1);
	}
	
	result->x__ = e->ownRobot__[index+1].x__;
	result->y__ = e->ownRobot__[index+1].y__;
	
	return( 0 );
}

int BE_clearListOfOwnRobot( BE *e ){

	memset( e->ownRobot__ , 0,  sizeof(Pos)*(e->maxNumberOfOwnRobot__+1) );
	e->nOwnRobot__ = 0;
	
	return( 0 );
}



int BE_setMaxNumberOfGoal(BE *e,  int maxNumberOfGoal ){
	if( (maxNumberOfGoal < 0)   ){
		fprintf(stderr, "ERROR: BE_setMaxNumberGoal couldn\'t set \"%d\" as a new maximum number of goal list! Invalid values!!\n", maxNumberOfGoal);
		return( -1 );
	}

   //cleaning, setting new values and allocating memory
	e->maxNumberOfGoal__ = maxNumberOfGoal;
	if( NULL != e->goal__ ) free( e->goal__ );
	if( 0 == maxNumberOfGoal ){
   	e->goal__=NULL;
	}
	else{
   	e->goal__ = (Pos *) myAllocator( e->maxNumberOfGoal__, sizeof(Pos) ) ;
   	//checking
	   if( e->goal__ == NULL ){
		   fprintf(stderr, "ERROR: BE_maxNumberOfGoal couldn\'t allocate memory to the goalList!\n" );
		   e->maxNumberOfGoal__ = 0;
		   return( -1 );
	   }
	}
	
	e->nGoal__ = 0;
	
	return( e->maxNumberOfGoal__ );
}



int BE_getMaxNumberOfGoal(BE *e){
	return( e->maxNumberOfGoal__ );
}


int BE_getNumberOfGoal(BE *e){
	return( e->nGoal__ );
}


int BE_addGoal( BE *e, Pos *newGoal){

	if(e->nGoal__ < e->maxNumberOfGoal__ ){
		e->goal__[ (e->nGoal__) ].x__ = newGoal->x__;
		e->goal__[ (e->nGoal__) ].y__ = newGoal->y__;

		e->nGoal__+=1;
	}
	else {
		fprintf(stderr, "ERROR: BE_addGoal couldn\'t add new goal! Maximum number of obstacles reached!!\n");
		return(-1);
	}
	
	
	return( 0 );
	
}

int BE_getGoalAtIndex( BE *e, int index, Pos *result ){
	if( index >= e->nGoal__  ){
		fprintf(stderr, "ERROR: BE_getGoalAtIndex couldn\'t get goal at index \"%d\"! Invalid values!!\n",  index);
		return(-1);
	}
	
	result->x__ = e->goal__[index].x__;
	result->y__ = e->goal__[index].y__;
	
	return( 0 );
}

int BE_clearListOfGoal( BE *e ){
	memset( e->goal__ , 0,  sizeof(Pos)*(e->maxNumberOfGoal__) );
	e->nGoal__ = 0;
	
	return( 0 );
}











int BE_test( FILE *out ){
	
	Pos posAux[2][100];

	fprintf(out, "\n\n-----------Testing all BE functions-----------\n\n");
	srand(time(NULL));
	int nObstacleTest[2], nEnemyRobotTest, nOwnRobotTest, nGoalTest;
	
	nObstacleTest[0] = (rand()%100)+1;
	nObstacleTest[1] = (rand()%100)+1;
	nEnemyRobotTest  = (rand()%100)+1;
	nOwnRobotTest    = (rand()%100)+1;
	nGoalTest        = (rand()%100)+1; 



	fprintf(out, "Creating a BE struct... (%d, %d, %d, %d, %d)\t", nObstacleTest[0], nObstacleTest[1], nEnemyRobotTest, nOwnRobotTest, nGoalTest);
	BE *e = BE_new( nObstacleTest[0], nObstacleTest[1], nEnemyRobotTest, nOwnRobotTest, nGoalTest);
	assert( e != NULL );
	fprintf(out, "[OK]\n");
	
	fprintf(out, "Testing obstacles lists... \n");
	fprintf(out, "\t Testing changing maximum number of obstacles... ");
	assert( BE_getMaxNumberOfObstacle(e,  0) == nObstacleTest[0] );
	assert( BE_getMaxNumberOfObstacle(e,  1) == nObstacleTest[1] );
	assert( BE_setMaxNumberOfObstacle(e, 50, 0) > 0 );
	assert( BE_setMaxNumberOfObstacle(e, 49, 1) > 0 );
	assert( BE_getMaxNumberOfObstacle(e,  0) == 50 );
	assert( BE_getMaxNumberOfObstacle(e,  1) == 49 );
	fprintf(out, "[OK]\n");

	fprintf(out, "\t Testing adding, getting and cleaning obstacles... ");
	assert( BE_getNumberOfObstacle(e,  0) == 0 );
	assert( BE_getNumberOfObstacle(e,  1) == 0 );
	
	//fullfill list 0
	for( int i=0; i<BE_getMaxNumberOfObstacle(e,  0); i++){
		posAux[0][i].x__ = ((rand()/((float) RAND_MAX)) - 0.5)*10000;
		posAux[0][i].y__ = ((rand()/((float) RAND_MAX)) - 0.5)*10000;
		
		assert( BE_addObstacle( e, &posAux[0][i], 0 ) == 0 );
	}
	//try extra obstacle
	assert( BE_addObstacle( e, &posAux[0][0], 0 ) < 0 );
	//fullfill list 1
	for( int i=0; i<BE_getMaxNumberOfObstacle(e,  1); i++){
		posAux[1][i].x__ = ((rand()/((float) RAND_MAX)) - 0.5)*10000;
		posAux[1][i].y__ = ((rand()/((float) RAND_MAX)) - 0.5)*10000;

		assert( BE_addObstacle( e, &posAux[1][i], 1 ) == 0 );
	}
	//try extra obstacle
	assert( BE_addObstacle( e, &posAux[1][0], 1 ) < 0 );

	assert( BE_getNumberOfObstacle(e,  0) == BE_getMaxNumberOfObstacle(e,  0) );
	assert( BE_getNumberOfObstacle(e,  1) == BE_getMaxNumberOfObstacle(e,  1) );
	
	//searching for corrupt entries
	for( int i=0; i<BE_getNumberOfObstacle(e,  0); i++){
		assert( BE_getObstacleAtIndex( e, i, &posAux[0][i+BE_getNumberOfObstacle(e,  0)], 0 ) == 0);
		assert( (fabs(posAux[0][i].x__ - posAux[0][i+BE_getNumberOfObstacle(e,  0)].x__) < 10e-10) && (fabs(posAux[0][i].y__ - posAux[0][i+BE_getNumberOfObstacle(e,  0)].y__) < 10e-10) );
	}
	
	for( int i=0; i<BE_getNumberOfObstacle(e,  1); i++){
		assert( BE_getObstacleAtIndex( e, i, &posAux[1][i+BE_getNumberOfObstacle(e,  1)], 1 ) == 0);
		assert( (fabs(posAux[1][i].x__ - posAux[1][i+BE_getNumberOfObstacle(e,  1)].x__) < 10e-10) && (fabs(posAux[1][i].y__ - posAux[1][i+BE_getNumberOfObstacle(e,  1)].y__) < 10e-10) );
	}
	
	//cleaning
	assert( BE_clearListOfObstacle( e, 0 ) == 0 );
	assert( BE_clearListOfObstacle( e, 1 ) == 0 );
	assert( BE_getNumberOfObstacle(e,  0) == 0 );
	assert( BE_getNumberOfObstacle(e,  1) == 0 );
	
	//fullfilling again
	for( int i=0; i<BE_getMaxNumberOfObstacle(e,  0); i++){
		posAux[0][i].x__ = ((rand()/((float) RAND_MAX)) - 0.5)*10000;
		posAux[0][i].y__ = ((rand()/((float) RAND_MAX)) - 0.5)*10000;
		
		assert( BE_addObstacle( e, &posAux[0][i], 0 ) == 0 );
	}
	for( int i=0; i<BE_getMaxNumberOfObstacle(e,  1); i++){
		posAux[1][i].x__ = ((rand()/((float) RAND_MAX)) - 0.5)*10000;
		posAux[1][i].y__ = ((rand()/((float) RAND_MAX)) - 0.5)*10000;

		assert( BE_addObstacle( e, &posAux[1][i], 1 ) == 0 );
	}
	assert( BE_getNumberOfObstacle(e,  0) == BE_getMaxNumberOfObstacle(e,  0) );
	assert( BE_getNumberOfObstacle(e,  1) == BE_getMaxNumberOfObstacle(e,  1) );
	fprintf(out, "[OK]\n");





	fprintf(out, "Testing EnemyRobots list... \n");
	fprintf(out, "\t Testing changing maximum number of EnemyRobots... ");
	assert( BE_getMaxNumberOfEnemyRobot(e) == nEnemyRobotTest );
	assert( BE_setMaxNumberOfEnemyRobot(e, 50) > 0 );
	assert( BE_getMaxNumberOfEnemyRobot(e) == 50 );
	fprintf(out, "[OK]\n");

	fprintf(out, "\t Testing adding, getting and cleaning EnemyRobots... ");
	assert( BE_getNumberOfEnemyRobot(e) == 0 );
	
	//fullfill list
	for( int i=0; i<BE_getMaxNumberOfEnemyRobot(e); i++){
		posAux[0][i].x__ = ((rand()/((float) RAND_MAX)) - 0.5)*10000;
		posAux[0][i].y__ = ((rand()/((float) RAND_MAX)) - 0.5)*10000;
		
		assert( BE_addEnemyRobot( e, &posAux[0][i] ) == 0 );
	}
	//try extra EnemyRobot
	assert( BE_addEnemyRobot( e, &posAux[0][0] ) < 0 );


	assert( BE_getNumberOfEnemyRobot(e) == BE_getMaxNumberOfEnemyRobot(e) );

	//searching for corrupt entries
	for( int i=0; i<BE_getNumberOfEnemyRobot(e); i++){
		assert( BE_getEnemyRobotAtIndex( e, i, &posAux[0][i+BE_getNumberOfEnemyRobot(e)]) == 0);
		assert( (fabs(posAux[0][i].x__ - posAux[0][i+BE_getNumberOfEnemyRobot(e)].x__) < 10e-10) && (fabs(posAux[0][i].y__ - posAux[0][i+BE_getNumberOfEnemyRobot(e)].y__) < 10e-10) );
	}
	
	//cleaning
	assert( BE_clearListOfEnemyRobot( e ) == 0 );
	assert( BE_getNumberOfEnemyRobot(e) == 0 );
	
	//fullfilling again
	for( int i=0; i<BE_getMaxNumberOfEnemyRobot(e); i++){
		posAux[0][i].x__ = ((rand()/((float) RAND_MAX)) - 0.5)*10000;
		posAux[0][i].y__ = ((rand()/((float) RAND_MAX)) - 0.5)*10000;
		
		assert( BE_addEnemyRobot( e, &posAux[0][i] ) == 0 );
	}

	assert( BE_getNumberOfEnemyRobot(e) == BE_getMaxNumberOfEnemyRobot(e) );
	fprintf(out, "[OK]\n");



	fprintf(out, "Testing OwnRobots list... \n");
	fprintf(out, "\t Testing changing maximum number of OwnRobots... ");
	assert( BE_getMaxNumberOfOwnRobot(e) == nOwnRobotTest );
	assert( BE_setMaxNumberOfOwnRobot(e, 50) > 0 );
	assert( BE_getMaxNumberOfOwnRobot(e) == 50 );
	fprintf(out, "[OK]\n");

	fprintf(out, "\t Testing adding, getting and cleaning OwnRobots... ");
	assert( BE_getNumberOfOwnRobot(e) == 0 );
	
	//fullfill list
	for( int i=0; i<BE_getMaxNumberOfOwnRobot(e); i++){
		posAux[0][i].x__ = ((rand()/((float) RAND_MAX)) - 0.5)*10000;
		posAux[0][i].y__ = ((rand()/((float) RAND_MAX)) - 0.5)*10000;
		
		assert( BE_addOwnRobot( e, &posAux[0][i] ) == 0 );
	}
	posAux[1][0].x__ = ((rand()/((float) RAND_MAX)) - 0.5)*10000;
	posAux[1][0].y__ = ((rand()/((float) RAND_MAX)) - 0.5)*10000;
	assert( BE_setMe( e, &posAux[1][0]) == 0 );


	
	

	//try extra OwnRobot
	assert( BE_addOwnRobot( e, &posAux[0][0] ) < 0 );


	assert( BE_getNumberOfOwnRobot(e) == BE_getMaxNumberOfOwnRobot(e) );

	//searching for corrupt entries
	for( int i=0; i<BE_getNumberOfOwnRobot(e); i++){
		assert( BE_getOwnRobotAtIndex( e, i, &posAux[0][i+BE_getNumberOfOwnRobot(e)]) == 0);
		assert( (fabs(posAux[0][i].x__ - posAux[0][i+BE_getNumberOfOwnRobot(e)].x__) < 10e-10) && (fabs(posAux[0][i].y__ - posAux[0][i+BE_getNumberOfOwnRobot(e)].y__) < 10e-10) );
	}
	assert( BE_getMe( e, &posAux[1][1]) == 0);
	assert( (fabs(posAux[1][0].x__ - posAux[1][1].x__) < 10e-10) && (fabs(posAux[1][0].y__ - posAux[1][1].y__) < 10e-10) );
	
	
	//cleaning
	assert( BE_clearListOfOwnRobot( e ) == 0 );
	assert( BE_getNumberOfOwnRobot(e) == 0 );
	
	//fullfilling again
	for( int i=0; i<BE_getMaxNumberOfOwnRobot(e); i++){
		posAux[0][i].x__ = ((rand()/((float) RAND_MAX)) - 0.5)*10000;
		posAux[0][i].y__ = ((rand()/((float) RAND_MAX)) - 0.5)*10000;
		
		assert( BE_addOwnRobot( e, &posAux[0][i] ) == 0 );
	}
	posAux[1][0].x__ = ((rand()/((float) RAND_MAX)) - 0.5)*10000;
	posAux[1][0].y__ = ((rand()/((float) RAND_MAX)) - 0.5)*10000;
	assert( BE_setMe( e, &posAux[1][0]) == 0 );

	assert( BE_getNumberOfOwnRobot(e) == BE_getMaxNumberOfOwnRobot(e) );
	fprintf(out, "[OK]\n");
	
	
	
	
	
	fprintf(out, "Testing Goals list... \n");
	fprintf(out, "\t Testing changing maximum number of Goals... ");
	assert( BE_getMaxNumberOfGoal(e) == nGoalTest );
	assert( BE_setMaxNumberOfGoal(e, 50) > 0 );
	assert( BE_getMaxNumberOfGoal(e) == 50 );
	fprintf(out, "[OK]\n");

	fprintf(out, "\t Testing adding, getting and cleaning Goals... ");
	assert( BE_getNumberOfGoal(e) == 0 );
	
	//fullfill list
	for( int i=0; i<BE_getMaxNumberOfGoal(e); i++){
		posAux[0][i].x__ = ((rand()/((float) RAND_MAX)) - 0.5)*10000;
		posAux[0][i].y__ = ((rand()/((float) RAND_MAX)) - 0.5)*10000;
		
		assert( BE_addGoal( e, &posAux[0][i] ) == 0 );
	}
	//try extra Goal
	assert( BE_addGoal( e, &posAux[0][0] ) < 0 );


	assert( BE_getNumberOfGoal(e) == BE_getMaxNumberOfGoal(e) );

	//searching for corrupt entries
	for( int i=0; i<BE_getNumberOfGoal(e); i++){
		assert( BE_getGoalAtIndex( e, i, &posAux[0][i+BE_getNumberOfGoal(e)]) == 0);
		assert( (fabs(posAux[0][i].x__ - posAux[0][i+BE_getNumberOfGoal(e)].x__) < 10e-10) && (fabs(posAux[0][i].y__ - posAux[0][i+BE_getNumberOfGoal(e)].y__) < 10e-10) );
	}
	
	//cleaning
	assert( BE_clearListOfGoal( e ) == 0 );
	assert( BE_getNumberOfGoal(e) == 0 );
	
	//fullfilling again
	for( int i=0; i<BE_getMaxNumberOfGoal(e); i++){
		posAux[0][i].x__ = ((rand()/((float) RAND_MAX)) - 0.5)*10000;
		posAux[0][i].y__ = ((rand()/((float) RAND_MAX)) - 0.5)*10000;
		
		assert( BE_addGoal( e, &posAux[0][i] ) == 0 );
	}

	assert( BE_getNumberOfGoal(e) == BE_getMaxNumberOfGoal(e) );
	fprintf(out, "[OK]\n");
	
	
	fprintf(out, "Testing cleaning all lists... ");
	assert( BE_clearAllLists( e ) == 0 );
	fprintf(out, "[OK]\n");

	fprintf(out, "Testing deleting a BE struct... ");
	BE_delete( e );
	fprintf(out, "[OK]\n");
	return( 0 );
}

