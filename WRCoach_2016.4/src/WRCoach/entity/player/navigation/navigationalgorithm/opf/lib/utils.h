#ifndef UTILS_H
#define UTILS_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>


#define DEGREE_TO_RADIAN(x) ( ((x)/180.0f)*M_PI )
#define RADIAN_TO_DEGREE(x) ( ((x)/M_PI)*180.0f )

#define UTILS_MAX2(x1, x2)             ( (x1)>(x2) ? (x1) : (x2) ) //maximum value between two elements - 1 comparison
#define UTILS_MAX3(x1, x2, x3)         ( UTILS_MAX2(UTILS_MAX2((x1),(x2)), (x3)) ) //maximum value between three elements - 2 comparisons
#define UTILS_MAX4(x1, x2, x3, x4)     ( UTILS_MAX2(UTILS_MAX2((x1),(x2)), UTILS_MAX2((x3),(x4))) ) //maximum value between four elements - 3 comparisons
#define UTILS_MAX5(x1, x2, x3, x4, x5) ( UTILS_MAX2(UTILS_MAX2((x1),(x2)), UTILS_MAX3((x3),(x4),(x5))) ) //maximum value between five elements - 4 comparisons

//#define UTILS_FLOAT_DATATYPE

//to debug or not to debug, thats the question
#define UTILS_WITH_DEBUG
#define UTILS_WITH_ERROR_MESSAGE

#define DEFAULT_ALLOCATOR 0


enum Booleano{ u_true, u_false };


#ifdef UTILS_FLOAT_DATATYPE

typedef struct{
	float x__, y__;
	} Pos;

#else 

typedef struct{
   double x__, y__;
	} Pos;

#endif

inline double euclideanDistance( double x1, double y1, double x2, double y2 ){
   return(  sqrt( ((x1-x2)*(x1-x2)) + ((y1-y2)*(y1-y2)) )   );
}

inline double euclideanDistancePos( Pos *a, Pos *b ){
   return(   euclideanDistance(a->x__, a->y__, b->x__, b->y__)  );
}

inline void* myAllocator( unsigned long int n, unsigned long int size, int type=DEFAULT_ALLOCATOR ){
   switch( type ){
      case 0:
         return( (void*)malloc(n*size) );
         break;
      case 1:
         return( (void*)calloc(n, size) );
         break;
      default:
         fprintf(stderr, "UTILS myAllocator ERROR: Unknow allocator type!\n");
         return( (void*)NULL );
         break;
   }

}


#endif //UTILS_H
