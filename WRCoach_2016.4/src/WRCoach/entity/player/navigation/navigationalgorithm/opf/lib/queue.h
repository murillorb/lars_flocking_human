#ifndef INT_QUEUE_H
#define INT_QUEUE_H

#include "utils.h"



typedef struct{
	int posArray__, radius__;
	} queueElement;
	
typedef struct{
	queueElement *array__;
	int sizeQueue__;
	int allocatedSizeQueue__;
	int minPointer__, maxPointer__;
	bool emptyQueue__;
	} queue;
	

int queuePush(queue *q, Pos *newElement, int *radius);
int queuePop(queue *q, Pos *newElement, int *radius);

int queueIsEmpty(queue *q);
int queueHasElement(queue *q, Pos *element);

//array__[sizeQueue__]



#endif
