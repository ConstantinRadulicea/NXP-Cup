#include "MedianFilter.h"
#include <stdlib.h> 
#include <stdio.h> 

int main() {
	MedianFilter filter(3);
	double next;
	double arr[] = {2, 3, 80, 6, 2, 3};
	
	for (size_t i = 0; i < sizeof(arr) / sizeof((*arr)); i++)
	{
		next = filter.next(arr[i]);
		printf("filtered: %f\n", next);
	}
	
	//for (size_t i = 0; i <1000; i++)
	//{
	//	next = filter.next(rand()%100);
	//	printf("filtered: %f\n", next);
	//}
	return 0;
}