#include "MedianFilter.h"
#include <stdlib.h> 
#include <stdio.h>
#include <time.h>

static void bubbleSort(float* _arr, size_t _n_elem)
{
    if (_n_elem <= 1) return;

    char ord;
    float *arr;
    size_t i, first_order;
    arr = _arr;
    first_order = _n_elem;
    float temp;

    do {
        ord = (char)1;
        for (i = (size_t)0; i < first_order - (size_t)1; i++)
        {
            if (arr[i] > arr[i + (size_t)1])
            {
                temp = arr[i];
                arr[i] = arr[i + (size_t)1];
                arr[i + (size_t)1] = temp;
                ord = (char)0;
            }
        }
        first_order -= 1;
    } while (!ord);
}

int main() {
	MedianFilter filter(3);
	double next;
	float arr[] = {2, 3, 80, 6, 2, 3};

	
	for (size_t i = 0; i < sizeof(arr) / sizeof((*arr)); i++)
	{
		next = filter.next(arr[i]);
		printf("value: %f\t\tfiltered: %f\n", arr[i], next);
	}
	
	//srand(time(NULL));
	//for (size_t i = 0; i <1000; i++)
	//{
	//	next = filter.next(rand()%100);
	//	printf("filtered: %f\n", next);
	//}
	return 0;
}