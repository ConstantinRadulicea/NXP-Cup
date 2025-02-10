/*
* Copyright 2024 Constantin Dumitru Petre RĂDULICEA
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*   http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

#ifndef __MEDIANFILTER_H__
#define __MEDIANFILTER_H__

#include <stddef.h>
//#include <new>


#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

class MedianFilter
{
private:
    unsigned int _windowSize;
    int totSamples;
    float* _queue = NULL;
    float* _tempQueue = NULL;


    // An optimized version of Bubble Sort
    static void bubbleSort(float arr[], int n)
    {
        int i, j;
        int swapped;
        float temp;
        for (i = 0; i < n - 1; i++) {
            swapped = 0;
            for (j = 0; j < n - i - 1; j++) {
                if (arr[j] > arr[j + 1]) {
                    //swap(arr[j], arr[j + 1]);
                    temp = arr[j];
                    arr[j] = arr[j + 1];
                    arr[j + 1] = temp;
                    swapped = 1;
                }
            }

            // If no two elements were swapped
            // by inner loop, then break
            if (swapped == 0)
                break;
        }
    }
public:

    MedianFilter(unsigned int windowSize) {
        this->totSamples = 0;
        this->_windowSize = windowSize;
        if (windowSize <= 0) {
            this->_queue = NULL;
            this->_tempQueue = NULL;
            return;
        }

        this->_queue = new float[windowSize];
        this->_tempQueue = new float[windowSize];
        for (unsigned int i = 0; i < windowSize; i++) {
            this->_queue[i] = 0.0f;
        }
    }

    float next(float curValue) {
        float nextValue;
        this->totSamples++;
        this->totSamples = MIN(this->totSamples, this->_windowSize);
        if (this->_windowSize <= 0) {
            return curValue;
        }
        if (this->_windowSize == 1) {
            return curValue;
        }

        for (unsigned int i = 0; i < (this->_windowSize - 1); i++) {
            this->_queue[i] = this->_queue[i + 1];
        }
        this->_queue[this->_windowSize - 1] = curValue;
        //for (int i = 0; i < this->_windowSize; i++) {
        //    printf("%f\n", this->_queue[i]);
        //}
        //printf("\n");
        unsigned int dst_index, src_index;

        for (dst_index = 0, src_index = this->_windowSize - this->totSamples; src_index < this->_windowSize; src_index++, dst_index++) {
            _tempQueue[dst_index] = _queue[src_index];
        }
        this->bubbleSort(this->_tempQueue, this->totSamples);
        nextValue = this->_tempQueue[(int)(this->totSamples / 2)];


        //for (int i = 0; i < this->_windowSize; i++) {
        //    printf("%f\n", this->_tempQueue[i]);
        //}
        //printf("\n");
        return nextValue;
    }

    float nextVolatile(float curValue) volatile {
        float nextValue;
        this->totSamples++;
        this->totSamples = MIN(this->totSamples, this->_windowSize);
        if (this->_windowSize <= 0) {
            return curValue;
        }
        if (this->_windowSize == 1) {
            return curValue;
        }

        for (unsigned int i = 0; i < (this->_windowSize - 1); i++) {
            this->_queue[i] = this->_queue[i + 1];
        }
        this->_queue[this->_windowSize - 1] = curValue;
        //for (int i = 0; i < this->_windowSize; i++) {
        //    printf("%f\n", this->_queue[i]);
        //}
        //printf("\n");
        unsigned int dst_index, src_index;

        for (dst_index = 0, src_index = this->_windowSize - this->totSamples; src_index < this->_windowSize; src_index++, dst_index++) {
            _tempQueue[dst_index] = _queue[src_index];
        }
        this->bubbleSort(this->_tempQueue, this->totSamples);
        nextValue = this->_tempQueue[(int)(this->totSamples / 2)];


        //for (int i = 0; i < this->_windowSize; i++) {
        //    printf("%f\n", this->_tempQueue[i]);
        //}
        //printf("\n");
        return nextValue;
    }

    MedianFilter() {
        this->totSamples = 0;
        this->_windowSize = 0;
    }

    void clear(){
        this->totSamples = 0;
    }

    ~MedianFilter() {
        if (this->_queue) {
            delete[] this->_queue;
        }
        if (this->_tempQueue) {
            delete[] this->_tempQueue;
        }
    }
};

#endif