#include <Arduino.h>
#include <math.h> 

float calculateMean(float arr[], int size) {
    float sum = 0;
    for (int i = 0; i < size; ++i) {
        sum += arr[i];
    }
    return sum / size;
}

// Function to calculate the variance of an array
float calculateVariance(float arr[], int size) {
    float mean = calculateMean(arr, size);
    float variance = 0;
    for (int i = 0; i < size; ++i) {
        variance += (arr[i] - mean) * (arr[i] - mean);
    }
    return variance / size;
}

float update_avg(float myArray[], float myScalar, int sample_pts) {
    for (int i = 0; i<sample_pts-1; i++) {
        myArray[i] = myArray[i+1];
    }
    myArray[sample_pts-1] = myScalar;
    float myAvg = calculateMean(myArray, sample_pts);
    return myAvg;
}