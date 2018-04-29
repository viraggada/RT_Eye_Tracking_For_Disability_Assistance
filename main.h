/**********************************************************
* Authors: Virag Gada and Shreyas Vasanhkumar
* Title: Real-time eye tracking for disability assisatance
* File Name: main.h
**********************************************************/

#ifndef MAIN_H
#define MAIN_H

#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <queue>
#include <iostream>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
//#include <opencv2/contrib/contrib.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#define NUM_THREADS   (3)
#define NUM_CPU_CORES (1)
#define FALSE         (0)
#define TRUE          (1)
#define NSEC          (1000000000)

typedef struct
{
    int threadIdx;
    unsigned long long sequencePeriods;
} threadParams_t;

void print_scheduler(void);
int delta_t(struct timespec *stop, struct timespec *start, struct timespec *delta_t);

#endif
