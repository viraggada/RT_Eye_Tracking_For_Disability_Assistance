/***************************************************************************************************************************
* Authors: Virag Gada and Shreyas Vasanhkumar
* Title: Real-time eye tracking for disability assisatance
* File Name: blink_detection.h
* Reference: Codes for this file taking from http://www.technolabsz.com/2013/05/eye-blink-detection-using-opencv-in.html and
*                                            http://sanyamgarg.blogspot.com/2016/03/a-blink-detection-technique-using.html
***************************************************************************************************************************/


#ifndef BLINK_DETECTION_H
#define BLINK_DETECTION_H


#include "cv.h"
#include "highgui.h"


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <float.h>
#include <limits.h>
#include <time.h>
#include <ctype.h>
#include <iostream>

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


/**********************************************
function defintions
**********************************************/

eyeState_t detect_and_draw( IplImage* image ,CvHaarClassifierCascade* cascade);
void MatchingMethod(cv::Mat templ,int id );
void detect_blink(cv::Mat roi);

typedef enum{
  EYES_OPEN,
  EYES_CLOSE
}eyeState_t;

#endif
