

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

// Create memory for calculations
static CvMemStorage* storage = 0;
// Create a new Haar classifier
static CvHaarClassifierCascade* cascade = 0;
Mat img1; Mat img2; Mat templ; Mat result;

int threshold_value = 200;
int threshold_type = 3;;
int const max_value = 255;
int const max_type = 4;
int const max_BINARY_value = 255;

const char* image_window = "Source Image";
const char* result_window = "Result window";

int match_method=0;
int max_Trackbar = 5;
int eye_open=0;
int eye_close=0;

/**********************************************
function defintions
**********************************************/

bool detect_and_draw( IplImage* image ,CvHaarClassifierCascade* cascade);
void MatchingMethod(cv::Mat templ,int id );
void detect_blink(cv::Mat roi);


#endif
