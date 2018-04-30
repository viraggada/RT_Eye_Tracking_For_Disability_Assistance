

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

bool detect_and_draw( IplImage* image ,CvHaarClassifierCascade* cascade);
void MatchingMethod(cv::Mat templ,int id );
void detect_blink(cv::Mat roi);


#endif
