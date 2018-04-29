/**********************************************************
* Authors: Virag Gada and Shreyas Vasanhkumar
* Title: Real-time eye tracking for disability assisatance
* File Name: eyeTracking.h
**********************************************************/

#ifndef EYETRACKING_H
#define EYETRACKING_H

#include <vector>
#include "main.h"
#include "constants.h"
#include "findEyeCenter.h"
#include "findEyeCorner.h"

/** Function Headers */
int detectAndDisplay( cv::Mat frame);
void findEyes(cv::Mat frame_gray, cv::Rect face);

/* Global Variable */
/*cv::String face_cascade_name = "haarcascade_frontalface_alt.xml";
cv::Mat debugImage;
cv::CascadeClassifier face_cascade;
std::string main_window_name = "Capture - Face detection";
std::string face_window_name = "Capture - Face";
cv::RNG rng(12345);
cv::Mat skinCrCbHist = cv::Mat::zeros(cv::Size(256, 256), CV_8UC1);
*/
#endif
