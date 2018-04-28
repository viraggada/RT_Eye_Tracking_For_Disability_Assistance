#include "cv.h"
#include "highgui.h"
#include "blink_detection.h"


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

/*******************************************************************
Function to detect and draw any faces that is present in an image
********************************************************************/

bool detect_and_draw( IplImage* img,CvHaarClassifierCascade* cascade )
{
    int scale = 1;
    Mat image;

    // Create a new image based on the input image
    IplImage* temp = cvCreateImage( cvSize(img->width/scale,img->height/scale), 8, 3 );

    // Create two points to represent the face locations
    CvPoint pt1, pt2;
    int i;

    // Clear the memory storage which was used before
    cvClearMemStorage( storage );

    // Find whether the cascade is loaded, to find the faces. If yes, then:
    if( cascade )
    {

        // There can be more than one face in an image. So create a growable sequence of faces.
        // Detect the objects and store them in the sequence
        CvSeq* faces = cvHaarDetectObjects( img, cascade, storage,
                                            1.1, 8, CV_HAAR_DO_CANNY_PRUNING,
                                            cvSize(40, 40) );

        // Loop the number of faces found.
        for( i = 0; i < (faces ? faces->total : 0); i++ )
        {
           // Create a new rectangle for drawing the face
            CvRect* r = (CvRect*)cvGetSeqElem( faces, i );

            // Find the dimensions of the face,and scale it if necessary
            pt1.x = r->x*scale;
            pt2.x = (r->x+r->width)*scale;
            pt1.y = r->y*scale;
            pt2.y = (r->y+r->height)*scale;

            // Draw the rectangle in the input image
            cvRectangle( img, pt1, pt2, CV_RGB(255,0,0), 3, 8, 0 );

	  cv::Mat image = cv::cvarrToMat(img);
	  cv::Rect rect;

	  rect = cv::Rect(pt1.x,pt1.y,(pt2.x-pt1.x),(pt2.y-pt1.y));

	  roiImg = image(rect);
    	  cv::imshow("roi",roiImg);
///Send to arduino

        }
    }
    // Show the image in the window named "result"
    cvShowImage( "original_frame", img );

   if(i  > 0)
		return 1;
	else
		return 0;
    // Release the temp image created.
    cvReleaseImage( &temp );

}

void detect_blink(cv::Mat roi)
{


	try
	{
	MatchingMethod(img1,0);
 	MatchingMethod(img2,1);

	}

	catch( cv::Exception& e )

	{
		std::cout<<"An exception occued"<<std::endl;
	}
}

//Matching with 2 images ,eye closed or open
void MatchingMethod(cv::Mat templ,int id )
{
  /// Source image to display
  cv::Mat img_display;
  roiImg.copyTo( img_display );

  /// Create the result matrix
  int result_cols =  roiImg.cols - templ.cols + 1;
  int result_rows = roiImg.rows - templ.rows + 1;

  result.create( result_cols, result_rows, CV_32FC1 );

  /// Do the Matching and Normalize
  cv::matchTemplate( roiImg, templ, result, match_method );
  cv::normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );

  /// Localizing the best match with minMaxLoc
  double minVal; double maxVal; Point minLoc; Point maxLoc;
  cv::Point matchLoc;

  cv::minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );



  ///Justing checkin the match template value reaching the threashold
  if(id == 0 && (minVal < 0))
	{
	eye_open=eye_open+1;
	if(eye_open == 10)
		{
		std::cout<<"Eye Open"<<std::endl;
		eye_open=0;
		eye_close=0;
		}
	}
   else if(id == 1 && (minVal < 0))
	eye_close=eye_close+1;
	if(eye_close == 10)
		{
		std::cout<<"Eye Closed"<<std::endl;
		eye_close=0;
		system("python send_arduino.py");
		}



  /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
  if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
    { matchLoc = minLoc; }
  else
    { matchLoc = maxLoc; }

  /// Show me what you got
  cv::rectangle( img_display, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );
  cv::rectangle( result, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );

  cv::imshow( image_window, img_display );
  cv::imshow( result_window, result );

  return;
}

