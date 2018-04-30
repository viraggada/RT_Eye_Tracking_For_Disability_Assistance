/**********************************************************
* Authors: Virag Gada and Shreyas Vasanhkumar
* Title: Real-time eye tracking for disability assisatance
* File Name: main.c
**********************************************************/

#include <stdlib.h>
#include <pthread.h>
#include <sched.h>
#include <atomic>
#include <vector>
#include <syslog.h>
#include <sys/time.h>
#include <semaphore.h>
#include "main.h"
#include "eyeTracking.h"
#include "findEyeCorner.h"
#include "blink_detection.h"
#include "cv.h"
#include "highgui.h"

std::atomic<int> checkVal(0);
//checkVal.store(1,std::memory_order_relaxed);

int abortTest=FALSE;
int abortS1=FALSE, abortS2=FALSE, abortS3=FALSE, abortS4=FALSE, abortS5=FALSE;
sem_t sem_takeImage, sem_detectFace, sem_detectEye, sem_detectBlink, sem_controlMouse;

void *captureImage(void *threadp);
void *extractFace(void *threadp);
void *eyeTracking(void *threadp);
void *eyeBlinking(void *threadp);
void *mouseControl(void *threadp);


cv::Mat frame; //frame for captured image
extern std::vector<cv::Rect> faces; // vector to hold face
extern cv::Mat frame_gray; // store red channel

extern cv::String face_cascade_name;
extern cv::Mat skinCrCbHist;
extern cv::Mat debugImage;
extern cv::CascadeClassifier face_cascade;
extern std::string main_window_name;
extern std::string face_window_name;

extern const char* result_window;

extern CvMemStorage* storage;

// Create a new Haar classifier
extern CvHaarClassifierCascade* cascade;

const char *cascade_name[1]={"eyes.xml"};

// Structure for getting video from camera or avi
extern CvCapture* capture;

// Images to capture the frame from video or camera or from file
//IplImage *frame_copy;
IplImage *copy_frame =NULL;

// Used for calculations
extern int optlen;

// Input file name for avi or image file.
extern const char* input_name;

using namespace cv; 

Mat img1 = imread( "open.jpg", 1 );
Mat img2 = imread( "close.jpg", 1 );

int main(int argc, const char** argv)
{
  struct timeval current_time_val;
  int i, rc, scope;
  cpu_set_t threadcpu;
  pthread_t threads[NUM_THREADS];
  threadParams_t threadParams[NUM_THREADS];
  pthread_attr_t rt_sched_attr[NUM_THREADS];
  int rt_max_prio, rt_min_prio;
  struct sched_param rt_param[NUM_THREADS];
  struct sched_param main_param;
  pthread_attr_t main_attr;
  pid_t mainpid;
  cpu_set_t allcpuset;


 // printf("System has %d processors configured and %d available.\n", get_nprocs_conf(), get_nprocs());

  CPU_ZERO(&allcpuset);

  for(i=0; i < NUM_CPU_CORES; i++)
     CPU_SET(i, &allcpuset);

  printf("Using CPUS=%d from total available.\n", CPU_COUNT(&allcpuset));


  // initialize the sequencer semaphores
  //
  if (sem_init (&sem_takeImage, 0, 1)) { printf ("Failed to initialize S1 semaphore\n"); exit (-1); }
  if (sem_init (&sem_detectFace, 0, 0)) { printf ("Failed to initialize S2 semaphore\n"); exit (-1); }
  if (sem_init (&sem_detectEye, 0, 0)) { printf ("Failed to initialize S3 semaphore\n"); exit (-1); }
  if (sem_init (&sem_detectBlink, 0, 0)) { printf ("Failed to initialize S4 semaphore\n"); exit (-1); }
  if (sem_init (&sem_controlMouse, 0, 0)) { printf ("Failed to initialize S5 semaphore\n"); exit (-1); }

  mainpid = getpid();

  rt_max_prio = sched_get_priority_max(SCHED_FIFO);
  rt_min_prio = sched_get_priority_min(SCHED_FIFO);

  rc=sched_getparam(mainpid, &main_param);
  main_param.sched_priority=rt_max_prio;
  rc=sched_setscheduler(getpid(), SCHED_FIFO, &main_param);
  if(rc < 0) perror("main_param");
  print_scheduler();


  pthread_attr_getscope(&main_attr, &scope);

  if(scope == PTHREAD_SCOPE_SYSTEM)
    printf("PTHREAD SCOPE SYSTEM\n");
  else if (scope == PTHREAD_SCOPE_PROCESS)
    printf("PTHREAD SCOPE PROCESS\n");
  else
    printf("PTHREAD SCOPE UNKNOWN\n");

  printf("rt_max_prio=%d\n", rt_max_prio);
  printf("rt_min_prio=%d\n", rt_min_prio);

  for(i=0; i < NUM_THREADS; i++)
  {

    CPU_ZERO(&threadcpu);
    CPU_SET(i, &threadcpu);

    rc=pthread_attr_init(&rt_sched_attr[i]);
    rc=pthread_attr_setinheritsched(&rt_sched_attr[i], PTHREAD_EXPLICIT_SCHED);
    rc=pthread_attr_setschedpolicy(&rt_sched_attr[i], SCHED_FIFO);
    //rc=pthread_attr_setaffinity_np(&rt_sched_attr[i], sizeof(cpu_set_t), &threadcpu);

    rt_param[i].sched_priority=rt_max_prio-i;
    pthread_attr_setschedparam(&rt_sched_attr[i], &rt_param[i]);

    threadParams[i].threadIdx=i;
  }

  // Create Service threads which will block awaiting release for:
  //

  // Image capture thread
  rt_param[0].sched_priority=rt_max_prio-1;
  pthread_attr_setschedparam(&rt_sched_attr[0], &rt_param[0]);
  rc=pthread_create(&threads[0],               // pointer to thread descriptor
                    &rt_sched_attr[0],         // use specific attributes
                    //(void *)0,               // default attributes
                    captureImage,                 // thread function entry point
                    (void *)&(threadParams[0]) // parameters to pass in
                   );
  if(rc < 0)
    perror("pthread_create for capture image");
  else
   {
    printf("pthread_create successful for capture image\n");
    //perror("capture imag");
   }

  // Face detection thread
  rt_param[1].sched_priority=rt_max_prio-2;
  pthread_attr_setschedparam(&rt_sched_attr[1], &rt_param[1]);
  rc=pthread_create(&threads[1], &rt_sched_attr[1], extractFace, (void *)&(threadParams[1]));
  if(rc < 0)
    perror("pthread_create for face detection");
  else
   {
     printf("pthread_create successful for face detection\n");
     //perror("Face detec");
   }
  // Eye tracking thread
  rt_param[2].sched_priority=rt_max_prio-3;
  pthread_attr_setschedparam(&rt_sched_attr[2], &rt_param[2]);
  rc=pthread_create(&threads[2], &rt_sched_attr[2], eyeTracking, (void *)&(threadParams[2]));
  if(rc < 0)
  perror("pthread_create for eye tracking");
  else
   {
     printf("pthread_create successful for eye tracking\n");
    // perror("Eye tracking");
    }

   // Eye Blinking thread
  rt_param[3].sched_priority=rt_max_prio-4;
  pthread_attr_setschedparam(&rt_sched_attr[3], &rt_param[3]);
  rc=pthread_create(&threads[3], &rt_sched_attr[3], eyeBlinking, (void *)&(threadParams[3]));
  if(rc < 0)
  perror("pthread_create for eye blinking");
  else
   {
     printf("pthread_create successful for eye blinking\n");
     //perror("Eye Blinking");
    }
  //Mouse control thread
  /*rt_param[3].sched_priority=rt_max_prio-2;
  pthread_attr_setschedparam(&rt_sched_attr[3], &rt_param[3]);
  rc=pthread_create(&threads[4], &rt_sched_attr[3], mouseControl, (void *)&(threadParams[3]));
  if(rc < 0)
    perror("pthread_create for service 4");
  else
    printf("pthread_create successful for mouse control\n");*/

 // while(1);

  for(i=0;i<NUM_THREADS;i++)
  {
   if( pthread_join(threads[i], NULL) !=0)
    {
      perror("pthread_join() error");
    }
  }

  return 0;
}

void *captureImage(void *threadp){
  
  std::cout << "capture image thread created" << std::endl;
  struct timespec startTime = {0, 0}, stopTime = {0, 0}, timeDifference = {0, 0};
  // Load the cascades
  if( !face_cascade.load( face_cascade_name ) ){ printf("--(!)Error loading face cascade, please change face_cascade_name in source code.\n");

   }
  else{

  cv::namedWindow(main_window_name,CV_WINDOW_NORMAL);
  cv::moveWindow(main_window_name, 400, 100);
  cv::namedWindow(face_window_name,CV_WINDOW_NORMAL);
  cv::moveWindow(face_window_name, 10, 100);
  //cv::namedWindow("Right Eye",CV_WINDOW_NORMAL);
  //cv::moveWindow("Right Eye", 10, 600);
  //cv::namedWindow("Left Eye",CV_WINDOW_NORMAL);
  //cv::moveWindow("Left Eye", 10, 800);
  //cv::namedWindow("aa",CV_WINDOW_NORMAL);
  //cv::moveWindow("aa", 10, 800);
  //cv::namedWindow("aaa",CV_WINDOW_NORMAL);
  //cv::moveWindow("aaa", 10, 800);
  //cv::namedWindow( result_window, CV_WINDOW_AUTOSIZE );

  storage = cvCreateMemStorage(0);

  createCornerKernels();
  ellipse(skinCrCbHist, cv::Point(113, 155.6), cv::Size(23.4, 15.2),
        43.0, 0.0, 360.0, cv::Scalar(255, 255, 255), -1);

  std::cout << "Accessing Camera..." << std::endl;

     cv::VideoCapture capture(0);
      if(capture.isOpened()) {
       std::cout << "Camera is opened" << std::endl;
       while( true ) {
      sem_wait(&sem_takeImage);
      clock_gettime(CLOCK_REALTIME, &startTime);
      //std::cout << "Image capture semaphore taken " << std::endl;
      capture.read(frame);

      // mirror it
      cv::flip(frame, frame, 1);
      frame.copyTo(debugImage);

      // Apply the classifier to the frame
      if( !frame.empty() ) {
        //detectAndDisplay( frame );
        clock_gettime(CLOCK_REALTIME, &stopTime);
        sem_post(&sem_detectFace);
        delta_t(&stopTime,&startTime,&timeDifference);
        syslog(LOG_INFO,"Image capture execution time - %ld sec, %ld nsec\n",timeDifference.tv_sec,timeDifference.tv_nsec);
        //imwrite("frame.png",frame);
      }
      else {
        printf(" --(!) No captured frame -- Break!");
        break;
      }
      imshow(main_window_name,frame);
      int c = cv::waitKey(10);
      if( (char)c == 'c' ) { break; }
      if( (char)c == 'f' ) {
        imwrite("frame.png",frame);
      }
    }
  }

  releaseCornerKernels();
  }
}

void *extractFace(void *threadp){
  std::cout << "extract face thread created" <<  std::endl;
  struct timespec startTime = {0, 0}, stopTime = {0, 0}, timeDifference = {0, 0};

  while(true){
    sem_wait(&sem_detectFace);
    clock_gettime(CLOCK_REALTIME, &startTime);
    //std::cout << "face extraction sem " << std::endl;
    if(detectAndDisplay(frame)==0){
      clock_gettime(CLOCK_REALTIME, &stopTime);
      sem_post(&sem_detectBlink);
      sem_post(&sem_detectEye);
    }
    else{
      clock_gettime(CLOCK_REALTIME, &stopTime);
      sem_post(&sem_takeImage);
    }
    delta_t(&stopTime,&startTime,&timeDifference);
    syslog(LOG_INFO,"Face extraction execution time - %ld sec, %ld nsec\n",timeDifference.tv_sec,timeDifference.tv_nsec);
    //std::cout << " face extracted " << std::endl;
  }
}

void *eyeTracking(void *threadp){
  std::cout << "eye tracking thread created" << std::endl;
  struct timespec startTime = {0, 0}, stopTime = {0, 0}, timeDifference = {0, 0};
  
  while(true){
    sem_wait(&sem_detectEye);
    clock_gettime(CLOCK_REALTIME, &startTime);
  //std::cout << "eye tracking sem" << std::endl;
    findEyes(frame_gray, faces[0]);
    imshow(main_window_name,debugImage);
    clock_gettime(CLOCK_REALTIME, &stopTime);
    if(checkVal.load(std::memory_order_relaxed) == 1){
      checkVal.store(0,std::memory_order_relaxed);
      sem_post(&sem_takeImage);
    }else
      checkVal.store(1,std::memory_order_relaxed);
  
    delta_t(&stopTime,&startTime,&timeDifference);
    syslog(LOG_INFO,"Eye detection execution time - %ld sec, %ld nsec\n",timeDifference.tv_sec,timeDifference.tv_nsec);
  }
}

void *mouseControl(void *threadp){


}

void *eyeBlinking(void *threadp){
  std::cout << "Eye blinking thread created" << std::endl;
  struct timespec startTime = {0, 0}, stopTime = {0, 0}, timeDifference = {0, 0};
  while(true){
  sem_wait(&sem_detectBlink);
  clock_gettime(CLOCK_REALTIME, &startTime);
   //std::cout << "Eye blinking check" << std::endl;
   
  cascade = (CvHaarClassifierCascade*)cvLoad( cascade_name[0], 0, 0, 0 );

    // Check whether the cascade has loaded successfully. Else report and error and quit
    if( !cascade )
        {
         fprintf( stderr, "ERROR: Could not load classifier cascade\n" );
         //return -1;
        }

	copy_frame = cvCreateImage(cvSize(frame.cols,frame.rows),8,3);
	IplImage ipltemp = frame;
	cvCopy(&ipltemp,copy_frame);
    // Call the function to detect and draw the face
        
        if(detect_and_draw(copy_frame,cascade))
        {
                ;
        }


    // Wait for a while before proceeding to the next frame
    //if( cvWaitKey( 1 ) >= 0 )
        clock_gettime(CLOCK_REALTIME, &stopTime);
        if(checkVal.load(std::memory_order_relaxed) == 1){
          checkVal.store(0,std::memory_order_relaxed);
          sem_post(&sem_takeImage);
        }else{
           checkVal.store(1,std::memory_order_relaxed);
        }
        delta_t(&stopTime,&startTime,&timeDifference);
        syslog(LOG_INFO,"Eye Blinking execution time - %ld sec, %ld nsec\n",timeDifference.tv_sec,timeDifference.tv_nsec);
      }
       cvReleaseHaarClassifierCascade(&cascade);
       cvReleaseImage( &copy_frame );
       //cvReleaseCapture( &capture );
       cvReleaseMemStorage(&storage);
}
void print_scheduler(void)
{
   int schedType;

   schedType = sched_getscheduler(getpid());

   switch(schedType)
   {
       case SCHED_FIFO:
           printf("Pthread Policy is SCHED_FIFO\n");
           break;
       case SCHED_OTHER:
           printf("Pthread Policy is SCHED_OTHER\n"); exit(-1);
         break;
       case SCHED_RR:
           printf("Pthread Policy is SCHED_RR\n"); exit(-1);
           break;
       default:
           printf("Pthread Policy is UNKNOWN\n"); exit(-1);
   }
}

// Function to calculate difference in time
int delta_t(struct timespec *stop, struct timespec *start, struct timespec *delta_t)
{
  int dt_sec=stop->tv_sec - start->tv_sec;
  int dt_nsec=stop->tv_nsec - start->tv_nsec;

  if(dt_sec >= 0)
  {
    if(dt_nsec >= 0)
    {
      delta_t->tv_sec=dt_sec;
      delta_t->tv_nsec=dt_nsec;
    }
    else
    {
      delta_t->tv_sec=dt_sec-1;
      delta_t->tv_nsec=NSEC+dt_nsec;
    }
  }
  else
  {
    if(dt_nsec >= 0)
    {
      delta_t->tv_sec=dt_sec;
      delta_t->tv_nsec=dt_nsec;
    }
    else
    {
      delta_t->tv_sec=dt_sec-1;
      delta_t->tv_nsec=NSEC+dt_nsec;
    }
  }

  return(1);
}
