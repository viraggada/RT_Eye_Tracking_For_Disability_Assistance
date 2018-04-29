/**********************************************************
* Authors: Virag Gada and Shreyas Vasanhkumar
* Title: Real-time eye tracking for disability assisatance
* File Name: main.c
**********************************************************/

#include <stdlib.h>
#include <pthread.h>
#include <sched.h>
#include <vector>
#include <syslog.h>
#include <sys/time.h>
#include <semaphore.h>
#include "main.h"
#include "eyeTracking.h"
#include "findEyeCorner.h"

int abortTest=FALSE;
int abortS1=FALSE, abortS2=FALSE, abortS3=FALSE, abortS4=FALSE, abortS5=FALSE;
sem_t sem_takeImage, sem_detectFace, sem_detectEye, sem_detectBlink, sem_controlMouse;

void *captureImage(void *threadp);
void *extractFace(void *threadp);
void *eyeTracking(void *threadp);
void *mouseControl(void *threadp);

cv::Mat frame; //frame for captured image
std::vector<cv::Rect> faces; // vector to hold face
cv::Mat frame_gray; // store red channel

extern cv::String face_cascade_name;
extern cv::Mat skinCrCbHist;
extern cv::Mat debugImage;
extern cv::CascadeClassifier face_cascade;
extern std::string main_window_name;
extern std::string face_window_name;

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

  //printf("System has %d processors configured and %d available.\n", get_nprocs_conf(), get_nprocs());

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
    CPU_SET(3, &threadcpu);

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
  rt_param[1].sched_priority=rt_max_prio-1;
  pthread_attr_setschedparam(&rt_sched_attr[1], &rt_param[1]);
  rc=pthread_create(&threads[1],               // pointer to thread descriptor
                    &rt_sched_attr[1],         // use specific attributes
                    //(void *)0,               // default attributes
                    captureImage,                 // thread function entry point
                    (void *)&(threadParams[1]) // parameters to pass in
                   );
  if(rc < 0)
    perror("pthread_create for capture image");
  else
    printf("pthread_create successful for capture image\n");

  // Face detection thread
  rt_param[2].sched_priority=rt_max_prio-2;
  pthread_attr_setschedparam(&rt_sched_attr[2], &rt_param[2]);
  rc=pthread_create(&threads[2], &rt_sched_attr[2], extractFace, (void *)&(threadParams[2]));
  if(rc < 0)
    perror("pthread_create for face detection");
  else
    printf("pthread_create successful for face detection\n");

  // Eye tracking thread
  rt_param[3].sched_priority=rt_max_prio-3;
  pthread_attr_setschedparam(&rt_sched_attr[3], &rt_param[3]);
  rc=pthread_create(&threads[3], &rt_sched_attr[3], eyeTracking, (void *)&(threadParams[3]));
  if(rc < 0)
  perror("pthread_create for eye tracking");
  else
    printf("pthread_create successful for eye tracking\n");

  //Mouse control thread
  /*rt_param[4].sched_priority=rt_max_prio-2;
  pthread_attr_setschedparam(&rt_sched_attr[4], &rt_param[4]);
  rc=pthread_create(&threads[4], &rt_sched_attr[4], mouseControl, (void *)&(threadParams[4]));
  if(rc < 0)
    perror("pthread_create for service 4");
  else
    printf("pthread_create successful for mouse control\n");*/

  for(i=0;i<NUM_THREADS;i++)
    pthread_join(threads[i], NULL);

  return 0;
}


void *captureImage(void *threadp){
  //cv::Mat frame;

  // Load the cascades
  if( !face_cascade.load( face_cascade_name ) ){ printf("--(!)Error loading face cascade, please change face_cascade_name in source code.\n");
 
   }
  else{

  cv::namedWindow(main_window_name,CV_WINDOW_NORMAL);
  cv::moveWindow(main_window_name, 400, 100);
  cv::namedWindow(face_window_name,CV_WINDOW_NORMAL);
  cv::moveWindow(face_window_name, 10, 100);
  cv::namedWindow("Right Eye",CV_WINDOW_NORMAL);
  cv::moveWindow("Right Eye", 10, 600);
  cv::namedWindow("Left Eye",CV_WINDOW_NORMAL);
  cv::moveWindow("Left Eye", 10, 800);
  cv::namedWindow("aa",CV_WINDOW_NORMAL);
  cv::moveWindow("aa", 10, 800);
  cv::namedWindow("aaa",CV_WINDOW_NORMAL);
  cv::moveWindow("aaa", 10, 800);

  createCornerKernels();
  ellipse(skinCrCbHist, cv::Point(113, 155.6), cv::Size(23.4, 15.2),
        43.0, 0.0, 360.0, cv::Scalar(255, 255, 255), -1);

  // I make an attempt at supporting both 2.x and 3.x OpenCV
  cv::VideoCapture capture(0);
  if( capture.isOpened() ) {
    while( true ) {

      sem_wait(&sem_takeImage);
      capture.read(frame);
      // mirror it
      cv::flip(frame, frame, 1);
      frame.copyTo(debugImage);

      // Apply the classifier to the frame
      if( !frame.empty() ) {
        //detectAndDisplay( frame );
        sem_post(&sem_detectFace);
      }
      else {
        printf(" --(!) No captured frame -- Break!");
        break;
      }
      //imshow(main_window_name,debugImage);
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

  while(true){
    sem_wait(&sem_detectFace);
    if(detectAndDisplay(frame,faces,frame_gray)==0)
      sem_post(&sem_detectEye);
  }
}

void *eyeTracking(void *threadp){

  while(true){
   sem_wait(&sem_detectEye);
   findEyes(frame_gray, faces[0]);
   imshow(main_window_name,debugImage);
   sem_post(&sem_takeImage);
  }
}

void *mouseControl(void *threadp){


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
