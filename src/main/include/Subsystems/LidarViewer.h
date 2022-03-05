#pragma once
#include <atomic>
#include <memory>
#include <thread>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

typedef struct Point_t {
	int x;
	int y;
	int tstamp;
	} tpPoint;

//#include "../../include/Subsystems/Lidar.h"

class LidarViewer {
 public:
  ~LidarViewer();
  static LidarViewer *Get();
  
  //void  setPoints(int numPoints, lidattp* lidarPts);
  //void  setLines(int numLines, tpLine * lines);
  void addPoint(double dist, double angle);
  void addPointXY(int x, int y, int spline);
  int m_numScoring = 0;
 private:
  LidarViewer();
  void  CameraStreamThread();
  void  convertToXY();

 private:
  std::atomic<bool> running;
  std::atomic<bool> shouldRun;
  std::thread cameraThread;

  //lidattp  m_lidarPts[1024];
  //int      m_numLidarPts = 0;

 	tpPoint  m_cartPts[1024];
  int      m_numCartPts = 0;

  tpPoint  m_scoring[1024];
  
  //tpLine   m_Lines[100];
  //int      m_numLines  = 0;

  static LidarViewer *self;
};
