#include <vector>

#include <cameraserver/CameraServer.h>
#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/Timer.h>
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

// #include "ApproxMath.h"
// #include "BallFinder.h"
//#include "../../include/Subsystems/Lidar.h"
#include "Subsystems/LidarViewer.h"
// #include "Log.h"
// #include "Settings.h"

// REGISTER_BOOL(EnableLidarViewer, "Lidar: Enable viewer", false)
// REGISTER_DOUBLE(LidarZoom, "Lidar: Zoom", 0.075)
// REGISTER_INT(LidarCameraWidth, "Lidar: Camera stream width", 480)
// REGISTER_INT(LidarCameraHeight, "Lidar: Camera stream height", 480)

static const bool ENABLE_LIDAR_VIEWER = true;
static const double LIDAR_ZOOM = 0.075;
static const int LIDAR_CAMERA_WIDTH = 800; // 480; // 800 x 480 works!
static const int LIDAR_CAMERA_HEIGHT = 480 ; // 480;



LidarViewer *LidarViewer::Get() {
  if (self == nullptr) {
    self = new LidarViewer();
  }
  return self;
}

LidarViewer::LidarViewer() {
  cameraThread = std::thread(&LidarViewer::CameraStreamThread, this);
}

LidarViewer::~LidarViewer() { shouldRun = false; }

void LidarViewer::CameraStreamThread() {
  
  shouldRun = true;
  running = true;
  // Log("Lidar viewer stream thread starting");

  // Lidar *lidar = Lidar::Get();
  
  const int cameraWidth = LIDAR_CAMERA_WIDTH;
  const int cameraHeight = LIDAR_CAMERA_HEIGHT;

  auto s = frc::CameraServer::GetInstance();
  cs::CvSource cameraStream = s->PutVideo("Lidar", cameraWidth, cameraHeight);

  double zoom = 2.0;
  static const cv::Scalar red(50, 50, 255);
  static const cv::Scalar blue(255, 100, 100);
  static const cv::Scalar green(50, 255, 50);
  static const cv::Scalar white(255, 255, 255);
  static const cv::Scalar orange(128, 128, 255);
  static const cv::Size size(cameraWidth, cameraHeight);
  static const int centerX = cameraWidth >> 1;
  static const int centerY = cameraHeight;
  static const auto type = CV_8UC3;

  int frames = 0;
  const double timeStarted = (double)frc::Timer::GetFPGATimestamp();
  double lastFrameTime = timeStarted;
  bool createBaseFrame = true;

  cv::Mat baseFrame;
  //LidarData::Buffer nodes;
  double lastLidarViewerCheck = 0;
  bool enabled = true;
  //m_numLidarPts = 0;
  m_numCartPts = 0;
  //m_numLines  = 0;
  // m_numScoring = 0;



  while (shouldRun) {
    //  Reset the class variables needed for displaying game objects

    const double now = (double)frc::Timer::GetFPGATimestamp();

    if (now - lastLidarViewerCheck > 5) {
      lastLidarViewerCheck = now;
      enabled = ENABLE_LIDAR_VIEWER;
      const double changeZoom = LIDAR_ZOOM;
      if (zoom != changeZoom) {
        zoom = changeZoom;
        createBaseFrame = true;
        // Log("Changing LIDAR zoom level to %f", zoom);
      }
    }
    if (!enabled) {
      frc::Wait(5.1_s);
      continue;
    }

    if (createBaseFrame) {
      baseFrame = cv::Mat::zeros(size, type);
      // cv::circle(baseFrame, cv::Point(centerX, centerY), 6, red, 1);
      cv::putText(baseFrame, "Field", cv::Point(10, 15),
                  cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, 0.5, white, 1);

//      const int radii[] = {500, 1000, 2000, 3000, 4000, 5000, 6000};
//      for (int i = 0; i < (int)(sizeof(radii) / sizeof(int)); ++i) {
//        const int radius = radii[i] * zoom;
//        cv::circle(baseFrame, cv::Point(centerX, centerY), radius, blue, 1);
      // zoom does get set to 0.075 at some point.  6000 * 0.075 = 450 so we're really dealing with
      // a camera feed of 480 x 480. Can we change it to be a bit more "field friendly"
      // say 800 x 400?  The 2.5ft dots on 30 ft -> 11 in the X (800/12 = 66.67), 6 in the y 
      // Draw the challenge field points
      int skip;
      for (int xd = 66;xd < 775;xd+=66) // Draw all the locations where cones might go.
        {
        for (int yd = 80;yd < 475;yd+=80)
          {
          skip = 0; // Don't skip by default.  On line 1 and 5, skip position 2, 5 and 11
          if (((yd == 80)||(yd == 80 * 5))&&((xd == 66*2)||(xd == 66 * 5)||(xd == 66 * 11)))
            skip = 1;
          if ((yd == 80 * 3)&&((xd != 66 * 3)&&(xd != 66 * 9))) // only include C3, C9 in center line.
            skip  = 1;
          if (!skip)
            cv::circle(baseFrame, cv::Point(xd, yd), 3, orange, 1);
          }
        }
      createBaseFrame = false;
    }

    if (0) {
      // Don't want this thread taking over everything.
      const static double minWait = 0.2;  // about 5 fps?
      const double elapsed = now - lastFrameTime;
      const double waitTime = elapsed < minWait ? minWait - elapsed : 0.01;
      //frc::Wait(waitTime);
      frc::Wait(0.3_s); //!!!!!WHAT WAS THIS FOR???
      lastFrameTime = now;
    } else {
      // The main thread is only going to update at most every 20ms.
      frc::Wait(0.2_s); // Trying for 5 fps.  We were getting 8 or 9.
//      printf("\nLidarViewer is running");
    }
    cv::Mat frame = baseFrame.clone();

//    lidar->GetDataPoints(nodes);

#ifdef SHOW_BOUNDARIES
    for (int i = 0; i < nodes.boundaryCount; ++i) {
      const double a = nodes.boundaryAngles[i];
      const double d = 2000;  //??
      const int x = centerX + (d * ApproxCos(a)) - 1;
      const int y = centerY + (d * ApproxSin(a)) - 1;
      cv::line(frame, cv::Point(centerX, centerY), cv::Point(x, y), red, 1);
    }
#endif

    // for (int i = 0; i < nodes.pointCount; i++) {
    //   const int x = centerX + nodes.points[i].x * zoom - 1;
    //   const int y = centerY + nodes.points[i].y * zoom - 1;
    //   if (x <= cameraWidth && x >= 0 && y <= cameraHeight && y >= 0) {
    //     cv::rectangle(frame, cv::Rect(x, y, 3, 3),
    //                   nodes.points[i].noise ? blue : white, 1);
    //   }
    // }

    // for (int i = 0; i < nodes.circleCount; i++) {
    //   const cv::Point c(centerX + nodes.circles[i].center.x * zoom,
    //                     centerY + nodes.circles[i].center.y * zoom);
    //   cv::circle(frame, c, nodes.circles[i].radius * zoom, red);
    // }

    // for (int i = 0; i < nodes.lineCount; i++) {
    //   const int x1 = centerX + nodes.lines[i].start.x * zoom;
    //   const int y1 = centerY + nodes.lines[i].start.y * zoom;
    //   const int x2 = centerX + nodes.lines[i].end.x * zoom;
    //   const int y2 = centerY + nodes.lines[i].end.y * zoom;
    //   cv::line(frame, cv::Point(x1, y1), cv::Point(x2, y2), red, 2);
    //   // Log("Line from %d, %d - %d, %d", x1, y1, x2, y2);
    // }

  //convertToXY();
  /*for (int i = 0; i < m_numCartPts; i++) {
      const int x = centerX + m_cartPts[i].x * zoom - 1;
      const int y = centerY + m_cartPts[i].y * zoom - 1;
      if (x <= cameraWidth && x >= 0 && y <= cameraHeight && y >= 0) {
        cv::rectangle(frame, cv::Rect(x, y, 3, 3), white, 1);
      }
    }

   for (int i = 0; i < m_numLines; i++) {
      const int x1 = centerX + m_Lines[i].start.x * zoom;
      const int y1 = centerY + (-m_Lines[i].start.y) * zoom;
      const int x2 = centerX + m_Lines[i].end.x * zoom;
      const int y2 = centerY + (-m_Lines[i].end.y) * zoom;
      cv::line(frame, cv::Point(x1, y1), cv::Point(x2, y2), red, 2);
      // Log("Line from %d, %d - %d, %d", x1, y1, x2, y2);
    }
    */
    int numPlotted = 0;
    for (int i = 0; i < m_numScoring; i++) {
      const int x = m_scoring[i].x-1; //  * zoom - 1;
      const int y = m_scoring[i].y-1; // ) * zoom - 1;
      if ((x <= cameraWidth) && (x >= 0) && (y <= cameraHeight) && (y >= 0)) { 
          if (m_scoring[i].tstamp == 1)
            cv::rectangle(frame, cv::Rect(x, y, 2, 2), green, 1);
          else
            cv::rectangle(frame, cv::Rect(x, y, 4, 4), red, 2);
//        else
//          cv::rectangle(frame, cv::Rect(x, y, 5, 5), blue, 1);
        numPlotted++;
      }
    else
    {
        // printf("\nSkipped (%d,%d)",x,y);
    }
    
//    printf("\nm_numScoring=%d, plotted=%d",m_numScoring,numPlotted);
    }

    cameraStream.PutFrame(frame);

    if (frames++ % 20 == 0) {
      const double elapsed = (double)frc::Timer::GetFPGATimestamp() - timeStarted;
      if (elapsed > 1.0) {
        frc::SmartDashboard::PutNumber("Lidar FPS", frames / elapsed);
      }
      //frc::SmartDashboard::PutNumber("Lidar lines found", m_numLines);
    }
  }
  running = false;
}

LidarViewer *LidarViewer::self = nullptr;

/*void  LidarViewer::setLines(int numLines, tpLine* lines){
  for (int i=0; i<(numLines+1); i++){
    m_Lines[i] = lines[i];
  }

  m_numLines = numLines+1;
  //printf("LidarViewer::setLines, numLines =%d\n", numLines);
}*/

/*void  LidarViewer::setPoints(int numPoints, lidattp* lidarPts){

  for (int i=0; i<numPoints; i++){
      m_lidarPts[i] = lidarPts[i];
	}

  m_numLidarPts = numPoints;
  //printf("LidarViewer::setPoints, numPoints =%d\n", numPoints);
}*/

void  LidarViewer::addPoint(double dist, double angle){
  if (m_numScoring >= 6)
    m_numScoring = 0;

  double rad = M_PI * (angle) / 180;
 
	m_scoring[m_numScoring].x = -(((dist) * std::sin(rad)));
	m_scoring[m_numScoring].y = (((dist) * std::cos(rad)));

  m_numScoring++;
}

void LidarViewer::addPointXY(int x, int y, int spline) {
  if (m_numScoring < 1024)
    {
    m_scoring[m_numScoring].x = x;
    m_scoring[m_numScoring].y = LIDAR_CAMERA_HEIGHT - y;
    m_scoring[m_numScoring].tstamp = spline;

    m_numScoring++;
    }
}

void  LidarViewer::convertToXY()
	{
	int j = 0;

	/*for(int i=0; i < m_numLidarPts; i++)  {
		if(!m_lidarPts[i].dist)
			continue;

		double rad = M_PI * ((double)m_lidarPts[i].angle / 64.0) / 180;
 
		m_cartPts[j].x = -((((double)m_lidarPts[i].dist) * std::sin(rad)));
		m_cartPts[j].y = ((((double)m_lidarPts[i].dist) * std::cos(rad)));
		//printf(" %i,%i",lidatXY[j].x,lidatXY[j].y);
		j++;
	}*/

	m_numCartPts = j;
	}

