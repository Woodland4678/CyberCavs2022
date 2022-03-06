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
static const int LIDAR_CAMERA_WIDTH = 480; // 800; // 480; // 800 x 480 works!
static const int LIDAR_CAMERA_HEIGHT = 800; // 480 ; // 480; // 480 x 800 works just fine too!  Good for 2022 (Gets X and Y working correctly.)

int hublines[][2] = { // 24 points, 24 lines.
      {159,208},
      {141,165},
      {162,169},
      {180,160},
      {203,95},
      {196,76},
      {178,63},
      {220,46},
      {216,68},
      {225,86},
      {290,111},
      {310,102},
      {320,84},
      {339,126},
      {317,121},
      {299,131},
      {275,196},
      {282,215},
      {301,227},
      {259,245},
      {263,224},
      {252,205},
      {190,180},
      {169,188},
    };

int tarmac1[][2] = {
  {91,226},
{190,182},
{252,206},
{297,305},
{177,308},
};

int tarmac2[][2] = {
  {320,294},
{276,196},
{301,132},
{398,89},
{401,209},
};

int hangar[][2] = {
  {0,430},
{170,430},
{170,625},
{0,625},
};

int balls[][3] = { // 9 balls.  3rd int is 0:Blue, 1:Red.
{19,195,0},
{118,337,0},
{370,331,0},
{461,184,0},
{414,563,0},
{17,107,1},
{55,276,1},
{290,366,1},
{460,96,1},
};

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
  int p2x,p2y;
  
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
      // 800 x 480 works.
      // Draw the challenge field points

      // The following was for 2020 Infinite Recharge at home.      
//      int skip;
//      for (int xd = 66;xd < 775;xd+=66) // Draw all the locations where cones might go.
//        {
//        for (int yd = 80;yd < 475;yd+=80)
//          {
//          skip = 0; // Don't skip by default.  On line 1 and 5, skip position 2, 5 and 11
//          if (((yd == 80)||(yd == 80 * 5))&&((xd == 66*2)||(xd == 66 * 5)||(xd == 66 * 11)))
//            skip = 1;
//          if ((yd == 80 * 3)&&((xd != 66 * 3)&&(xd != 66 * 9))) // only include C3, C9 in center line.
//            skip  = 1;
//          if (!skip)
//            cv::circle(baseFrame, cv::Point(xd, yd), 3, orange, 1);
//          }
//        }

      // Here's some elements to get us a field that looks more like this year's field.
      for(int xd = 0;xd < 24;xd++)
        {
        if(xd < 23)
          {
          p2x = hublines[xd+1][0];
          p2y = hublines[xd+1][1];
          }
        else
          {
          p2x = hublines[0][0];
          p2y = hublines[0][1];
          }
        cv::line(baseFrame,cv::Point(hublines[xd][0],hublines[xd][1]),cv::Point(p2x,p2y),white,1,8,0);
//        printf("line(%d,%d) - (%d,%d)\n\r",hublines[xd][0],hublines[xd][1],p2x,p2y);
        }
      for(int xd = 0;xd < 5;xd++)
        {
        if(xd < 4)
          {
          p2x = tarmac1[xd+1][0];
          p2y = tarmac1[xd+1][1];
          }
        else
          {
          p2x = tarmac1[0][0];
          p2y = tarmac1[0][1];
          }
        cv::line(baseFrame,cv::Point(tarmac1[xd][0],tarmac1[xd][1]),cv::Point(p2x,p2y),red,5,8,0);
        if(xd < 4)
          {
          p2x = tarmac2[xd+1][0];
          p2y = tarmac2[xd+1][1];
          }
        else
          {
          p2x = tarmac2[0][0];
          p2y = tarmac2[0][1];
          }
        cv::line(baseFrame,cv::Point(tarmac2[xd][0],tarmac2[xd][1]),cv::Point(p2x,p2y),red,5,8,0);
        }
      // Hangar
      for(int xd = 0;xd < 4;xd++)
        {
        if(xd < 3)
          {
          p2x = hangar[xd+1][0];
          p2y = hangar[xd+1][1];
          }
        else
          {
          p2x = hangar[0][0];
          p2y = hangar[0][1];
          }
        cv::line(baseFrame,cv::Point(hangar[xd][0],hangar[xd][1]),cv::Point(p2x,p2y),orange,3,8,0);
        }
      
      // Balls
      for(int xd = 0;xd<9;xd++)
      {
        if (balls[xd][2] == 0)
          cv::circle(baseFrame, cv::Point(balls[xd][0],balls[xd][1]), 6, blue, 4);
          
        else
          cv::circle(baseFrame, cv::Point(balls[xd][0],balls[xd][1]), 6, red, 4);

      }

      // Div Line {479,1},{0,292},
      cv::line(baseFrame,cv::Point(479,40),cv::Point(0,252),white,3,8,0);

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
    m_scoring[m_numScoring].y = y; // LIDAR_CAMERA_HEIGHT - y;
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

