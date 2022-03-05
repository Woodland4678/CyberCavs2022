/**************************************************************************************************
 * Program: Woodland 4678 Path finding calculator 
 * Purpose: Calculate the path through waypoints and the required trajectory
 * Author: William Veldhuis
 * Version: 0
 * Date: 2019
 * Changes: Initial Release
**************************************************************************************************/

#ifndef PATHFINDER_H
#define PATHFINDER_H

//#include "frc/WPILib.h"

#define MAX_PATH_POINTS 32 // Define how many spline and circle segements we're allowed to string togehter.

#define MAX_TRAJ_POINTS 16 // Only need 16 (or less) since we're generating them dynamically as fast as we need them.

class PathFinder {
private:
    // pathData holds values related to the overall path algorithm
    typedef struct 
        {
        double startX;
        double startY;
        // double startAngle;

        double calculatedX; // Current X,Y and Angle as calculated by the path traversal
        double calculatedY;
        double calculatedAngle;

        double measuredX; // X,Y and Angle as measured (and tracked) based on encoder and gyro readings.
        double measuredY;
        double measuredAngle;

//        int state; // main processPath state counter.

        int pathPoints; // Number of splines in the current path.
        int pointsGenerated; // increases to match pathPoints as data is generated for that path.
        int activeIdx; // Index of spline or circle that we are generating traversal data for.
        int trajPointIdx; // index to where next trajectory point will be placed in the trajPoints circular array
        int trajPointToUse; // when running the trajectory, this is the trajectory point to use to set motor speeds.
        // Note, when trajPointIdx == trajPointToUse then array is empty and there's room for 16 of them.
        // trajPointidx is incremented when trajectory points are generated.  trajPointToUse is incremented as the points are
        // used and the data is send to the motor controller.  When trajPointIdx == trajPointToUse - 1, cyclic array is full and we need to wait
        // before adding more.
        int runRobot; // 0=not running, just creating path for display on cameraview, 1=running the robot.
        double acceleration;
        double deceleration;
        double distanceBetweenWheels;
        double cycleTime; // will be 0.02 for now.
        int havePrevTraj; // 0=no, 1=yes 
        int startedTraverse; // 0=not started yet, 1=started.
        double timeReference; // fpga timer reference.
        double vel; // manage velocity as we accelerate and decelerate (each segment)
        double nextVel; // velocity of next traversal section.
        double heading; // track the direction of this segment as we crank out trajectory points.
        double nextHeading;
        double arcPos; // track arc position 
        double pos; // tracks arc length position during traversal generation
        double nextPos; // next position value.
        double prevPos; // previous position value.
        double arcLength;
        double arcSegment;
        double lastArcLength;
        double last_integrand;
        double lastPosIncrement;

        double currentX; // Used in circle trajectory generation.
        double currentY;

        } pathData_t;

    typedef struct 
        {
        int segmentType; // 1=spline, 2=circle, 
        int segmentID;
        double x; // This is ending x,y
        double y;
        double theta; // Angle in radians at ending position.
        double targetVelocity;
        double finalVelocity;
        int useActual;   
        double theta_offset;
        double startX; // starting point gets added in here when spline or circle is first generated.
        double startY;
        double startTheta; // Angle at starting point.
        double linearDistance; // Straight Line distance start to end point.
        double theta_S_hat; // angle values for calculations.
        double theta_E_hat; // angle values for calculations.
        double m_S_Hat;
        double m_E_Hat;
        double a;
        double b;
        double c;
        double d;
        double e;
        int done; // gets set to 1 when we've generated all the traversal data for this one.
        int sampleCount; // counts from 0 to numberOfSamples
        int numberOfSamples; // we're done when sampleCount >= numberOfSamples
        int sampleNum;
        int prevSampleNum;
        double percentage;
        int goBackwards; // 0=forward, 1=backwards (velocity specified as negative)
        int turnDirection; // 0 = turn left, 1=turn right
        double thetaSpan; // the angle range we go through.
        double endTheta; // heading angle when this segement finishes.
		
		double centerX; // co-ords of the center of the circle.
		double centerY;
		double startAngle; // These are start and end angles for the circle that match the circleAround and splineTo angles,
		double endAngle; // not the theta angles which seem to be 180degrees off (due to using -X co-ords).  Next step for this code is to fix that ...

//        double currentX; // track current X and Y position info.
//        double currentY; // These never ended up getting used.
        } segmentData_t;
    
    segmentData_t segmentData[MAX_PATH_POINTS];

    typedef struct 
        {
        double pos;
        double vel;
        double acc;
        double heading;
        double x;
        double y;

        double velR;
        double velL;
        double curRX;
        double curRY;
        double curLX;
        double curLY;
        double posL;
        double posR;
        double accL;
        double accR;
        }trajPoint_t;

    trajPoint_t trajPoints[MAX_TRAJ_POINTS];

public:
    int m_WayPoint_Cnt = 0;

    pathData_t pathData;

    PathFinder(double acceleration, double deceleration, double distanceBetweenWheels,int runRobot);
    double angleDiffRadians(double from, double to);
    int generatePath2(int idx);
    int processPath(void);
    void setStartPoint(double x, double y, double angle);
    int splineTo(int segmentID, double x, double y, double angle, double targetVelocity, double finalVelocity, int useActual, int samples);
    int circleAround(int segmentID, double x, double y, double angle, double targetVelocity, double finalVelocity, int useActual, int samples);
    bool generateSpline2(int idx);
    double angleToTheta(double angle);
    bool generateCircle2(int idx);
    int generateCircPath2(int idx);
    double spline_derivativeAt2(int idx, double percentage);
    int trajectorySpace();
    void setNextVel2(int idx);
    bool spline_getXY2(int idx, double percentage, double *outX, double *outY);

    double gyroReading = 0;
    double degree = 0;

};

#endif //PATHFINDER_H