
#include "Subsystems/PathFinder/Path.h"
#include "Subsystems/LidarViewer.h"
#include "frc/commands/Command.h"
#include "frc/commands/Subsystem.h"
#include "Robot.h"

// for PC compile, comment out LidarViewer stuff and the pathdata.runRobot portion of processPath
// be sure to un-comment when putting back on the robot for testing.  Don't forget the velocity to 0 later in processPath
// un-comment out the pc version of main()

//#include "Path.h"
//#include "math.h"
//#include "stdio.h"
#include "Subsystems/LidarViewer.h"


PathFinder::PathFinder(double acceleration, double deceleration, double distanceBetweenWheels,int runRobot)
    { // if runRobot is 0, only generates graphics data for cameraview.  A 1 will send velocity data to motors to actually run the robot.
    pathData.acceleration = acceleration; // Set acceleration for the entire path.
    pathData.deceleration = deceleration; // and deceleration;
    pathData.distanceBetweenWheels = distanceBetweenWheels;
    pathData.runRobot = runRobot;
    if (!runRobot)
        LidarViewer::Get()->m_numScoring = 0; // init the lidar viewer.
    pathData.startedTraverse = 0;
    }

double PathFinder::angleDiffRadians(double from, double to){
    double angle = to - from;
    while (angle >= M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
}

bool PathFinder::spline_getXY2(int idx, double percentage, double *outX, double *outY) {
    percentage = std::fmax(std::fmin(percentage, 1.0), 0.0);
    double x_hat = segmentData[idx].linearDistance * percentage;

    double dist2 = x_hat * x_hat;
    double dist3 = x_hat * x_hat * x_hat;
    double dist4 = x_hat * x_hat * x_hat * x_hat;

    // The spline is quintic y = a.x^5 + b.x^4 + c.x^3 + d.x^2 + e.x.  We add in an x_offset and y_offset later.
    double y_hat = (segmentData[idx].a * x_hat + segmentData[idx].b) * dist4 + segmentData[idx].c * dist3 + segmentData[idx].d * dist2 + segmentData[idx].e * x_hat;
    
    double cos_theta = cos(segmentData[idx].theta_offset);
    double sin_theta = sin(segmentData[idx].theta_offset);

    double x = x_hat * cos_theta - y_hat * sin_theta + segmentData[idx].startX; // .currentX;
    double y = x_hat * sin_theta + y_hat * cos_theta + segmentData[idx].startY; // .currentY;
    //printf("\nidx = %d, Points=(%f,%f)",idx,x,y);

    *outX = x;
    *outY = y;
    return true;
}

double PathFinder::spline_derivativeAt2(int idx, double percentage) {
    percentage = std::fmax(std::fmin(percentage, 1.0), 0.0);
    double x_hat = segmentData[idx].linearDistance * percentage;
    double dist2 = x_hat * x_hat;
    double dist3 = x_hat * x_hat * x_hat;
    double yp_slope = (5 * segmentData[idx].a * x_hat + 4 * segmentData[idx].b)
        * dist3 + 3 * segmentData[idx].c * dist2 + 2 * segmentData[idx].d * x_hat + segmentData[idx].e;
    //(5 * a * dist + 4 * b) + dist * dist * dist + 3 * c * dist * dist + 2 * d * dist + e
    return yp_slope;
}

void PathFinder::setNextVel2(int idx)
    {
    // printf("\nvel=%f, targ=%f",m_Traj.segments[ti].vel,m_Splines[idx].targetVelocity);
    if (pathData.vel > segmentData[idx].targetVelocity) // Decelerate.  We're going faster than target.
        {
        //printf("\n Decelerate");
        pathData.nextVel = pathData.vel - pathData.deceleration * pathData.cycleTime;
        if (pathData.nextVel < segmentData[idx].targetVelocity)
            pathData.nextVel =  segmentData[idx].targetVelocity; // Limit velocity to target.
        }
    else if (pathData.vel < segmentData[idx].targetVelocity) // Accelerate, We're slower than target
        {
        // printf("\n Accelerate");
        pathData.nextVel = pathData.vel + pathData.acceleration * pathData.cycleTime;
        if (pathData.nextVel > segmentData[idx].targetVelocity)
            pathData.nextVel =  segmentData[idx].targetVelocity; // Limit velocity to target.
        }
    else // target speed reached, stay at this one for now.
        {
        //printf("\n Coast");
        pathData.nextVel = pathData.vel;
        }
    // Set next pos value using avg velocity.  This is position into entire traversal of the spline.
    pathData.lastPosIncrement =  (pathData.vel + pathData.nextVel) / 2.0 * pathData.cycleTime; 
    pathData.nextPos = pathData.lastPosIncrement + pathData.pos;
//    printf("\nsetNextVel2: vel=%f, nextVel=%f, pos=%f, nextPos=%f",pathData.vel,pathData.nextVel,pathData.pos,pathData.nextPos);
    }

int PathFinder::generateCircPath2(int idx)
    { // Starting with splitting Spline into 10,000 little segments for arc distance calculation

    // Traveling in a circle is a lot simpler than traversing a spline.  Ratio of left:right speeds is constant when doing a circle.
    // We can create a trajectory entry every time.  pathData will have existing velocity info and we can modify that.
    bool done;
    double curad,err,pcterr,theta,vel,velL,velR,ratio,radius,avgHeading,arcDist,linDist,multR,multL,startHeading;
    double prevVelL, prevVelR;
    double sinHeading,cosHeading;
    int previdx,iterCnt;

    done = false;
    iterCnt = 0;
    if (idx == 0)
        startHeading = (segmentData[0].startAngle - 180) * M_PI / 180; // pathData.startAngle * M_PI/180;
    else
        startHeading = segmentData[idx-1].endTheta;
    
    while((!done)&&(iterCnt < 15)&&(trajectorySpace() > 2))
        {
        // pathData.Calculated will have X,Y and heading (from any previous spline activity and we'll update it here.)
        setNextVel2(idx);  // Make sure next velocity has been set so we can produce the left and right velocities for this next 50Hz section.

        // Calculate how we're doing wrt being on the specified radius.  We can be inside the circle, on the circle or outside the circle.
        curad = sqrt((pathData.calculatedX - segmentData[idx].centerX)*(pathData.calculatedX - segmentData[idx].centerX) + (pathData.calculatedY - segmentData[idx].centerY)*(pathData.calculatedY - segmentData[idx].centerY));
//        printf("\nC(%f,%f) r=%f theta=%f (%f), Stheta=%f (%f) turn=%d a=%f, b=%f",segmentData[idx].x,segmentData[idx].y,segmentData[idx].linearDistance,segmentData[idx].theta,segmentData[idx].theta * 180/M_PI,segmentData[idx].startTheta,segmentData[idx].startTheta * 180/M_PI,
//                segmentData[idx].turnDirection,segmentData[idx].a,segmentData[idx].b);
        err = curad - segmentData[idx].linearDistance; // get the difference + = outside, - = inside.  Within certain limits, we can adjust the left / right ratio to correct.
        //    pcterr = err / segmentData[idx].linearDistance; // get a percentage error for the correction calculation we want to apply to velL and velR.
        pcterr = err / pathData.distanceBetweenWheels; // this is probably a better indicator as to how much we should change our left / right ratio
        // 0 will mean we're right on.  -1 would be inside by a whole wheel base.  +1 would be outside by a whole wheel base.  Let's say +/- 0.25 would be the maximum compensation
        // points (where we'd bring either velL or velR right down to 0 and the other at 2 * vel)
        pathData.heading = pathData.calculatedAngle * M_PI / 180; // get heading in radians from calculatedAngle.

        vel = (pathData.vel + pathData.nextVel) / 2; // Get avg velocity for this section.

        // adjustments to x and y position takes a bit of thinking.
        // When velL = velR, it's a straight forward x += vel * t * sin(heading), y += vel * t * cos(heading)
        // when velL != velR, the distance traveled is really a small arc, not a straight line and we want max accuracy so we need to convert the arc traversal to change in x,y
        // using ratio velL:velR = r+1/2b:r-1/2b we can eventually work out that r = b(velL + velR) / 2(velL - velR).  We can abs velL-velR if we want ot get r without sign problems
        // since arc dist is avg vel * t and Theta x r = avg vel * t = (velL + velR)/2 we can eventually get a simplified eqn: Theta = (velR - velL) * t / b
        // so we know the radius and the angle of the tiny arc.  This can get us correct calculations for change in x, change in y.  We just need to work out the
        // precise formulas.  We should use the angle that is 1/2 (start angle + this added angle) and velocity that is 1/2 (start + end), just in case we're in an acceleration / deceleration section.
        // this theta does match the change in heading so we can certainly use that here.

        // normally, when traveling around a circle, the ratio of left and right velocities is r - 1/2 b : r + 1/2 b.  r is radius of the circle, b is wheel base.
        // radius in this case was calculated as segmentData[idx].linearDistance.  b is pathdata.distanceBetweenWheels

        multR = segmentData[idx].a; // multR + multL will = 2.0
        multL = segmentData[idx].b;

        if (pcterr < -0.25) // this is max error for being inside the circle, set one of multR, multL to 0, the other to 2
            { // it may be necessary to do more here.  If we're headed for a tangent point but not there yet, drive straight.  If we passed a tangent point ... go backwards?
            if (segmentData[idx].turnDirection == 1) // turning right
                {
                multL = 0;
                multR = 2;
                }
            else
                {
                multL = 2;
                multR = 0;                    
                }
            }
        else if (pcterr > 0.25) // max error outside
            {
            if (segmentData[idx].turnDirection == 1) // turning right
                {
                multL = 2;
                multR = 0;
                }
            else
                {
                multL = 0;
                multR = 2;                    
                }
            }
        else // somewhere between on inside / outside error.  scale multL and multR by a factor
            { // let's simply add or subtract up to 0.5 to the larger multiplier over the 0.25 range. so 2 * pcterr
            if (segmentData[idx].turnDirection == 1) // turning right (multL will normally be the larger.  If pcterr is > 0, outside, make multL even larger)
                { // if inside the circle, make multL smaller
                multL += pcterr * 2.0;
                multR = 2 - multL;
                }
            else // turning left.  multR will normally be larger.
                {
                multR += pcterr * 2.0;
                multL = 2 - multR;
                }        
            }
        if (multL < 0.0) // limit multL and multR to the range 0.0 to 2.0
            multL = 0.0;
        if (multL > 2)
            multL = 2;
        if (multR < 0.0)
            multR = 0.0;
        if (multR > 2)
            multR = 2;
        
        velR = vel * multR; // segmentData[idx].a; // use previously calculated multipliers
        velL = vel * multL; // segmentData[idx].b;

        previdx = pathData.trajPointIdx - 1;
        if (previdx < 0)
            previdx = MAX_TRAJ_POINTS - 1; 

        // We've noticed that the acceleration value at start of circle can be rather excessive.  We may want to limit how much we can change velL and velR
        // from the previous trajectory entry to smooth this out a bit (not sure how badly we'll get out of the circle if we do this.)
        // if acceleration limit is something like 10m/s^2, in 0.02 seconds, max change in velocity should be v=at = 10 * 0.02 = 0.2 m/s
        #define MAX_ACCEL 10.0
        if (pathData.havePrevTraj)
            {
            prevVelL = trajPoints[previdx].velL;
//            prevVelR = trajPoints[previdx].velR; Only need to check one since the 2 need to add up to vel.

            if (prevVelL - velL > MAX_ACCEL * pathData.cycleTime)
                { // exceeding acceleration limit.  we need to reduce the difference.
                velL = prevVelL - MAX_ACCEL * pathData.cycleTime;
                velR = vel*2 - velL;
                }
            else if (prevVelL - velL < -MAX_ACCEL * pathData.cycleTime)
                {
                velL = prevVelL + MAX_ACCEL * pathData.cycleTime;
                velR = vel*2 - velL;
                }
            }

        // Once velL and velR have been adjusted, do calculations for where that brings us for the next point.    
        if (velL == velR) // If they're the same, do calculations a bit different to avoid div by 0 problems
            {
            avgHeading = pathData.heading; // Heading doesn't change when moving straight.
            theta = 0; // no change to heading.
            linDist = vel * pathData.cycleTime; // straight line distance from start to end of this short segment.
            }
        else // do circular curve calculations.
            {
            theta = (velL - velR) * pathData.cycleTime / pathData.distanceBetweenWheels;
            radius = pathData.distanceBetweenWheels * (velL + velR) / 2 / fabs(velL - velR); // this is the short arc radius, not the circle
            avgHeading = pathData.heading + theta/2; // Add half of theta to get the average heading.
            pathData.nextHeading = pathData.heading + theta;// * 0.8888888; // Set up for next heading.  27-Mar-2021 - fudge factor of 0.88888 to correct for under-rotation.  Needed to change calculatedAngle below instead.
            arcDist = vel * pathData.cycleTime;
            linDist = fabs(2 * radius * sin(theta/2)); // linear distance from start to end points of this segment (so we can adjust x,y).  Get fabs so len is always +

            }

        if (segmentData[idx].turnDirection == 1) // if turning right, we're done when heading > segmentData[].theta
            {
            if (avgHeading >= startHeading + segmentData[idx].thetaSpan)
                {
                done = true;
                printf("\nCirc Done Right: avgHeading=%f, startHeading=%f, thetaSpan=%f",avgHeading * 180/M_PI,startHeading * 180/M_PI,segmentData[idx].thetaSpan * 180/M_PI);
                }
            }
        else
            { // turning left, < theta.
            if (avgHeading <= startHeading - segmentData[idx].thetaSpan)
                {
                done = true;                
                printf("\nCirc Done Left: avgHeading=%f, startHeading=%f, thetaSpan=%f",avgHeading * 180/M_PI,startHeading * 180/M_PI,segmentData[idx].thetaSpan * 180/M_PI);
                }
            }
        

        // update calculated path info.
        sinHeading = sin(avgHeading);
        cosHeading = cos(avgHeading);
        pathData.calculatedX += linDist * cosHeading;
        pathData.calculatedY += linDist * sinHeading;
        pathData.calculatedAngle += ((theta*180/M_PI)*0.74); // new heading (in degrees) (with fudge factor 27-Mar-2021)

        trajPoints[pathData.trajPointIdx].vel = vel; // average velocity for this segment.
        trajPoints[pathData.trajPointIdx].heading = avgHeading;
        trajPoints[pathData.trajPointIdx].velL = velL;
        trajPoints[pathData.trajPointIdx].velR = velR;
        trajPoints[pathData.trajPointIdx].x = pathData.calculatedX;
        trajPoints[pathData.trajPointIdx].y = pathData.calculatedY;
        // The following are needed for smooth transition to spline
        trajPoints[pathData.trajPointIdx].curLX = trajPoints[pathData.trajPointIdx].x + pathData.distanceBetweenWheels / 2 * sinHeading;
        trajPoints[pathData.trajPointIdx].curLY = trajPoints[pathData.trajPointIdx].y - pathData.distanceBetweenWheels / 2 * cosHeading;
        trajPoints[pathData.trajPointIdx].curRX = trajPoints[pathData.trajPointIdx].x - pathData.distanceBetweenWheels / 2 * sinHeading;
        trajPoints[pathData.trajPointIdx].curRY = trajPoints[pathData.trajPointIdx].y + pathData.distanceBetweenWheels / 2 * cosHeading;
        //printf("\nXY(%f,%f) CURL (%f,%f), CURR(%f,%f)",trajPoints[pathData.trajPointIdx].x,trajPoints[pathData.trajPointIdx].y,trajPoints[pathData.trajPointIdx].curLX,trajPoints[pathData.trajPointIdx].curLY,trajPoints[pathData.trajPointIdx].curRX,trajPoints[pathData.trajPointIdx].curRY);


        if (!pathData.runRobot) // only send info to lidar viewer if we're not running the path.
            LidarViewer::Get()->addPointXY((int)(trajPoints[pathData.trajPointIdx].x * -87.489),(int)(trajPoints[pathData.trajPointIdx].y * 104.98687),1); // lidarviewer has an 800 x 480 display area (9.144m x 4.572m).  plot x,y accordingly.

        if (pathData.havePrevTraj)
            { // calculate accelerations so we can see the numbers.  Really high or low values would present an issue.
            trajPoints[pathData.trajPointIdx].accL = (trajPoints[pathData.trajPointIdx].velL - trajPoints[previdx].velL) / pathData.cycleTime;
            trajPoints[pathData.trajPointIdx].accR = (trajPoints[pathData.trajPointIdx].velR - trajPoints[previdx].velR) / pathData.cycleTime;
            }

        printf("\nCIRC: %d (%f,%f) %f Deg v=%f Lvel=%f Rvel=%f Lacc=%f Racc=%f, err=%f pcterr=%f linDist=%f",pathData.trajPointIdx,trajPoints[pathData.trajPointIdx].x,trajPoints[pathData.trajPointIdx].y,trajPoints[pathData.trajPointIdx].heading * 180 / M_PI,trajPoints[pathData.trajPointIdx].vel,
                trajPoints[pathData.trajPointIdx].velL,trajPoints[pathData.trajPointIdx].velR,trajPoints[pathData.trajPointIdx].accL,trajPoints[pathData.trajPointIdx].accR,err,pcterr,linDist,
                Robot::driveTrain->positionX,Robot::driveTrain->positionY,Robot::driveTrain->thetaHeading * 180/M_PI);

        pathData.trajPointIdx++; // increase point count.
        if (pathData.trajPointIdx >= MAX_TRAJ_POINTS)
            pathData.trajPointIdx = 0; // limit range to size of array.                
        pathData.havePrevTraj = 1;
        pathData.vel = pathData.nextVel;
		segmentData[idx].sampleCount++; // Only used so when iterating, we know if this is the first iteration (set up starting stuff needed - pathData.calculatedX)
        iterCnt++;
        }
    return(done);
    }


int PathFinder::generatePath2(int idx)
    { // Starting with splitting Spline into 10,000 little segments for arc distance calculation
    bool done;
    int iterCnt,j,previdx;
    double arcLength,lastArcLength,percentage;
    double t, dydt, integrand, last_integrand; //  = sqrt(1 + spline_derivativeAt(idx, 0) * spline_derivativeAt(idx, 0)) / samples;
    double dist,dist2;
    double decelerateTime,timeLeft;
    double decelerateDist,distLeft;                    

    done = false;
    iterCnt = 0; // Used to limit how many calculations we do at this time.
    last_integrand = pathData.last_integrand; // Resume from where we left off.
    j = segmentData[idx].sampleCount; // Resume from where we left off with j.
    arcLength = pathData.arcLength; 
    lastArcLength = pathData.lastArcLength;
    while((!done)&&(iterCnt < 500)&&(trajectorySpace() > 2)) // only do this for up to 500 times or till trajectory space is getting rather full.
        { // 
        t = ((double) j) / segmentData[idx].numberOfSamples;
        dydt = spline_derivativeAt2(idx, t);
//        printf("\nj=%d, arcLength=%f segPos/dist=%f dydt=%f, lastint=%f",j,arcLength,pathData.pos / segmentData[idx].linearDistance,dydt,last_integrand);
        integrand = sqrt(1 + dydt * dydt) / segmentData[idx].numberOfSamples;
        lastArcLength = arcLength; // may need this last one if we have exceeded the target distance into the Spline.
        arcLength += (integrand + last_integrand) / 2;
        last_integrand = integrand;
        pathData.arcSegment += (integrand + last_integrand) / 2; //  * segmentData[idx].linearDistance; // arcLength; // Total up arcLength for this CycleTime
        // If we've gotten far enough following the arc of the spline that we've reached a distance sufficient for adding another trajectory point, let's add it.
        if (arcLength >= pathData.pos / segmentData[idx].linearDistance) // Need to div by Spline Distance in order for this part to work correctly!
            {
            segmentData[idx].prevSampleNum = segmentData[idx].sampleNum; // keep track of last couple so we can get the difference.
            segmentData[idx].sampleNum = j;

            // Manage deceleration.  The system will accelerate at the start of a segment but we would like to have deceleration to a final velocity happen
            // more at the end of the segment.  To do this, we need to "estimate" the time left then figure out how much time is needed to complete the
            // deceleration and then start it at the right moment.
            // d = 1/2 a t^2.  to go from v1 (down) to v2 given acceleration a, v2 = v1 + at.  d = (v1 + v2) / 2 * t
            // we have values for v1, v2 and a.  Using v2 = v1 + at, t = (v2 - v1) / a and d = (v1 + v2) / 2 x t
            // so we can find t and d.  We will also have segmentData[idx].sampleNum and segmentData[idx].prevSampleNum.  Time between samples is 0.02 
            // so it's easy to convert time to a sample count.
            // We will want to do this calculation at each trajectory point generation due to the fact we may be accelerating and need to begin decelerate
            // even before we hit the target velocity.
            if (segmentData[idx].finalVelocity < segmentData[idx].targetVelocity) // if there is a need to decelerate 
                {
                // printf("\nHave Deceleration.  sampleNum=%d, prev=%d",segmentData[idx].sampleNum,segmentData[idx].prevSampleNum);
                if ((segmentData[idx].sampleNum > segmentData[idx].prevSampleNum)&&(segmentData[idx].sampleNum * 10 > segmentData[idx].numberOfSamples))
                    { // make sure we have at least a couple of samples, just from this segment.  if prevSampleNum is from last segment, it will be a larger number
                    // Also, make sure we're not in the first 10% of a segment.  There might not be enough reliable data to for a proper deceleration decistion to be made.
                    decelerateTime = (pathData.vel - segmentData[idx].finalVelocity) / pathData.deceleration; // time needed for deceleration in seconds
                    decelerateDist = (pathData.vel - segmentData[idx].finalVelocity) / 2 * decelerateTime;
                    // based on samples between 0.02sec trajectory point generation, predict approximately how much time is left for this segment.
                    // will be (number of samples - current sample) / (.sampleNum - .prevSampleNum) * pathData.cycleTime
                    timeLeft = (double)(segmentData[idx].numberOfSamples - segmentData[idx].sampleNum) / double(segmentData[idx].sampleNum - segmentData[idx].prevSampleNum) * pathData.cycleTime;
                    distLeft = timeLeft * pathData.vel; // distance at current velocity
                    printf("\ndecelTime=%f, timeLeft=%f, decelDist=%f, distLeft=%f",decelerateTime,timeLeft,decelerateDist,distLeft);
                    if (distLeft  <= decelerateDist * 1.17) // allow 17% extra distance.  We weren't slowing enough the other way.  25% is too much 10% not quite enough
                        segmentData[idx].targetVelocity = segmentData[idx].finalVelocity; // Looks like it's time to decelerate, let's do that.
                    }
                }
                

                //printf("\narcLength=%f, pos/dist=%f",arcLength,m_Traj.segments[ti].pos/m_Splines[idx].distance);
            setNextVel2(idx);  // Make sure next has been set as well as position (distance into the Spline arc)
            // Using avg velocity, determine position into the spline arc.  This is where we will match arcLength
            // now part of setNextVel m_Traj.segments[ti].pos = (m_Traj.segments[ti-1].vel + m_Traj.segments[ti].vel) / 2.0 * m_Config.cycleTime + m_Traj.segments[ti-1].pos;                
            trajPoints[pathData.trajPointIdx].pos = pathData.pos; // get traj pos from segmentData pos
            trajPoints[pathData.trajPointIdx].vel = pathData.vel; // and velocity from segmentData vel

            if (arcLength != lastArcLength)
                {
                percentage = t + ((pathData.pos - lastArcLength)/(arcLength - lastArcLength - 1.0))/((double)segmentData[idx].numberOfSamples);
//                printf("\nt=%f,add=%f,tot=%f",t,((m_Traj.segments[ti].pos - lastArcLength)/(arcLength - lastArcLength - 1.0))/((double)m_Splines[idx].samples),percentage);
                }
            else
                {
                percentage = t;
                }
            segmentData[idx].percentage = percentage;
            spline_getXY2(idx,percentage,&trajPoints[pathData.trajPointIdx].x, &trajPoints[pathData.trajPointIdx].y); // Get true X,Y at position x_hat on this Spline.
//            printf("\ngetXY(%d,%f,%f,%f)",idx,percentage,m_Traj.segments[ti].x,m_Traj.segments[ti].y);

            pathData.calculatedX = trajPoints[pathData.trajPointIdx].x;
            pathData.calculatedY = trajPoints[pathData.trajPointIdx].y;

            trajPoints[pathData.trajPointIdx].velR = 0;
            trajPoints[pathData.trajPointIdx].velL = 0;

            trajPoints[pathData.trajPointIdx].heading = atan(dydt) + segmentData[idx].theta_offset; 
            pathData.calculatedAngle = trajPoints[pathData.trajPointIdx].heading * 180 / M_PI;

            // traverse only uses left and right velocity values and heading (for gyro correction).  We don't need to set any other values really.
            // We do need some for calculation of distance travelled for each size of the robot.
            // copySegment(ti);
            double cos_angle = cos(trajPoints[pathData.trajPointIdx].heading);
            double sin_angle = sin(trajPoints[pathData.trajPointIdx].heading);

            //printf("\nheading=%f theta_offset=%f, cos=%f, sin=%f",m_Traj.segments[ti].heading,m_Splines[idx].theta_offset,cos_angle,sin_angle);
            if (!pathData.runRobot) // only send info to lidar viewer if we're not running the path.
                LidarViewer::Get()->addPointXY((int)(trajPoints[pathData.trajPointIdx].x * -87.489),(int)(trajPoints[pathData.trajPointIdx].y * 104.98687),1); // lidarviewer has an 800 x 480 display area (9.144m x 4.572m).  plot x,y accordingly.
            // printf("\nAdded (%d,%d)",(int)(m_Traj.segments[ti].x * -87.489),(int)(m_Traj.segments[ti].y * 104.98687));

            trajPoints[pathData.trajPointIdx].curLX = trajPoints[pathData.trajPointIdx].x + pathData.distanceBetweenWheels / 2 * sin_angle;
            trajPoints[pathData.trajPointIdx].curLY = trajPoints[pathData.trajPointIdx].y - pathData.distanceBetweenWheels / 2 * cos_angle;
            
            trajPoints[pathData.trajPointIdx].curRX = trajPoints[pathData.trajPointIdx].x - pathData.distanceBetweenWheels / 2 * sin_angle;
            trajPoints[pathData.trajPointIdx].curRY = trajPoints[pathData.trajPointIdx].y + pathData.distanceBetweenWheels / 2 * cos_angle;

//            printf("\nXY(%f,%f) CURL (%f,%f), CURR(%f,%f)",trajPoints[pathData.trajPointIdx].x,trajPoints[pathData.trajPointIdx].y,trajPoints[pathData.trajPointIdx].curLX,trajPoints[pathData.trajPointIdx].curLY,trajPoints[pathData.trajPointIdx].curRX,trajPoints[pathData.trajPointIdx].curRY);

            if (pathData.havePrevTraj)
                {
                previdx = pathData.trajPointIdx - 1;
                if (previdx < 0)
                    previdx = MAX_TRAJ_POINTS - 1; 
                dist = sqrt(((trajPoints[pathData.trajPointIdx].curLX - trajPoints[previdx].curLX)
                    *(trajPoints[pathData.trajPointIdx].curLX - trajPoints[previdx].curLX))
                    +((trajPoints[pathData.trajPointIdx].curLY - trajPoints[previdx].curLY)
                    *(trajPoints[pathData.trajPointIdx].curLY - trajPoints[previdx].curLY)));
//                printf("\ndist=%f",dist);
                trajPoints[pathData.trajPointIdx].posL = trajPoints[previdx].posL + dist;
                if (segmentData[idx].goBackwards)
                    trajPoints[pathData.trajPointIdx].velR = -dist / pathData.cycleTime; // need to swap left and right when going backwards
                else
                    trajPoints[pathData.trajPointIdx].velL = dist / pathData.cycleTime;
                trajPoints[pathData.trajPointIdx].accL = (trajPoints[pathData.trajPointIdx].velL - trajPoints[previdx].velL) / pathData.cycleTime;


                dist2 = sqrt(((trajPoints[pathData.trajPointIdx].curRX - trajPoints[previdx].curRX)
                    *(trajPoints[pathData.trajPointIdx].curRX - trajPoints[previdx].curRX))
                    +((trajPoints[pathData.trajPointIdx].curRY - trajPoints[previdx].curRY)
                    *(trajPoints[pathData.trajPointIdx].curRY - trajPoints[previdx].curRY)));
                trajPoints[pathData.trajPointIdx].posR = trajPoints[previdx].posR + dist2;
                if (segmentData[idx].goBackwards)
                    trajPoints[pathData.trajPointIdx].velL = -dist2 / pathData.cycleTime; // need to swap left and right when going backwards.
                else
                    trajPoints[pathData.trajPointIdx].velR = dist2 / pathData.cycleTime;
                trajPoints[pathData.trajPointIdx].accR = (trajPoints[pathData.trajPointIdx].velR - trajPoints[previdx].velR) / pathData.cycleTime;

                if ((dist > 0.5)||(dist2 > 0.5))
                    {
                    printf("\nLarge Dist %f, %f from co-ords (%f,%f) - (%f,%f), (%f,%f) - (%f-%f)",dist,dist2,trajPoints[previdx].curLX,trajPoints[previdx].curLY,trajPoints[pathData.trajPointIdx].curLX,trajPoints[pathData.trajPointIdx].curLY,
                        trajPoints[previdx].x,trajPoints[previdx].y,trajPoints[pathData.trajPointIdx].x,trajPoints[pathData.trajPointIdx].y);
                    }

                }
            else
                {
                previdx = 0;
                dist = 0;
                dist2 = 0;                        
                }
            
            pathData.havePrevTraj = 1;
    printf("\n%d/%d (%f,%f) %f%% %f Deg pos=%f v=%f Lpos=%f Rpos=%f Lvel=%f Rvel=%f Lacc=%f Racc=%f d1=%f d2=%f XY(%f,%f) %fDeg",pathData.trajPointIdx,j,trajPoints[pathData.trajPointIdx].x,trajPoints[pathData.trajPointIdx].y,percentage,trajPoints[pathData.trajPointIdx].heading * 180 / M_PI,trajPoints[pathData.trajPointIdx].pos,trajPoints[pathData.trajPointIdx].vel,
            trajPoints[pathData.trajPointIdx].posL,trajPoints[pathData.trajPointIdx].posR,trajPoints[pathData.trajPointIdx].velL,trajPoints[pathData.trajPointIdx].velR,trajPoints[pathData.trajPointIdx].accL,trajPoints[pathData.trajPointIdx].accR,
            dist,dist2,
            Robot::driveTrain->positionX,Robot::driveTrain->positionY,Robot::driveTrain->thetaHeading * 180/M_PI);
//    printf("\n     curL(%f,%f) - prvL(%f,%f) d=%f, curR(%f,%f) - prvR(%f,%f) d=%f",trajPoints[pathData.trajPointIdx].curLX,trajPoints[pathData.trajPointIdx].curLY,
//                    trajPoints[previdx].curLX,trajPoints[previdx].curLY,dist,
//                    trajPoints[pathData.trajPointIdx].curRX,trajPoints[pathData.trajPointIdx].curRY,
//                    trajPoints[previdx].curRX,trajPoints[previdx].curRY,dist2);
            pathData.trajPointIdx++;
            if (pathData.trajPointIdx >= MAX_TRAJ_POINTS)
                pathData.trajPointIdx = 0; // limit range to size of array.                
            pathData.arcPos += pathData.arcSegment;
            pathData.arcSegment = 0; // Clear for the next piece of the trajectory
            pathData.prevPos = pathData.pos;
            pathData.pos = pathData.nextPos; // copy over next position and next velocity for next iteration.
            pathData.vel = pathData.nextVel;

            }
        iterCnt++; // Incrementer iteration count
        j++; // j increments
        if (j >= segmentData[idx].numberOfSamples)
            {
            segmentData[idx].endTheta = trajPoints[pathData.trajPointIdx].heading;
            done = true; // Generation is now complete.
//            printf("\nOn done, arcLength=%f, pos/dist=%f diff=%f, lastIncrement=%f",arcLength,m_Traj.segments[ti].pos/m_Splines[idx].distance,arcLength - m_Traj.segments[ti].pos/m_Splines[idx].distance,m_Splines[idx].lastIncrement);
            }
        }
    pathData.last_integrand = last_integrand;
    pathData.lastArcLength = lastArcLength;
    pathData.arcLength = arcLength;
    segmentData[idx].sampleCount = j;

    return(done);
    }

double PathFinder::angleToTheta(double angle) // convert angle to radians and limit to +/-2pi range.
    {
    while(angle <=-360.0)
        angle += 360.0;
    while(angle >= 360.0)
        angle -= 360.0;
    return(angle * M_PI / 180.0); // return result in radians.
    }

bool PathFinder::generateSpline2(int idx) // was idx, way1, way2.  Now, we just use start point from pathData.  idx is index into segmentData
    { // way1 will be startX, startY.  way 2 will be segmentData[idx].x, y
    //Setup offsets.  

// For pass 1 math stuff, segmentData[idx].startX, startY and startTheta should all have been set up already as well as x,y, theta end angles.
//    if ((segmentData[idx].useActual)&&(pathData.runRobot)) // use Actual, but only if this is a real run.
//        {
//        segmentData[idx].startX = pathData.measuredX; // Note, we need to wait till robot has nearly finished previous trajectory before this data is available.
//        segmentData[idx].startY = pathData.measuredY;
//        segmentData[idx].startTheta = angleToTheta(pathData.measuredAngle); // Convert to radians and make sure it's in the +/-2pi range. * M_PI / 180.0;
//        }
//    else // use calculated.  If idx is 0, these would already have been set by startPoint()
//        {
//        segmentData[idx].startX = pathData.calculatedX;
//        segmentData[idx].startY = pathData.calculatedY;
//        segmentData[idx].startTheta = angleToTheta(pathData.calculatedAngle); //  * M_PI/180.0;
//        }

    segmentData[idx].startTheta = segmentData[idx].startAngle * M_PI / 180;
    
    //Calculate length (straight line distance)
    segmentData[idx].linearDistance = sqrt((segmentData[idx].x - segmentData[idx].startX)*(segmentData[idx].x - segmentData[idx].startX) + (segmentData[idx].y - segmentData[idx].startY)*(segmentData[idx].y - segmentData[idx].startY));
//    printf("\n    Dist=%f",segmentData[idx].linearDistance);
    if(segmentData[idx].linearDistance == 0)
        {
        printf("\nSpline Generation failed.  Segment has 0 length index=%d, SegmentID=%d",idx,segmentData[idx].segmentID);
        return false;
        }

    // set up angles
    segmentData[idx].theta_offset = atan2(segmentData[idx].y - segmentData[idx].startY, segmentData[idx].x - segmentData[idx].startX); //  m_WayPoints[way2].y - m_WayPoints[way1].y, m_WayPoints[way2].x - m_WayPoints[way1].x);
    segmentData[idx].theta_S_hat = angleDiffRadians(segmentData[idx].theta_offset, segmentData[idx].startTheta); // m_Splines[idx].theta_offset, m_WayPoints[way1].theta);
    segmentData[idx].theta_E_hat = angleDiffRadians(segmentData[idx].theta_offset, segmentData[idx].theta); 
//    m_Splines[idx].theta_S_hat =  angleDiffRadians(m_Splines[idx].theta_offset, m_WayPoints[way1].theta);
//    m_Splines[idx].theta_E_hat =  angleDiffRadians(m_Splines[idx].theta_offset, m_WayPoints[way2].theta);
    //printf("\n    theta Offset=%f | theta S offset=%f theta E offset=%f| ",m_Splines[idx].theta_offset,m_Splines[idx].theta_S_hat,m_Splines[idx].theta_E_hat);
    if((fabs(segmentData[idx].theta_S_hat - (M_PI / 2)) < 0.0001)||(fabs(segmentData[idx].theta_E_hat - (M_PI / 2)) < 0.0001))
        {
        printf("\nAngle out of range error (90 Deg) on index=%d, SegmentID=%d",idx,segmentData[idx].segmentID);
        return false;
        }
    double dumbVariable = angleDiffRadians(segmentData[idx].theta_S_hat, segmentData[idx].theta_E_hat);
    double dumbVariable2 = (M_PI / 2);
    if(dumbVariable >= dumbVariable2)
        {
        printf("\nAngle between start and end is 90 Deg index=%d, SegmentID=%d Start=%f (%f), End=%f (%f)",idx,segmentData[idx].segmentID,
                segmentData[idx].startTheta,segmentData[idx].startTheta * 180 / M_PI,
                segmentData[idx].theta,segmentData[idx].theta * 180 / M_PI);
        return false;
        }

    //Slopes
    segmentData[idx].m_S_Hat = tan(segmentData[idx].theta_S_hat);
    segmentData[idx].m_E_Hat = tan(segmentData[idx].theta_E_hat);

    //Cofficients
    segmentData[idx].a = -(3*(segmentData[idx].m_S_Hat + segmentData[idx].m_E_Hat))/(segmentData[idx].linearDistance * segmentData[idx].linearDistance * segmentData[idx].linearDistance * segmentData[idx].linearDistance);
    segmentData[idx].b = (8*segmentData[idx].m_S_Hat + 7 * segmentData[idx].m_E_Hat)/(segmentData[idx].linearDistance * segmentData[idx].linearDistance * segmentData[idx].linearDistance);
    segmentData[idx].c = -(6*segmentData[idx].m_S_Hat + 4*segmentData[idx].m_E_Hat) / (segmentData[idx].linearDistance * segmentData[idx].linearDistance);
    segmentData[idx].d = 0;
    segmentData[idx].e = segmentData[idx].m_S_Hat;

    printf("\n Spline: %d ID %d   %f | %f | %f | %f | %f | linDist=%f (%f,%f) %f Deg - (%f,%f) %f Deg",idx,segmentData[idx].segmentID,segmentData[idx].a,segmentData[idx].b,segmentData[idx].c,segmentData[idx].d,segmentData[idx].e,segmentData[idx].linearDistance,
        segmentData[idx].startX,segmentData[idx].startY,segmentData[idx].startTheta * 180/ M_PI,segmentData[idx].x,segmentData[idx].y,segmentData[idx].theta * 180 / M_PI);

    return true;
}

bool PathFinder::generateCircle2(int idx) // set up math for the circular path
    { // This is an add-on.  How to make things work?  We have current point startX, startY.  This point is on the circle.
	// using .x and .y as the center point is problematic for transition from spline to cicle and back again.  x, y, theta are always going to be the end points.
	// startX, startY and startTheta are the starting point.  We're adding centerX, centerY for center of the circular path.
    // We have the center of the circle as specified by the point.  segmentData[idx].x,y.  linear distance between the 2 is radius.  This gets stored in linearDistance.
    // The angle we want to draw the circle through is specified by theta.  In this case, angle can be more than +/-2pi
    // if angle is > startAngle we'll turn right.  If it's <, turn left, going around the specified point as the center.
    // theta_offset will match the angle used for splines.  It will be the direction the robot is pointing in at the start of the path.
    // angle of 0 (robot front is up) if turning left (angle increases as we move)
    // angle of 180 (robot front is up) if turning right (angle decreases as we move)
    // position information will be relatively straight forward.  SIN and COS * radius of an angle that we increment by a very small amount each time.
    // 
    double aplusb;
    double endTheta;
    double ctrToRobot;
    double endTheta2;

// segmentData[idx].startX, startY, startTheta should all have been set up already.  No need to reference pathData.  It may not be correct at this point.
//    if ((segmentData[idx].useActual)&&(pathData.runRobot)) // use Actual, but only if this is a real run.
//        {
//        segmentData[idx].startX = pathData.measuredX; // Note, we need to wait till robot has nearly finished previous trajectory before this data is available.
//        segmentData[idx].startY = pathData.measuredY;
//        segmentData[idx].startTheta = angleToTheta(pathData.measuredAngle); // Convert to radians and make sure it's in the +/-2pi range. * M_PI / 180.0;
//        }
//    else // use calculated.
//        {
//        segmentData[idx].startX = pathData.calculatedX;
//        segmentData[idx].startY = pathData.calculatedY;
//        segmentData[idx].startTheta = angleToTheta(pathData.calculatedAngle); //  * M_PI/180.0;
//        }
    
    // linearDistance will be the radius.  We certainly need that.  Let's just crank out trajectory points using velocity and distance traveled to go around the circle.
    // heading data will need to basically match what we see in the spline rotate.  Won't be a whole lot different than the spline code, really.
    segmentData[idx].linearDistance = sqrt((segmentData[idx].centerX - segmentData[idx].startX)*(segmentData[idx].centerX - segmentData[idx].startX) + (segmentData[idx].centerY - segmentData[idx].startY)*(segmentData[idx].centerY - segmentData[idx].startY));

    // Approach for circle can be pretty direct (only need to do calcs at 50Hz)
    // given circle of radius r, wheel base of b ...
    // For a right turn, ratio of L:R is r + 1/2b : r - 1/2b
    // For a left turn,  ratio of L:R is r - 1/2b : r + 1/2b
    // This gives us reference velocity ratios.  
    // Monitor position and if we're outside the circle, increase the ratio difference.
    // If we're inside the circle, decrease the ratio difference.
    // use gyro and / or calculated heading to track our x,y position.  
    // when heading has reached the target, we are done.  We may want to do a bit better than this for progress so we can properly handle a deceleration on the circle.
    // 
    // if (segmentData[idx].theta > segmentData[idx].startTheta)
	if (segmentData[idx].endAngle > segmentData[idx].startAngle) // Now using angle specs, not theta.  Theta will be heading.
        { // right turn.  Calculate multipliers, put right in a, left in b
        segmentData[idx].a = segmentData[idx].linearDistance - 0.5*pathData.distanceBetweenWheels;
        segmentData[idx].b = segmentData[idx].linearDistance + 0.5*pathData.distanceBetweenWheels;
        segmentData[idx].turnDirection = 1; // Indicate we're turning right
        }
    else
        { // left turn.  Calculate ratios, put right in a, left in b
        segmentData[idx].a = segmentData[idx].linearDistance + 0.5*pathData.distanceBetweenWheels;
        segmentData[idx].b = segmentData[idx].linearDistance - 0.5*pathData.distanceBetweenWheels;                
        segmentData[idx].turnDirection = 0; // Indicate we're turning left
        }

    aplusb = segmentData[idx].a + segmentData[idx].b;
    segmentData[idx].a = 2 * segmentData[idx].a / aplusb; // calculate the actual multipliers
    segmentData[idx].b = 2 * segmentData[idx].b / aplusb; // for how to split vel into velR (*a) and velL (*b)

//    segmentData[idx].currentX = segmentData[idx].startX; // get starting X,Y into currentX, currentY (not sure if this gets used)
//    segmentData[idx].currentY = segmentData[idx].startY; // No, turns out we were only every placing values into these variables and never using them.
        
    // In order to keep generating Spline and Circle setup stuff, we need to have the final x,y and theta.  For
    // the splines, this was easy but for circles, it's a bit trickier.  Angle isn't too hard but the final X,Y
    // is a bit interesting.

    // segmentData[idx].centerX,centerY are the center.  We'll end up at
    // .x + r cos(theta2), .y + r sin(theta2).  Where theta2 is our ending angle around the circle.  We need to consider end angle and turn direction to get this right.
    // When turning right, the angle of the line from the center of the circle to the robot location is heading(in deg) + 90 deg
    // When turning left, the angle of the line from the center of the circle to the robot location is heading(in deg) - 90 deg.
    // We're not really dealing with robot headings in this part though.  That comes later in the traj generation code.
    // At this point, the radius from the center co-ords will be .endAngle +/- 90 deg.
    //
    // Not working really well for the second circle. We may need to calculate the angle of the vector from center to robot start
    // position.  We can't just use the start Angle.  We can be off by 180 deg if center is above robot vs below.
    // The actual starting angle would be an ATAN2(center to robot), I think.
    ctrToRobot = atan2(segmentData[idx].startY - segmentData[idx].centerY,segmentData[idx].startX - segmentData[idx].centerX);
    endTheta2 = ctrToRobot - segmentData[idx].thetaSpan; // rotate by the span to get new direction of vector from center to robot.
    printf("\nRob(%f,%f) - Ctr(%f,%f) atan2(%f,%f) = %f (%f). theta2=%f (%f) -> (%f,%f)",segmentData[idx].startX,segmentData[idx].startY,segmentData[idx].centerX,segmentData[idx].centerY,
        segmentData[idx].startY - segmentData[idx].centerY,segmentData[idx].startX - segmentData[idx].centerX,ctrToRobot,ctrToRobot*180/M_PI,
        endTheta2,endTheta2 * 180 / M_PI,
        segmentData[idx].centerX + segmentData[idx].linearDistance * cos(endTheta2),
        segmentData[idx].centerY + segmentData[idx].linearDistance * sin(endTheta2)
        );

    // Probably don't need this part.        
    if (segmentData[idx].turnDirection)
        {
        endTheta = (segmentData[idx].endAngle + 90) * M_PI / 180;// segmentData[idx].theta + M_PI/2; // +90 for right turn
        }
    else
        {
        endTheta = (segmentData[idx].endAngle - 90) * M_PI / 180;//segmentData[idx].theta - M_PI/2; // -90 for right turn
        }
    
//    segmentData[idx].x = segmentData[idx].centerX + segmentData[idx].linearDistance * cos(endTheta);
//    segmentData[idx].y = segmentData[idx].centerY + segmentData[idx].linearDistance * sin(endTheta);
    segmentData[idx].x = segmentData[idx].centerX + segmentData[idx].linearDistance * cos(endTheta2);
    segmentData[idx].y = segmentData[idx].centerY + segmentData[idx].linearDistance * sin(endTheta2);
    printf("\nCircle (%f,%f) %fRad %fDeg Center(%f,%f)  End (%f,%f) %fRad %fDeg",segmentData[idx].startX,segmentData[idx].startY,segmentData[idx].startTheta,segmentData[idx].startTheta*180/M_PI,
			segmentData[idx].centerX,segmentData[idx].centerY,
            segmentData[idx].x,segmentData[idx].y,endTheta,endTheta*180/M_PI);
        

	if(segmentData[idx].linearDistance == 0)
        {
        printf("\nCircle Generation failed.  Radius has 0 length index=%d, SegmentID=%d",idx,segmentData[idx].segmentID);
        return false;
        }
    printf("\nCircGen: robot(%f,%f) center(%f,%f) radius=%f a=%f b(rad)=%f c(tan)=%f",segmentData[idx].startX,segmentData[idx].startY,segmentData[idx].x,segmentData[idx].y,segmentData[idx].linearDistance,segmentData[idx].a,segmentData[idx].b,segmentData[idx].c);

    // ***************** need setup stuff here to allow us to traverse the portion of a circle.  set up numbers to make this work *******************

    // Figure out starting and ending angle and direction.  We'll do a few 1000 points using incrementing or decrementing angle
    // get sin and cos of said angle, add to center point (segmentData[idx].x, segmentData[idx].y) and we have our new x,y position.
    // use the starting angle, add the angle used to get sin and cos, maybe need to get the normal.  This becomes new heading for each point.
    // if distance to this new point meets the current velocity requirements, we have a new point to add to the trajectory.  Add it.

    return true;
    }

void PathFinder::setStartPoint(double x, double y, double angle) // establish the start point for the robot.
    { // Angle at this point is degrees and there's no limit as to what the start angle is (can be <-360 or >+360)
    // Since this is a true starting point, intialize the calculated and measured values as well.
    pathData.startX = x;
    pathData.startY = y;
//    pathData.startAngle = angle;

    pathData.calculatedX = x; // probably a good idea to init this stuff for when traj points get generated.
    pathData.calculatedY = y; // we should not be using these for math pass 1 stuff.  Everything should come from segmentData[].
    pathData.calculatedAngle = angle;

    pathData.measuredX = x;
    pathData.measuredY = y;
    pathData.measuredAngle = angle;

    pathData.trajPointIdx = 0; // reset the trajectory pointers so there's no points in the queue
    pathData.havePrevTraj = 0; // indicate we don't have any prior trajectory points yet.
    pathData.trajPointToUse = 0;
    pathData.pathPoints = 0; // No paths to start with.
    pathData.activeIdx = 0; // Starting at path 0 when creating trajectory points.
    pathData.pointsGenerated = 0; // And no math done to generate the path data (spline or circle)

    segmentData[0].startX = x; // need proper start position for spline and circle path generation
    segmentData[0].startY = y;
//    segmentData[0].theta = angle * M_PI/180; // This is end angle, not start.  Probably don't need this!
	segmentData[0].startAngle = angle; // In case first segment is a circle.
	segmentData[0].startTheta = angleToTheta(angle);

    pathData.cycleTime = 0.02; // 50 Hz for now
    }

int PathFinder::splineTo(int segmentID, double x, double y, double angle, double targetVelocity, double finalVelocity, int useActual, int samples)
    {
    if (pathData.pathPoints < MAX_PATH_POINTS)
        {
        segmentData[pathData.pathPoints].segmentType = 1; // This is a spline.
        segmentData[pathData.pathPoints].segmentID = segmentID;
        segmentData[pathData.pathPoints].x = x;
        segmentData[pathData.pathPoints].y = y;
        segmentData[pathData.pathPoints].theta = angleToTheta(angle); //  * M_PI / 180; // angle will always be degrees.  theta will always be radians
        segmentData[pathData.pathPoints].segmentID = segmentID;
        if (targetVelocity < 0) // Backwards
            segmentData[pathData.pathPoints].goBackwards = 1;
        else
            segmentData[pathData.pathPoints].goBackwards = 0;
        segmentData[pathData.pathPoints].targetVelocity = fabs(targetVelocity); // always deal with + velocities.
        segmentData[pathData.pathPoints].finalVelocity = fabs(finalVelocity);
        segmentData[pathData.pathPoints].useActual = useActual;
        segmentData[pathData.pathPoints].done = 0; // Just made this one.  It has not yet been processed for traversal points.
        segmentData[pathData.pathPoints].numberOfSamples = samples;
        segmentData[pathData.pathPoints].sampleCount = 0; // start at 0 and count to numberofSamples
		segmentData[pathData.pathPoints].endAngle = angle; // Circle may need end angle so we're going to save it.

        pathData.pathPoints++;
        return true;
        }
    else
        {
        // too many paths.
        printf("\nExceeded Maximum Number of path segments=%d",MAX_PATH_POINTS);
        return false;
        }
    }

int PathFinder::trajectorySpace()  // returns number of spots available in the circular trajectory information array.
    { // should be room for 16 but just to be safe, we'll set this up for a maximum of 15.
    if (pathData.trajPointIdx > pathData.trajPointToUse)
        { // normal.  points being added come after point to be used.  Space is 15 - the difference.
        return(15 - (pathData.trajPointIdx - pathData.trajPointToUse));
        }
    else if (pathData.trajPointIdx < pathData.trajPointToUse)
        { // case where we have wrap around.  Space is simply the difference.  Eg. Use=15, Point=0 -> space for 14.
        return(pathData.trajPointToUse - pathData.trajPointIdx - 1);
        }
    else
        {
        return 15; // When equal, circular array is empty, we have room for 15.                
        }    
    }

int PathFinder::circleAround(int segmentID, double x, double y, double angle, double targetVelocity, double finalVelocity, int useActual, int samples)
    {
    if (pathData.pathPoints  < MAX_PATH_POINTS)
        {
        segmentData[pathData.pathPoints].segmentType = 2; // This is a circular path.
        segmentData[pathData.pathPoints].segmentID = segmentID;
        segmentData[pathData.pathPoints].centerX = x;
        segmentData[pathData.pathPoints].centerY = y;
		segmentData[pathData.pathPoints].endAngle = angle; // Actual ending angle now gets put in endAngle.
        segmentData[pathData.pathPoints].theta = angleToTheta(angle - 180); //  * M_PI / 180; // angle will always be degrees.  theta will always be radians and it will be heading that matches spline headings.
//        if (pathData.pathPoints != 0)
//            segmentData[pathData.pathPoints].thetaSpan = segmentData[pathData.pathPoints].theta - segmentData[pathData.pathPoints - 1].theta;
//        else
//            segmentData[pathData.pathPoints].thetaSpan = segmentData[pathData.pathPoints].theta - segmentData[pathData.pathPoints].startAngle * M_PI / 180;
        segmentData[pathData.pathPoints].thetaSpan = fabs((segmentData[pathData.pathPoints].endAngle -  segmentData[pathData.pathPoints].startAngle) * M_PI / 180);
        
        segmentData[pathData.pathPoints].segmentID = segmentID;
        if (targetVelocity < 0) // Backwards
            segmentData[pathData.pathPoints].goBackwards = 1;
        else
            segmentData[pathData.pathPoints].goBackwards = 0;
        segmentData[pathData.pathPoints].targetVelocity = fabs(targetVelocity);
        segmentData[pathData.pathPoints].finalVelocity = fabs(finalVelocity);
        segmentData[pathData.pathPoints].useActual = useActual;
        segmentData[pathData.pathPoints].done = 0; // Just made this one.  It has not yet been processed for traversal points.
        segmentData[pathData.pathPoints].numberOfSamples = samples;
        segmentData[pathData.pathPoints].sampleCount = 0; // start at 0 and count to numberofSamples

        pathData.pathPoints++;

        return true;
        }
    else
        {
        // too many circles.
        printf("\nExceeded Maximum Number of Circles =%d",MAX_PATH_POINTS);
        return false;
        }
    }


int PathFinder::processPath(void) // This needs to be called at 50Hz any time path functionality of any kind is needed
    {
    int idx,done,idxstep;
    double timediff,sref;

    done = false; // Assume we're not done.
    if (pathData.runRobot)
        {
        // follow trajectory to run the robot (as long as we have points to follow)
        if (pathData.startedTraverse == 0) // If traverse has not yet started,  
            { // and we have at least a few points to start with, start the traversal stuff.
            printf("\nCheck for Start, trajSpace=%d, PTU=%d. PTIdx=%d started=%d",trajectorySpace(),pathData.trajPointToUse,pathData.trajPointIdx,pathData.startedTraverse);
            if (trajectorySpace() < 8) // Wait till we have at least 8 points in the queue.
                {
                printf("\nSpace has dropped below 8");
                pathData.timeReference = (double)frc::Timer::GetFPGATimestamp();   
                pathData.startedTraverse = 1; // Only do the above at the very beginning.
                pathData.trajPointToUse = 0; // For some reason, this doesn't seem to start off at 0.  Set it at 0.
                }

            }
        if (pathData.startedTraverse != 0) // Once things have started, check timer to get proper index into trajectory array.
            {
            // printf("\nStarted Traverse: ");
            sref = (double)frc::Timer::GetFPGATimestamp();
            timediff = sref - pathData.timeReference; // will be in seconds with lots of decimal places.
            // normally, we should see this executed every 0.02 seconds but sometimes we skip 1, 2 or 3 of them.  We'll want to skip a few trajectory entries then too.
            idxstep = (int)(timediff/0.02 + 0.5); // get number of steps to skip, rounded to nearest integer.
            if (idxstep <  8)
                { // Only do this if idxstep is resonable.
                gyroReading = Robot::driveTrain->getGyroReading();
                pathData.trajPointToUse += idxstep; // for now, we'll just assume we have enough points for this to simply work.
                if (pathData.trajPointToUse >= MAX_TRAJ_POINTS)
                    pathData.trajPointToUse -= MAX_TRAJ_POINTS; // make sure we wrap around the circular array.
                // Simply output the velocity values to the motors.
                degree = trajPoints[pathData.trajPointToUse].heading * (180 / M_PI);
                double err = 0;
                if (trajPoints[pathData.trajPointToUse].velR >= 0) // if going forwards                    
                    err =  degree - (180 - gyroReading); 
                else
                    err = degree + gyroReading; // watching data shows degree is pretty much equal to -gyroReading when going backwards.
                // Warning. err can come out as 180 if velR is 0 and this is a backwards section.
                // It's more likely we should modify the heading value to be correct when going backwards but that has
                // a number of more complex direction implications that need more-careful examination.
    

                // Correct err if it's outside of +/-180
                while(err < - 180.0)
                    err += 360.0;
                while(err > 180.0)
                    err -= 360.0;

                printf("\nTraj %d idxstep=%d, velR=%f, velL=%f sref=%f, timeRef=%f, timediff=%f, space=%d, toUse=%d, PtIdx=%d XY(%f,%f) %fDeg Gyro=%f, err=%f",pathData.trajPointToUse,idxstep,trajPoints[pathData.trajPointToUse].velR,trajPoints[pathData.trajPointToUse].velL,
                            sref,pathData.timeReference,timediff,trajectorySpace(),pathData.trajPointToUse,pathData.trajPointIdx,
                            Robot::driveTrain->positionX,Robot::driveTrain->positionY,Robot::driveTrain->thetaHeading * 180 / M_PI,gyroReading,err);
//                printf("\nGyro: heading=%f, gyro=%f, err=%f",degree,gyroReading,err);
                // When going backwards (.velR and .velL are negative)
                // gyroReading = -degree is pretty much right on.
//                if(gyroReading < 0)
//                    gyroReading = (gyroReading + (360 * (float)abs(((int)(gyroReading / 360)))));;
//                if(gyroReading > 360)
//                    gyroReading = (gyroReading - (360 * (float)((int)(gyroReading / 360))));
//                if(gyroReading < 0)
//                    gyroReading = 360 + gyroReading;
//                //if(degree >= 270){
//                //    degree -= 360;
//                //}	
//                err = degree - gyroReading;
//                if(err > 180){ //Either we are way off course or header past 0 and is now something like 356
//                    err -= 360;
//                }
//                if(err < -180){ //Either we are way off course or header past 0 and is now something like 356
//                    err += 360;
//                }
                double g_mod = 0;
                if ((err > -60)&&(err < 60)) // Only consider errors in the +/-60 range.
                    {
                    g_mod = 0.02 * err;
                    }
//                g_mod = 0; // Disable gyro for now.

                //Modify left and right power

                Robot::driveTrain->setRightVelocity(trajPoints[pathData.trajPointToUse].velR - g_mod); 
                Robot::driveTrain->setLeftVelocity(trajPoints[pathData.trajPointToUse].velL + g_mod);
                }
            pathData.timeReference = sref; // update the reference for next time.
            }

        }  
    else // not really running.  increment pathData.trajPointToUse so we just generate the path and send the data to the cameraviewer
        {
        if (pathData.trajPointToUse != pathData.trajPointIdx)
            {
            pathData.trajPointToUse++; // 
            if (pathData.trajPointToUse >= MAX_TRAJ_POINTS)
                pathData.trajPointToUse = 0;
            }
        }
    
    if (trajectorySpace() > 4) // If we have space for trajectory entries, go make more,  Otherwise, wait till these are used up.
        { // we're expecting the trajectory generation stuff to recoginze when it's producing the last one for the segment and we can get the measured data
        idx = pathData.activeIdx; 
        if (pathData.pointsGenerated > idx) // makes sure math has been generated for this one.
            {
            if (!segmentData[idx].done) // if we have not yet generated all the traversal data for this one, go do some more.
                {
                if (segmentData[idx].sampleCount == 0) // if this is the first crack at this segment, init a bit of information ...
                    {
					// We'll need to prep the segments differently depending on segment type and previous segment type.
					if (segmentData[idx].segmentType == 1) // if this is a spline segment ... 
						{
						if (idx == 0) // If this is the very first segment, velocity will be starting at 0
							{ // and we can init a bunch of other stuff that needs to be set when starting a segment.
							pathData.pos = pathData.acceleration * pathData.cycleTime * pathData.cycleTime;
							pathData.arcPos = pathData.pos / segmentData[idx].linearDistance;
							pathData.vel = 0;
							}
						else
							{
							// This wasn't quite right.  The more-recent (closest to the 100% of segment) the trajectory point was generated, the more we want to delay 
							// coming up the with the first one of this path.  We have pos and nextPos.  Pos should be initiated with 
							// nextPos - pos times some factor based on how recent pos was used.  *( 1.0 - (samples - j) / (last diff between samples)
							// Nice!! This works for smoothing transition between spline segments.
	//                        printf("\npos=%f, prevPos=%f, sampleNum=%d, prevSampleNum=%d, numSamples=%d",pathData.pos,pathData.prevPos,segmentData[idx-1].sampleNum,segmentData[idx-1].prevSampleNum,segmentData[idx-1].numberOfSamples);
							if (segmentData[idx-1].segmentType == 1) // If previous was a spline, 
								{
								if (segmentData[idx-1].sampleNum != segmentData[idx-1].prevSampleNum)
									pathData.pos = (pathData.pos - pathData.prevPos) * (1.0 - (double)(segmentData[idx-1].numberOfSamples - segmentData[idx-1].sampleNum)/(double)(segmentData[idx-1].sampleNum - segmentData[idx-1].prevSampleNum));
								else
									pathData.pos = 0;                        
		//                        printf("\npos=%f",pathData.pos);
								}
							else // Previous was a circle.  Do things a bit different.
								{
								pathData.pos = 0;
								}								
							pathData.arcLength = 0;                                
							}
						
						// Other things that need to be set at the start of trajectory generation 
						pathData.heading = segmentData[idx].theta_offset;
	//                    segmentData[idx].currentX = segmentData[idx].startX; // We were storing these but never using them.
	//                    segmentData[idx].currentY = segmentData[idx].startY;
	//                    pathData.lastArcLength = 0;
						pathData.last_integrand = sqrt(1 + spline_derivativeAt2(idx, 0) * spline_derivativeAt2(idx, 0)) / segmentData[idx].numberOfSamples;
						setNextVel2(idx); // prepare nextPos and nextVel.
						}
					else // This is a circle
						{
						pathData.currentX = segmentData[idx].startX;
						pathData.currentY = segmentData[idx].startY;
						}						
					}

                if (segmentData[idx].segmentType == 1)
                    segmentData[idx].done = generatePath2(idx); // Go make some traversal points.  Only do a few each time.  we'll return true when we're done.
                else
                    segmentData[idx].done = generateCircPath2(idx);
                }
            else
                {
                printf("\n Completed segment %d, ID=%d",pathData.activeIdx,segmentData[pathData.activeIdx].segmentID);
                if (pathData.activeIdx < pathData.pointsGenerated - 1)
                    {
                    pathData.activeIdx++; // Move on to the next one.
                    printf("\nStarting segment %d, ID=%d",pathData.activeIdx,segmentData[pathData.activeIdx].segmentID);
                    // Other things that need to initiate traversal points should go here.
                    }
                else
                    {
                    done = true; // We've done all we can.  Time to end.
                    Robot::driveTrain->setRightVelocity(0); // Make sure things stop.
                    Robot::driveTrain->setLeftVelocity(0);
                    }
                
                }
            
            }
        else
            { // when we have another segment to run, get ready to run that one.
            if (pathData.activeIdx < pathData.pointsGenerated)
                {
                pathData.activeIdx++; // Move on to the next one.
                printf("\nStarting segment %d, ID=%d",pathData.activeIdx,segmentData[pathData.activeIdx].segmentID);
                // Other things that need to initiate traversal points should go here.
                }
            }
        
        // Generate the coefficents and any other initial data for spline or circles.
        if (pathData.pointsGenerated < pathData.pathPoints) // If we have another path segment to calculate and generate, go calculate it.
            { // could be a spline (segmentType 1) or circle (segmentType 2)
            // We may need to wait here till existing path is nearly complete so we can get measured x, y and angle.
            // If we're using calculated X,Y and angle, no need to wait.

            if (pathData.pointsGenerated != 0)
                { // use end point of previous segment as start pont for this one (as long as this isn't the first one)
                if (segmentData[pathData.pointsGenerated].useActual)
                    {
                    // wait till we can confidently predict actual position and angle before we proceed.    
                    }
                else 
                    { // no need to wait.  Use the final x,y and angle from previous spline as our start point.
                    pathData.calculatedX = segmentData[pathData.pointsGenerated-1].x;
                    pathData.calculatedY = segmentData[pathData.pointsGenerated-1].y;
                    pathData.calculatedAngle = segmentData[pathData.pointsGenerated-1].theta * 180/ M_PI;                        
                    segmentData[pathData.pointsGenerated].startX = pathData.calculatedX;
                    segmentData[pathData.pointsGenerated].startY = pathData.calculatedY;
                    segmentData[pathData.pointsGenerated].startTheta = segmentData[pathData.pointsGenerated-1].theta;
					segmentData[pathData.pointsGenerated].startAngle = segmentData[pathData.pointsGenerated-1].endAngle; // segmentData[pathData.pointsGenerated].startTheta * 180 / M_PI + 180; // For circles we will need the circular start angle.
					
                    }
                }
            if (segmentData[pathData.pointsGenerated].segmentType == 1)
                { // do math to set up for a spline
                generateSpline2(pathData.pointsGenerated); // Generate math info for this spline segment
                }
            else if (segmentData[pathData.pointsGenerated].segmentType == 2)
                { // do math to set up for a circular path.
                generateCircle2(pathData.pointsGenerated); // Generate math info for this circle segment
                }

            pathData.pointsGenerated++; // increment the paths we have set up the math for.
            }
        }
    // if the tractory array has data in it and we're to run the path with the robot, go get the correct trajectory data and send it.
    // if we're not to run it (this is a displayThePath run), set up the data for display on the cameraview only.

    // if trajectory array is not full, process data that will allow us to fill it, provided we have splines and circles to do that with
    return(done);
    }

// Path.cpp interface planning:
// To program a path, we need a starting point and then any number of waypoints for Splines but also allow for circular and possibly linear paths (spline should be able to handle a straight line)
// We also want to be able to get a quick and easy position report (which segment and how far (%) along that line to allow other systems to be activated)
// Subsequent path segments will use either the calculated position as the start point (and direction) for the next path segment or it can use the measured position and direction
// There is a small risk here in that the spline may fail or may created something odd if the start point changes by quite a bit.
// We will want very good calculation failure messages so it's quick and easy to see where things are not working.
// We also want to be able to plot the path on the camera viewer so there's a good idea as to what the robot is going to do BEFORE running it.
// 
// internally, we don't need to hold on to too much data (unlike the original version)
// We do want a deceleration option but it doesn't need to be perfectly accurate.  If we don't quite reach the end velocity or we hit it a bit earlier than 
// expected, that's not a big deal.  We'll use the distance per segment and how far we're into the number of samples (10,000 used for setting everything up)
// To predict distance to end of the path segment.  Using deceleration and calculating time, we can do a pretty good job of figuring out when to apply the 
// deceleration.
//
// We'll want to use the new PathFinder() function but we won't be passing all the same parameters.  We're going to hard-code cycle time a 50Hz
// Target Velocity and Final Velocity are going to be per segment, not the whole path.  Max acceleration and deceleration are pretty universal and it works
// to make them part of the whole path system, not per path segment.  No need for max jerk (don't think it was every used).  Distance between wheels we do need.
// path1 = new PathFinder(0.02,0,2,2,1.5,1,0.7112);  // cycle time (s), max velocity (m/s), max acceleration (m/s^2), max jerk (m/s^3), distance between wheels (m)
// path1 = new PathFinder(double acceleration, double deceleration, double distanceBetweenWheels) is what we're going with for now.
// instead of CreateNewPath, we'll have a setStartPoint(double x, double y, double angle) to establish where the robot is positioned at start-up.
// path1->addWayPoint(-5.713, 2.250, 0) now becomes 
// path1->splineTo(int segmentID, double x, double y, double angle, double targetVelocity, double finalVelocity, int useActual, int samples) 
// useActual is 0:uses Calculated Start Point.  1:uses Actual Start Point
// samples is normally something like 10000.  Longer segments should have fairly high samples.  Shorter ones can be lower.  Probably don't go much below 1000 or above 10000
// Original code required us to use - as forward and + as reverse.  Let's make a few changes to make this more intuitive and have + go forward and - go backwards.
// In fact, it might be necessary for us to specify if we want the robot to move forward or backwards.  We can probably calculate it based on the angle information
// but we need to conside this quite carefully.  Existing velocity information may need to be used to ensure we get it right.
// in order for Actual Start Point to work correctly, we'll need to have routines that read encoders (or motor positions) and gyro angle to track positions.  Future users of this code
// may have a number of different ways of collecting this data so we'll need to document it reasonably well.
// path1->circleAround(int segmentID, double x, double y, double toangle, double targetVelocity, double finalVelocity, int useActual) //  will plan a circular path
//       around x,y as the center of the circle.  toangle is the ending angle.  If it's > starting angle, we'll turn right.  If less, we'll turn left.  
//       the plan is to implement this stuff so angles are not really limited (can be < -360 and >+360) and the code will take care of the details.
// segmentID can be any value and is purely used to identify which segment of the path is currently being followed by the robot.  It's recommended that the segments be
// kept in order and that they be unique just to ensure ease of programming.  (probaby a good place for an enum so it would be relatively simple to insert path segments)
// path1->reportPosition(int useActual,&x,&y,&angle,&segmentID,&percentage); // will return current x,y and angle (either 0:calculated or 1:actual measured)
// it will also give the segmentID that the robot is currently following and the % (0.0 to 100.0) that the robot is currently at.  Warning: the robot might not quite
// get to 100% of the path segment so use this information accordingly.
// path1->followThePath() will activate the path following of the robot.  We're going to set things up so that even after path1->followThePath() starts the robot
// movement, additional splintTo() and circleAround() calls can be made and these will simply add to the path the robot is on at the time.
// Should the robot get to the end of the data it's been given without any additional data, the motors will be set to velocity of 0
// The system will remain ready for added paths although the starting velocity will be assumed to be 0.  It's best to ensure paths are added before the 
// robot reached 90% of the path it's already on.  If it's a short, high-speed path, it may be necessary to reduce the % by which the next segment should be added.
// Instead of path1->followThePath, we're also going to provide path1->displayThePath().  This one will go through the path generation process but will only
// generate the path data to the cameraviewer instead of running the robot.  This will allow for graphical verification of the intended path BEFORE sending the
// robot into a wall.
// path1->processPath() is the main periodic function of the path planning process. Set things up so that processPath() is called at 50Hz to ensure
// the path programming code can do all the necessary math fast enough to feed the traversal code.
//
// To have robot move in forward direction, it might make sense to specify a + target velocity
// To have robot move in reverse direction, how about a - target velocity.
//
// Testing by adding main() here ...
//
// OK.  We sort of broke things on the robot version of this.  We need to re-examine the path.cpp stuff and get the circle
// Generation working much more similar to the spline generation.
// Here's a few of the key things:
// GenerateSpline2 and GenerateCircle2 are meant to do some math calcs prior to generating the trajectory points.
// Gets StartX, StartY, StartTheta from pathData.measured or calculated and uses angleToTheta which is probably a good idea.   segmentData[idx].startTheta = angleToTheta(pathData.calculatedAngle); //  * M_PI/180.0;
// for Spline, .x and .y are the end point with end angle .theta.  These are given by the ToSpline call.  So this is how we get our start and end positions.
// just above the call to GenerateSpline2, you can see where we set the Start stuff for calculated
//                    pathData.calculatedX = segmentData[pathData.pointsGenerated-1].x;
//                    pathData.calculatedY = segmentData[pathData.pointsGenerated-1].y;
//                    pathData.calculatedAngle = segmentData[pathData.pointsGenerated-1].theta * 180/ M_PI; // End angle of last segment becomes start angle of next segment.
// These 3 lines probably do nothing since GenerateSpline2 sets StartX, StartY and startTheta at the very start of generateSpline2.  We can try commenting them out and see what happens.
//                    segmentData[pathData.pointsGenerated].startX = pathData.calculatedX;
//                    segmentData[pathData.pointsGenerated].startY = pathData.calculatedY;
//                    segmentData[pathData.pointsGenerated].startTheta = segmentData[pathData.pointsGenerated-1].theta;
//
// We did manage to come up with a way to calculate the circle end point and heading.  This will need to be implemented properly for us to be able to traverse from spline to circular
// and back to spline again without there being problems.
// 
// It's recommended to keep with the calculated and measured plans since we are likely to want to work with the idea of "measured" start points eventually.
// 
// To-Do: 
// 1.  adjust GenerateCircle2 to have .x, .y and .theta be the ending point of the circle traversal so the above code for setting "Calculated" works the same for both.
// 2.  For the pass 1 math stuff, don't use pathData for anything.  This should all be segmentData stuff.  Leave pathData for the actual trajectory point generation (if we need it)
// right now, there's a chance things could interfere with each other.
// 3. The math for circular should provide us with everything we need to crank out trajectory points and have that in the segmentData[] information (center X, Y, start X, Y, start angle, end angle are the critical items)
// 		we can calculate radius and maybe a couple of other things if it makes sense to do so.
// One issue is that the spline headings are NOT the same as the angles we feed into the spline (mostly due to use -X that's cranking out an angle that's 180 degrees from what we put in the splineTo information)
// circle stuff will need to work with this.  For now, let's have the actual circle start and end angles stored in segmentData[].startAngle, segmentData[].endAngle.
// These will be used for the math pass 1 stuff but we'll establish theta values for the traj part that matches what the spline stuff generates.
//     
// In order to keep generating Spline and Circle setup stuff, we need to have the final x,y and theta.  For
// the splines, this was easy but for circles, it's a bit trickier.  Angle isn't too hard but the final X,Y
// is a bit interesting.  We've figured that part out now too.

    // segmentData[idx].x,y are the center.  We'll end up at
    // .x + r cos(theta2), .y + r sin(theta2).  Where theta2 is our ending angle around the circle.  We need to consider end angle and turn direction to get this right.
    // When turning right, the angle of the line from the center of the circle to the robot location is heading(in deg) + 90 deg
    // When turning left, the angle of the line from the center of the circle to the robot location is heading(in deg) - 90 deg.
// 
// GeneratePath2 and GenerateCircPath2 ... These build the traj points
// GeneratePath2 does not appear to use segmentData[].currentX,Y,Angle.  The coefficients set for the segment would provide the correct
// location and heading information so there's no need to initiate it.  This may have to be a bit different with the circle path stuff.
// GenerateCircPath2 uses too much information from pathData.  It all needs to originate from segmentData.  We can use pathData values to do the traversal.
// We may need to be a bit more careful in transitions, especially between circles and splines.
// 
