#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Twist.h>

//#define WRENCH_VIEW_DEBUG 0

// >>>>> Functions declarations
void load_params(ros::NodeHandle& nh);
void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan);
void exitMinimum();
// <<<<< Functions declarations

#ifndef DEG2RAD
#define DEG2RAD 0.017453293f
#define RAD2DEG 57.295777937f
#endif

#ifndef SIGN
#define SIGN(X) (((X)>=0)?1:-1);
#endif

// >>>>> Params
std::string robot_name = ""; /// Name of the robot as prefix

std::string command = "command";
std::string velocity = "velocity";

std::string name_node = "robot_wandering_node";

float repThresh = 1.5f; /// Over this distance the point become attractive
float dangerThresh = 0.6; /// If there are more than @ref dangerPtsCount the robot stops and turn
float maxVal = 5.0f; /// Max Value used to normalize distances
int dangerPtsMax = 5;

float secureWidth = 0.70f; /// Used to detect dangerous obstacles

float maxFwSpeed = 0.8f; /// Max forward speed (m/sec)
float maxRotSpeed = 3*M_PI; /// Max rotation speed (rad/sec)
// <<<<< Params

// >>>>> Globals
ros::NodeHandle* nhPtr=NULL;
ros::Publisher* wrenchPubPtr=NULL;
ros::Publisher* twistPubPtr=NULL;
float normVal = 8.0f;
float last_omega_valid=0.0f;
float last_forceRot_danger=0.0f;

bool firstScan=true;
// <<<<< Globals

// >>>>> Nav
typedef struct _nav
{
    float forceFw;
    float forceRot;
    bool valid;
    bool danger;
} Nav;
Nav navInfo;
// <<<<< Nav

int main(int argc, char** argv)
{
    ros::init(argc, argv, name_node);
    ros::NodeHandle nh;

    nhPtr = &nh;

    load_params( nh );

    // >>>>> Subscribers
    ros::Subscriber scanSub;
    scanSub = nh.subscribe<sensor_msgs::LaserScan>("/scan",1,&processLaserScan); // TODO add namespace before message!
    // <<<<< Subscribers

    ros::Publisher wrenchPub = nh.advertise<geometry_msgs::WrenchStamped>("nav_force", 10, false);
    wrenchPubPtr = &wrenchPub;
    ros::Publisher twistPub = nh.advertise<geometry_msgs::Twist>("/" + robot_name + "/" + command + "/" + velocity, 10);
    twistPubPtr = &twistPub;

    ros::Rate r(30);

    ROS_INFO("robot_wandering_node node starting...!");

    navInfo.valid = false;
    navInfo.danger = false;

    geometry_msgs::Twist vel;

    while(nh.ok())
    {
        if( navInfo.valid )
        {
            vel.linear.x = navInfo.forceFw*maxFwSpeed;
            vel.linear.y = 0;
            vel.linear.z = 0;

            vel.angular.x = 0;
            vel.angular.y = 0;
            vel.angular.z = navInfo.forceRot*maxFwSpeed;

            last_omega_valid = vel.angular.z;

            if( fabs( vel.linear.x) < 0.03f && fabs(vel.angular.z) < 3.0f*DEG2RAD )
            {
                ROS_INFO_STREAM( "V: " << vel.linear.x << "m/sec - Omega: " << vel.angular.z << " rad/sec" );
                exitMinimum();
            }
            else
                twistPub.publish( vel );

            navInfo.valid = false;
        }
        /*else
        {
            exitMinimum();
        }*/

        ros::spinOnce();
        r.sleep();
    }
}

void exitMinimum()
{
    geometry_msgs::Twist vel;

    ROS_INFO_STREAM("Exiting from minimum...");
    vel.linear.x = -0.1;
    vel.linear.y = 0;
    vel.linear.z = 0;

    vel.angular.x = 0;
    vel.angular.y = 0;
    vel.angular.z = 0;

    twistPubPtr->publish( vel );

    ros::Duration(1.0).sleep();

    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.linear.z = 0;

    vel.angular.x = 0;
    vel.angular.y = 0;
    vel.angular.z = (M_PI/2.0f)*SIGN(last_omega_valid);

    twistPubPtr->publish( vel );

    ros::Duration(1.0).sleep();
}

void load_params(ros::NodeHandle& nh)
{
    if (nh.hasParam(name_node + "/velocity"))
    {
        nh.getParam(name_node + "/velocity", velocity);
    } else {
        nh.setParam(name_node + "/velocity", velocity);
    }

    if (nh.hasParam("/info/robot_name"))
    {
        nh.getParam("/info/robot_name", robot_name );
    } else {
        nh.setParam("/info/robot_name", robot_name);
    }
}

void processLaserScan( const sensor_msgs::LaserScan::ConstPtr& scan)
{
    /*                      X 0°
     *                      ^
     *                      |
     *                      |
     *                      |
     *                      |
     * 90° +Y  <------------|-----------> Y -90°
     */

    float angle = scan->angle_min;

    float forceX = 0.0f;
    float forceY = 0.0f;

    if( firstScan )
    {
        firstScan=false;

        float forceX=0.0f, forceY=0.0f;

        for( int i=0; i<scan->ranges.size(); i++ )
        {
            float range = 1.0; // Max normalized value

            float Fx = range*cos(angle);
            float Fy = range*sin(angle);

            forceX += Fx;
            forceY += Fy;
        }

        normVal = sqrt(forceY*forceY+forceX*forceX);

        ROS_INFO_STREAM( "Force normalization value: " << normVal );
    }

    int dangerPtsCount = 0;
    int validCount = 0;

    for( int i=0; i<scan->ranges.size(); i++)
    {
        angle += scan->angle_increment;

        float range = scan->ranges[i];        

        if( isnan(range) )
        {
            // TODO replace with "scan->range_max"?
            //range = 1.0f;
	    continue;
        }
        else
        {
	    if( range > maxVal )
		range = maxVal;

            validCount++;

            float ptForce = range;

            if( ptForce < repThresh)
            {
                // >>>>> Projections
                float Fx = ptForce*cos(angle);
                float Fy = ptForce*sin(angle);
                // <<<<< Projections

                if( (Fx < dangerThresh) && (fabs(Fy) < secureWidth/2.0f) ) // Point in the danger rectangular Area in front of the robot
                {
                    dangerPtsCount++;

                    ptForce *= -2.0f; // Amplifield repulsion
                    //ROS_INFO_STREAM( "Fy: " << Fy << " Fx: " << Fx << " dangerPtsCount:" << dangerPtsCount );
                }
                else
                    ptForce *= -1; // Simple repulsion
            }

            //ptForce = (range > repThresh)?range:-range; // Repulsion!
            float ptForceNorm = ptForce/maxVal; // normalization

            // >>>>> Reprojection with normalization
            float Fx_n = ptForceNorm*cos(angle);
            float Fy_n = ptForceNorm*sin(angle);
            // <<<<< Reprojection with normalization

#ifdef WRENCH_VIEW_DEBUG
            geometry_msgs::WrenchStamped forceMsg;
            forceMsg.header.stamp = scan->header.stamp;
            forceMsg.header.frame_id = "base_link";

            forceMsg.wrench.force.x = Fx_n;
            forceMsg.wrench.force.y = Fy_n;
            forceMsg.wrench.force.z = 0;

            wrenchPubPtr->publish(forceMsg);

            ros::Duration(0.01).sleep();
#endif

            forceX += Fx_n;
            forceY += Fy_n;
        }
    }

    geometry_msgs::WrenchStamped forceMsg;
    forceMsg.header.stamp = scan->header.stamp;
    forceMsg.header.frame_id = "base_link";



    if( dangerPtsCount >= dangerPtsMax || validCount<20) // Danger... stop forwarding
    {
        navInfo.forceFw = 0.0f;
        //navInfo.forceRot = -2.0*SIGN(forceY);

        if( !navInfo.danger )
        {
            navInfo.forceRot = 2.0*SIGN(forceY);
            last_forceRot_danger = navInfo.forceRot;
        }
        else
        {
            navInfo.forceRot = last_forceRot_danger;
        }

        navInfo.danger = true;


    }
    else
    {
        navInfo.forceFw = forceX/(float)validCount;
        navInfo.forceRot = forceY/(float)validCount;
        navInfo.danger = false;
    }



    navInfo.valid = true;

    forceMsg.wrench.force.x = navInfo.forceFw;
    forceMsg.wrench.force.y = navInfo.forceRot;
    forceMsg.wrench.force.z = 0;

    wrenchPubPtr->publish(forceMsg);

    ROS_INFO_STREAM( "Force FW: " << navInfo.forceFw << " - Force ROT: " << navInfo.forceRot );

#ifdef WRENCH_VIEW_DEBUG
    ros::Duration(0.01).sleep();
#endif

    ROS_DEBUG_STREAM( "Force: " << navInfo.forceFw << " - Ang: " << navInfo.forceRot*RAD2DEG );
}



