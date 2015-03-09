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
    #define SIGN(X) (X)>=0?(1):(-1);
#endif

// >>>>> Params
std::string robot_name = ""; /// Name of the robot as prefix

std::string command = "command";
std::string velocity = "velocity";

std::string name_node = "robot_wandering_node";

float repThresh = 1.5f; /// Over this distance the point become attractive
float dangerThresh = 0.5; /// If there are more than @ref dangerPtsCount the robot stops and turn
float maxVal = 5.0f; /// Max Value used to normalize distances
int dangerPtsMax = 10;

float maxFwSpeed = 1.0f; /// Max forward speed (m/sec)
float maxRotSpeed = M_PI; /// Max rotation speed (rad/sec)
// <<<<< Params

// >>>>> Globals
ros::NodeHandle* nhPtr=NULL;
ros::Publisher* wrenchPubPtr=NULL;
ros::Publisher* twistPubPtr=NULL;
float normVal = 8.0f;
float last_omega_valid=0.0f;

bool firstScan=true;
// <<<<< Globals

// >>>>> Nav
typedef struct _nav
{
    float forceMod;
    float forceAng;
    bool valid;
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

    geometry_msgs::Twist vel;

    while(nh.ok())
    {        
        if( navInfo.valid )
        {
            vel.linear.x = navInfo.forceMod*maxFwSpeed;
            vel.linear.y = 0;
            vel.linear.z = 0;

            vel.angular.x = 0;
            vel.angular.y = 0;
            vel.angular.z = navInfo.forceAng*maxFwSpeed;

            if( fabs(navInfo.forceAng > 0.5 ) )
                vel.linear.x *= -1.0f;

            last_omega_valid = vel.angular.z;

            if( fabs( vel.linear.x) < 0.05f && fabs(vel.angular.z) < 0.05f )
                exitMinimum();
            else
                twistPub.publish( vel );
        }
        else
        {
            exitMinimum();
        }



        ros::spinOnce();
        r.sleep();
    }
}

void exitMinimum()
{
    geometry_msgs::Twist vel;

    ROS_INFO_STREAM("Exiting from minimum...");
    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.linear.z = 0;

    vel.angular.x = 0;
    vel.angular.y = 0;
    vel.angular.z = M_PI*SIGN(last_omega_valid);;

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

        float forceX, forceY;

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

    for( int i=0; i<scan->ranges.size(); i++)
    {
        angle += scan->angle_increment;

        float range = scan->ranges[i];

        if( isnan(range) )
        {

            // TODO replave with "scan->range_max"?
            range = 1.0f;
        }
        else
        {
            float ptForce = (range > repThresh)?range:-3*range;
            ptForce /= maxVal; // normalization

            if( range < dangerThresh )
                dangerPtsCount++;

            float Fx = ptForce*cos(angle);
            float Fy = ptForce*sin(angle);

#ifdef WRENCH_VIEW_DEBUG
            geometry_msgs::WrenchStamped forceMsg;
            forceMsg.header.stamp = scan->header.stamp;
            forceMsg.header.frame_id = "base_link";

            forceMsg.wrench.force.x = Fx;
            forceMsg.wrench.force.y = Fy;
            forceMsg.wrench.force.z = 0;

            wrenchPubPtr->publish(forceMsg);

            ros::Duration(0.01).sleep();
#endif

            forceX += Fx;
            forceY += Fy;
        }
    }

    navInfo.forceMod = sqrt(forceY*forceY+forceX*forceX)/normVal;
    navInfo.forceAng = -atan2( forceY, forceX )/(2*M_PI);

    if( dangerPtsCount<dangerPtsMax )
        navInfo.valid = true;
    else
        navInfo.valid = false;

    geometry_msgs::WrenchStamped forceMsg;
    forceMsg.header.stamp = scan->header.stamp;
    forceMsg.header.frame_id = "base_link";

    forceMsg.wrench.force.x = forceX/normVal;
    forceMsg.wrench.force.y = forceY/normVal;
    forceMsg.wrench.force.z = 0;

    wrenchPubPtr->publish(forceMsg);

    ROS_INFO_STREAM( "Force FW: " << navInfo.forceMod << " - Force ROT: " << navInfo.forceAng );

#ifdef WRENCH_VIEW_DEBUG
    ros::Duration(0.01).sleep();
#endif

    ROS_DEBUG_STREAM( "Force: " << navInfo.forceMod << " - Ang: " << navInfo.forceAng*RAD2DEG );
}



