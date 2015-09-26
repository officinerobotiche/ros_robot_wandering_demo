#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Twist.h>
#include <csignal>

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
std::string cmd_vel_string = "cmd_vel"; ///< Speed command to publish (Topic: geometry_msgs::Twist)
double repThresh = 1.5f;                 ///< Over this distance the point become attractive
double dangerThresh = 0.6;               ///< Points nearest than this distance are really dangerous and repulsion is doubled
double maxLaserVal = 8.0f;               ///< Max Laser distance
int dangerPtsMax = 5;                   ///< If there are more than @ref dangerPtsMax the robot stops and turn
double secureWidth = 0.70f;              ///< Width used to detect dangerous obstacles
double maxFwSpeed = 0.8f;                ///< Max forward speed (m/sec)
double maxRotSpeed = 3*M_PI;             ///< Max rotation speed (rad/sec)
// <<<<< Params

// >>>>> Globals
std::string name_node = "robot_wandering_node";
ros::NodeHandle* nhPtr=NULL;
ros::Publisher* wrenchPubPtr=NULL;
ros::Publisher* twistPubPtr=NULL;
double normVal = 8.0f;
double last_omega_valid=0.0f;
double last_forceRot_danger=0.0f;

bool firstScan=true;

struct sigaction sigAct;
bool stop = false;
// <<<<< Globals

// >>>>> Nav
typedef struct _nav
{
    double forceFw;
    double forceRot;
    bool valid;
    bool danger;
} Nav;
Nav navInfo;
// <<<<< Nav

// >>>>> Ctrl+C handler
/*! Ctrl+C handler
 */
static void sighandler(int signo)
{
    stop = true;
}
// <<<<< Ctrl+C handler

int main(int argc, char** argv)
{
    ros::init(argc, argv, name_node);
    ros::NodeHandle nh;
    ros::NodeHandle nhPriv("~"); // Private node handler to retrieve parameters

    // >>>>> Ctrl+C handling
    memset( &sigAct, 0, sizeof(sigAct) );
    sigAct.sa_handler = sighandler;
    sigaction(SIGINT, &sigAct, 0);
    // <<<<< Ctrl+C handling

    nhPtr = &nh;

    load_params( nhPriv );

    // >>>>> Subscribers
    ros::Subscriber scanSub;
    scanSub = nh.subscribe<sensor_msgs::LaserScan>("scan",1,&processLaserScan); // TODO add namespace before message!
    // <<<<< Subscribers

    ros::Publisher wrenchPub = nh.advertise<geometry_msgs::WrenchStamped>("nav_force", 10, false);
    wrenchPubPtr = &wrenchPub;
    ros::Publisher twistPub = nh.advertise<geometry_msgs::Twist>( cmd_vel_string, 10);
    twistPubPtr = &twistPub;

    ros::Rate r(30);

    ROS_INFO("robot_wandering_node node starting...!");

    navInfo.valid = false;
    navInfo.danger = false;

    geometry_msgs::Twist vel;

    stop = false;

    while(nh.ok())
    {
        if(stop)
        {
            ROS_INFO("... robot_wandering_node node stopping!");

            vel.linear.x = 0;
            vel.linear.y = 0;
            vel.linear.z = 0;

            vel.angular.x = 0;
            vel.angular.y = 0;
            vel.angular.z = 0;

            twistPub.publish( vel );

            break;
        }

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

    exit( EXIT_SUCCESS );
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
    if( nh.hasParam( "cmd_vel_string" ) )
    {
        nh.getParam( "cmd_vel_string", cmd_vel_string );
    }
    else
    {
        nh.setParam( "cmd_vel_string", cmd_vel_string );
        ROS_INFO_STREAM( "cmd_vel_string" << " not present. Default value set: " << cmd_vel_string );
    }

    if( nh.hasParam( "dangerThresh" ) )
    {
        nh.getParam( "dangerThresh", dangerThresh );
    }
    else
    {
        nh.setParam( "dangerThresh", dangerThresh );
        ROS_INFO_STREAM( "dangerThresh" << " not present. Default value set: " << dangerThresh );
    }

    if( nh.hasParam( "maxLaserVal" ) )
    {
        nh.getParam( "maxLaserVal", maxLaserVal );
    }
    else
    {
        nh.setParam( "maxLaserVal", maxLaserVal );
        ROS_INFO_STREAM( "maxLaserVal" << " not present. Default value set: " << maxLaserVal );
    }

    if( nh.hasParam( "dangerPtsMax" ) )
    {
        nh.getParam( "dangerPtsMax", dangerPtsMax );
    }
    else
    {
        nh.setParam( "dangerPtsMax", dangerPtsMax );
        ROS_INFO_STREAM( "dangerPtsMax" << " not present. Default value set: " << dangerPtsMax );
    }

    if( nh.hasParam( "secureWidth" ) )
    {
        nh.getParam( "secureWidth", secureWidth );
    }
    else
    {
        nh.setParam( "secureWidth", secureWidth );
        ROS_INFO_STREAM( "secureWidth" << " not present. Default value set: " << secureWidth );
    }

    if( nh.hasParam( "maxFwSpeed" ) )
    {
        nh.getParam( "maxFwSpeed", maxFwSpeed );
    }
    else
    {
        nh.setParam( "maxFwSpeed", maxFwSpeed );
        ROS_INFO_STREAM( "maxFwSpeed" << " not present. Default value set: " << maxFwSpeed );
    }

    if( nh.hasParam( "maxRotSpeed" ) )
    {
        nh.getParam( "maxRotSpeed", maxRotSpeed );
    }
    else
    {
        nh.setParam( "maxRotSpeed", maxRotSpeed );
        ROS_INFO_STREAM( "maxRotSpeed" << " not present. Default value set: " << maxRotSpeed );
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

    double angle = scan->angle_min;

    double forceX = 0.0f;
    double forceY = 0.0f;

    if( firstScan )
    {
        firstScan=false;

        double forceX=0.0f, forceY=0.0f;

        for( int i=0; i<scan->ranges.size(); i++ )
        {
            double range = 1.0; // Max normalized value

            double Fx = range*cos(angle);
            double Fy = range*sin(angle);

            forceX += Fx;
            forceY += Fy;
        }

        normVal = sqrt(forceY*forceY+forceX*forceX); // Value to normalize the module of the forcce vector

        ROS_INFO_STREAM( "Force normalization value: " << normVal );
    }

    int dangerPtsCount = 0;
    int validCount = 0;

    for( int i=0; i<scan->ranges.size(); i++)
    {
        angle += scan->angle_increment; // Angle update

        double range = scan->ranges[i];

        if( isnan(range) )
        {
            // TODO replace with "scan->range_max"?
            //range = 1.0f;
            continue;
        }
        else
        {
            if( range > maxLaserVal )
                range = maxLaserVal;

            validCount++;

            double ptForce = range;

            if( ptForce < repThresh)
            {
                // >>>>> Projections
                double Fx = ptForce*cos(angle);
                double Fy = ptForce*sin(angle);
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
            double ptForceNorm = ptForce/maxLaserVal; // normalization

            // >>>>> Reprojection with normalization
            double Fx_n = ptForceNorm*cos(angle);
            double Fy_n = ptForceNorm*sin(angle);
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
        navInfo.forceFw = forceX/(double)validCount;
        navInfo.forceRot = forceY/(double)validCount;
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



