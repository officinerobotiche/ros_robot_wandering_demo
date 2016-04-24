#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PolygonStamped.h>
#include <csignal>
#include <visualization_msgs/Marker.h>

#include <dynamic_reconfigure/server.h>
#include <ros_robot_wandering_demo/wandering_dyn_paramsConfig.h>

//#define WRENCH_VIEW_DEBUG 0

// >>>>> Functions declarations
void load_params(ros::NodeHandle& nh);
void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan);
void exitMinimum();
void dynReconfCallback(ros_robot_wandering_demo::wandering_dyn_paramsConfig &config, uint32_t level);
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
std::string frame_link = "base_link";   ///< Referene frame for rviz visualization
double repThresh = 1.5f;                ///< Over this distance the point become attractive
double dangerThresh = 0.6;              ///< Points nearest than this distance are really dangerous and repulsion is doubled
double maxLaserVal = 8.0f;              ///< Max Laser distance
int dangerPtsPerc = 25;                  ///< If there are more than @ref dangerPtsPerc the robot stops and turn
double secureWidth = 0.70f;             ///< Width used to detect dangerous obstacles
double maxFwSpeed = 0.8f;               ///< Max forward speed (m/sec)
double maxRotSpeed = 3*M_PI;            ///< Max rotation speed (rad/sec)
bool noInfoTurnAround = false;          ///< Indicates if the robot must stop and turn around when no laser info are available (obstacle too near)
// <<<<< Params

// >>>>> Globals
std::string name_node = "robot_wandering_node";
ros::NodeHandle* nhPtr=NULL;
ros::Publisher* wrenchPubPtr=NULL;
ros::Publisher* twistPubPtr=NULL;
double normVal = 1.0;
double last_omega_valid=0.0;
double last_forceRot_danger=0.0;

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

    // >>>>> Dynamic reconfigure
    dynamic_reconfigure::Server<ros_robot_wandering_demo::wandering_dyn_paramsConfig> server;
    dynamic_reconfigure::Server<ros_robot_wandering_demo::wandering_dyn_paramsConfig>::CallbackType f;

    f = boost::bind(&dynReconfCallback, _1, _2);
    server.setCallback(f);
    // <<<<< Dynamic reconfigure

    nhPtr = &nh;

    load_params( nhPriv );

    // >>>>> Subscribers
    ros::Subscriber scanSub;
    scanSub = nh.subscribe<sensor_msgs::LaserScan>("scan",1,&processLaserScan);
    // <<<<< Subscribers

    ros::Publisher wrenchPub = nh.advertise<geometry_msgs::WrenchStamped>("nav_force", 10, false);
    wrenchPubPtr = &wrenchPub;

    ros::Publisher twistPub = nh.advertise<geometry_msgs::Twist>( cmd_vel_string, 10);
    twistPubPtr = &twistPub;

    ros::Publisher dangerZonePub = nh.advertise<visualization_msgs::Marker>("danger_zone", 1, false );
    ros::Publisher repLimitPub = nh.advertise<visualization_msgs::Marker>("repulsive_limit", 1, false );
    ros::Publisher laserLimitPub = nh.advertise<visualization_msgs::Marker>("laser_limit", 1, false );
    ros::Publisher forceVectorPub = nh.advertise<visualization_msgs::Marker>("force_vector", 1, false );

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
            // >>>>> Transform force values to speed commands
            vel.linear.x = navInfo.forceFw*maxFwSpeed;
            vel.linear.y = 0;
            vel.linear.z = 0;

            vel.angular.x = 0;
            vel.angular.y = 0;
            vel.angular.z = navInfo.forceRot*maxFwSpeed;

            last_omega_valid = vel.angular.z;

            if( fabs( vel.linear.x) < 0.03f && fabs(vel.angular.z) < 3.0f*DEG2RAD ) // Local minimum???
            {
                ROS_INFO_STREAM( "V: " << vel.linear.x << "m/sec - Omega: " << vel.angular.z << " rad/sec" );
                exitMinimum();
            }
            else
                twistPub.publish( vel );
            // <<<<< Transform force values to speed commands

            navInfo.valid = false;
        }

        // >>>>> Robot areas for Rviz

        ros::Time now = ros::Time::now();

        if( dangerZonePub.getNumSubscribers()>0 )
        {
            /*geometry_msgs::PolygonStamped plgMsg;
            plgMsg.header.frame_id = frame_link;
            plgMsg.header.stamp = now;

            geometry_msgs::Point32 pt1;
            pt1.x = 0.0;
            pt1.y = -secureWidth/2.0;
            pt1.z = 0.0;

            geometry_msgs::Point32 pt2;
            pt2.x = 0.0;
            pt2.y = secureWidth/2.0;
            pt2.z = 0.0;

            geometry_msgs::Point32 pt3;
            pt3.x = dangerThresh;
            pt3.y = secureWidth/2.0;
            pt3.z = 0.0;

            geometry_msgs::Point32 pt4;
            pt4.x = dangerThresh;
            pt4.y = -secureWidth/2.0;
            pt4.z = 0;

            plgMsg.polygon.points.push_back( pt1 );
            plgMsg.polygon.points.push_back( pt2 );
            plgMsg.polygon.points.push_back( pt3 );
            plgMsg.polygon.points.push_back( pt4 );

            dangerZonePub.publish( plgMsg );*/

            visualization_msgs::Marker marker;

            marker.header.frame_id = frame_link;
            marker.header.stamp = now;
            marker.ns = "wandering";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = dangerThresh/2;
            marker.pose.position.y = 0.0;
            marker.pose.position.z = 0.0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = dangerThresh;
            marker.scale.y = secureWidth;
            marker.scale.z = 0.001;
            marker.color.a = 0.4;
            marker.color.r = 1.0;
            marker.color.g = 0.3;
            marker.color.b = 0.3;

            dangerZonePub.publish( marker );
        }

        if( repLimitPub.getNumSubscribers()>0 )
        {
            visualization_msgs::Marker marker;

            marker.header.frame_id = frame_link;
            marker.header.stamp = now;
            marker.ns = "wandering";
            marker.id = 1;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = 0.0;
            marker.pose.position.y = 0.0;
            marker.pose.position.z = 0.0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = repThresh*2;
            marker.scale.y = repThresh*2;
            marker.scale.z = 0.001;
            marker.color.a = 0.3;
            marker.color.r = 0.7;
            marker.color.g = 0.4;
            marker.color.b = 0.3;

            repLimitPub.publish( marker );
        }

        if( laserLimitPub.getNumSubscribers()>0 )
        {
            visualization_msgs::Marker marker;

            marker.header.frame_id = frame_link;
            marker.header.stamp = now;
            marker.ns = "wandering";
            marker.id = 2;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = 0.0;
            marker.pose.position.y = 0.0;
            marker.pose.position.z = 0.0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = maxLaserVal*2;
            marker.scale.y = maxLaserVal*2;
            marker.scale.z = 0.001;
            marker.color.a = 0.3;
            marker.color.r = 0.7;
            marker.color.g = 0.7;
            marker.color.b = 0.3;

            laserLimitPub.publish( marker );
        }

        if( forceVectorPub.getNumSubscribers()>0 )
        {
            visualization_msgs::Marker marker;

            marker.header.frame_id = frame_link;
            marker.header.stamp = now;
            marker.ns = "wandering";
            marker.id = 2;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = 0.0;
            marker.pose.position.y = 0.0;
            marker.pose.position.z = 0.0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = navInfo.forceFw*maxFwSpeed;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 0.7;
            marker.color.b = 0.3;

            forceVectorPub.publish( marker );
        }
        // <<<<< Robot areas for Rviz

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


void dynReconfCallback(ros_robot_wandering_demo::wandering_dyn_paramsConfig &config, uint32_t level)
{
    repThresh = config.repThresh;
    dangerThresh = config.dangerThresh;
    maxLaserVal = config.maxLaserVal;
    dangerPtsPerc = config.dangerPtsPerc;
    secureWidth = config.secureWidth;
    maxFwSpeed = config.maxFwSpeed;
    maxRotSpeed = config.maxRotSpeed;
    noInfoTurnAround = config.noInfoTurnAround;

    if( config.restore_defaults )
    {
        config.repThresh = 1.5f;                // Over this distance the point become attractive
        config.dangerThresh = 0.6;              // Points nearest than this distance are really dangerous and repulsion is doubled
        config.maxLaserVal = 8.0f;              // Max Laser distance
        config.dangerPtsPerc = 5;                // If there are more than @ref dangerPtsPerc the robot stops and turn
        config.secureWidth = 0.70f;             // Width used to detect dangerous obstacles
        config.maxFwSpeed = 0.8f;               // Max forward speed (m/sec)
        config.maxRotSpeed = 3*M_PI;            // Max rotation speed (rad/sec)
        config.noInfoTurnAround = false;        // Indicates if the robot must stop and turn around when no laser info are available (obstacle too near)

        config.restore_defaults = false;
    }
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

    if( nh.hasParam( "frame_link" ) )
    {
        nh.getParam( "frame_link", frame_link );
    }
    else
    {
        nh.setParam( "frame_link", frame_link );
        ROS_INFO_STREAM( "frame_link" << " not present. Default value set: " << frame_link );
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

    if( nh.hasParam( "dangerPtsPerc" ) )
    {
        nh.getParam( "dangerPtsPerc", dangerPtsPerc );
    }
    else
    {
        nh.setParam( "dangerPtsPerc", dangerPtsPerc );
        ROS_INFO_STREAM( "dangerPtsPerc" << " not present. Default value set: " << dangerPtsPerc );
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

    if( nh.hasParam( "noInfoTurnAround" ) )
    {
        nh.getParam( "noInfoTurnAround", noInfoTurnAround );
    }
    else
    {
        nh.setParam( "noInfoTurnAround", noInfoTurnAround );
        ROS_INFO_STREAM( "noInfoTurnAround" << " not present. Default value set: " << noInfoTurnAround );
    }

}

void processLaserScan( const sensor_msgs::LaserScan::ConstPtr& scan )
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

        normVal = sqrt(forceY*forceY+forceX*forceX); // Value to normalize the module of the force vector

        ROS_INFO_STREAM( "Force normalization value: " << normVal );
    }

    int dangerPtsCount = 0;
    int validCount = 0;

    int totPts = scan->ranges.size();

    for( int i=0; i<totPts; i++)
    {
        angle += scan->angle_increment; // Angle update

        double range = scan->ranges[i];

        if( isnan(range) )
        {
            // Ignore not valid measures
            continue;
        }
        else
        {
            if( range > maxLaserVal )
                range = maxLaserVal;

            validCount++;

            double ptForce = range;

            if( ptForce < repThresh) // Point is actractive or repulsive?
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

            double ptForceNorm = ptForce/maxLaserVal; // normalization

            // >>>>> Reprojection with normalization
            double Fx_n = ptForceNorm*cos(angle);
            double Fy_n = ptForceNorm*sin(angle);
            // <<<<< Reprojection with normalization

#ifdef WRENCH_VIEW_DEBUG
            geometry_msgs::WrenchStamped forceMsg;
            forceMsg.header.stamp = scan->header.stamp;
            forceMsg.header.frame_id = frame_link;

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
    forceMsg.header.frame_id = frame_link;

    int ptsDangerThresh = ((double)totPts)*((double)dangerPtsPerc)/100.0;

    if( dangerPtsCount >= ptsDangerThresh ) // Danger... stop forwarding
    {
        navInfo.forceFw = 0.0f;

        if( !navInfo.danger ) // If the robot was not previously in danger...
        {
            navInfo.forceRot = 1.0*SIGN(forceY);
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

    // TODO When noInfoTurnAround is TRUE the robot must turn on the place to search for info
    // TODO When noInfoTurnAround is FALSE the robot must go straight to search for info

    navInfo.valid = true;

    // >>>>> Force vector visualization on Rviz
    if(wrenchPubPtr->getNumSubscribers()>0)
    {
        forceMsg.wrench.force.x = navInfo.forceFw*maxFwSpeed;
        forceMsg.wrench.force.y = navInfo.forceRot*maxRotSpeed;
        forceMsg.wrench.force.z = 0;

        wrenchPubPtr->publish(forceMsg);

        ROS_INFO_STREAM( "Force FW: " << navInfo.forceFw*maxFwSpeed << " - Force ROT: " << navInfo.forceRot*maxRotSpeed );

#ifdef WRENCH_VIEW_DEBUG
        ros::Duration(0.01).sleep();
#endif

        ROS_DEBUG_STREAM( "Force: " << navInfo.forceFw << " - Ang: " << navInfo.forceRot*RAD2DEG );


    }
    // <<<<< Force vector visualization on Rviz
}



