#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include "ros/ros.h"


using namespace std;

class NodeSync
{
 public:
  NodeSync()
  {
    noGoal = true;
    scan_sub_.subscribe(nh_, "/scan", 1);
    pose_sub_.subscribe(nh_, "/pose", 1);
    goal_sub_ = nh_.subscribe("goal", 1, &NodeSync::goalCb, this);
    velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    sync_.reset(new Sync(MySyncPolicy(10), scan_sub_, pose_sub_));
    sync_->registerCallback(boost::bind(&NodeSync::callback, this, _1, _2));
  }



  //Callback para las metas
  void goalCb(const geometry_msgs::PoseStamped& msg) {

		std::cout << " Goal Update: "<< msg.pose.position.x << endl;
		std::cout << " Goal Update: "<< msg.pose.position.y << endl;
		noGoal = false; // hay una meta.
	//	upadte the goal
		Goal.pose.position.x = msg.pose.position.x;
		Goal.pose.position.y = msg.pose.position.y;
	}


  /*Callback sincrono de la odometria e informacion del láser.
    Se ha modificado para implementar un control de velocidad en el propio callback,
    que controla el robot en su camino a una meta Goal. Además si el laser detecta un 
    obstaculo cerca, lo detiene
    */
  void callback(const sensor_msgs::LaserScan::ConstPtr& laser_scan, const nav_msgs::Odometry::ConstPtr& estimate_pose)
  {
    ROS_INFO("Synchronization successful");
    
    // Get position info
    tf::Quaternion q(	estimate_pose->pose.pose.orientation.x, 
    			estimate_pose->pose.pose.orientation.y, 
    			estimate_pose->pose.pose.orientation.z, 
    			estimate_pose->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    ROS_INFO("Robot Pose x:%f ; y:%f ; th:%f ;", estimate_pose->pose.pose.position.x,estimate_pose->pose.pose.position.y,yaw);


    /*BUCLE DE CONTROL DE VELOCIDADES*/


		if(!noGoal){
			std::cout << "meta x: "<< Goal.pose.position.x << " ";
		std::cout << "meta y: "<< Goal.pose.position.y << " ";
		
    //Cogemos la odometria del robot
		float ex = estimate_pose->pose.pose.position.x - Goal.pose.position.x;
		float ey = estimate_pose->pose.pose.position.y - Goal.pose.position.y;
		float rho = sqrt(ex*ex+ey*ey);
		float beta = atan2(ey,ex) + M_PI;
		if (beta < -M_PI) beta = 2*M_PI - abs(beta);
		if (beta > M_PI) beta = -2*M_PI + beta;
		//angle to the goal
		float alpha = beta - tf::getYaw(estimate_pose->pose.pose.orientation);

		std::cout << "ex: "<< ex << " ";
		std::cout << "ey: "<< ey << " ";

		std::cout << "Rho: "<< rho << " ";
		std::cout << "Alpha: "<< alpha << " ";
		std::cout << "Beta: "<< beta << endl;

		std::cout << "X: "<< estimate_pose->pose.pose.position.x << " ";
		std::cout << "Y: "<< estimate_pose->pose.pose.position.y << " ";
		std::cout << "Th: "<< tf::getYaw(estimate_pose->pose.pose.orientation) << endl;

		geometry_msgs::Twist input; //to send the velocities

		//No hay que corregir angulo
		if(abs(alpha) < 0.1){
			input.linear.x = 0.5;
			input.angular.z = 0;
		}else{
			input.linear.x = 0;
			input.angular.z = 0.5;
		}
		//Hemos llegado al goal, parar.
		if(abs(Goal.pose.position.x - estimate_pose->pose.pose.position.x) < 0.1 
		&& abs(Goal.pose.position.y - estimate_pose->pose.pose.position.y) < 0.1 ){
			input.linear.x = 0;
			input.angular.z = 0;
		}
		//input.angular = alpha;
		//input.linear = rho;


    /*HAY QUE CHEKEAR EL LASER ANTES DE PUBLICAR LA VELOCIDAD*/

    //Se mira si los 3 rayos centrales intersectan con algo a menos de 15 unidades de distancia
    if(laser_scan->ranges[laser_scan->ranges.size()/2] < 2 &&  ((laser_scan->ranges[laser_scan->ranges.size()/2] - 1) < 2)
       && ((laser_scan->ranges[laser_scan->ranges.size()/2] + 1 )< 2)){
        ROS_INFO("OBSTACULO DETECTADO");
        input.linear.x = 0;
        input.angular.z = 0;
    }




		velocity_pub_.publish(input);

		}







    
    /*
    // Print first and last value of the laser scan
    ROS_INFO("First range data scan: %f",laser_scan->ranges[0]);
    ROS_INFO("Last range data scan: %f",laser_scan->ranges[laser_scan->ranges.size()-1]);



    

    for(int i=0;i<(laser_scan->ranges.size());i++){
      if(laser_scan->ranges[laser_scan->ranges.size()/2] < 15 &&  ((laser_scan->ranges[laser_scan->ranges.size()/2] - 1) < 15)
       && ((laser_scan->ranges[laser_scan->ranges.size()/2] + 1 )< 15)){
        ROS_INFO("OBSTACULO DETECTADO %d", i);
      }
    }
    */


    /*
     //Example loop over laser scan data
    for(int i=0;i<(laser_scan->ranges.size());i++){
      if(!(laser_scan->ranges.empty()))
         ROS_INFO("data %f",laser_scan->ranges[i]);
      else{
         ROS_INFO("NO VALUE");
      }
    } */
      
  }

 private:
  ros::NodeHandle nh_;
  message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> pose_sub_;
  ros::Subscriber goal_sub_;
  ros::Publisher velocity_pub_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry> MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;
  ros::Publisher laser_data_pub;
  geometry_msgs::PoseStamped Goal;

  bool noGoal; 

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "read_odom_scan");

  NodeSync synchronizer;

  ros::spin();
}












