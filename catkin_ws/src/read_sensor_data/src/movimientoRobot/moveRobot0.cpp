#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include "ros/ros.h"
#include "std_msgs/Bool.h"


using namespace std;

class NodeSync
{
 public:
  NodeSync()
  {
    noGoal = true;
    //Estos dos no se ponen a robot_0 porque ya se remapearon en el start.launch
    scan_sub_.subscribe(nh_, "/scan", 1);
    pose_sub_.subscribe(nh_, "/pose", 1);
    goal_sub_ = nh_.subscribe("/robot_0/goal", 1, &NodeSync::goalCb, this);

    //Canal que manda si cada robot quiere una nueva meta( True = si quiere False = En caso contrario)
    //Al principio siempre quiere una nueva meta
    
    pedirSiguienteGoal_pub = nh_.advertise<std_msgs::Bool>("/pedirSiguienteGoal", 1, true);

    primerMensaje = true;
    ros::Duration(0.5).sleep();

    

    //Tampoco se pone robot0 ya esta remapeado
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
    //ROS_INFO("Synchronization successful");
    if(primerMensaje == true){
      //Se mandan N mensajes siendo N el num de robots para pedir la primera meta.
      //Esto se hace hasta que nos responda el circularTrajectory, despues solo se hara una vez
      //alcancemos la meta
      primerMensaje = false;
      std_msgs::Bool msg;
      msg.data = true; // Aquí asignas True si quieres recibir un nuevo valor de meta
      pedirSiguienteGoal_pub.publish(msg);
      ROS_INFO("Se ha publicado el mensaje");
    }
    
      
    
    


    // Get position info
    tf::Quaternion q(	estimate_pose->pose.pose.orientation.x, 
    			estimate_pose->pose.pose.orientation.y, 
    			estimate_pose->pose.pose.orientation.z, 
    			estimate_pose->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    //ROS_INFO("Robot Pose x:%f ; y:%f ; th:%f ;", estimate_pose->pose.pose.position.x,estimate_pose->pose.pose.position.y,yaw);


    /*BUCLE DE CONTROL DE VELOCIDADES*/


		if(!noGoal){
			//std::cout << "meta x: "<< Goal.pose.position.x << " ";
		//std::cout << "meta y: "<< Goal.pose.position.y << " ";
		/*
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
  */


        // Calculating the difference angle using atan2
    float ex = Goal.pose.position.x - estimate_pose->pose.pose.position.x;
    float ey = Goal.pose.position.y - estimate_pose->pose.pose.position.y;
    float beta = atan2(ey, ex);

    // Get the current yaw of the robot
    float current_yaw = tf::getYaw(q);

    // Calculate the difference in yaw (alpha)
    float alpha = beta- current_yaw;

    // Ensure alpha is within the range -π to π
    if (alpha > M_PI) {
        alpha -= 2 * M_PI;
    } else if (alpha < -M_PI) {
        alpha += 2 * M_PI;
    }
		geometry_msgs::Twist input; //to send the velocities

    float tolerance = 0.1;  // Tolerancia para detener el giro cuando esté cerca del objetivo

    // Comprueba si alpha indica que el objetivo está a la izquierda o a la derecha
    if (alpha > tolerance) {
        // Gira a la izquierda
        // Asigna una velocidad angular positiva para girar a la izquierda
        // Utiliza angular_speed como la velocidad angular que necesitas
        input.linear.x = 0;
        input.angular.z = 0.5;

    } else if (alpha < -tolerance) {
        // Gira a la derecha
        // Asigna una velocidad angular negativa para girar a la derecha
        // Utiliza -angular_speed como la velocidad angular que necesitas

        input.linear.x = 0;
        input.angular.z = -0.5;
    } else {
      input.linear.x = 0.5;
			input.angular.z = 0;

      for(int laser : laser_scan->ranges){
      //Se mira si los 3 rayos centrales intersectan con algo a menos de 15 unidades de distancia
        if(laser < 0.1){
            ROS_INFO("OBSTACULO DETECTADO");
            input.linear.x = 0;
            input.angular.z = 0;
        }
      }
    }
		//Hemos llegado al goal, parar.
		if(abs(Goal.pose.position.x - estimate_pose->pose.pose.position.x) < 0.1 
		&& abs(Goal.pose.position.y - estimate_pose->pose.pose.position.y) < 0.1 ){
      noGoal = true;
			input.linear.x = 0;
			input.angular.z = 0;
      std_msgs::Bool msg;
      msg.data = true; // Aquí asignas True si quieres recibir un nuevo valor de meta
      pedirSiguienteGoal_pub.publish(msg);
		}
		//input.angular = alpha;
		//input.linear = rho;


    /*HAY QUE CHEKEAR EL LASER ANTES DE PUBLICAR LA VELOCIDAD*/

    

    




		velocity_pub_.publish(input);
    anteriorAlpha = alpha;

		}
      
  }

 private:
  ros::NodeHandle nh_;
  message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> pose_sub_;
  ros::Subscriber goal_sub_;
  ros::Publisher velocity_pub_;
  ros::Publisher pedirSiguienteGoal_pub;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry> MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;
  ros::Publisher laser_data_pub;
  geometry_msgs::PoseStamped Goal;

  bool noGoal; 
  bool primerMensaje;


  //Angulo antiguo para mejorar el giro si falla
  double anteriorAlpha; 

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "read_odom_scan");

  NodeSync synchronizer;

  ros::spin();
}













