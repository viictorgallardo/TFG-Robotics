#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <bits/stdc++.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

using namespace std;

class FollowTargetsClass {
	ros::NodeHandle nh_;
	ros::Publisher goal_pub_;
	ros::Subscriber position_sub_;
	geometry_msgs::PoseStamped Goal;
	std::vector<std::vector<float> > targets;
	ifstream inFile;
	int currentTarget; //index with the next target to reach
	int tamanyo;


public:
	FollowTargetsClass() { 
		//TODO: CAMBIAR LA RUTA, NO PUEDE SER ABSOLUTA SI NO NO COMPILA EN OTRO PC
		inFile.open("/home/arob/Desktop/TFG-Robotics/catkin_ws/src/arob_lab2/src/targets.txt");
		tamanyo = 0;
		cout << "Hola" << endl;
		string aux;
		string lineaX, lineaY;
		if (inFile.is_open()){
			cout << "OK" << endl;
		}else{
			cout << "NOT OK " << endl;
		}
		while(!inFile.eof()){
			getline(inFile,lineaX, ';');
			getline(inFile, lineaY, ';');
			cout << "Linea x es " << lineaX << endl;
			int x = stof(lineaX);
			int y = stof(lineaY);
			
			std::vector<float> auxVector;
			auxVector.push_back(x);
			auxVector.push_back(y);
			targets.push_back(auxVector);
			tamanyo++; // tamanyo en lineas del archivo(num de posis que hay)
		}

		inFile.close();
		currentTarget = 0;
		cout << "Hemos leido el fichero y targets tiene size " << targets.size() <<  endl;

		goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("goal", 1);
		//Se accede a la primera meta
		std::vector<float> target0 = targets[currentTarget];
		Goal.pose.position.x = target0[0];
		Goal.pose.position.y = target0[1];
	
	
		goal_pub_.publish(Goal);
	


		cout << "Goal x " << Goal.pose.position.x << endl;
		cout << "Goal y " << Goal.pose.position.y << endl;	
		//Cuando le llegue mensaje de odometria ejecutara positionCb
		//Se pone robot_0 para que coja al primer robot, ya que hay varios
		position_sub_ = nh_.subscribe("/robot_0/base_pose_ground_truth", 1, &FollowTargetsClass::positionCb, this);

		//Leo target

		//Publico

		//Me duermo hasta que sea target == odometria
//in the contructor you can read the targets from the text file
	}

	~FollowTargetsClass() {
	}

	//complete the class by adding the functio that you need

	void positionCb(const nav_msgs::Odometry& msg) {
		//Cada vez que llega una actualizacion de la odometria, miro si estoy demasiado cerca
		//y en ese caso se cambia la meta

		if(abs(Goal.pose.position.x - msg.pose.pose.position.x) < 0.1 ){
			//Se ha llegado ya
			if(currentTarget < tamanyo - 1){
				currentTarget++;
				std::vector<float> aux = targets[currentTarget];
				Goal.pose.position.x = aux[0];
				Goal.pose.position.y = aux[1];

				goal_pub_.publish(Goal);

				cout << "Goal x " << Goal.pose.position.x << endl;
				cout << "Goal y " << Goal.pose.position.y << endl;
			}
			
		}else{
			//No se hace nada
		}

	}



};


int main(int argc, char** argv) {


	ros::init(argc, argv, "followTargets");
	ros::NodeHandle nh("~");
	FollowTargetsClass FT;

	ros::spin();
	return 0;
}

