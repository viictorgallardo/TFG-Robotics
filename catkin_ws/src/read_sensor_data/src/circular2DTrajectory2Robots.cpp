#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include "ros/ros.h"
#include <cmath>
#include <unistd.h>

#include <iostream>
#include <fstream>

#include <csignal>
#include <cstdlib>
//#include <rviz_visual_tools/rviz_visual_tools.h>


using namespace std;



//Calcula el valor de alpha(s) según la figura 19 del paper 
double calcularAlpha(double wVecino, double r, double R ){
    cout  << "Wvecino:  "<< wVecino  << endl;
    if (wVecino > R){
        return 0;
    }else if(wVecino < R){
        return 1000000;
    }else if(wVecino > r && wVecino < R){
        return (1/(wVecino -r)) - (1/(R-r));
    }
}

//Calcula el modulo de 1 coordenada en 2d
double calcularModulo(double x, double y){
    return  sqrt( pow(x,2) +  pow(y,2));
}




void signalHandler(int signum) {
    cout << " Señal capturada " << endl;
    exit(signum);
}


//Se define un caso hardcodeado de trayectoria circular 2d para 2 robots
// Va a ir publicando los controladores(nuevas posiciones) en los topics goal para cada robot.
// Asi otro nodo irá moviendo los robots respectivamente. Si se calculan bien los controladores debería realizar una trayectoria circular.
//TODO: PROBAR CON 3 Y PARAMETRIZARLO
int main(int argc, char **argv)
{
    
    ofstream archivo("coordenadas.txt");
    ofstream archivo1("coordenadas1.txt");
    if (!archivo.is_open() || !archivo1.is_open()) {
        cerr << "Error al abrir el archivo" << endl;
        return 1; // Sal del programa si no se pueden abrir los archivos
    }

    /* signal(SIGINT, signalHandler);
    signal(SIGTSTP, signalHandler);
    if (!archivo.is_open()){
        cerr << "Error al abrir el archivo" << endl;
    } */

    ros::init(argc, argv, "read_odom_scan");


    ros::NodeHandle nh_;
    message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> pose_sub_;

    ros::Publisher velocity_pub_;

    ros::Publisher goal_pub_0;
    ros::Publisher goal_pub_1;

    /*// Inicializa un objeto RViz Visual Tools
    rviz_visual_tools::RvizVisualToolsPtr visual_tools;
    visual_tools.reset(new rviz_visual_tools::RvizVisualTools("base_link", "/visualization_marker"));
    visual_tools->loadMarkerPub();
*/
    geometry_msgs::PoseStamped Goal;



    goal_pub_0 = nh_.advertise<geometry_msgs::PoseStamped>("robot0/goal", 1);

    goal_pub_1 = nh_.advertise<geometry_msgs::PoseStamped>("robot1/goal", 1);

    double T = 0.1; // 100 milisegundos
    double wTarget = 0; // w*
    double ki1 = 3.5; // gains 1 
    double ki2 = 3.5; // gains 2
    double ci = 2; // 

    //Hay que diferenciar el publicador de cada robot
    
    double posRobot0[3] = {0 , 0, 0}; // x0,1   ,   x0,2 ,  w0
    double posRobot1[3] = {10 , 10, 1};
    double u01,u02,mu0,w0;
    double u11, u12, mu1, w1;
    int i = 0;
    int r = 1;
    int R = 100;
    while(i < 1000){
        i++;
        cout << "ITERACION " << i << " DEL BUCLE"   << endl;
        // ... calcular controladores

        //Para robot0... mas tarde se hara con odometria
        u01 = -800*  sin(posRobot0[2]) - ki1 * posRobot0[0] + ki1*800* cos(posRobot0[2]);
        u02 = 800* cos(posRobot0[2]) - ki2*posRobot0[1] + ki2*800* sin(posRobot0[2]);

        // Sumatorio de todos los vecinos
        mu0 =  calcularAlpha(posRobot1[2], r , R)* posRobot1[2]/calcularModulo(posRobot1[0], posRobot1[1]);

        w0 = 1 + ki1 * (posRobot0[0] - 800* cos(posRobot0[2])) * (-800* sin(posRobot0[2]))
                + ki2 * (posRobot0[1] - 800* sin(posRobot0[2])) * (800* cos(posRobot0[2]))
                - ci * (posRobot0[2] - wTarget) + mu0;

        

        posRobot0[0] = posRobot0[0] + T * u01;
        posRobot0[1] = posRobot0[1] + T * u02;
        posRobot0[2] = posRobot0[2] + T* w0;

        ROS_INFO("POS ROBOT 0 %f , %f , %f" , posRobot0[0], posRobot0[1], posRobot0[2]);

        // ¿Que valor se le da a wtarget?

        wTarget = posRobot0[2];
 
        // poner velocidad

        Goal.pose.position.x = posRobot0[0];
		Goal.pose.position.y = posRobot0[1];

        goal_pub_0.publish(Goal);


        //Vamos a escribir la trayectoria del robot 0
        //archivo << posRobot0[0] << "," << posRobot0[1] << "," << posRobot0[2] << "," << wTarget << endl;


        //Para robot1... mas tarde se hara con odometria
        u11 = -800* sin(posRobot1[2]) - ki1 * posRobot1[0] + ki1*800* cos(posRobot1[2]);
        u12 = 800*cos(posRobot1[2]) - ki2*posRobot1[1] + ki2*800* sin(posRobot1[2]);

        // Sumatorio de todos los vecinos
        mu1 =  calcularAlpha(posRobot0[2], r , R)* posRobot0[2]/calcularModulo(posRobot0[0], posRobot0[1]);

        w1 = 1 + ki1 * (posRobot1[0] - 800* cos(posRobot1[2])) * (-800* sin(posRobot1[2]))
                + ki2 * (posRobot1[1] - 800* sin(posRobot1[2])) * (800* cos(posRobot1[2]))
                - ci * (posRobot1[2] - wTarget) + mu1;

        // ¿Que valor se le da a wtarget?

        posRobot1[0] = posRobot1[0] + T * u11;
        posRobot1[1] = posRobot1[1] + T * u12;
        posRobot1[2] = posRobot1[2] + T* w1;

        ROS_INFO("POS ROBOT 1 %f , %f , %f" , posRobot1[0], posRobot1[1], posRobot1[2]);
 
        // poner velocidad

        Goal.pose.position.x = posRobot1[0];
		Goal.pose.position.y = posRobot1[1];

        goal_pub_1.publish(Goal);

        //Vamos a escribir la trayectoria del robot 1
        //archivo1 << posRobot1[0] << "," << posRobot1[1] << "," << posRobot1[2] << "," << wTarget << endl;


        // Visualiza las coordenadas como un marcador en RViz
        //visual_tools->publishSphere(Eigen::Affine3d(Eigen::Translation3d(posRobot0[0], posRobot0[1], posRobot0[2])), rviz_visual_tools::colors::GREEN, rviz_visual_tools::LARGE);
        //visual_tools->trigger();

        //simula el periodo cada cuanto se controla
        sleep(100);
        T += 100;
        
    }
}