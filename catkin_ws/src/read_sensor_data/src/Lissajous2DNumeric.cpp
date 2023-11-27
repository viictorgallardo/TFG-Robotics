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


double normalizarAngulo(double angulo) {
    const double pi = M_PI; // Valor constante de π



    double theta_out = angulo;
    
    while (theta_out > pi) {
        theta_out -= 2 * pi;
    }
    
    while (theta_out < -pi) {
        theta_out += 2 * pi;
    }
    
    return theta_out;


    // Calcula el ángulo normalizado entre -pi y pi
    //double anguloNormalizado = fmod(angulo + pi, 2*pi) - pi;
   


  

    //if (anguloNormalizado < 0) {
     //   anguloNormalizado += dosPi; // Asegura que el ángulo esté en el rango [0, 2π]
    //}
    //cout << "Angulo normalizado " << anguloNormalizado << endl;
    //return anguloNormalizado;
}




//Calcula el valor de alpha(s) según la figura 19 del paper 
double calcularAlpha(double wVecino, double r, double R ){
    //cout  << "Wvecino:  "<< wVecino  << endl;
    if (wVecino > R){
        cout << "LIMITE SUPERIOR R PASADO" << endl;
        return 0;
    }else if(wVecino < r){
        cout << "LIMITE SUPERIOR r PASADO" << endl;
        return 1000000;
    }else if(wVecino > r && wVecino < R){
        return (1/(normalizarAngulo(wVecino -r)) - (1/(R-r)));
    }
}

/* //Calcula el modulo de 1 coordenada en 2d
double calcularModulo(double x, double y){
    return  sqrt( pow(x,2) +  pow(y,2));
}
 */



void signalHandler(int signum) {
    cout << " Señal capturada " << endl;
    exit(signum);
}

struct PosiRobot{
            double x;
            double y;
            double w;
};


//Se define un caso hardcodeado de trayectoria circular 2d para 3 robots
// Va a ir publicando los controladores(nuevas posiciones) en los topics goal para cada robot.
// Asi otro nodo irá moviendo los robots respectivamente. Si se calculan bien los controladores debería realizar una trayectoria circular.
//TODO: PROBAR CON 3 Y PARAMETRIZARLO
int main(int argc, char **argv)
{
    
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
    ros::Publisher goal_pub_2;

    /*// Inicializa un objeto RViz Visual Tools
    rviz_visual_tools::RvizVisualToolsPtr visual_tools;
    visual_tools.reset(new rviz_visual_tools::RvizVisualTools("base_link", "/visualization_marker"));
    visual_tools->loadMarkerPub();
*/
    geometry_msgs::PoseStamped Goal;

    ofstream trazasSalida("src/read_sensor_data/src/aux/salida_logs.txt");
    ofstream c("src/read_sensor_data/src/aux/coordenadasR0.txt");
    ofstream c1("src/read_sensor_data/src/aux/coordenadasR1.txt");
    ofstream c2("src/read_sensor_data/src/aux/coordenadasR2.txt");
    /*
    vector<ofstream> coordenadas;
    coordenadas.push_back(c);
    coordenadas.push_back(c1);
    coordenadas.push_back(c2);
    */



    goal_pub_0 = nh_.advertise<geometry_msgs::PoseStamped>("robot0/goal", 1);

    goal_pub_1 = nh_.advertise<geometry_msgs::PoseStamped>("robot1/goal", 1);

    goal_pub_2 = nh_.advertise<geometry_msgs::PoseStamped>("robot2/goal", 1);

    double T = 0.1; // 100 milisegundos
    double wTarget = 0; // w*
    double ki1 = 2; // gains 1 
    double ki2 = 2; // gains 2
    double kw = 1; // ganancia de la w para evitar que cambie mucho
    double ci = 2; // ganancia ci

    //Hay que diferenciar el publicador de cada robot

    vector<PosiRobot> posicionesRobots;
    int numRobots = 3;

    double radioCirculo = 1.6;

    posicionesRobots.push_back({-4,-4,0.5});
    posicionesRobots.push_back({-3.5,4,1});
    posicionesRobots.push_back({3.75,-3.75,1.5});
    
    double u1,u2,w0;

    //Derivadas obtenidas del paper
    double derivadaXi1 = -(8000*sin(wi)*(3*pow(sin(wi),2) + 6*pow(cos(wi),2) + 10))/(pow(3*pow(sin(wi),2) + 10,2));
    double derivadaXi2 = -(8000*(3*pow(sin(wi),4) + (3*pow(cos(wi),2) + 10)* pow(sin(wi),2) - 10*pow(cos(wi),2)))/(pow(3*pow(sin(wi),2) + 10,2));


    double wimenosj;

    int iter = 0;
    int r = 0.2;
    int R = 20;
    double mu = 0;
    while(iter <10000){
        iter++;

        //ROS_INFO("ITERACION %d",iter);
        trazasSalida << "ITERACION " << iter << " DEL BUCLE"   << endl;

        for(int i = 0; i < numRobots; i++){
                //Para robot0... mas tarde se hara con odometria
            u1 = derivadaXi1 - ki1 * posicionesRobots[i].x + ki1* radioCirculo* cos(posicionesRobots[i].w);
            u2 =  derivadaXi2 - ki2*posicionesRobots[i].y + ki2* radioCirculo* sin(posicionesRobots[i].w);


            mu = 0;
            //Vecindario para un robot
            for(int j = 0 ; j < numRobots; j++){
                if(i != j){
                    double wimenosj = posicionesRobots[i].w - posicionesRobots[j].w;
                    if(abs(wimenosj ) < R){
                        wimenosj = normalizarAngulo(wimenosj);
                        cout << "Valores : " << wimenosj << endl;
                        mu += calcularAlpha( abs(wimenosj),  r , R)* (wimenosj/abs(wimenosj));
                    }
                }
            }
        

            //Ahora mismo se hace sin el termino de la repulsion para ver si hacen rendezvous con w*
            w0 = 1 + ki1 * (posicionesRobots[i].x -  radioCirculo* cos(posicionesRobots[i].w)) * (- radioCirculo* sin(posicionesRobots[i].w))
                    + ki2 * (posicionesRobots[i].y -  radioCirculo* sin(posicionesRobots[i].w)) * ( radioCirculo* cos(posicionesRobots[i].w))
                    - (ci * (normalizarAngulo(posicionesRobots[i].w - wTarget)) + normalizarAngulo(mu*kw)); 

            

            posicionesRobots[i].x = posicionesRobots[i].x + T * u1;
            posicionesRobots[i].y = posicionesRobots[i].y + T * u2;
            posicionesRobots[i].w = normalizarAngulo(posicionesRobots[i].w + normalizarAngulo(T* w0));

            

            trazasSalida << "Posicion robot "<< i << " "<< posicionesRobots[i].x <<"   " <<  posicionesRobots[i].y
             << "   "  << posicionesRobots[i].w <<  "  wi - w* " 
            <<  normalizarAngulo(posicionesRobots[i].w - wTarget) << "   " <<   mu*kw << endl;

            if(i == 0){
                c << posicionesRobots[i].x << "," << posicionesRobots[i].y << "," <<
            posicionesRobots[i].w << "," << wTarget << endl;
            }else if( i == 1){
                c1 << posicionesRobots[i].x << "," << posicionesRobots[i].y << "," <<
            posicionesRobots[i].w << "," << wTarget << endl;
            }else if(i == 2){
                c2 << posicionesRobots[i].x << "," << posicionesRobots[i].y << "," <<
            posicionesRobots[i].w << "," << wTarget << endl;
            }
            

        }
        wTarget = normalizarAngulo(wTarget + (T * 1));
        sleep(0.1);
        
    }

    trazasSalida.close();
    c.close();
    c1.close();
    c2.close();
}











