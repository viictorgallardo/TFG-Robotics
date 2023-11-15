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

    // Calcula el ángulo normalizado entre -pi y pi
    double anguloNormalizado = fmod(angulo + pi, 2*pi) - pi;
   


    //if (anguloNormalizado < 0) {
     //   anguloNormalizado += dosPi; // Asegura que el ángulo esté en el rango [0, 2π]
    //}
    cout << "Angulo normalizado " << anguloNormalizado << endl;
    return anguloNormalizado;
}




//Calcula el valor de alpha(s) según la figura 19 del paper 
double calcularAlpha(double wVecino, double r, double R ){
    cout  << "Wvecino:  "<< wVecino  << endl;
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



//Se define un caso hardcodeado de trayectoria circular 2d para 3 robots
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
    ros::Publisher goal_pub_2;

    /*// Inicializa un objeto RViz Visual Tools
    rviz_visual_tools::RvizVisualToolsPtr visual_tools;
    visual_tools.reset(new rviz_visual_tools::RvizVisualTools("base_link", "/visualization_marker"));
    visual_tools->loadMarkerPub();
*/
    geometry_msgs::PoseStamped Goal;

    ofstream trazasSalida("salida_logs.txt");
    ofstream coordenadas("coordenadasR0.txt");
    ofstream coordenadas1("coordenadasR1.txt");
    ofstream coordenadas2("coordenadasR2.txt");



    goal_pub_0 = nh_.advertise<geometry_msgs::PoseStamped>("robot0/goal", 1);

    goal_pub_1 = nh_.advertise<geometry_msgs::PoseStamped>("robot1/goal", 1);

    goal_pub_2 = nh_.advertise<geometry_msgs::PoseStamped>("robot2/goal", 1);

    double T = 0.001; // 100 milisegundos
    double wTarget = 0; // w*
    double ki1 = 3.5; // gains 1 
    double ki2 = 3.5; // gains 2
    double kw = 0.01; // ganancia de la w para evitar que cambie mucho
    double ci = 2; // ganancia ci

    //Hay que diferenciar el publicador de cada robot
    
    double posRobot0[3] = {-0.8*cos((0*M_PI)/180) , -0.8*sin((0*M_PI)/180),(0*M_PI)/180 }; // x0,1   ,   x0,2 ,  w0
    double posRobot1[3] = {-0.8*cos((45*M_PI)/180) , -0.8*sin((45*M_PI)/180), (45*M_PI)/180};
    double posRobot2[3] = {-0.8*cos((-45*M_PI)/180) , -0.8*sin((-45*M_PI)/180), (-45*M_PI)/180};
    double u01,u02,mu0,w0;
    double u11, u12, mu1, w1;
    double u21, u22, mu2, w2;
    double w0menos1, w0menos2, w1menos0, w1menos2, w2menos0, w2menos1;

    int i = 0;
    int r = 0.2;
    int R = 20;
    while(i < 400){
        i++;
        //cout << "ITERACION " << i << " DEL BUCLE"   << endl;

        trazasSalida << "ITERACION " << i << " DEL BUCLE"   << endl;
        // ... calcular controladores

        //Para robot0... mas tarde se hara con odometria
        u01 = - 0.8*  sin(posRobot0[2]) - ki1 * posRobot0[0] + ki1* 0.8* cos(posRobot0[2]);
        u02 =  0.8* cos(posRobot0[2]) - ki2*posRobot0[1] + ki2* 0.8* sin(posRobot0[2]);

        //Necesitamos la diferencia de angulos entre el robot y sus vecinos
        w0menos1 = normalizarAngulo(posRobot0[2] - posRobot1[2]);
        w0menos2 = normalizarAngulo(posRobot0[2] - posRobot2[2]);
        // Sumatorio de todos los vecinos r1 y r2
        mu0 =  calcularAlpha( abs(w0menos1),  r , R)* (w0menos1/abs(w0menos1))
                + calcularAlpha(abs(w0menos2), r , R)* (w0menos2/abs(w0menos2));

        //Ahora mismo se hace sin el termino de la repulsion para ver si hacen rendezvous con w*
        w0 = 1 + ki1 * (posRobot0[0] -  0.8* cos(posRobot0[2])) * (- 0.8* sin(posRobot0[2]))
                + ki2 * (posRobot0[1] -  0.8* sin(posRobot0[2])) * ( 0.8* cos(posRobot0[2]))
                - (ci * (normalizarAngulo(posRobot0[2] - wTarget)) * kw )+ mu0*kw; 

        //ROS_INFO("VALOR DEL PRIMER OPERANDO %f" , ki1 * (posRobot0[0] -  0.8* cos(posRobot0[2])) * (- 0.8* sin(posRobot0[2])));
        //ROS_INFO("Ganancia ki1 %f", ki1);
        //ROS_INFO("x de robot 0 %f", posRobot0[0]);
        //double aux =  0.8* cos(posRobot0[2]);
        //double aux1 =  0.8* sin(posRobot0[2]);
        //ROS_INFO("Tercer termino %f" , aux);
        //ROS_INFO("cuarto termino %f", aux1);
        //ROS_INFO("VALOR DEL SEGUNDO OPERANDO %f",  ki2 * (posRobot0[1] -  0.8* sin(posRobot0[2])) * ( 0.8* cos(posRobot0[2])));
        //ROS_INFO("VALOR DEL TERCER OPERANDO %f" , ci * (posRobot0[2] - wTarget));
        //ROS_INFO("VALOR DE MU %f" , mu0);

        

        posRobot0[0] = posRobot0[0] + T * u01;
        posRobot0[1] = posRobot0[1] + T * u02;
        posRobot0[2] = normalizarAngulo(posRobot0[2] + normalizarAngulo(T* w0));

        //ROS_INFO("POS ROBOT 0 %f , %f , %f" , posRobot0[0], posRobot0[1], posRobot0[2]);

        trazasSalida << "Posicion robot 0 " << posRobot0[0] << "   " <<  posRobot0[1] << "   " << posRobot0[2] << "  wi - w* " <<  normalizarAngulo(posRobot0[2] - wTarget)  << endl;

        // ¿Que valor se le da a wtarget?

        
 
        // poner velocidad

        Goal.pose.position.x = posRobot0[0];
		Goal.pose.position.y = posRobot0[1];

        goal_pub_0.publish(Goal);


        //Vamos a escribir la trayectoria del robot 0
        coordenadas << posRobot0[0] << "," << posRobot0[1] << "," << posRobot0[2] << "," << wTarget << endl;


        //Para robot1... mas tarde se hara con odometria
        u11 = - 0.8* sin(posRobot1[2]) - ki1 * posRobot1[0] + ki1* 0.8* cos(posRobot1[2]);
        u12 =  0.8*cos(posRobot1[2]) - ki2*posRobot1[1] + ki2* 0.8* sin(posRobot1[2]);



        //Necesitamos la diferencia de angulos entre el robot y sus vecinos
        w1menos0 = normalizarAngulo(posRobot1[2] - posRobot0[2]);
        w1menos2 = normalizarAngulo(posRobot1[2] - posRobot2[2]);


        // Sumatorio de todos los vecinos r0 y r2
        mu1 =  calcularAlpha(abs(w1menos0), r , R)* (w1menos0/abs(w1menos0)) +
                calcularAlpha(abs(w1menos2), r , R)* w1menos2/abs(w1menos2);

        //Ahora mismo se hace sin el termino de la repulsion para ver si hacen rendezvous con w*
        w1 = 1 + ki1 * (posRobot1[0] -  0.8* cos(posRobot1[2])) * (- 0.8* sin(posRobot1[2]))
                + ki2 * (posRobot1[1] -  0.8* sin(posRobot1[2])) * ( 0.8* cos(posRobot1[2]))
                - (ci * (normalizarAngulo(posRobot1[2] - wTarget)) * kw) + mu1*kw;

        // ¿Que valor se le da a wtarget?

        //ROS_INFO("VALOR DEL PRIMER OPERANDO %f" , ki1 * (posRobot1[0] -  0.8* cos(posRobot1[2])) * (- 0.8* sin(posRobot1[2])));
        //ROS_INFO("VALOR DEL SEGUNDO OPERANDO %f",  ki2 * (posRobot1[1] -  0.8* sin(posRobot1[2])) * ( 0.8* cos(posRobot1[2])));
        //ROS_INFO("VALOR DEL TERCER OPERANDO %f" , ci * (posRobot1[2] - wTarget));
        //ROS_INFO("VALOR DE MU %f" , mu1);

        posRobot1[0] = posRobot1[0] + T * u11;
        posRobot1[1] = posRobot1[1] + T * u12;
        posRobot1[2] = normalizarAngulo(posRobot1[2] + normalizarAngulo(T* w1));

        //ROS_INFO("POS ROBOT 1 %f , %f , %f" , posRobot1[0], posRobot1[1], posRobot1[2]);
 
        // poner velocidad


         //Vamos a escribir la trayectoria del robot 1
        coordenadas1 << posRobot1[0] << "," << posRobot1[1] << "," << posRobot1[2] << "," << wTarget << endl;


        trazasSalida << "Posicion robot 1 " << posRobot1[0] << "   " << posRobot1[1] << "   " <<  posRobot1[2] <<  "  wi - w* " <<  normalizarAngulo(posRobot1[2] - wTarget) <<  endl;

        Goal.pose.position.x = posRobot1[0];
		Goal.pose.position.y = posRobot1[1];

        goal_pub_1.publish(Goal);

        //Vamos a escribir la trayectoria del robot 1
        //archivo1 << posRobot1[0] << "," << posRobot1[1] << "," << posRobot1[2] << "," << wTarget << endl;


        // Visualiza las coordenadas como un marcador en RViz
        //visual_tools->publishSphere(Eigen::Affine3d(Eigen::Translation3d(posRobot0[0], posRobot0[1], posRobot0[2])), rviz_visual_tools::colors::GREEN, rviz_visual_tools::LARGE);
        //visual_tools->trigger();

        //simula el periodo cada cuanto se controla



        //Para robot2... mas tarde se hara con odometria
        u21 = - 0.8* sin(posRobot2[2]) - ki1 * posRobot2[0] + ki1* 0.8* cos(posRobot2[2]);
        u22 =  0.8*cos(posRobot2[2]) - ki2*posRobot2[1] + ki2* 0.8* sin(posRobot2[2]);



        //Necesitamos la diferencia de angulos entre el robot y sus vecinos
        w2menos0 = normalizarAngulo(posRobot2[2] - posRobot0[2]);
        w2menos1 = normalizarAngulo(posRobot2[2] - posRobot1[2]);


        // Sumatorio de todos los vecinos
        mu2 =  calcularAlpha(abs(w2menos0), r , R)* w2menos0/(abs(w2menos0))
                + calcularAlpha(abs(w2menos1), r , R)* w2menos1/(abs(w2menos1));

        //Ahora mismo se hace sin el termino de la repulsion para ver si hacen rendezvous con w*
        w2 = 1 + ki1 * (posRobot2[0] -  0.8* cos(posRobot2[2])) * (- 0.8* sin(posRobot2[2]))
                + ki2 * (posRobot2[1] -  0.8* sin(posRobot2[2])) * ( 0.8* cos(posRobot2[2]))
                - (ci * (normalizarAngulo(posRobot2[2] - wTarget)) * kw) + mu2*kw;

        // ¿Que valor se le da a wtarget?

        posRobot2[0] = posRobot2[0] + T * u21;
        posRobot2[1] = posRobot2[1] + T * u22;
        posRobot2[2] = normalizarAngulo(posRobot2[2] + normalizarAngulo(T* w2));

         //Vamos a escribir la trayectoria del robot 2
        coordenadas2 << posRobot2[0] << "," << posRobot2[1] << "," << posRobot2[2] << "," << wTarget << endl;

        //ROS_INFO("POS ROBOT 2 %f , %f , %f" , posRobot2[0], posRobot2[1], posRobot2[2]);

        trazasSalida << "Posicion robot 2 " << posRobot2[0] <<"   " <<  posRobot2[1] << "   "  << posRobot2[2] <<  "  wi - w* " <<  normalizarAngulo(posRobot2[2] - wTarget) << endl;
 
        // poner velocidad

        Goal.pose.position.x = posRobot2[0];
		Goal.pose.position.y = posRobot2[1];

        goal_pub_2.publish(Goal);


        //Se actualiza la w virtual segun su derivada ( mirar paper )
        // Se normaliza para que no salga de la esfera, es el angulo con el que deberia ir el platooning
        wTarget = normalizarAngulo(wTarget + (T * 1) * kw);

        sleep(0.001);
        T += 0.001;
        
    }

    trazasSalida.close();
    coordenadas.close();
    coordenadas1.close();
    coordenadas2.close();
}











