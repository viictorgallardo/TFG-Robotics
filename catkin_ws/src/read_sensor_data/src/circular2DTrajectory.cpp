#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include "ros/ros.h"
#include <cmath>
#include <unistd.h>
#include "std_msgs/Bool.h"

#include <iostream>
#include <fstream>

#include <csignal>
#include <cstdlib>
//#include <rviz_visual_tools/rviz_visual_tools.h>


using namespace std;


class CircularTrajectory{
    public:
        CircularTrajectory(){
            numPedirSiguienteRecibidos = 0;

            trazasSalida.open("salida_logs.txt", ios::out);
            coordenadas.open("coordenadasR0.txt", ios::out);
            coordenadas1.open("coordenadasR1.txt", ios::out);
            coordenadas2.open("coordenadasR2.txt", ios::out);

            goal_pub_0 = nh_.advertise<geometry_msgs::PoseStamped>("robot_0/goal", 1);
            goal_pub_1 = nh_.advertise<geometry_msgs::PoseStamped>("robot_1/goal", 1);
            goal_pub_2 = nh_.advertise<geometry_msgs::PoseStamped>("robot_2/goal", 1);

            //TODO:Se podria usar solo uno y remapear en el launch
            pedirSiguienteGoal_sub0_ = nh_.subscribe("/robot_0/pedirSiguienteGoal", 1, &CircularTrajectory::pedirSiguienteGoal, this);
            pedirSiguienteGoal_sub1_ = nh_.subscribe("/robot_1/pedirSiguienteGoal", 1, &CircularTrajectory::pedirSiguienteGoal, this);
            pedirSiguienteGoal_sub2_ = nh_.subscribe("/robot_2/pedirSiguienteGoal", 1, &CircularTrajectory::pedirSiguienteGoal, this);

            
        }

        double normalizarAngulo(double angulo) {
            const double pi = M_PI; // Valor constante de π

            // Calcula el ángulo normalizado entre -pi y pi
            double anguloNormalizado = fmod(angulo + pi, 2*pi) - pi;
        


            //if (anguloNormalizado < 0) {
            //   anguloNormalizado += dosPi; // Asegura que el ángulo esté en el rango [0, 2π]
            //}
            //cout << "Angulo normalizado " << anguloNormalizado << endl;
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



        ~CircularTrajectory() {
            trazasSalida.close();
            coordenadas.close();
            coordenadas1.close();
            coordenadas2.close();
    }


    protected:

        

        void pedirSiguienteGoal(const std_msgs::Bool msg){
            
            ROS_INFO("He recibido un pedir siguiente");
            
            //Si el robot 0 quiere una nueva meta se calcula...
            if(msg.data == true){

                numPedirSiguienteRecibidos++;
                //Los 3 robots han llegado a sus metas, asignar nuevas
                if(numPedirSiguienteRecibidos == 3){
                    numPedirSiguienteRecibidos = 0;
                    //Para robot0... mas tarde se hara con odometria
                    u01 = - 1.6*  sin(posRobot0[2]) - ki1 * posRobot0[0] + ki1* 1.6* cos(posRobot0[2]);
                    u02 =  1.6* cos(posRobot0[2]) - ki2*posRobot0[1] + ki2* 1.6* sin(posRobot0[2]);

                    //Necesitamos la diferencia de angulos entre el robot y sus vecinos
                    w0menos1 = normalizarAngulo(posRobot0[2] - posRobot1[2]);
                    w0menos2 = normalizarAngulo(posRobot0[2] - posRobot2[2]);
                    // Sumatorio de todos los vecinos r1 y r2
                    mu0 =  calcularAlpha( abs(w0menos1),  r , R)* (w0menos1/abs(w0menos1))
                            + calcularAlpha(abs(w0menos2), r , R)* (w0menos2/abs(w0menos2));

                    //Ahora mismo se hace sin el termino de la repulsion para ver si hacen rendezvous con w*
                    w0 = 1 + ki1 * (posRobot0[0] -  1.6* cos(posRobot0[2])) * (- 1.6* sin(posRobot0[2]))
                            + ki2 * (posRobot0[1] -  1.6* sin(posRobot0[2])) * ( 1.6* cos(posRobot0[2]))
                            - (ci * (normalizarAngulo(posRobot0[2] - wTarget)) * kw )+ mu0*kw; 

                    

                    posRobot0[0] = posRobot0[0] + T * u01;
                    posRobot0[1] = posRobot0[1] + T * u02;
                    posRobot0[2] = normalizarAngulo(posRobot0[2] + normalizarAngulo(T* w0));

                    ROS_INFO("POS ROBOT 0 %f , %f , %f" , posRobot0[0], posRobot0[1], posRobot0[2]);

                    trazasSalida << "Posicion robot 0 " << posRobot0[0] << "   " <<  posRobot0[1] << "   " << posRobot0[2] << "  wi - w* " <<  normalizarAngulo(posRobot0[2] - wTarget)  << endl;

                    // ¿Que valor se le da a wtarget?

                    
            
                    // poner velocidad

                    Goal.pose.position.x = posRobot0[0];
                    Goal.pose.position.y = posRobot0[1];

                    goal_pub_0.publish(Goal);


                    //Vamos a escribir la trayectoria del robot 0
                    coordenadas << posRobot0[0] << "," << posRobot0[1] << "," << posRobot0[2] << "," << wTarget << endl;


                    //Para robot1... mas tarde se hara con odometria
                    u11 = - 1.6* sin(posRobot1[2]) - ki1 * posRobot1[0] + ki1* 1.6* cos(posRobot1[2]);
                    u12 =  1.6*cos(posRobot1[2]) - ki2*posRobot1[1] + ki2* 1.6* sin(posRobot1[2]);



                    //Necesitamos la diferencia de angulos entre el robot y sus vecinos
                    w1menos0 = normalizarAngulo(posRobot1[2] - posRobot0[2]);
                    w1menos2 = normalizarAngulo(posRobot1[2] - posRobot2[2]);


                    // Sumatorio de todos los vecinos r0 y r2
                    mu1 =  calcularAlpha(abs(w1menos0), r , R)* (w1menos0/abs(w1menos0)) +
                            calcularAlpha(abs(w1menos2), r , R)* w1menos2/abs(w1menos2);

                    //Ahora mismo se hace sin el termino de la repulsion para ver si hacen rendezvous con w*
                    w1 = 1 + ki1 * (posRobot1[0] -  1.6* cos(posRobot1[2])) * (- 1.6* sin(posRobot1[2]))
                            + ki2 * (posRobot1[1] -  1.6* sin(posRobot1[2])) * ( 1.6* cos(posRobot1[2]))
                            - (ci * (normalizarAngulo(posRobot1[2] - wTarget)) * kw) + mu1*kw;

                    // ¿Que valor se le da a wtarget?

                    //ROS_INFO("VALOR DEL PRIMER OPERANDO %f" , ki1 * (posRobot1[0] -  1.6* cos(posRobot1[2])) * (- 1.6* sin(posRobot1[2])));
                    //ROS_INFO("VALOR DEL SEGUNDO OPERANDO %f",  ki2 * (posRobot1[1] -  1.6* sin(posRobot1[2])) * ( 1.6* cos(posRobot1[2])));
                    //ROS_INFO("VALOR DEL TERCER OPERANDO %f" , ci * (posRobot1[2] - wTarget));
                    //ROS_INFO("VALOR DE MU %f" , mu1);

                    posRobot1[0] = posRobot1[0] + T * u11;
                    posRobot1[1] = posRobot1[1] + T * u12;
                    posRobot1[2] = normalizarAngulo(posRobot1[2] + normalizarAngulo(T* w1));

                    ROS_INFO("POS ROBOT 1 %f , %f , %f" , posRobot1[0], posRobot1[1], posRobot1[2]);
            
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
                    u21 = - 1.6* sin(posRobot2[2]) - ki1 * posRobot2[0] + ki1* 1.6* cos(posRobot2[2]);
                    u22 =  1.6*cos(posRobot2[2]) - ki2*posRobot2[1] + ki2* 1.6* sin(posRobot2[2]);



                    //Necesitamos la diferencia de angulos entre el robot y sus vecinos
                    w2menos0 = normalizarAngulo(posRobot2[2] - posRobot0[2]);
                    w2menos1 = normalizarAngulo(posRobot2[2] - posRobot1[2]);


                    // Sumatorio de todos los vecinos
                    mu2 =  calcularAlpha(abs(w2menos0), r , R)* w2menos0/(abs(w2menos0))
                            + calcularAlpha(abs(w2menos1), r , R)* w2menos1/(abs(w2menos1));

                    //Ahora mismo se hace sin el termino de la repulsion para ver si hacen rendezvous con w*
                    w2 = 1 + ki1 * (posRobot2[0] -  1.6* cos(posRobot2[2])) * (- 1.6* sin(posRobot2[2]))
                            + ki2 * (posRobot2[1] -  1.6* sin(posRobot2[2])) * ( 1.6* cos(posRobot2[2]))
                            - (ci * (normalizarAngulo(posRobot2[2] - wTarget)) * kw) + mu2*kw;

                    // ¿Que valor se le da a wtarget?

                    posRobot2[0] = posRobot2[0] + T * u21;
                    posRobot2[1] = posRobot2[1] + T * u22;
                    posRobot2[2] = normalizarAngulo(posRobot2[2] + normalizarAngulo(T* w2));

                    //Vamos a escribir la trayectoria del robot 2
                    coordenadas2 << posRobot2[0] << "," << posRobot2[1] << "," << posRobot2[2] << "," << wTarget << endl;

                    ROS_INFO("POS ROBOT 2 %f , %f , %f" , posRobot2[0], posRobot2[1], posRobot2[2]);

                    trazasSalida << "Posicion robot 2 " << posRobot2[0] <<"   " <<  posRobot2[1] << "   "  << posRobot2[2] <<  "  wi - w* " <<  normalizarAngulo(posRobot2[2] - wTarget) << endl;
            
                    // poner velocidad

                    Goal.pose.position.x = posRobot2[0];
                    Goal.pose.position.y = posRobot2[1];

                    goal_pub_2.publish(Goal);


                    //Se actualiza la w virtual segun su derivada ( mirar paper )
                    // Se normaliza para que no salga de la esfera, es el angulo con el que deberia ir el platooning
                    wTarget = normalizarAngulo(wTarget + (T * 1) * kw);

                    sleep(0.1);
                    
                }
                        
                
            }
        }


    private:

        ros::NodeHandle nh_;


        ros::Publisher goal_pub_0;
        ros::Publisher goal_pub_1;
        ros::Publisher goal_pub_2;

        ros::Subscriber pedirSiguienteGoal_sub0_;
        ros::Subscriber pedirSiguienteGoal_sub1_;
        ros::Subscriber pedirSiguienteGoal_sub2_;

        geometry_msgs::PoseStamped Goal;

        int numPedirSiguienteRecibidos;

        ofstream trazasSalida;
        ofstream coordenadas;
        ofstream coordenadas1;
        ofstream coordenadas2;


        double T = 0.1; // 100 milisegundos
        double wTarget = 0; // w*
        double ki1 = 3.5; // gains 1 
        double ki2 = 3.5; // gains 2
        double kw = 0.01; // ganancia de la w para evitar que cambie mucho
        double ci = 2; // ganancia ci

        //Hay que diferenciar el publicador de cada robot
        
        double posRobot0[3] = {-4 , -4, 0.5}; // x0,1   ,   x0,2 ,  w0
        double posRobot1[3] = {-3.5 , 4, 1};
        double posRobot2[3] = {3.75 , -3.75, 1.5};
        double u01,u02,mu0,w0;
        double u11, u12, mu1, w1;
        double u21, u22, mu2, w2;
        double w0menos1, w0menos2, w1menos0, w1menos2, w2menos0, w2menos1;

        int i = 0;
        int r = 0.2;
        int R = 20;

};


//Se define un caso hardcodeado de trayectoria circular 2d para 3 robots
// Va a ir publicando los controladores(nuevas posiciones) en los topics goal para cada robot.
// Asi otro nodo irá moviendo los robots respectivamente. Si se calculan bien los controladores debería realizar una trayectoria circular.
//TODO: PROBAR CON 3 Y PARAMETRIZARLO
int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "circularTrajectory");

    CircularTrajectory synchronizer;

    ros::spin();

}












