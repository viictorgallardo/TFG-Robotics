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

#include <vector>

#include <iostream>
#include <fstream>
#include <string>

#include <csignal>
#include <cstdlib>
//#include <rviz_visual_tools/rviz_visual_tools.h>


using namespace std;


class CircularTrajectory{
    public:
        //Constructor que utiliza el nodo para comunicarse con los robots
        CircularTrajectory(){

        
            posicionesRobots.push_back({-4,-4,0.5});
            posicionesRobots.push_back({-3.5,4,1});
            posicionesRobots.push_back({2,3.75,1.5});
            

            numRobots = 3;

            radioCirculo = 2;

            numPedirSiguienteRecibidos = 0;

            goal_pub_0 = nh_.advertise<geometry_msgs::PoseStamped>("robot_0/goal", 1);
            goal_pub_1 = nh_.advertise<geometry_msgs::PoseStamped>("robot_1/goal", 1);
            goal_pub_2 = nh_.advertise<geometry_msgs::PoseStamped>("robot_2/goal", 1);
            goal_pub_3 = nh_.advertise<geometry_msgs::PoseStamped>("robot_3/goal", 1);

            //TODO:Se podria usar solo uno y remapear en el launch
            pedirSiguienteGoal_sub0_ = nh_.subscribe("/robot_0/pedirSiguienteGoal", 1, &CircularTrajectory::pedirSiguienteGoal, this);
            pedirSiguienteGoal_sub1_ = nh_.subscribe("/robot_1/pedirSiguienteGoal", 1, &CircularTrajectory::pedirSiguienteGoal, this);
            pedirSiguienteGoal_sub2_ = nh_.subscribe("/robot_2/pedirSiguienteGoal", 1, &CircularTrajectory::pedirSiguienteGoal, this);
            pedirSiguienteGoal_sub3_ = nh_.subscribe("/robot_3/pedirSiguienteGoal", 1, &CircularTrajectory::pedirSiguienteGoal, this);


            hayObstaculo_sub = nh_.subscribe("/hayObstaculo", 1, &CircularTrajectory::hayObstaculo, this);

            
        }


        //Constructor que realiza unicamente la fase de calculo
        CircularTrajectory(int iters){

        
            posicionesRobots.push_back({-4,-4,0.5});
            posicionesRobots.push_back({-3.5,4,1});
            posicionesRobots.push_back({2,3.75,1.5});
            

            numRobots = 3;

            radioCirculo = 2;


            numPedirSiguienteRecibidos = 0;



            calcularIGoal(iters);

            
        }

                
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




    protected:

        double derivadaXi1(double x){
            double numerator = radioCirculo*(-sin(x)*(1 + 0.3*pow(sin(x),2)) - 0.6*pow(cos(x),2)*sin(x));
            double denominator = pow(1 + 0.3 * pow(sin(x), 2), 2);
            double result = numerator / denominator;
            return result;
        }

        double derivadaXi2(double x){
            double numerator = radioCirculo * (cos(2*x) * ( 1 + 0.3*pow(sin(x),2)) -0.6*pow(cos(x),2)*pow(sin(x),2));
            double denominator = pow(1 + 0.3 * pow(sin(x), 2), 2);
            double result = numerator / denominator;
            return result;
        }


        void hayObstaculo(const std_msgs::Bool msg){
            if(msg.data == true){
                ROS_INFO("HAY OBSTACULO Y VOY A MANDAR METAS");
                numPedirSiguienteRecibidos=numRobots -1;
                pedirSiguienteGoal(msg);
            }
        } 

        void pedirSiguienteGoal(const std_msgs::Bool msg){
            
            ROS_INFO("He recibido un pedir siguiente");
            
            //Si el robot 0 quiere una nueva meta se calcula...
            if(msg.data == true){

                numPedirSiguienteRecibidos++;
                //Los 3 robots han llegado a sus metas, asignar nuevas
                if(numPedirSiguienteRecibidos == numRobots){
                    numPedirSiguienteRecibidos = 0;


                    for(int i = 0; i < numRobots; i++){
                            //Para robot0... mas tarde se hara con odometria
                        u1 = derivadaXi1(posicionesRobots[i].w) - ki1 * posicionesRobots[i].x + ki1* ((radioCirculo*cos(posicionesRobots[i].w))/(1+ 0.3*pow(sin(posicionesRobots[i].w),2)));
                        u2 =  derivadaXi2(posicionesRobots[i].w) - ki2*posicionesRobots[i].y + ki2* ((radioCirculo*sin(posicionesRobots[i].w)*cos(posicionesRobots[i].w))/(1+0.3*pow(sin(posicionesRobots[i].w),2)));

                        
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
                        w0 = 1 + ki1 * (posicionesRobots[i].x - (radioCirculo*cos(posicionesRobots[i].w)/(1+ 0.3*pow(sin(posicionesRobots[i].w),2))))
                        * (derivadaXi1(posicionesRobots[i].w))
                                + ki2 * (posicionesRobots[i].y -  (radioCirculo*sin(posicionesRobots[i].w)*cos(posicionesRobots[i].w)/(1+0.3*pow(sin(posicionesRobots[i].w),2))))
                                * ( derivadaXi2(posicionesRobots[i].w))
                                - (ci * (normalizarAngulo(posicionesRobots[i].w - wTarget)) + normalizarAngulo(mu*kw)); 

                        

                        

                        posicionesRobots[i].x = posicionesRobots[i].x + T * u1;
                        posicionesRobots[i].y = posicionesRobots[i].y + T * u2;
                        posicionesRobots[i].w = normalizarAngulo(posicionesRobots[i].w + normalizarAngulo(T* w0));

            
                        ROS_INFO("POS ROBOT %f, %f , %f " , posicionesRobots[i].x, posicionesRobots[i].y, posicionesRobots[i].w);

                        //trazasSalida << "Posicion robot 0 " << posRobot0[0] << "   " <<  posRobot0[1] << "   " << posRobot0[2] << "  wi - w* " <<  normalizarAngulo(posRobot0[2] - wTarget)  << endl;

                        // ¿Que valor se le da a wtarget?

                        
                
                        // poner velocidad

                        Goal.pose.position.x = posicionesRobots[i].x;
                        Goal.pose.position.y = posicionesRobots[i].y;

                        if(i == 0){
                            goal_pub_0.publish(Goal);
                        }else if(i == 1){
                            goal_pub_1.publish(Goal);
                        }else if(i == 2){
                            goal_pub_2.publish(Goal);
                        }else if(i == 3){
                            goal_pub_3.publish(Goal);
                        }

                        


                        //Vamos a escribir la trayectoria del robot 0
                        //coordenadas << posRobot0[0] << "," << posRobot0[1] << "," << posRobot0[2] << "," << wTarget << endl;
                    }


                    //Se actualiza la w virtual segun su derivada ( mirar paper )
                    // Se normaliza para que no salga de la esfera, es el angulo con el que deberia ir el platooning
                    wTarget = normalizarAngulo(wTarget + (T * 0.5));

                    sleep(0.1);
                    
                }
                        
                
            }
        }



        void calcularIGoal(int N){

            ofstream trazasSalida("src/read_sensor_data/src/aux/salida_logs.txt", ios::out);
            ofstream coordenadas("src/read_sensor_data/src/aux/coordenadasR0.txt",ios::out);
            ofstream coordenadas1("src/read_sensor_data/src/aux/coordenadasR1.txt",ios::out);
            ofstream coordenadas2("src/read_sensor_data/src/aux/coordenadasR2.txt",ios::out);
            //ofstream coordenadas3("src/read_sensor_data/src/aux/coordenadasR3.txt",ios::out);

            int iter = 0;
            while(iter <N){
                iter++;

                trazasSalida << "ITERACION " << iter << " DEL BUCLE"   << endl;

                for(int i = 0; i < numRobots; i++){
                        //Para robot0... mas tarde se hara con odometria
                    u1 = derivadaXi1(posicionesRobots[i].w) - ki1 * posicionesRobots[i].x + ki1* ((radioCirculo*cos(posicionesRobots[i].w))/(1+ 0.3*pow(sin(posicionesRobots[i].w),2)));
                    u2 =  derivadaXi2(posicionesRobots[i].w) - ki2*posicionesRobots[i].y + ki2* ((radioCirculo*sin(posicionesRobots[i].w)*cos(posicionesRobots[i].w))/(1+0.3*pow(sin(posicionesRobots[i].w),2)));

                    
                    mu = 0;
                    //Vecindario para un robot
                    for(int j = 0 ; j < numRobots; j++){
                        if(i != j){
                            double wimenosj = posicionesRobots[i].w - posicionesRobots[j].w;
                            if(abs(wimenosj) < R){
                                wimenosj = normalizarAngulo(wimenosj);
                                cout << "Valores : " << wimenosj << endl;
                                mu += calcularAlpha( abs(wimenosj),  r , R)* (wimenosj/abs(wimenosj));
                            }
                        }
                    }
                

                    //Ahora mismo se hace sin el termino de la repulsion para ver si hacen rendezvous con w*
                    w0 = 1 + ki1 * (posicionesRobots[i].x - (radioCirculo*cos(posicionesRobots[i].w)/(1+ 0.3*pow(sin(posicionesRobots[i].w),2))))
                    * (derivadaXi1(posicionesRobots[i].w))
                            + ki2 * (posicionesRobots[i].y -  (radioCirculo*sin(posicionesRobots[i].w)*cos(posicionesRobots[i].w)/(1+0.3*pow(sin(posicionesRobots[i].w),2))))
                            * ( derivadaXi2(posicionesRobots[i].w))
                            - (ci * (normalizarAngulo(posicionesRobots[i].w - wTarget)) + normalizarAngulo(mu*kw)); 

                    

                    

                    posicionesRobots[i].x = posicionesRobots[i].x + T * u1;
                    posicionesRobots[i].y = posicionesRobots[i].y + T * u2;
                    posicionesRobots[i].w = normalizarAngulo(posicionesRobots[i].w + normalizarAngulo(T* w0));

                                                        

                    trazasSalida << "Posicion robot "<< i << " "<< posicionesRobots[i].x <<"   " <<  posicionesRobots[i].y
                    << "   "  << posicionesRobots[i].w <<  "  wi - w* " 
                    <<  normalizarAngulo(posicionesRobots[i].w - wTarget) << "   " <<   mu*kw << endl;

                    if(i == 0){
                        coordenadas << posicionesRobots[i].x << "," << posicionesRobots[i].y << "," <<
                    posicionesRobots[i].w << "," << wTarget << endl;
                    }else if( i == 1){
                        coordenadas1 << posicionesRobots[i].x << "," << posicionesRobots[i].y << "," <<
                    posicionesRobots[i].w << "," << wTarget << endl;
                    }else if(i == 2){
                        coordenadas2 << posicionesRobots[i].x << "," << posicionesRobots[i].y << "," <<
                    posicionesRobots[i].w << "," << wTarget << endl;
                    }else if(i == 3){
                        //coordenadas3 << posicionesRobots[i].x << "," << posicionesRobots[i].y << "," <<
                    //posicionesRobots[i].w << "," << wTarget << endl;
                    }
                    

                }
                wTarget = normalizarAngulo(wTarget + (T * 1));
                sleep(0.1);
        
            }

            trazasSalida.close();
            coordenadas.close();
            coordenadas1.close();
            coordenadas2.close();
        }


    private:


        struct PosiRobot{
            double x;
            double y;
            double w;
        };

        ros::NodeHandle nh_;


        ros::Publisher goal_pub_0;
        ros::Publisher goal_pub_1;
        ros::Publisher goal_pub_2;
        ros::Publisher goal_pub_3;

        ros::Subscriber pedirSiguienteGoal_sub0_;
        ros::Subscriber pedirSiguienteGoal_sub1_;
        ros::Subscriber pedirSiguienteGoal_sub2_;
        ros::Subscriber pedirSiguienteGoal_sub3_;

        ros::Subscriber hayObstaculo_sub;

        geometry_msgs::PoseStamped Goal;

        int numPedirSiguienteRecibidos;


        vector<PosiRobot> posicionesRobots;
        int numRobots;

        double radioCirculo;


        double T = 0.1; // 100 milisegundos
        double wTarget = 0; // w*
        double ki1 = 2; // gains 1 
        double ki2 = 2; // gains 2
        double kw = 2; // ganancia de la w para evitar que cambie mucho
        double ci = 2; // ganancia ci

        //Hay que diferenciar el publicador de cada robot

    
        
        double u1,u2,mu,w0;
    
        

        int i = 0;
        int r = 0.2;
        int R = 10;

        

};


//Se define un caso hardcodeado de trayectoria circular 2d para 3 robots
// Va a ir publicando los controladores(nuevas posiciones) en los topics goal para cada robot.
// Asi otro nodo irá moviendo los robots respectivamente. Si se calculan bien los controladores debería realizar una trayectoria circular.
//TODO: PROBAR CON 3 Y PARAMETRIZARLO
int main(int argc, char **argv)
{
    
    if(argc == 2){
        string aux = argv[1];
        if(aux == "calcular"){
            ros::init(argc, argv, "circularTrajectory");
            CircularTrajectory synchronizer(10000);
        }else{
            cout << " PArametro " << argv[1] << endl;
            cout << "Argumento fallido: Introduce calcular o nada depende la opción que quieras hacer." << endl;
        }
    }else{
        
        ros::init(argc, argv, "circularTrajectory");
        CircularTrajectory synchronizer;
        ros::spin();

    }
   
    

}











