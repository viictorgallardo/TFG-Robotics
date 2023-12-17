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
#include "std_msgs/String.h"

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

        
            posicionesRobots.push_back({-4,-4,0});
            posicionesRobots.push_back({-3.5,4,0.5});
            //posicionesRobots.push_back({2,2,1.5});
            
            posicionesRobotsNext.push_back({-4,-4,0});
            posicionesRobotsNext.push_back({-3.5,4,0.5});


            numRobots = 2;

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

        
            posicionesRobots.push_back({-4,-4,0});
            posicionesRobots.push_back({-3.5,4,0.5});

            posicionesRobotsNext.push_back({-4,-4,0});
            posicionesRobotsNext.push_back({-3.5,4,0.5});
            //posicionesRobots.push_back({2,2,1.5});
            

            numRobots = 2;



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
        double calcularAlpha(double wVecino ){
            cout  << "Wvecino:  "<< wVecino  << endl;
            if (wVecino > R){
                cout << "LIMITE SUPERIOR R PASADO" << endl;
                return 0;
            }else if(wVecino < r){
                return 1;
            }else if(wVecino > r && wVecino < R){
                return (1/(normalizarAngulo(wVecino -r)) - (1/(R-r)));
            }
        }



        //Nueva implementación de alpha 
        double calcularAlpha2(double wVecino){
            cout  << "Nueva Impl, Wvecino:  "<< wVecino  << " r es " << r << "y R: " << R << endl;
            if (wVecino > R){
                //cout << "LIMITE SUPERIOR R PASADO" << endl;
                return 0;
            }else if(wVecino > r && wVecino <= R){
                return (R-wVecino)/((wVecino-r_tope)*(R-r));
            }else if( wVecino > 0 && wVecino <= r){
                return 1/(r-r_tope);
            }
            
        }
        

        // Función para calcular la distancia entre dos puntos (x, y) y (x1, y1)
        double calcularDistancia(double x, double y, double x1, double y1) {
            return std::sqrt(std::pow(x1 - x, 2) + std::pow(y1 - y, 2));
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


        void hayObstaculo(const std_msgs::String msg){
            string aux = msg.data;
            if(isdigit(aux[0])){
                sleep(3);
                //ROS_INFO("HAY OBSTACULO Y VOY A MANDAR METAS");
                //numPedirSiguienteRecibidos=numRobots -1;
                //pedirSiguienteGoal(msg);
            }
        } 

        void pedirSiguienteGoal(const std_msgs::String msg){
            
            string aux = msg.data;
            double info_msg[3]; // guarda id, x , y
            //Si el robot 0 quiere una nueva meta se calcula...
            if(isdigit(aux[0])){

                numPedirSiguienteRecibidos++;
                stringstream ss(aux);
                string token;
                int x = 0;
                while (std::getline(ss, token, ',')) {
                    double numero = std::stod(token);
                    info_msg[x] = numero;
                    x++;
                }
                //Se actualiza las posiciones de los robots con la odometria real
                posicionesRobots[info_msg[0]].x = info_msg[1];
                posicionesRobots[info_msg[0]].y = info_msg[2]; 

                ROS_INFO("He recibido un pedir siguiente de %f con posiciones %f, %f" ,info_msg[0], info_msg[1], info_msg[2]);
                
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
                                wimenosj = normalizarAngulo(wimenosj);
                                
                                    
                                    //cout << "Valores : " << wimenosj << endl;
                                mu += calcularAlpha2( abs(wimenosj))* (wimenosj/abs(wimenosj));
                            
                            }
                        }
                    

                        //Ahora mismo se hace sin el termino de la repulsion para ver si hacen rendezvous con w*
                        w0 = 1 + ki1 * (posicionesRobots[i].x - (radioCirculo*cos(posicionesRobots[i].w)/(1+ 0.3*pow(sin(posicionesRobots[i].w),2))))
                        * (derivadaXi1(posicionesRobots[i].w))
                                + ki2 * (posicionesRobots[i].y -  (radioCirculo*sin(posicionesRobots[i].w)*cos(posicionesRobots[i].w)/(1+0.3*pow(sin(posicionesRobots[i].w),2))))
                                * ( derivadaXi2(posicionesRobots[i].w))
                                - (ci * (normalizarAngulo(posicionesRobots[i].w - wTarget)) + normalizarAngulo(mu*kw)); 

                        

                        

                        posicionesRobotsNext[i].x = posicionesRobots[i].x + T * u1;
                        posicionesRobotsNext[i].y = posicionesRobots[i].y + T * u2;
                        posicionesRobotsNext[i].w = normalizarAngulo(posicionesRobots[i].w + normalizarAngulo(T* w0));

            
                        //ROS_INFO("POS ROBOT %f, %f , %f " , posicionesRobots[i].x, posicionesRobots[i].y, posicionesRobots[i].w);

                        //trazasSalida << "Posicion robot 0 " << posRobot0[0] << "   " <<  posRobot0[1] << "   " << posRobot0[2] << "  wi - w* " <<  normalizarAngulo(posRobot0[2] - wTarget)  << endl;



                        //Vamos a escribir la trayectoria del robot 0
                        //coordenadas << posRobot0[0] << "," << posRobot0[1] << "," << posRobot0[2] << "," << wTarget << endl;
                    }

                    for(int i = 0; i < numRobots; i++){
                        posicionesRobots[i].x = posicionesRobotsNext[i].x;
                        posicionesRobots[i].y = posicionesRobotsNext[i].y;
                        posicionesRobots[i].w = posicionesRobotsNext[i].w;

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
                    }
                    //Se actualiza la w virtual segun su derivada ( mirar paper )
                    // Se normaliza para que no salga de la esfera, es el angulo con el que deberia ir el platooning
                    wTarget = normalizarAngulo(wTarget + (T * multiplicadorW));

                    
                }
                        
                
            }
        }



        void calcularIGoal(int N){

            ofstream trazasSalida("src/read_sensor_data/src/aux/salida_logs.txt", ios::out);
            ofstream coordenadas("src/read_sensor_data/src/aux/coordenadasR0.txt",ios::out);
            ofstream coordenadas1("src/read_sensor_data/src/aux/coordenadasR1.txt",ios::out);
            ofstream coordenadas2("src/read_sensor_data/src/aux/coordenadasR2.txt",ios::out);
            ofstream distancias("src/read_sensor_data/src/aux/distancias.txt",ios::out);
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
                            wimenosj = normalizarAngulo(wimenosj);
                           
                            cout << "Valores : " << wimenosj << endl;
                            mu += calcularAlpha2( abs(wimenosj))* (wimenosj/abs(wimenosj));
                            
                        }
                    }
                

                    //Ahora mismo se hace sin el termino de la repulsion para ver si hacen rendezvous con w*
                    w0 = 1 + ki1 * (posicionesRobots[i].x - (radioCirculo*cos(posicionesRobots[i].w)/(1+ 0.3*pow(sin(posicionesRobots[i].w),2))))
                    * (derivadaXi1(posicionesRobots[i].w))
                            + ki2 * (posicionesRobots[i].y -  (radioCirculo*sin(posicionesRobots[i].w)*cos(posicionesRobots[i].w)/(1+0.3*pow(sin(posicionesRobots[i].w),2))))
                            * ( derivadaXi2(posicionesRobots[i].w))
                            - (ci * (normalizarAngulo(posicionesRobots[i].w - wTarget)) + normalizarAngulo(mu*kw)); 

                    

                    

                    posicionesRobotsNext[i].x = posicionesRobots[i].x + T * u1;
                    posicionesRobotsNext[i].y = posicionesRobots[i].y + T * u2;
                    posicionesRobotsNext[i].w = normalizarAngulo(posicionesRobots[i].w + normalizarAngulo(T* w0));

                                                        

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

                double muestraW1 = normalizarAngulo(posicionesRobots[1].w - posicionesRobots[0].w);
                double muestraW2 = calcularDistancia(posicionesRobots[1].x,posicionesRobots[1].y,posicionesRobots[0].x,posicionesRobots[0].y);
                //double muestraW2  = normalizarAngulo(posicionesRobots[2].w - posicionesRobots[0].w);
                distancias << abs(muestraW1) <<  "," << abs(muestraW2) <<
                        endl;


                for(int i = 0; i < numRobots; i++){
                    posicionesRobots[i].x = posicionesRobotsNext[i].x;
                    posicionesRobots[i].y = posicionesRobotsNext[i].y;
                    posicionesRobots[i].w = posicionesRobotsNext[i].w;
                }

                wTarget = normalizarAngulo(wTarget + (T * multiplicadorW));
                sleep(0.1);
        
            }

            trazasSalida.close();
            coordenadas.close();
            coordenadas1.close();
            coordenadas2.close();
            distancias.close();
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
        vector<PosiRobot> posicionesRobotsNext;
        int numRobots;

        double radioCirculo = 2.1;


        double T = 0.1; // 100 milisegundos
        double wTarget = 0; // w*
        double ki1 = 2; // gains 1 
        double ki2 = 2; // gains 2

        //double ki1 = 2; // gains 1 
        //double ki2 = 2; // gains 2
        //double ci = 2;

        double kw = 1.5; // ganancia de la w para evitar que cambie mucho
        double ci = 0.5; // ganancia ci

        //Hay que diferenciar el publicador de cada robot

        double multiplicadorW = 1;
        
        double u1,u2,mu,w0;
    
        

        int i = 0;

        double r_tope = 0.35;
        double r = 0.7;
        double R = 1;

        

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












