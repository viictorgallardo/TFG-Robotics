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
#include <sstream>

#include <csignal>
#include <cstdlib>
//#include <rviz_visual_tools/rviz_visual_tools.h>


using namespace std;


class CircularTrajectory{
    public:
        //Constructor que utiliza el nodo para comunicarse con los robots
        CircularTrajectory(){

            /*
            double x0 = 0 + radioCirculo * cos(0);
            double y0 = 0 + radioCirculo * sin(0);

            cout << "X0 es " << x0 << " Y0 es " << y0 << endl;

            double x1 = 0 + radioCirculo * cos(1);
            double y1 = 0 + radioCirculo * sin(1);


            cout << "X1 es " << x1 << " Y1 es " << y1 << endl;

            double x2 = 0 + radioCirculo * cos(2);
            double y2 = 0 + radioCirculo * sin(2);

            cout << "X2 es " << x2 << " Y2 es " << y2 << endl;

            posicionesRobots.push_back({x0,y0,0});
            posicionesRobots.push_back({x1,y1,1});
            posicionesRobots.push_back({x2,y2,2});

            posicionesRobotsNext.push_back({x0,y0,0});
            posicionesRobotsNext.push_back({x1,y1,1});
            posicionesRobotsNext.push_back({x2,y2,2});
            */
            posicionesRobots.push_back({-2,-2,0.5});
            //posicionesRobots.push_back({-1.5,2,1});
            //posicionesRobots.push_back({2,3.75,1.5});

            posicionesRobotsNext.push_back({-2,-2,0.5});

            cout << "El tamaño del vector es "<< posicionesRobotsNext.size() << endl;
            //posicionesRobotsNext.push_back({-1.5,2,1});
            //posicionesRobotsNext.push_back({2,3.75,1.5});
        

            
            if(numRobots == 1){
                C = 1;
            }

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
            /*
            double x0 = 0 + radioCirculo * cos(0);
            double y0 = 0 + radioCirculo * sin(0);

            cout << "X0 es " << x0 << " Y0 es " << y0 << endl;

            double x1 = 0 + radioCirculo * cos(1);
            double y1 = 0 + radioCirculo * sin(1);


            cout << "X1 es " << x1 << " Y1 es " << y1 << endl;

            double x2 = 0 + radioCirculo * cos(2);
            double y2 = 0 + radioCirculo * sin(2);

            cout << "X2 es " << x2 << " Y2 es " << y2 << endl;

            posicionesRobots.push_back({x0,y0,0});
            posicionesRobots.push_back({x1,y1,1});
            posicionesRobots.push_back({x2,y2,2});

            posicionesRobotsNext.push_back({x0,y0,0});
            posicionesRobotsNext.push_back({x1,y1,1});
            posicionesRobotsNext.push_back({x2,y2,2});
            */
            
            posicionesRobots.push_back({-2,-2,0.5});
            //posicionesRobots.push_back({-1.5,2,1});
            //posicionesRobots.push_back({2,3.75,1.5});

            posicionesRobotsNext.push_back({-2,-2,0.5});
            //posicionesRobotsNext.push_back({-1.5,2,1});
            //posicionesRobotsNext.push_back({2,3.75,1.5});
            
            //posicionesRobots.push_back({2.75,-3.75,2});

            if(numRobots == 1){
                C = 1;
            }

            cout << "Calculos para r : " << r << "y R: " << R << endl;





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
        double calcularAlpha(double wVecino){
            cout  << "Wvecino:  "<< wVecino  << " r es " << r << "y R: " << R << endl;
            //if(wVecino < r){
                //cout << "LIMITE INFERIOR r PASADO" << endl;
                //return (A/(pow(r,12)) - (B/pow(r,6)));
                //return 1 - (1/(normalizarAngulo(wVecino -r)) - (1/(R-r)));
                //return 1;
            //}
            if (wVecino > R){
                cout << "LIMITE SUPERIOR R PASADO" << endl;
                 
                return 0;
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
                return ((R-wVecino)/((wVecino-r_tope)*(R-r))) *(r-r_tope); // este factor normaliza
            }else{
                return (1/(r-r_tope)) * (r-r_tope);
            }
            
        }



        // Función para calcular la distancia entre dos puntos (x, y) y (x1, y1)
        double calcularDistancia(double x, double y, double x1, double y1) {
            return std::sqrt(std::pow(x1 - x, 2) + std::pow(y1 - y, 2));
        }




    protected:

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

            double u1,u2,mu,w0;
            
            
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
                //posicionesRobots[info_msg[0]].x = info_msg[1];
                //posicionesRobots[info_msg[0]].y = info_msg[2]; 

                ROS_INFO("He recibido un pedir siguiente de %f con posiciones %f, %f" ,info_msg[0], info_msg[1], info_msg[2]);
                //Los 3 robots han llegado a sus metas, asignar nuevas
                if(numPedirSiguienteRecibidos == numRobots){
                    
                    numPedirSiguienteRecibidos = 0;


                    for(int i = 0; i < numRobots; i++){
                        
                            //Para robot0... mas tarde se hara con odometria
                        u1 = dot_w_star * (-radioCirculo*  sin(posicionesRobots[i].w)) - ki1 * posicionesRobots[i].x + ki1* radioCirculo* cos(posicionesRobots[i].w);
                        u2 =  dot_w_star * (radioCirculo* cos(posicionesRobots[i].w)) - ki2*posicionesRobots[i].y + ki2* radioCirculo* sin(posicionesRobots[i].w);


                        mu = 0;
                        //Vecindario para un robot
                        for(int j = 0 ; j < numRobots; j++){
                            if(i != j){
                               
                                //Repulsión para dicha pareja de robots
                                double wimenosj = posicionesRobots[i].w - posicionesRobots[j].w;
                                wimenosj = normalizarAngulo(wimenosj);
                                cout << "Valores : " << wimenosj << endl;
                                double repulsionAux = calcularAlpha2(abs(wimenosj));
                                //Mirar si por lo que sea han coincidido las 2 posiciones w
                                if(abs(wimenosj) == 0){
                                    if( i < j){
                                        mu += repulsionAux* -1;
                                    }else{
                                        mu += repulsionAux* 1;
                                    }
                                    
                                }else{
                                // Comportamiento normal
                                    mu += repulsionAux* (wimenosj/abs(wimenosj));
                                }
                                
                                
                            }
                        }

                            //Ahora mismo se hace sin el termino de la repulsion para ver si hacen rendezvous con w*
                        w0 = dot_w_star + ki1 * (posicionesRobots[i].x -  radioCirculo* cos(posicionesRobots[i].w)) * (- radioCirculo* sin(posicionesRobots[i].w))
                                + ki2 * (posicionesRobots[i].y -  radioCirculo* sin(posicionesRobots[i].w)) * ( radioCirculo* cos(posicionesRobots[i].w))
                                - (C* (normalizarAngulo(posicionesRobots[i].w - wTarget) * Kw)) + normalizarAngulo(mu) * Kw * D; 


                       

                        posicionesRobotsNext[i].x = posicionesRobots[i].x + T * u1;
                        posicionesRobotsNext[i].y = posicionesRobots[i].y + T * u2;
                        posicionesRobotsNext[i].w = normalizarAngulo(posicionesRobots[i].w + normalizarAngulo(T* w0));

                        
                        cout << "xnueva: " << posicionesRobotsNext[i].x << "ynueva: " << posicionesRobotsNext[i].y << endl;
                        cout << "He actualizado" << endl;


                        //ROS_INFO("POS ROBOT %f, %f , %f , %f" , i , posicionesRobots[i].x, posicionesRobots[i].y, posicionesRobots[i].w);

                        //trazasSalida << "Posicion robot 0 " << posRobot0[0] << "   " <<  posRobot0[1] << "   " << posRobot0[2] << "  wi - w* " <<  normalizarAngulo(posRobot0[2] - wTarget)  << endl;

                        // ¿Que valor se le da a wtarget?

                        
                
                        // poner velocidad

                        //Vamos a escribir la trayectoria del robot 0
                        //coordenadas << posRobot0[0] << "," << posRobot0[1] << "," << posRobot0[2] << "," << wTarget << endl;
                    }

                    //Actualizamos las posiciones de los robots
                    for(int i = 0; i < numRobots; i++){
                        
                        posicionesRobots[i].x = posicionesRobotsNext[i].x;
                        posicionesRobots[i].y = posicionesRobotsNext[i].y;
                        posicionesRobots[i].w = posicionesRobotsNext[i].w;

                        Goal.pose.position.x = posicionesRobots[i].x;
                        Goal.pose.position.y = posicionesRobots[i].y;
                        
                        if(i == 0){
                            cout << "Voy a publicar en 0" << endl;
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
                    wTarget = normalizarAngulo(wTarget + (T * dot_w_star));

                    sleep(0.1);
                    
                }
                        
                
            }
        }



        void calcularIGoal(int N){
            double u1,u2,mu,w0;

            ofstream trazasSalida("src/robot_core/src/aux/salida_logs.txt", ios::out);
            ofstream coordenadas("src/robot_core/src/aux/coordenadasR0.txt",ios::out);
            ofstream coordenadas1("src/robot_core/src/aux/coordenadasR1.txt",ios::out);
            ofstream coordenadas2("src/robot_core/src/aux/coordenadasR2.txt",ios::out);
            ofstream coordenadas3("src/robot_core/src/aux/coordenadasR3.txt",ios::out);
            ofstream distancias("src/robot_core/src/aux/distancias.txt",ios::out);
            ofstream medidas("src/robot_core/src/aux/medidas.txt",ios::out);


            int iter = 0;
            while(iter <N){
                iter++;

                //ROS_INFO("ITERACION %d",iter);
                trazasSalida << "ITERACION " << iter << " DEL BUCLE"   << endl;

                for(int i = 0; i < numRobots; i++){
                        //Para robot0... mas tarde se hara con odometria
                    u1 = dot_w_star * (-radioCirculo*  sin(posicionesRobots[i].w)) - ki1 * posicionesRobots[i].x + ki1* radioCirculo* cos(posicionesRobots[i].w);
                    u2 =  dot_w_star * (radioCirculo* cos(posicionesRobots[i].w)) - ki2*posicionesRobots[i].y + ki2* radioCirculo* sin(posicionesRobots[i].w);


                    mu = 0;
                    //Vecindario para un robot
                    for(int j = 0 ; j < numRobots; j++){
                        if(i != j){
                            //Repulsión para dicha pareja de robots
                            double wimenosj = posicionesRobots[i].w - posicionesRobots[j].w;
                            wimenosj = normalizarAngulo(wimenosj);
                            cout << "Valores : " << wimenosj << endl;
                            double repulsionAux = calcularAlpha2(abs(wimenosj));
                            medidas << repulsionAux << endl;
                            //Mirar si por lo que sea han coincidido las 2 posiciones w
                            if(abs(wimenosj) == 0){
                                if( i < j){
                                    mu += repulsionAux* -1;
                                }else{
                                    mu += repulsionAux* 1;
                                }
                                
                            }else{
                               // Comportamiento normal
                                mu += repulsionAux* (wimenosj/abs(wimenosj));
                            }
                            
                            
                        }
                    }


                

                    //Ahora mismo se hace sin el termino de la repulsion para ver si hacen rendezvous con w*
                    w0 = dot_w_star + ki1 * (posicionesRobots[i].x -  radioCirculo* cos(posicionesRobots[i].w)) * (- radioCirculo* sin(posicionesRobots[i].w))
                            + ki2 * (posicionesRobots[i].y -  radioCirculo* sin(posicionesRobots[i].w)) * ( radioCirculo* cos(posicionesRobots[i].w))
                            - (C* (normalizarAngulo(posicionesRobots[i].w - wTarget) * Kw)) + normalizarAngulo(mu) * Kw * D; 

                 

                    posicionesRobotsNext[i].x = posicionesRobots[i].x + T * u1;
                    posicionesRobotsNext[i].y = posicionesRobots[i].y + T * u2;
                    posicionesRobotsNext[i].w = normalizarAngulo(posicionesRobots[i].w + normalizarAngulo(T* w0));

               

                    trazasSalida << "Posicion robot "<< i << " "<< posicionesRobots[i].x <<"   " <<  posicionesRobots[i].y
                    << "   "  << posicionesRobots[i].w << " w* "  <<  wTarget<<   "  wi - w* " 
                    <<  normalizarAngulo(posicionesRobots[i].w - wTarget) << "   " <<   mu*Kw << endl;
                    /*if(i == 0){
                        trazasSalida << calcularDistancia(posicionesRobots[i].x,posicionesRobots[i].y,posicionesRobots[1].x,posicionesRobots[1].y) << endl;;
                    }else{
                        trazasSalida << calcularDistancia(posicionesRobots[i].x,posicionesRobots[i].y,posicionesRobots[0].x,posicionesRobots[0].y) << endl;
                    }*/


                    
                    
                    if(i == 0){
                        errorX =  posicionesRobots[i].x - radioCirculo* cos(posicionesRobots[i].w);
                        errorY = posicionesRobots[i].y - radioCirculo* sin(posicionesRobots[i].w);
                        coordenadas << posicionesRobots[i].x << "," << posicionesRobots[i].y << "," <<
                    posicionesRobots[i].w << "," << wTarget << "," << errorX << "," << errorY <<  endl;
                    }else if( i == 1){
                        errorX =  posicionesRobots[i].x - radioCirculo* cos(posicionesRobots[i].w);
                        errorY = posicionesRobots[i].y - radioCirculo* sin(posicionesRobots[i].w);
                        coordenadas1 << posicionesRobots[i].x << "," << posicionesRobots[i].y << "," <<
                    posicionesRobots[i].w << "," << wTarget << ","  << errorX << "," << errorY <<  endl;
                    }else if(i == 2){
                        errorX =  posicionesRobots[i].x - radioCirculo* cos(posicionesRobots[i].w);
                        errorY = posicionesRobots[i].y - radioCirculo* sin(posicionesRobots[i].w);
                        coordenadas2 << posicionesRobots[i].x << "," << posicionesRobots[i].y << "," <<
                    posicionesRobots[i].w << "," << wTarget << ","  <<  errorX << "," << errorY <<  endl;
                    }else if(i == 3){
                        errorX =  posicionesRobots[i].x - radioCirculo* cos(posicionesRobots[i].w);
                        errorY = posicionesRobots[i].y - radioCirculo* sin(posicionesRobots[i].w);
                        coordenadas3 << posicionesRobots[i].x << "," << posicionesRobots[i].y << "," <<
                    posicionesRobots[i].w << "," << wTarget << "," <<  errorX << "," << errorY <<  endl;
                    }
                    

                }



                //trazasSalida << calcularDistancia(posicionesRobots[0].x,posicionesRobots[0].y,posicionesRobots[1].x,posicionesRobots[1].y) << endl;
                
                //double muestraW1 = normalizarAngulo(posicionesRobots[1].w - posicionesRobots[2].w);
                //double muestraW2  = normalizarAngulo(posicionesRobots[2].w - posicionesRobots[0].w);

                //double muestraW1 = normalizarAngulo(posicionesRobots[1].w - posicionesRobots[0].w);
                //double muestraW2 = calcularDistancia(posicionesRobots[1].x,posicionesRobots[1].y,posicionesRobots[0].x,posicionesRobots[0].y);
                //double muestraW2  = normalizarAngulo(posicionesRobots[2].w - posicionesRobots[0].w);
                //distancias << abs(muestraW1) <<  "," << abs(muestraW2) <<
                 //       endl;
                
                //Actualizamos las posiciones de los robots
                for(int i = 0; i < numRobots; i++){
                    posicionesRobots[i].x = posicionesRobotsNext[i].x;
                    posicionesRobots[i].y = posicionesRobotsNext[i].y;
                    posicionesRobots[i].w = posicionesRobotsNext[i].w;
                }

                //Siguiente valor de w*
                wTarget = normalizarAngulo(wTarget + (T * dot_w_star));
                //sleep(0.1);
        
            }

            trazasSalida.close();
            coordenadas.close();
            coordenadas1.close();
            coordenadas2.close();
            distancias.close();
            medidas.close();
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
        int numRobots = 1;
        double radioCirculo = 1.6;


    
        double wTarget = 0; // w*
        double ki1 = 3; // gains 1 
        double ki2 = 3; // gains 2

        //double ki1 = 2; // gains 1 
        //double ki2 = 2; // gains 2
        //double C= 2;

        double T=0.1; // segundos
        double dot_w_star = pow(-1,numRobots); // 1 rad per second
        
        double Kw= 0.5;
        double D = 1;

	    double d_opt = 30*M_PI/180; // W deseado al final.
        double R = 1.5*d_opt; // 15 grados. % distancia en grados entre los angulos para ser considerados vecinos.
        double r = 15 * M_PI/180; // por ejemplo son factibles.
        double r_tope = 5*M_PI/180; 

        //CUIDADO! DA UN INFINITO SI NUMROBOTS = 1
        double factor_C = (r-r_tope) / ( (numRobots-1) * (d_opt - r_tope) * (1.5 * d_opt - r) );
        double C = D * factor_C;



        //METRICAS A SACAR

        double errorX = 0;
        double errorY = 0;

        

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
            CircularTrajectory synchronizer(1000);
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












