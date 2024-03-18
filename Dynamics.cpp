#include <ros/ros.h>
#include <string>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

void c_matrix();
void c_omegapunto();
void c_omega();
void c_thetapunto();
void c_theta();
void c_fuerza();
void c_vlineal();
void c_v();
void c_vinercial();
void c_posicion();

Eigen::Matrix3d inercias; //Matriz de inercias
Eigen::Matrix3d inv_inercias; //Matriz inversa de inercias
Eigen::Matrix3d rotacion; // Matriz de rotacion
Eigen::Matrix3d rot_trans; //Matriz de rotacion transpuesta
Eigen::Matrix3d r2; 

Eigen::Vector3d theta; //Vector de posiciones angulares inerciales
Eigen::Vector3d thpunto; //Vector de velocidades angulares incerciales 
Eigen::Vector3d torques; //Matriz de torques
Eigen::Vector3d omega; //Vector de velocidad angular
Eigen::Vector3d opunto; //Vector de velocidad angular derivada
Eigen::Vector3d e3;
Eigen::Vector3d vpunto; //Vector de velocidad derivada local
Eigen::Vector3d v; //Vector de velocidad lineal local
Eigen::Vector3d vinercial; //Vector de velocidad inercial vector p punto
Eigen::Vector3d posicion; //Vector de posicion
Eigen::Vector3d f;

double Th = 0; //Thrust
double step = 0.01;
double mass = 2;
double g = 9.81;

geometry_msgs::Twist p_real;
geometry_msgs::Twist v_real;

void torquesCallback(const geometry_msgs::Vector3::ConstPtr& torq) //Obtener datos de torque
{
    torques(0) = torq -> x;
    torques(1) = torq -> y;
    torques(2) = torq -> z;
}

void thrustCallback(const std_msgs::Float32::ConstPtr& th) //Obtener datos de Thrust
{
    Th = th-> data;
}

int main(int argc, char**argv)
{
    omega << 0,0,0;
    theta << 0,0,0;
    thpunto << 0,0,0;
    inercias <<0.0411,0,0,0,0.0478,0,0,0,0.0599;
    inv_inercias = inercias.inverse();
    e3 << 0,0,1;
    v << 0,0,0;
    posicion << 0,0,0;

    ros::init(argc, argv, "Dynamics"); //Declarar subscriptor
    ros::NodeHandle nh4; //Inicializar nodo

    ros::Subscriber torque_sub = nh4.subscribe("/torque", 100, &torquesCallback); //Suscribirse a nodo de torque
    ros::Subscriber thrust_sub = nh4.subscribe("/thrust", 100, &thrustCallback);

    ros::Publisher posicion_pub = nh4.advertise<geometry_msgs::Twist>("/pos_real", 10); //Inicializar publisher
    ros::Publisher velocidad_pub = nh4.advertise<geometry_msgs::Twist>("/vels_real", 10);

    ros::Rate loop_rate(100);

    while(ros::ok())
    {
        ros::spinOnce();
        c_matrix();
        c_omegapunto();
        c_omega();
        c_thetapunto();
        c_theta();
        c_fuerza();
        c_vlineal();
        c_v();
        c_vinercial();
        c_posicion();

        p_real.linear.x = posicion(0),
        p_real.linear.y = posicion(1);
        p_real.linear.z = posicion(2);
        p_real.angular.x = theta(0);
        p_real.angular.y = theta(1);
        p_real.angular.z = theta(2);

        v_real.linear.x = vinercial(0);
        v_real.linear.y = vinercial(1);
        v_real.linear.z = vinercial(2);
        v_real.angular.x = thpunto(0);
        v_real.angular.y = thpunto(1);
        v_real.angular.z = thpunto(2);

        std::cout << posicion << std::endl;
        std::cout << "----------" << std::endl;
        std::cout << vinercial << std::endl;

        posicion_pub.publish(p_real);
        velocidad_pub.publish(v_real);

        loop_rate.sleep();
    }

    return 0;
}

//------Calculo de matriz de rotacion, r2 y matriz de rotacion transpuesta------
void c_matrix()
{
    r2 << 1, sin(theta(0))*tan(theta(1)), cos(theta(0))*tan(theta(1)), 0, cos(theta(0)), -sin(theta(0)), 0, sin(theta(0))/cos(theta(1)), cos(theta(0))/cos(theta(1));
    rotacion << cos(theta(2))*cos(theta(1)), cos(theta(2))*sin(theta(0))*sin(theta(1))-cos(theta(0))*sin(theta(2)), sin(theta(2))*sin(theta(0))+cos(theta(2))*cos(theta(0))*sin(theta(1)),
            cos(theta(1))*sin(theta(2)), cos(theta(2))*cos(theta(0))+sin(theta(2))*sin(theta(0))*sin(theta(1)), cos(theta(0))*sin(theta(2))*sin(theta(1))-cos(theta(2))*sin(theta(0)),
           -sin(theta(1)), cos(theta(1))*sin(theta(0)), cos(theta(0))*cos(theta(1));
    rot_trans = rotacion.inverse();
}

//---------Dinamica Angular--------
void c_omegapunto() //Aceleracion angular local 
{
    opunto = inv_inercias*(torques - (omega.cross((inercias*omega))));
}

void c_omega() //Velocidad angular local
{
    omega = omega + (step*opunto);
}

void c_thetapunto() //Velocdiad angular inercial
{
    thpunto = r2*omega;
}

void c_theta() //Calculo de posicion angular 
{
    theta = theta + step*thpunto;
}

//--------Dinamica Lineal--------
void c_fuerza()
{
    f = (Th*e3) + (rot_trans*mass*g*e3);
}

void c_vlineal()
{
    vpunto = (f/mass) - (omega.cross(v));
}

void c_v()
{
    v = v + (step*vpunto);
}

void c_vinercial()
{
    vinercial = rotacion*v;
}

void c_posicion()
{
    posicion = posicion + (step*vinercial);
}
