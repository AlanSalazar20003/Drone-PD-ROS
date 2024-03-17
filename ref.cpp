/*              NODO DE REFERENCIAS
Este nodo envia la posicion, velocidad deseadas (lineales y angulares), incluyendo condiciones inciales.
*/
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <cmath>

// Variables globales para las posiciones iniciales
double x_des;
double y_des, z_des;
double yaw_des, roll_des, pitch_des;

// Step para integraciones numericas
double step = 0.01;

// Funciones de referencia
double x_punto_ref(double t) {
    if (t < 5) {
        return 0;
    } else if (t >= 5 && t <= 65) {
        return 0.5 * sin(0.1 * (t - 5));
    } else {
        return 0;
    }
}

double y_punto_ref(double t) {
    if (t < 5) {
        return 0;
    } else if (t >= 5 && t <= 65) {
        return 0.5 * cos(0.1 * (t - 5));
    } else {
        return 0;
    }
}

double z_punto_ref(double t) {
    if (t < 5) {
        return -0.5;
    }
   else {
        return 0;
    }
}

double yaw_punto_ref(double t) {
    if (t < 5) {
        return 0;
    } else {
        return 0.1;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ref");
    ros::NodeHandle nh;

    ros::Publisher position_pub = nh.advertise<geometry_msgs::Twist>("/pos_des", 100);
    ros::Publisher velocity_pub = nh.advertise<geometry_msgs::Twist>("/vels_des", 100);

    ros::Rate loop_rate(100); 

    double t = 0;
    while (ros::ok()) {
        geometry_msgs::Twist vels_des_var;
        geometry_msgs::Twist pos_des_var;

        // Calcular las referencias de velocidades (roll y pitch = 0)
        double x_punto_des = x_punto_ref(t);
        double y_punto_des = y_punto_ref(t);
        double z_punto_des = z_punto_ref(t);
        double yaw_punto_des = yaw_punto_ref(t);

        // Calcular las posociones lineales y angular 
        x_des += step*x_punto_des;
        y_des += step*y_punto_des;
        z_des += step*z_punto_des;
        yaw_des += step*yaw_punto_des;

        // Publicar las velocidades lineales y angular
        vels_des_var.linear.x = x_punto_des;
        vels_des_var.linear.y = y_punto_des;
        vels_des_var.linear.z = z_punto_des;
        vels_des_var.angular.x = 0; // velocidad = 0, ya que la posicion queremos que sea 0
        vels_des_var.angular.y = 0; // velocidad = 0, ya que la posicion queremos que sea 0
        vels_des_var.angular.z = yaw_punto_des;
        velocity_pub.publish(vels_des_var);

        // Publicar posiciones y yaw
        pos_des_var.linear.x = x_des;
        pos_des_var.linear.y = y_des;
        pos_des_var.linear.z = z_des;
        pos_des_var.angular.z = yaw_des;
        position_pub.publish(pos_des_var);

        ros::spinOnce();
        loop_rate.sleep();

        // Reiniciar el tiempo si t > 65
        if (t > 65) {
            t = 0;
        }
        
        t += loop_rate.expectedCycleTime().toSec(); // Incrementa el tiempo en función de la frecuencia, si no sirve cambiar a t+=0.01;
    }

    return 0;
}
