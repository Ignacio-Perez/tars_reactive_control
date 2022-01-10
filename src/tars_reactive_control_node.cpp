/*
MIT License

Copyright (c) 2021 Ignacio Pérez Hurtado de Mendoza

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tars/AgentsMsg.h>


// Frecuencia del bucle principal (Hz)
#define FREQ 30

// Radio del robot (m)
#define ROBOT_RADIUS 0.2

// Velocidad lineal del robot (m/s)
#define ROBOT_LIN_VEL 0.6

// Velocidad angular del robot (rad/s)
#define ROBOT_ANG_VEL 0.5


// Posibles estados del robot
enum State {INIT, MOVING_FORWARD, ROTATION_IN_PLACE};


// Estructura para almacenar toda la informacion sobre el robot
struct Robot
{
	State state; // El estado actual del robot
	bool collision; // Se pone a true si el robot va a colisionar en caso de avanzar en linea recta
	bool poseReceived; // Se pone a true cuando se recibe la primera pose del robot
	bool scanReceived; // Se pone a true cuando se recibe la primera lectura de obstaculos
	double x; // La posicion x del robot en el frame "map"
	double y; // La posicion y del robot en el frame "map"
	double yaw; // El angulo del robot en el frame "map"
} robot;



/***************************/
/* DEFINICION DE FUNCIONES */
/***************************/

// Funcion main
int main(int argc, char** argv);

// Funcion para procesar la recepcion de un mensaje del sensor LiDAR
void scanReceivedCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

// Funcion para procesar la recepcion de un mensaje de tracking de agentes
void agentsReceivedCallback(const tars::AgentsMsg::ConstPtr& agents);

// Funcion que controla al robot, segun la informacion actual, decide el siguiente comando de movimiento y actualiza el estado del robot
void control(ros::Publisher& cmdVelPublisher);

/****************/
/* FUNCION MAIN */
/****************/
int main(int argc, char** argv) {

	/*******************/
	/* INICIALIZACION  */
	/*******************/
	// La inicializacion de ROS, es completamente necesaria antes de ejecutar ninguna otra funcion de ROS
	ros::init(argc,argv,"tars_reactive_control_node");

	// Definimos dos manejadores, uno publico y otro privado, para comunicarnos con el sistema ROS
	ros::NodeHandle n, pn("~");


	// Inicializacion del robot
	robot.state = INIT;
	robot.collision = false;
	robot.poseReceived = false;
	robot.scanReceived = false;
	robot.x = 0;
	robot.y = 0;
	robot.yaw = 0;


	/*************************/
	/* SUBSCRIPCION A TOPICS */
	/*************************/
	
	// Topic del LiDAR (obstaculos estaticos)
	ros::Subscriber scanSubscriber = n.subscribe<sensor_msgs::LaserScan>("/tars/09/scan",1,scanReceivedCallback);


	// Topic de deteccion de agentes (obstaculos dinamicos) 
	ros::Subscriber trackingSubscriber = n.subscribe<tars::AgentsMsg>("/tars/09/agents_tracking",1,agentsReceivedCallback);


	/*************************/
	/* PUBLICACION DE TOPICS */
	/*************************/

	// Topic de comandos de velocidad (cmd_vel)
	ros::Publisher cmdVelPublisher = pn.advertise<geometry_msgs::Twist>("/tars/09/cmd_vel",1);


 	/**********************/
 	/* BUCLE PRINCIPAL    */
 	/**********************/
	// La frecuencia a la que queremos que se ejecute el bucle
	ros::Rate r(FREQ);	

	// El Bucle principal, que funcionara idealmente a una tasa constante r
	while (n.ok()) {

		// Controlamos al robot
		control(cmdVelPublisher); 

		// El nodo se duerme para ajustarse a la frecuencia deseada
		r.sleep();

		// Se leen y procesan los mensajes que llegan por los topics
		ros::spinOnce();
	}

	return 0;
}

/************************/
/* FUNCIONES CALLBACK   */
/************************/

/* LiDAR */
void scanReceivedCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
	robot.collision = false;
	double angleIncrement = scan->angle_increment;
	double angle = scan->angle_min;
	for (unsigned i=0;i<scan->ranges.size();i++) {
		if (angle >= -M_PI/4 && angle <= M_PI/4 && scan->ranges[i]<=0.4) {
			robot.collision = true;
			break;
		}
		angle += angleIncrement;
	}
	robot.scanReceived = true;
}


/* Tracking */
void agentsReceivedCallback(const tars::AgentsMsg::ConstPtr& agents) {
	for (unsigned i=0;i<agents->size;i++) {
		// El robot se percibe a si mismo como agente, podemos tomar su pose en el frame "map" de esta forma
		// Esto es una facilidad debido al simulador TARS, en un robot real habria que subscribirse al topic "odom" y transformar la pose al frame "map"
		if (agents->agents[i].id == "09") {
			robot.x = agents->agents[i].position.x;
			robot.y = agents->agents[i].position.y;
			robot.yaw = agents->agents[i].yaw;
			robot.poseReceived = true;
		}
	}
}

/**********************/
/* CONTROL DEL ROBOT  */
/*********************/
void control(ros::Publisher& cmdVelPublisher) {
	switch (robot.state) {
		case INIT:
			if (robot.poseReceived && robot.scanReceived) {
				robot.state = MOVING_FORWARD;
				ROS_INFO("MOVING FORWARD");
			}
		break;
		case ROTATION_IN_PLACE:
			if (!robot.collision) {
				robot.state = MOVING_FORWARD;
				ROS_INFO("MOVING FORWARD");
			} else {
				geometry_msgs::Twist cmdVel;
				cmdVel.angular.z = ROBOT_ANG_VEL;
				cmdVelPublisher.publish(cmdVel);
			}

		break;
		case MOVING_FORWARD:
			if (robot.collision) {
				robot.state = ROTATION_IN_PLACE;
				ROS_INFO("ROTATION IN_PLACE");
			} else {
				geometry_msgs::Twist cmdVel;
				cmdVel.linear.x = ROBOT_LIN_VEL;
				cmdVelPublisher.publish(cmdVel);
			}

		break;
	}
}