/*********************************************************************************************
 * Progetto per l'azionamento di un braccio 6 servo con arduino ricevendo i dati dal
 * topic JointState
 * 
 */



#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

//allegare le parti del programma ce ci servono

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/JointState.h>

//inizializzazione di ros e la creazioni di 6 classi servo

ros::NodeHandle  nh;
Servo servo_rotz;
Servo con_up_1;
Servo con_up_2;
Servo con_up_3;
Servo rotation_grip;
Servo plan_move;

//inizializzazione posizione iniziale

double base_angle=90;
double shoulder_angle=90;
double elbow_angle=90;
double wrist_angle=90;
double gripper_angle=90;

//aspetta ricezione messaggio da ros

void servo_cb(const sensor_msgs::JointState& cmd_msg){
  
  //Conversione angolo da radianti a gradi
  
  base_angle=radiansToDegrees(cmd_msg.position[0]);
  shoulder_angle=radiansToDegrees(cmd_msg.position[1]);
  elbow_angle=radiansToDegrees(cmd_msg.position[2]);
  wrist_angle=radiansToDegrees(cmd_msg.position[3]);
  gripper_angle=radiansToDegrees(cmd_msg.position[4]);

  
  
  base.write(base_angle);
  shoulder1.write(shoulder_angle);
  shoulder2.write(shoulder_angle);
  elbow.write(elbow_angle);
  wrist.write(wrist_angle);
  gripper.write(gripper_angle);
  
}

//nodo a cui sono sottoscritto
ros::Subscriber<sensor_msgs::JointState> sub("joint_states", servo_cb);

void setup(){
  //selezione della frequenza 
  nh.getHardware()->setBaud(115200);
  
  //settaggio del nodo 
  nh.initNode();
  nh.subscribe(sub);

  //Conessioni comandi servo 
  base.attach(2); 
  gripper.attach(8); 
  wrist.attach(11);
  elbow.attach(6); 
  shoulder2.attach(4);
  shoulder1.attach(3); 

  //posizione iniziale 
  delay(1);
  base.write(90);
  shoulder1.write(90);
  shoulder2.write(90);
  elbow.write(90);
  wrist.write(90);
  gripper.write(90);
}

//loop
void loop(){
  nh.spinOnce();
}

double radiansToDegrees(float position_radians)
{

  //position_radians = position_radians + 1.6;

  //return position_radians * 57.2958;
  return position_radians*360/2/3.1415

}