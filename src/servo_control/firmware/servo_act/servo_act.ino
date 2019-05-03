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


Servo base;
Servo servo_up1;
Servo servo_up2;
Servo servo_up3;
Servo gripper_angle;
Servo gripper_chiuso;

//inizializzazione posizione iniziale

double base_angle=0;
double servo_up1_angle=0;
double servo_up2_angle=0;
double servo_up3_angle=0;
double gripper_angle_angle=0;
double gripper_chiuso_angle=0;





//aspetta ricezione messaggio da ros

void servo_cb(const sensor_msgs::JointState& cmd_msg){
  
  //Conversione angolo da radianti a gradi
  
  base_angle=radiansToDegrees(cmd_msg.position[0]);
  servo_up1_angle=radiansToDegrees(cmd_msg.position[1]);
  servo_up2_angle=radiansToDegrees(cmd_msg.position[2]);
  servo_up3_angle=radiansToDegrees(cmd_msg.position[3]);
  gripper_angle_angle=radiansToDegrees(cmd_msg.position[4]);
  gripper_chiuso_angle=cmd_msg.position[5];


  //Conversione corretta tra l'angolo reale e quello generato
  //4 gradi per essere ortogonale

  base_angle=dato_to_real(base_angle-90)+10;
  servo_up1_angle=dato_to_real(servo_up1_angle+90);
  servo_up2_angle=dato_to_real(servo_up2_angle+90);
  servo_up3_angle=dato_to_real(servo_up3_angle);
  gripper_angle_angle=dato_to_real(180-gripper_angle_angle);
  gripper_chiuso_angle=gripper_control(gripper_chiuso_angle);
  
  
  base.write(base_angle);
  servo_up1.write(servo_up1_angle);
  servo_up2.write(servo_up2_angle);
  servo_up3.write(servo_up3_angle);
  gripper_angle.write(gripper_angle_angle);
  gripper_chiuso.write(gripper_chiuso_angle);

  Serial.print(base_angle);
  
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
  base.attach(3);
  servo_up1.attach(5);
  servo_up2.attach(6);
  servo_up3.attach(9);
  gripper_angle.attach(10);
  gripper_chiuso.attach(11);

  //posizione iniziale 
  delay(1);
  base.write(90);
  servo_up1.write(50);
  servo_up2.write(90);
  servo_up3.write(90);
  gripper_angle.write(0);
  gripper_chiuso.write(60);
}

//loop
void loop(){
  nh.spinOnce();
}

double radiansToDegrees(float position_radians)
{

  //position_radians = position_radians + 1.6;

  //return position_radians * 57.2958;
  return position_radians*360/2/3.1415;

}

double dato_to_real(float position_degre)
{
  return position_degre*145/180;
}

double gripper_control(float position_degre)
{
  //il modello va da -0.55 to -0.05 -->
  //--> converto tale valore per usarlo in angoli
  //attualmente ha un angolo che varia tra i 55 e i 10 gradi
  position_degre=0.05-position_degre;
  position_degre=position_degre*45/0.5;
  position_degre=55-position_degre+10;
  return position_degre;
}
