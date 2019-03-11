
// A C/C++ program for splitting a string 
// using strtok() 
#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <Servo.h>
#include <stdio.h>

#include <ros.h>
#include <std_msgs/String.h>
#include <string.h>

#include <stdlib.h>//dovrebbe essere la libreria di atoi

//c++ file for atoi
//#include <iostream>
//#include <cstring>
//#include <cstdlib>

int a;

ros::NodeHandle  nh;

Servo servo9, servo10, servo11, servo3, servo5, servo6;

void servo_cb( const std_msgs::String& cmd_msg){
    char str[100] = {*cmd_msg.data};
    int arr[6]; //valori da impostare ai servo
    char* token;

    // Returns first token
    char *ptr = strtok(str, "-");
    // Keep printing tokens while one of the
    // delimiters present in str[].
    while (ptr != NULL)
    {
        token = ptr;
        arr[a] = atoi(token);
        ptr = strtok(NULL, "-");
	a++;
    }

    //Test serial aoutput
    Serial.print(arr[0]);

    servo9.write(arr[0]); //set servo angle, should be from 0-180
    servo10.write(arr[1]); //se valore maggiore di 180 usare map
    servo11.write(arr[2]); //val = map(val, 0, 1023, 0, 179) da 0 a 180 con val il valore da mettere nel servo
    servo3.write(arr[3]);
    servo5.write(arr[4]);
    servo6.write(arr[5]);
}

ros::Subscriber<std_msgs::String> sub("servo", servo_cb);

void setup()
{
    Serial.begin(9600);
    nh.initNode();
    nh.subscribe(sub);
    servo9.attach(9); //attach it to pin 9
    servo10.attach(10); //attach it to pin 10
    servo11.attach(11); //attach it to pin 11
    servo3.attach(3); //attach it to pin 3
    servo5.attach(5); //attach it to pin 5
    servo6.attach(6); //attach it to pin 6
}

void loop()
{
    nh.spinOnce();
    delay(1000);
}

