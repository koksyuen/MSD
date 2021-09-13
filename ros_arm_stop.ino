#define servo0pin 3
#define servo1pin 5
#define servo2pin 6
#define servo3pin 9
#define servo4pin 10
#define gripperpin 11

#define btn1 A1 //stop btn
#define btn2 A2 //Gantry system input signal
#define btn3 A3 //Nut Fastening input signal 
#define led1 A4 //Nut Fastening output signal
#define led2 A5 //Gantry system output signal
#define ledStop A0 //Stop led

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <msd_arm/armAngle.h>

/* Gripper Angle */
#define RELEASE 90
#define GRIP_SPACER 138
#define GRIP_BOLT 140
#define GRIP_BOLT_HEAD 128

// Servo motor declartion
Servo servo0, servo1, servo2, servo3, servo4, gripper;

/**********************************************************************/
/* gripper_state (based on gripping object)                           */ 
/* 0 - release object                                                 */       
/* 1 - grip spacer                                                    */        
/* 2 - grip bolt                                                      */
/* 3 - grip bolt head                                                 */         
/**********************************************************************/
uint8_t gripState = 0;

//Stop Btn
uint8_t lastBtn1State;    // the previous state of button
uint8_t currentBtn1State; // the current state of button

//Gantry system input signal
uint8_t lastBtn2State;    // the previous state of button
uint8_t currentBtn2State; // the current state of button

//Nut Fastening input signal
uint8_t lastBtn3State;    // the previous state of button
uint8_t currentBtn3State; // the current state of button



bool stopping;

/*================================================*/
/*          ROS RELATED DECLARATION               */
/*================================================*/

ros::NodeHandle nh;
std_msgs::UInt8 state;
std_msgs::Bool stop_system;

void servo_cb(const msd_arm::armAngle&);
void state_cb(const std_msgs::UInt8&);

ros::Publisher statePub("arm_state", &state);
ros::Publisher stopPub("arm_stop", &stop_system);
ros::Subscriber<std_msgs::UInt8> stateSub("arm_state", state_cb);
ros::Subscriber<msd_arm::armAngle> sub("joint_angle", servo_cb);

void setup(){
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(stateSub);
  nh.advertise(statePub);
  nh.advertise(stopPub);

  servo0.attach(3,500,2500); 
  servo1.attach(5,500,2500); 
  servo2.attach(6,550,2500);
  servo3.attach(9,550,2500); 
  servo4.attach(10,575,2450);
  gripper.attach(11,500,2500); 

  delay(1);
  servo0.write(90);
  servo1.write(135);
  servo2.write(180);
  servo3.write(90);
  servo4.write(180);
  gripper.write(RELEASE);

  pinMode(btn1, INPUT);
  pinMode(btn2, INPUT);
  pinMode(btn3, INPUT);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(ledStop, OUTPUT);

  currentBtn1State = digitalRead(btn1);
  currentBtn2State = digitalRead(btn2);
  currentBtn3State = digitalRead(btn3);

  stopping = true;  // the system is stopping initially
}

void loop(){
  lastBtn1State    = currentBtn1State;      // save the last state
  currentBtn1State = digitalRead(btn1); // read new state

  lastBtn2State    = currentBtn2State;      // save the last state
  currentBtn2State = digitalRead(btn2); // read new state
  
  lastBtn3State    = currentBtn3State;      // save the last state
  currentBtn3State = digitalRead(btn3); // read new state

  /**********************************************************************/
  /* Stop Button Mechanism                                              */
  /**********************************************************************/

  //when stop button is pressed
  if(lastBtn1State == LOW && currentBtn1State == HIGH) stopping = true;

  if (stopping){
    /* Stop Btn pressed once */
    stop_system.data = stopping;
    stopPub.publish( &stop_system ); // inform moveit to stop robot arm movement
    digitalWrite(ledStop, HIGH);
    
    while (stopping){ //infinity loop until the stop button is pressed again
      lastBtn1State    = currentBtn1State;      // save the last stop btn state
      currentBtn1State = digitalRead(btn1);     // read new stop btn state

      // if stop button is pressed again
      if(lastBtn1State == LOW && currentBtn1State == HIGH) stopping = false;
      nh.spinOnce();
    }

    /* Stop Btn pressed twice */
    stop_system.data = stopping; // inform moveit to resume robot arm movement
    stopPub.publish( &stop_system );
    digitalWrite(ledStop, LOW);
  }
  
  /**********************************************************************/
  /* Input signal from Gantry System                                    */
  /* publish arm_state to main.py                                       */
  /* arm_state:                                                         */        
  /* 1 - insert spacer (by main.py)                                     */        
  /* 2 - inform gantry system to start its task                         */
  /* 3 - insert bolt   (by main.py)                                     */         
  /* 4 - inform nut fastening system to start its task                  */
  /* 5 - go back home position   (by main.py)                           */
  /**********************************************************************/

  // detect rising signal
  if(lastBtn2State == LOW && currentBtn2State == HIGH) {

    // after Gantry system has move the completely assembled to the final destination
    if (state.data == 5) { 
      state.data = 1; // inform main.py to insert spacer
      statePub.publish( &state );
    }
    // after Gantry system has move the wheel to castor yoke
    else if (state.data == 2) {
      state.data += 1; // inform main.py to insert bolt
      statePub.publish( &state );
    }
    
  }


  /**********************************************************************/
  /* Input signal from Nut Fastening System                             */
  /* publish arm_state to main.py                                       */
  /* arm_state:                                                         */        
  /* 1 - insert spacer (by main.py)                                     */        
  /* 2 - inform gantry system to start its task                         */
  /* 3 - insert bolt   (by main.py)                                     */         
  /* 4 - inform nut fastening system to start its task                  */
  /* 5 - go back home position   (by main.py)                           */
  /**********************************************************************/

  // detect rising signal
  if(lastBtn3State == LOW && currentBtn3State == HIGH) {
    // after Nut Fastening System has fasten the nut
    if (state.data == 4) {
      state.data += 1;  // inform main.py go back to home position
      statePub.publish( &state );
    }
  }


  /**********************************************************************/
  /* Adjust gripper angle based on gripping object                      */
  /* gripper_state:                                                     */ 
  /* 0 - release object                                                 */       
  /* 1 - grip spacer                                                    */        
  /* 2 - grip bolt                                                      */
  /* 3 - grip bolt head                                                 */         
  /**********************************************************************/
  switch (gripState) {
  case 0: 
    gripper.write(RELEASE);
    break;
  case 1: 
    gripper.write(GRIP_SPACER);
    break;
  case 2: 
    gripper.write(GRIP_BOLT);
    break;
  case 3: 
    gripper.write(GRIP_BOLT_HEAD);
    break;
  }
  
  nh.spinOnce();
}

/**********************************************************************/
/* Subscribe joint angle calculated by moveit                         */
/* command servo motor to execute the joint angle                     */         
/**********************************************************************/

void servo_cb(const msd_arm::armAngle& angle){

  servo0.write(angle.joint0);
  servo1.write(angle.joint1);
  servo2.write(angle.joint2);
  servo3.write(angle.joint3);
  servo4.write(angle.joint4);
  gripState = angle.gripper;
}

  /**********************************************************************/
  /* Subscribe arm_state published by main.                             */
  /* arm_state:                                                         */        
  /* 1 - insert spacer (by main.py)                                     */        
  /* 2 - inform gantry system to start its task                         */
  /* 3 - insert bolt   (by main.py)                                     */         
  /* 4 - inform nut fastening system to start its task                  */
  /* 5 - go back home position   (by main.py)                           */
  /**********************************************************************/

void state_cb(const std_msgs::UInt8& arm_state){

  state.data = arm_state.data;
  
  switch (arm_state.data) {
  case 1: // inserting spacer   
    digitalWrite(led2, LOW);    // no any output signal   
    break;
  case 2: // inform gantry system to start its task    
    digitalWrite(led2, HIGH);  // output signal to Gantry system 
    break;
  case 3: // inserting bolt
    digitalWrite(led2, LOW);   // no any output signal   
    break;
  case 4: // inform nut fastening system to start its task  
    digitalWrite(led1, HIGH);  // output signal to Nut Fastening system
    break;
  case 5: // going back home position
    digitalWrite(led1, LOW);   // no output signal to Nut Fastening system
    digitalWrite(led2, HIGH);  // inform Gantry System to move completely assembled castor wheel to final destination  
    break;
  }
}
