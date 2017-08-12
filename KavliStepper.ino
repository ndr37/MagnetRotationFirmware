
#include <Arduino.h>
#include "DRV8825.h"
#include <Servo.h>
#include <digitalWriteFast.h>  // library for high performance reads and writes by jrraines
                               // see http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1267553811/0
                               // and http://code.google.com/p/digitalwritefast/
 

// Quadrature encoder
#define c_EncoderInterruptApin 3
#define c_EncoderInterruptBpin 2
//#define EncoderIsReversed
volatile bool _EncoderASet;
volatile bool _EncoderBSet;
volatile long _EncoderTicks = 0;

bool homed = false;
bool in_motion = false;

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200

// All the wires needed for full functionality
#define DIR 11
#define STEP 12
#define ENBL 13


// microstep control for DRV8825
 #define MODE0 A3
 #define MODE1 A2
 #define MODE2 A1
  DRV8825 stepper(MOTOR_STEPS, DIR, STEP, ENBL, MODE0, MODE1, MODE2);


 //setup conversion factors
 #define MAGNET_RADIUS   27.0     // in cm
 #define TIMING_GEAR_TEETH  10 // NUMBER OF TEETH ON THE PULLEY
 #define ENCODER_RADIUS 1.905   // in cm
 double gear_ratio = MAGNET_RADIUS*2*3.14159265/2.54/0.2/TIMING_GEAR_TEETH*100/89.50; //Circumference in cm to inches divided by belt pitch
 double encoder_ratio = MAGNET_RADIUS/ENCODER_RADIUS;
 double degrees_per_pulse = 1000*360.0/(1024*2*encoder_ratio)*50/46.09;


 //interlocks
 #define OPTO_ENDSTOP1_PIN 7
 #define OPTO_ENDSTOP2_PIN 8

 bool opto1_state;
 bool opto2_state;

void setup() {
    stepper.setRPM(15);
    stepper.setMicrostep(8);
    stepper.enable();

     Serial.begin(9600);
     Serial.println(gear_ratio);
     Serial.println(degrees_per_pulse);
     
    homed=false;
 
  // Quadrature encoder
  pinMode(c_EncoderInterruptApin, INPUT);      // sets pin A as input
  digitalWrite(c_EncoderInterruptApin, LOW);  // turn on pullup resistors
  pinMode(c_EncoderInterruptBpin, INPUT);      // sets pin B as input
  digitalWrite(c_EncoderInterruptBpin, LOW);  // turn on pullup resistors
  attachInterrupt(digitalPinToInterrupt(c_EncoderInterruptApin), HandleInterruptA, RISING);
  attachInterrupt(digitalPinToInterrupt(c_EncoderInterruptBpin), HandleInterruptB, RISING);

  pinMode(OPTO_ENDSTOP1_PIN, INPUT);      // sets pin A as input
  digitalWrite(OPTO_ENDSTOP1_PIN, LOW);  // turn on pullup resistors
  pinMode(OPTO_ENDSTOP2_PIN, INPUT);      // sets pin B as input
  digitalWrite(OPTO_ENDSTOP2_PIN, LOW);  // turn on pullup resistors
  
 
}

void loop() {
  
    if( Serial.available()){
      serial_parse();
    }
    delay(100);
    
}

void safe_rotate(double angle)
{
  /* I ran out of interrupts on the arduino pro mini (they are all used for 
   *  the quadrature encoder) so this function breaks every rotation down into
   *  small pieces and 
   *  
   */
  double stepper_angle = gear_ratio*angle;
  double current = 0.0;
  double dir = 1;
  stepper.enable();
  if(angle < 0){
    dir = -1;
  }
  in_motion = true;
  while(current < abs(stepper_angle)){
    opto1_state = digitalRead(OPTO_ENDSTOP1_PIN);
    opto2_state = digitalRead(OPTO_ENDSTOP2_PIN);
    if( Serial.available()){
      serial_parse();
    }
    if(!opto2_state && (dir==1)){
      stepper.rotate(1*dir);
      current+=1;
    }
    if(!opto1_state && (dir==-1)){
      stepper.rotate(1*dir);
      current+=1;
    }
    if(opto1_state){
      _EncoderTicks=0;
      homed=true;
    }
    if(!(!opto2_state && (dir==1)) && !(!opto1_state && (dir==-1))){
      break;
    }
  }
  in_motion = false;
}

int home_magnet(void){
  int i = 0;
  while(!opto1_state){
    safe_rotate(-1);
    i+=1;
    if(i>(365)){
      return -1;
    }
  }
  _EncoderTicks=0;
  homed=true;
  return 1;
  
}

int rotate_absolute(double angle){
  if(!homed){
   home_magnet();
  }
  double angle_dif = angle - _EncoderTicks*degrees_per_pulse/1000;
  safe_rotate(angle_dif);
  if(opto1_state || opto2_state){
    return -1;
  }
  return 1;
}

void serial_parse(void){
 String content;
 char character;
     while(Serial.available()) {
      character = Serial.read();
      content.concat(character);
    }

    if(content.startsWith("HOME")){
      int stat = home_magnet();
      if(stat<0){
        stepper.disable();
        //Serial.println("Homing Failed, disabling motors");
      }
      else{
        //Serial.println(stat);
      }
    }
    
    if(content.startsWith("LOC?")){
      if(!in_motion){
        Serial.println(_EncoderTicks*degrees_per_pulse/1000);
        }
      else{
        Serial.println(-1);
      }
       
    }
    
    if(content.startsWith("GOTO ")){
       content = content.substring(5);
       int stat = rotate_absolute(content.toFloat());
      /*if(stat <0){
        Serial.println(stat);
      }
      else{
        Serial.println(_EncoderTicks*degrees_per_pulse/1000);
      }*/
    }
 }


void HandleInterruptA()
{
  _EncoderASet = digitalReadFast(c_EncoderInterruptApin);    //get quadrature states
  _EncoderBSet = digitalReadFast(c_EncoderInterruptBpin);   //get quadrature states
 
  // and adjust counter + if A leads B
  #ifdef EncoderIsReversed
    if((_EncoderASet && !_EncoderBSet) || (!_EncoderASet && _EncoderBSet)){
      _EncoderTicks -= 1;
    }
    else{
      _EncoderTicks += 1;
    }
  #else
    if((_EncoderASet && !_EncoderBSet) || (!_EncoderASet && _EncoderBSet)){
      _EncoderTicks += 1;
    }
    else{
      _EncoderTicks -= 1;
    }
  #endif
}

void HandleInterruptB()
{
  _EncoderASet = digitalReadFast(c_EncoderInterruptApin);    //get quadrature states
  _EncoderBSet = digitalReadFast(c_EncoderInterruptBpin);   //get quadrature states
 
  // and adjust counter + if A leads B
  #ifdef EncoderIsReversed
    if((_EncoderASet && _EncoderBSet) || (!_EncoderASet && !_EncoderBSet)){
      _EncoderTicks -= 1;
    }
    else{
      _EncoderTicks += 1;
    }
  #else
    if((_EncoderASet && _EncoderBSet) || (!_EncoderASet && !_EncoderBSet)){
      _EncoderTicks += 1;
    }
    else{
      _EncoderTicks -= 1;
    }
  #endif
}
 
