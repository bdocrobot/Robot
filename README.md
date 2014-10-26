//Robot: PID Third Implement
//=====

//senior design

//--------------------------------------
//This code will instruct the robot to move forward following a line and lighting an LED when in center (error=0)
//currently, the motor control and turn functions are commented out so I can tweak the settings without it movoing. uncomment 
//the motor control function in the main loop.
//The robot will find the cross section and run in circles untill the counter is met. Exit loop is suppose to occur 
//when sensor 4 finds two other lines.
//Let me know if there is any confusion and I will attempt to address it.


#include <QTRSensors.h>
#include <DRV8835MotorShield.h>
DRV8835MotorShield motors;



//may not use these at all
//---------sensor header file used--------------------
#define NUM_SENSORS   7     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   30     // emitter is controlled by digital pin 2               

QTRSensorsRC qtrrc((unsigned char[]) {31,32,33,34,35,36,37},                    
 NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensor[NUM_SENSORS];
//----------------------------------------------------

float Kp=11; //magnitude of change required to get to set point
            //if error is high, increase
//float Ki=0.02; //vary rate of change of the physical quantity to get set point
            //if not responding fast enough, increase slightly
float Kd=7; //vary stability of system
            //increase little to a lot depending on oscillation
            
float error=0; //difference b/w set point and actual point
float P=0;     //proportional to error term
//float I=0;     //integral term, sums all previous error
float D=0;     //differential term b/w instantaneous error from set point and error from previous instant
float PID_value=0; //calculates error and sends to motor
float previous_error=0;
//int sensor[7]={0,0,0,0,0,0,0};
int initial_motor_speed=100;
int ledCount=0;

void read_sensor_value(void);
void calculate_PID(void);
void motor_control(void);

void setup()
{
  pinMode(9,OUTPUT);  //PWM for M1 speed
  pinMode(10,OUTPUT); //PWM for M2 speed
  pinMode(7,OUTPUT);  //direction of M1
  pinMode(8,OUTPUT);  //direction of M2
  pinMode(43,OUTPUT); //in-line LED
  Serial.begin(9600);
}

void loop(){
  
  read_sensor_value();
  if((sensor[0]<2400)&&(sensor[1]<2400)&&(sensor[2]<2400)&&(sensor[3]>=2400)&&(sensor[4]<2400)&&(sensor[5]<2400)&&(sensor[6]<2400))  //condition for 0 error
  {
    digitalWrite(43,HIGH);
    ledCount++;
  }else
  {digitalWrite(43,LOW);}
  calculate_PID();
 //motor_control();    //comment to stop from moving 
//  Serial.print(PID_value);
//  Serial.println();
 //stopMotors();       //function to stop (and turn)
}

void stopMotors()
{
  if((sensor[0]>=2400)&&(sensor[1]>=2400)&&(sensor[2]>=2400)&&(sensor[3]>=2400)&&(sensor[4]>=2400)&&(sensor[5]>=2400)&&(sensor[6]>=2400))
    {
      motors.setSpeeds(0,0);
      delay(1000);
      int i=0;
      while(i<2)      //**************look into this... infinite loop with ledCount in if statement
      {
        if(ledCount<2){  //attempting a counter for when the center sensor 'sees' the black line
        i++;                  //once it sees two lines, that means it has turned 180 degrees
        delay(500);
        }
        motors.setSpeeds(100,100);
        Serial.print(i);
      }
      
      delay(1500);
      error=0;
      ledCount=0;
    }
}

void read_sensor_value()
{
  
  qtrrc.read(sensor);
  
//-----------start error statements-------------------sensor[4] is no error, center of bot
   if((sensor[0]>=2400)&&(sensor[1]<2400)&&(sensor[2]<2400)&&(sensor[3]<2400)&&(sensor[4]<2400)&&(sensor[5]<2400)&&(sensor[6]<2400))
  error=-6;  //indicator of line on right side: 1000000 done
  else if((sensor[0]>=2400)&&(sensor[1]>=2400)&&(sensor[2]<2400)&&(sensor[3]<2400)&&(sensor[4]<2400)&&(sensor[5]<2400)&&(sensor[6]<2400))
  error=-5;  //line still on right              1100000 done
  else if((sensor[0]<2400)&&(sensor[1]>=2400)&&(sensor[2]<2400)&&(sensor[3]<2400)&&(sensor[4]<2400)&&(sensor[5]<2400)&&(sensor[6]<2400))
  error=-4;  //                                 0100000 done
else if((sensor[0]<2400)&&(sensor[1]>=2400)&&(sensor[2]>=2400)&&(sensor[3]<2400)&&(sensor[4]<2400)&&(sensor[5]<2400)&&(sensor[6]<2400))
  error=-3;  //                                 0110000 done
  else if((sensor[0]<2400)&&(sensor[1]<2400)&&(sensor[2]>=2400)&&(sensor[3]<2400)&&(sensor[4]<2400)&&(sensor[5]<2400)&&(sensor[6]<2400))
  error=-2;  //                                 0010000 done
  else if((sensor[0]<2400)&&(sensor[1]<2400)&&(sensor[2]>=2400)&&(sensor[3]>=2400)&&(sensor[4]<2400)&&(sensor[5]<2400)&&(sensor[6]<2400))
  error=-1; //                                  0011000 done
  else if((sensor[0]<2400)&&(sensor[1]<2400)&&(sensor[2]<2400)&&(sensor[3]>=2400)&&(sensor[4]<2400)&&(sensor[5]<2400)&&(sensor[6]<2400))
  error=0;  //                                 0001000 done
  else if((sensor[0]<2400)&&(sensor[1]<2400)&&(sensor[2]<2400)&&(sensor[3]>=2400)&&(sensor[4]>=2400)&&(sensor[5]<2400)&&(sensor[6]<2400))
  error=1;  //                                0001100 done
  else if((sensor[0]<2400)&&(sensor[1]<2400)&&(sensor[2]<2400)&&(sensor[3]<2400)&&(sensor[4]>=2400)&&(sensor[5]<2400)&&(sensor[6]<2400))
  error=2;  //                                0000100 done
  else if((sensor[0]<2400)&&(sensor[1]<2400)&&(sensor[2]<2400)&&(sensor[3]<2400)&&(sensor[4]>=2400)&&(sensor[5]>=2400)&&(sensor[6]<2400))
  error=3;  //                                0000110 done
  else if((sensor[0]<2400)&&(sensor[1]<2400)&&(sensor[2]<2400)&&(sensor[3]<2400)&&(sensor[4]<2400)&&(sensor[5]>=2400)&&(sensor[6]<2400))
  error=4;  //                                0000010 done
  else if((sensor[0]<2400)&&(sensor[1]<2400)&&(sensor[2]<2400)&&(sensor[3]<2400)&&(sensor[4]<2400)&&(sensor[5]>=2400)&&(sensor[6]>=2400))
  error=5;  //                                0000011 done
  else if((sensor[0]<2400)&&(sensor[1]<2400)&&(sensor[2]<2400)&&(sensor[3]<2400)&&(sensor[4]<2400)&&(sensor[5]<2400)&&(sensor[6]>=2400))
  error=6;  //                                0000001 done
  else if((sensor[0]<2400)&&(sensor[1]<2400)&&(sensor[2]<2400)&&(sensor[3]<2400)&&(sensor[4]<2400)&&(sensor[5]<2400)&&(sensor[6]<2400))
   {
     motors.setSpeeds(0,0);
//     if(error==6) //                           0000000
//    error=7;
//    else error=-7; 
   }
   //  //                           
//-----------end error statements---------- 
  
}

void calculate_PID()
{
  P=error;  
  //I=I+error;   //error adds to itself for every iteration
  D=error-previous_error;  //how far away from correct
  PID_value=(Kp*P)+(Kd*D);  //value to be sent to motor after scaling, pos or neg value
  previous_error=error;  //current error becomes previous error
}

void motor_control()
{
  //if PID_value is pos, left motor speed increases and right decreases
  //if PID_value is neg, right motor speed increases and left decreases
  
  //---------effective motor speed-------
  int M1_speed=initial_motor_speed-PID_value;
  int M2_speed=initial_motor_speed+PID_value;
  
  //---------speed should not exceed max PWM value----
  constrain(M1_speed,0,255);
  constrain(M2_speed,0,255);
  
  analogWrite(9,M1_speed); //left motor speed
  analogWrite(10,M2_speed); //right motor speed
  
  digitalWrite(7,HIGH);  //polarity M1
  digitalWrite(8,LOW);  //polarity M2
  
  
}





