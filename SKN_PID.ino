//Libraries
#include <PID_v1.h>
#include <Chrono.h>
Chrono timer;
//Pins 
//pins 2,3 error handling of motor driver
#define E1 6  // right engine pwm
#define E2 5  // left engine pwm
#define M1 7  //right direction
#define M2 4  //left direction
#define encoderPinA 3     //Encoder1 - right engine A
#define encoderPinB 11    //Encoder1 B
#define encoderPinC 2     //Encoder2 - left engine A
#define encoderPinD 10    //Encoder2 B

//variables for engines 
int counter=0; //error_handling
int val_x; //serial_incoming_data
int base_speed;//base_speed
int left_motor_pwm=0; //pwm handling
int right_motor_pwm=0;
int left_motor_pwm_c=0;
int right_motor_pwm_c=0;
int left_motor_direction;
int right_motor_direction;
bool LMD,RMD;//direction handling
int val;

//variables for encoders
double encoderPos1 = 0; //right
double encoderPos2 = 0; //left
int encoderPinALast = LOW;
int encoderPinANow = LOW;
int encoderPinCLast = LOW;
int encoderPinCNow = LOW;
int counterR=0,counterL=0;//rotation counter (encoder)

//PID
double w_left=0, w_right=0;
double y_left=0, y_right=0;
int sampletime=20;
double Kpl = 0.3038 + 1, Kpp =  0.2971 + 1;
double Kil = 2.7033 + 3.6, Kip = 2.9430 + 3.6;
double Kdl =  0.0039 + 0.02, Kdp =  0.0037 + 0.02;
double Setpoint1=0, Setpoint2=0, Input1, Output1, Input2, Output2;
//PID myPID1(&Input1, &Output1, &Setpoint1, Kpp, Kip, Kdp, DIRECT); //right engine
//PID myPID2(&Input2, &Output2, &Setpoint2, Kpl, Kil, Kdl, DIRECT); //left
PID myPID1(&Input2, &Output2, &Setpoint2, Kpp, Kip, Kdp, DIRECT); //right engine
PID myPID2(&Input1, &Output1, &Setpoint1, Kpl, Kil, Kdl, DIRECT); //left
unsigned short iterator=0;

void setup(void)
{
  int i;
  for(i=4;i<=7;i++)
  pinMode(i, OUTPUT);  
  Serial.begin(19200);      //Set Baud Rate
  Serial.setTimeout(7);
  digitalWrite(E1,LOW);  
  digitalWrite(E2,LOW);
  
  attachInterrupt(digitalPinToInterrupt(encoderPinA), enco1, FALLING);
  attachInterrupt(digitalPinToInterrupt(encoderPinC), enco2, FALLING);
  myPID1.SetMode(AUTOMATIC);
  myPID2.SetMode(AUTOMATIC);
  myPID1.SetSampleTime(20);
  myPID2.SetSampleTime(20);
  Setpoint1 = w_left;
  Setpoint2 = w_right;
  timer.restart();
  delay(500);
}


void loop(void)
{
 
  //read DATA via Serial
  if (Serial.available())
  {
    val_x=Serial.readString().toInt();  
   
  //sth like dataframe, but in only one number (from serial)
  // multiply-PWM_R-PWM_L-Direction_R-Direction_L
  base_speed=(val_x/10000); // 0-2
  if(base_speed != 1 && base_speed != 2)
  {
    //Serial.println ("Bad base speed");
    base_speed=0;
  }
//
  right_motor_pwm=(val_x/1000 - base_speed*10);   //0-9
  left_motor_pwm=(val_x/100 - base_speed*100 - right_motor_pwm*10); //0-9
  right_motor_direction=val_x/10 - base_speed*1000 - right_motor_pwm*100 - left_motor_pwm*10; //1 or 2
  left_motor_direction=val_x - base_speed*10000 - right_motor_pwm*1000 - left_motor_pwm*100 - right_motor_direction*10; //1 or 2

  if(base_speed==2)
  {
    left_motor_pwm=left_motor_pwm+9;
    right_motor_pwm=right_motor_pwm+9;
  } 
  if(right_motor_pwm == 0 && left_motor_pwm == 0)
  {
     w_right = 0;
    w_left = 0;
 
  }
  else
  {
    if(right_motor_pwm == 0)
    {
      w_right = 0;
      w_left = map(left_motor_pwm, 1, 9, 30, 210);
    }else
    if(left_motor_pwm == 0)
    {
      w_left = 0;
      w_right = map(right_motor_pwm, 1, 9, 30, 210);
    }else
    {
      w_right = map(right_motor_pwm, 1, 9, 30, 210);
      w_left = map(left_motor_pwm, 1, 9, 30, 210);
    }
  }  

  }

  
 if(timer.hasPassed(20))
 {
  iterator=iterator+1;   
  y_right = (60*encoderPos1/(500*0.02));//rpm
  y_left = (60*encoderPos2/(500*0.02));//rpm
  encoderPos1 = encoderPos2 = 0;
  timer.restart();
  }

  if(iterator==45)
  {
    iterator=0;
  }
   
   //manual int to bool conversion;
   if(left_motor_direction==1) LMD=LOW;
   if(right_motor_direction==1) RMD=LOW;
   if(left_motor_direction==2) LMD=HIGH;  
   if(right_motor_direction==2) RMD=HIGH;
 Setpoint1= w_left;
 Input1 = y_left;
 Setpoint2 = w_right;
 Input2 = y_right;
 myPID1.Compute();
 myPID2.Compute();
 //applying new motor parameters
 //digitalWrite(M1,LMD);
 //digitalWrite(M2,RMD);
 digitalWrite(M2,LMD);
 digitalWrite(M1,RMD);
 right_motor_pwm_c = map(right_motor_pwm, 1, 18, 50, 255);
 //analogWrite(E1,Output1); //lewy
 //analogWrite(E2,Output2); //prawy
 analogWrite(E2,Output1); //lewy
 analogWrite(E1,Output2); //prawy
}

void enco1 ()
{
    encoderPinANow = digitalRead(encoderPinA);
    if (digitalRead(encoderPinB) == HIGH) {
      encoderPos1++;
    } else {
      encoderPos1++;//--
    }
     encoderPinALast = encoderPinANow;
  }
  
void enco2 ()
{
    encoderPinCNow = digitalRead(encoderPinC);
    if (digitalRead(encoderPinD) == HIGH) {
      encoderPos2++;//--
    } else {
      encoderPos2++;
    }
  encoderPinCLast = encoderPinCNow;
}

void stop(void)                    //Stop
{
  digitalWrite(E1,0);
  digitalWrite(M1,LOW);    
  digitalWrite(E2,0);  
  digitalWrite(M2,LOW);    
}  


void current_sense()                  // current sense and diagnosis
{
  int val1=digitalRead(2);
  int val2=digitalRead(3);
  if(val1==HIGH || val2==HIGH){
    counter++;
    if(counter==3){
      counter=0;
      Serial.println("Motror Driver Warning");
    }  
  }
}
