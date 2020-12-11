 //============================================ IO Pins ============================================
// Pins used on Arduino Uno

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"


#define rEncoder 2
#define lEncoder 3

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myMotorG = AFMS.getMotor(2);
Adafruit_DCMotor *myMotorD = AFMS.getMotor(1);

//======================================== Ultrasons Variables =======================================
/* Constantes pour les broches */
const byte TRIGGER_PIN = 6; // Broche TRIGGER
const byte ECHO_PIN = 7;    // Broche ECHO

const byte TRIGGER_PIN1 = 9; // Broche TRIGGER
const byte ECHO_PIN1 = 8;    // Broche ECHO

const byte TRIGGER_PIN2 = 10; // Broche TRIGGER
const byte ECHO_PIN2 = 11;    // Broche ECHO
 
/* Constantes pour le timeout */
const unsigned long MEASURE_TIMEOUT = 25000UL; // 25ms = ~8m à 340m/s

/* Vitesse du son dans l'air en mm/us */
const float SOUND_SPEED = 340.0 / 1000;
String distance_left="";
String distance_right="";
String distance_center="";
String new_inner_command ="";
 String inner_message ;
//======================================== Global Variables =======================================

// Variables used in interrupts
volatile unsigned long rightWheel;
volatile unsigned long leftWheel;

 double somPid=0;
 double diffPid=0;

unsigned long previousMillis=0;
unsigned long timer1=0;
int interval1=100;
int interval =250;

// PID Values
double inputG=0,inputD=0, LastErrorRot=0,LastErrorLin=0, proportionalRot=0,proportionalLin=0, derivativeRot=0,derivativeLin=0, integralRot=0,integralLin=0;



double outputsPid[2];


// PID Multipliers
double kpRot = 0.4 ; //0.4
double kiRot = 0.1; //0.1
double kdRot = 0.05; //0.05

double kpLin = 0.4; // 0.4
double kiLin = 0.1; //0.1 
double kdLin = 0.05;//0.05
  
int rotation=0;
int lineaire=0;
int stopM=0;
int i=0;
int j=0;
//============================================= Setup =============================================
void setup(){
 
  Serial.begin(9600);
   AFMS.begin();
  pinMode(rEncoder, INPUT);
  pinMode(lEncoder, INPUT);
  
  pinMode(4,INPUT_PULLUP); // pin pour test
  pinMode(5,INPUT_PULLUP);
  pinMode(12,INPUT_PULLUP);
  
 
  // Enable the pull up resistors for the encoders
  digitalWrite(rEncoder, HIGH);
  digitalWrite(lEncoder, HIGH);
  //Enable interrupts 0 and 1 on pins D2 and D3
  attachInterrupt(1, rightEncoderISR, RISING);
  attachInterrupt(0, leftEncoderISR, RISING);

  /* Initialise les broches */
  
  //ULTRASON 0
  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, LOW); // La broche TRIGGER doit être à LOW au repos
  pinMode(ECHO_PIN, INPUT);
//ULTRASON 1
 pinMode(TRIGGER_PIN1, OUTPUT);
  digitalWrite(TRIGGER_PIN1, LOW); // La broche TRIGGER doit être à LOW au repos
  pinMode(ECHO_PIN1, INPUT);
//ULTRASON 2
pinMode(TRIGGER_PIN2, OUTPUT);
  digitalWrite(TRIGGER_PIN2, LOW); // La broche TRIGGER doit être à LOW au repos
  pinMode(ECHO_PIN2, INPUT);
  
}// End Setup

//============================================= Loop ==============================================
void loop(){


unsigned long currentMillis=millis();
if ( currentMillis-previousMillis>= interval){
   previousMillis=currentMillis;
      
   calculPid(rotation,lineaire);//setPointRot[rad/s] et  setPointLin [mm/s] 
  
   distance_left=ultrason0();
   distance_right=ultrason1();
   distance_center= ultrason2();
   /*Serial.println(distance_left);
   Serial.println(distance_right);
   Serial.println(distance_center);*/

      

  
}

 new_inner_command = get_serial_command_from_pi();

 if (stopM==2 && i>=72){
    Serial.println("in");  
   stopM=0;
    rotation=0;
   lineaire=0;
   integralLin=0;
   integralRot=0;
   LastErrorRot=0;
   LastErrorLin=0;
   i=0;
   
   }
if (stopM==3 && j>=72){
    Serial.println("in j");
     stopM=0;
    rotation=0;
   lineaire=0;
   integralLin=0;
   integralRot=0;
   LastErrorRot=0;
   LastErrorLin=0;
  j=0;  
   }
  if(stopM==4){
  diffPid=lineaire+10; // E : faible déviation vers la droite 
  stopM=1;
  }
if(stopM==5){
  somPid=lineaire+10;// F: faible déviation vers la gauche
  stopM=1;
  }

    refresh();



if( new_inner_command != ""){
messageDecode(new_inner_command);

}

new_inner_command="";

}

String get_serial_command_from_pi(){
  int var;
 if(Serial.available()){
  var = Serial.read();
  inner_message=(char)var;
  }
  else{
    inner_message= "";
    }
  return inner_message;
}

void messageDecode(String &cmd){
  //Serial.println(cmd);
   if(cmd=="a"){ 
    
   rotation=0;
   Serial.println("got A");
   lineaire=140;//310 si signe+, vers l'avant, si signe - vers l'arriére
 stopM=1;
  }
  else if(cmd=="b"){
    rotation=3;// si signe +, tourne à gauche
   lineaire=0;
   stopM=2;
   Serial.println("got B");

  }
  else if(cmd=="c"){
   
  Serial.println("got C");
   rotation=-3;// si signe -, tourne à droite
   lineaire=0;
stopM=3;

  
  }
  else if(cmd=="d"){
    
 Serial.println("got D");
   stopM=0;
    rotation=0;
   lineaire=0;
   integralLin=0;
   integralRot=0;
   LastErrorRot=0;
   LastErrorLin=0;
  
  }
   else if(cmd=="e"){
    
 Serial.println("got E");
    rotation=0;
   lineaire=140;
   stopM=4;

  }
   else if(cmd=="f"){
    
 Serial.println("got F");
    rotation=0;
   lineaire=140;
   stopM=5;

  }
  
  }

String ultrason0(){
 ////////////////////////////////////////////ULTRASON 0//////////////////////////////////////////////
  /* 1. Lance une mesure de distance en envoyant une impulsion HIGH de 10µs sur la broche TRIGGER */
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  
  /* 2. Mesure le temps entre l'envoi de l'impulsion ultrasonique et son écho (si il existe) */
  long measure = pulseIn(ECHO_PIN, HIGH, MEASURE_TIMEOUT);
   
  /* 3. Calcul la distance à partir du temps mesuré */
  float distance_mm = (measure / 2.0 * SOUND_SPEED)/10;
   
  /* Affiche les résultats en mm, cm et m */
return String(distance_mm);
  }

 String ultrason1(){
  
  ////////////////////////////////////////////ULTRASON 1//////////////////////////////////////////////
digitalWrite(TRIGGER_PIN1, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN1, LOW); 

  long measure1 = pulseIn(ECHO_PIN1, HIGH, MEASURE_TIMEOUT);

  float distance_mm1 = (measure1 / 2.0 * SOUND_SPEED)/10;
  
  return String(distance_mm1);
  }
  String ultrason2(){
  ////////////////////////////////////////////ULTRASON 2//////////////////////////////////////////////
digitalWrite(TRIGGER_PIN2, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN2, LOW); 

  long measure2 = pulseIn(ECHO_PIN2, HIGH, MEASURE_TIMEOUT);

  float distance_mm2 = (measure2 / 2.0 * SOUND_SPEED)/10;
  
  return String(distance_mm2);
  }
 
  
void calculPid(double setPointRot, double setPointLin){
  
        inputG = leftWheel;
        double vitG= ((inputG*4*20.7)/20); //inputG = le nombre de ticks pour 0.2s  et vitG la vitesse en tr/s car *5 pour 1sec et /20 pr 1tr et *20.7 pour passer de tr/s à mm/s
    // Serial.println("TICKS G ");
    //  Serial.println(inputG);
   
    //  Serial.println("VIT MESURE G");
      // Serial.println(vitG); 
        inputD= rightWheel;
        double vitD= ((inputD*4*20.7)/20); // vit en mm/s
    //  Serial.println("TICKS D ");
    //    Serial.println(inputD);
     // Serial.println("VIT MESURE D");
      //  Serial.println(vitD);
        
        leftWheel = 0;
        rightWheel=0;
        
      
      double mesureVitRot= (vitD-vitG)/150;// 15cm= D roue
      double mesureVitLin= (vitD+vitG)/2;
      
      /* Serial.println("LAST EROR Rot");""
        Serial.println(LastErrorRot); */
        // Calculate the PID values Rotation
        proportionalRot = setPointRot - mesureVitRot; //Erreur vit desire et vit reele
      /*Serial.println("ERROR ROT :  ");
      Serial.println(proportionalRot); */
        derivativeRot =  mesureVitRot- LastErrorRot;
        integralRot = (integralRot + proportionalRot);
      /* Serial.println("INTEGRAL ROT :  ");
      Serial.println(integralRot); */
        // Scale the PID values and save total as output
        outputsPid[0] = kpRot * proportionalRot + kdRot * derivativeRot + kiRot * integralRot;
        outputsPid[0]= outputsPid[0]*(150/2);
        
        // Save variables for next time
        LastErrorRot =mesureVitRot;
       
      
      ///////////////////////////////////////////////////////////////////////////////////////////////
         // Calculate the PID values Linear
        proportionalLin = setPointLin- mesureVitLin;//Erreur vit desire et vit reele
        derivativeLin = mesureVitLin - LastErrorLin;
        integralLin = (integralLin + proportionalLin);
        /*Serial.println("INTEGRAL LIN :  ");
      Serial.println(integralLin); 
        Serial.println("ERROR LIN :  ");
        Serial.println(proportionalLin); */
        // Scale the PID values and save total as output
        outputsPid[1] = kpLin * proportionalLin + kdLin * derivativeLin + kiLin * integralLin;
        
        // Save variables for next time
       /* Serial.println("LAST EROR Lin ");
        Serial.println(LastErrorLin); */
        LastErrorLin = mesureVitLin;
      
        somPid= (outputsPid[0]+outputsPid[1] ); // Rot+Lin  => M1=Droite
        diffPid= (outputsPid[1]-outputsPid[0]); // Lin -Rot => M2=Gauche

  

}

  
 

void refresh(){
    if(somPid>255){
      somPid=255;
    }
    if (diffPid>255){
      diffPid=255;
    }
    if(somPid<0){
      somPid=0;
    }
    if(diffPid<0){
      diffPid=0;
      }

     if(stopM==0) {
       myMotorG->setSpeed(0);
       myMotorD->setSpeed(0);
       i=0;
       j=0;
      
      }
   
        
    if(somPid<0){
        myMotorD->setSpeed(abs(somPid));// set speed of motor (0-255)
        myMotorD->run(BACKWARD);
     //  Serial.println("PWM D:  ");
      //Serial.println(somPid);
      
        }
      if(somPid>0) {
        myMotorD->setSpeed(somPid);// set speed of motor (0-255)
      myMotorD->run(FORWARD);
      // Serial.println("PWM D:  ");
     // Serial.println(somPid);
      }
     if(diffPid<0){
       myMotorG->setSpeed(abs(diffPid));// set speed of motor (0-255)
      myMotorG->run(BACKWARD);
      //Serial.println("PWM G:  ");
      //Serial.println(diffPid);
      }
     if(diffPid>0){
      myMotorG->setSpeed(diffPid);// set speed of motor (0-255)
      myMotorG->run(FORWARD);
      //Serial.println("PWM G:  ");
      //Serial.println(diffPid);
      } 
     
}



//========================================== Encoder ISRs =========================================
// Interrupts
void rightEncoderISR(){
  rightWheel++;
  i++;
}

void leftEncoderISR(){
  leftWheel++;
  j++;
}

//=================================================================================================
