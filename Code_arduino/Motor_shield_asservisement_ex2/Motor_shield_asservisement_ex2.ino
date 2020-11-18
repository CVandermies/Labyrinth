//============================================ IO Pins ============================================
// Pins used on Arduino Uno

#include <Wire.h>
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


//======================================== Global Variables =======================================

// Variables used in interrupts
volatile unsigned long rightWheel;
volatile unsigned long leftWheel;

 double somPid=0;
 double diffPid=0;

unsigned long previousMillis=0;

int interval =100;

// PID Values
double inputG=0,inputD=0, LastErrorRot=0,LastErrorLin=0, proportionalRot=0,proportionalLin=0, derivativeRot=0,derivativeLin=0, integralRot=0,integralLin=0;



double outputsPid[2];
// pidcount is used to divide the total error (integral formula)
int pidcountRot = 1;
int pidcountLin =1;

// PID Multipliers
double kpRot = 0.1; // 0.1
double kiRot = 0; 
double kdRot = 0; 

double kpLin = 0.6; // 0.6
double kiLin = 0; //entre 0.2 et 0.6 ?
double kdLin = 0;
  
int rotation=0;
int lineaire=0;

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
  attachInterrupt(0, rightEncoderISR, RISING);
  attachInterrupt(1, leftEncoderISR, RISING);

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

 ultrason0();
 ultrason1();
 ultrason2();

 
}

  if(digitalRead(4)==0){ // ici viens la commande aller tout droit de la raspbery
   rotation=0;
   lineaire=350;//350
   goStraigth();
  }
  else if(digitalRead(5)==0){// ici viens la commande aller à droite de la raspbery
  
    rotation=20;// si signe +, tourne à droite
   lineaire=0;
  turn();
  delay(250);
  }
  else if(digitalRead(12)==0){// ici viens la commande aller à gauche de la raspbery
  
   rotation=-20;// si signe -, tourne à gauche 
   lineaire=0;
  turn();
  delay(250);
  
  }
  else{
   Stop();
    
  }



}

void ultrason0(){
 ////////////////////////////////////////////ULTRASON 0//////////////////////////////////////////////
  /* 1. Lance une mesure de distance en envoyant une impulsion HIGH de 10µs sur la broche TRIGGER */
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  
  /* 2. Mesure le temps entre l'envoi de l'impulsion ultrasonique et son écho (si il existe) */
  long measure = pulseIn(ECHO_PIN, HIGH, MEASURE_TIMEOUT);
   
  /* 3. Calcul la distance à partir du temps mesuré */
  float distance_mm = measure / 2.0 * SOUND_SPEED;
   
  /* Affiche les résultats en mm, cm et m */
  Serial.print(F("Distance 0: "));
  Serial.print(distance_mm);
  }

  void ultrason1(){
  
  ////////////////////////////////////////////ULTRASON 1//////////////////////////////////////////////
digitalWrite(TRIGGER_PIN1, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN1, LOW); 

  long measure1 = pulseIn(ECHO_PIN1, HIGH, MEASURE_TIMEOUT);

  float distance_mm1 = measure1 / 2.0 * SOUND_SPEED;
  Serial.print(F("Distance 1: "));
  Serial.print(distance_mm1);
  
  
  
  }
  void ultrason2(){
  ////////////////////////////////////////////ULTRASON 2//////////////////////////////////////////////
digitalWrite(TRIGGER_PIN2, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN2, LOW); 

  long measure2 = pulseIn(ECHO_PIN2, HIGH, MEASURE_TIMEOUT);

  float distance_mm2 = measure2 / 2.0 * SOUND_SPEED;
  Serial.print(F("Distance 2: "));
  Serial.println(distance_mm2);
  
  
  }
 
  
void calculPid(double setPointRot, double setPointLin){
  
         
           Serial.println("PID");
        inputG = leftWheel;
        double vitG= ((inputG*10*20.7)/20); //inputG = le nombre de ticks pour 0.1s  et vitG la vitesse en tr/s car *10 pour 1sec et /20 pr 1tr et *20.7 pour passer de tr/s à mm/s
      /*Serial.println("TICKS G ");
        Serial.println(inputG);
      Serial.println("VIT MESURE G");
       Serial.println(vitG); */
        inputD= rightWheel;
        double vitD= ((inputD*10*20.7)/20); // vit en mm/s
      /* Serial.println("TICKS D ");
        Serial.println(inputD);
      Serial.println("VIT MESURE D");
        Serial.println(vitD);*/
        
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
        derivativeRot = proportionalRot - LastErrorRot;
        integralRot = (integralRot + proportionalRot)/pidcountRot; 
      /* Serial.println("INTEGRAL ROT :  ");
      Serial.println(integralRot); */
        // Scale the PID values and save total as output
        outputsPid[0] = kpRot * proportionalRot + kdRot * derivativeRot + kiRot * integralRot;
        outputsPid[0]= outputsPid[0]*(150/2);
        
        // Save variables for next time
        LastErrorRot = proportionalRot;
        pidcountRot++;
      
      ///////////////////////////////////////////////////////////////////////////////////////////////
         // Calculate the PID values Linear
        proportionalLin = setPointLin- mesureVitLin;//Erreur vit desire et vit reele
        derivativeLin = proportionalLin - LastErrorLin;
        integralLin = (integralLin + proportionalLin/pidcountLin);
        /*Serial.println("INTEGRAL LIN :  ");
      Serial.println(integralLin); 
        Serial.println("ERROR LIN :  ");
        Serial.println(proportionalLin); */
        // Scale the PID values and save total as output
        outputsPid[1] = kpLin * proportionalLin + kdLin * derivativeLin + kiLin * integralLin;
        
        // Save variables for next time
       /* Serial.println("LAST EROR Lin ");
        Serial.println(LastErrorLin); */
        LastErrorLin = proportionalLin;
        pidcountLin++;
      
        somPid= (outputsPid[0]+outputsPid[1] ); // Rot+Lin  => M1=Droite
        diffPid= (outputsPid[1]-outputsPid[0]); // Lin -Rot => M2=Gauche

  

}

  
 
void turn(){
  
Serial.println("je tourne");
if(somPid<0){
        myMotorD->setSpeed(abs(somPid));// set speed of motor (0-255)
        myMotorD->run(FORWARD);
     //  Serial.println("PWM D:  ");
     // Serial.println(somPid);
      
        }
      if(somPid>0) {
        myMotorD->setSpeed(somPid);// set speed of motor (0-255)
      myMotorD->run(BACKWARD);
      // Serial.println("PWM D:  ");
      //Serial.println(somPid);
      }
     if(diffPid<0){
       myMotorG->setSpeed(abs(diffPid));// set speed of motor (0-255)
      myMotorG->run(FORWARD);
      //Serial.println("PWM G:  ");
      //Serial.println(diffPid);
      }
     if(diffPid>0){
      myMotorG->setSpeed(diffPid);// set speed of motor (0-255)
      myMotorG->run(BACKWARD);
      //Serial.println("PWM G:  ");
      //Serial.println(diffPid);
      } 
  
  }
void goStraigth(){
  Serial.println("GO STRAIGT");
    if(somPid<0){
        myMotorD->setSpeed(abs(somPid));// set speed of motor (0-255)
        myMotorD->run(BACKWARD);
     //  Serial.println("PWM D:  ");
     // Serial.println(somPid);
      
        }
      if(somPid>0) {
        myMotorD->setSpeed(somPid);// set speed of motor (0-255)
      myMotorD->run(FORWARD);
      // Serial.println("PWM D:  ");
      //Serial.println(somPid);
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

void Stop(){
  Serial.println("STOP");
  myMotorD->run(RELEASE);
  myMotorG->run(RELEASE);
  }

//========================================== Encoder ISRs =========================================
// Interrupts
void rightEncoderISR(){
  rightWheel++;
 
}

void leftEncoderISR(){
  leftWheel++;
}

//=================================================================================================
