 //============================================ IO Pins ============================================
// Pins used on Arduino Uno

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"


#define rEncoder 2 
#define lEncoder 3

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myMotorG = AFMS.getMotor(2); // M2 du shield
Adafruit_DCMotor *myMotorD = AFMS.getMotor(1); // M1 du shield

//======================================== Ultrasons Variables =======================================
/* Constantes pour les broches */
const byte TRIGGER_PIN = 6; // Broche TRIGGER  LEFT ultrason
const byte ECHO_PIN = 7;    // Broche ECHO

const byte TRIGGER_PIN1 = 9; // Broche TRIGGER  RIGHT ultason
const byte ECHO_PIN1 = 8;    // Broche ECHO

const byte TRIGGER_PIN2 = 10; // Broche TRIGGER  CENTER ultrason
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
int i=0;
int j=0;

// PID Values
double inputG=0,inputD=0, LastErrorRot=0,LastErrorLin=0, proportionalRot=0,proportionalLin=0, derivativeRot=0,derivativeLin=0, integralRot=0,integralLin=0;
double outputsPid[2];
double somPid=0;
double diffPid=0;
int rotation=0;
int lineaire=0;

// PID Multipliers
double kpRot = 0.4 ; //0.4
double kiRot = 0.1; //0.1
double kdRot = 0.05; //0.05

double kpLin = 0.4; // 0.4
double kiLin = 0.1; //0.1 
double kdLin = 0.05;//0.05

//Other variables
int stopM=0;
unsigned long previousMillis=0;
int interval =250;

//============================================= Setup =============================================
void setup(){
 
  Serial.begin(9600);
  AFMS.begin();
  pinMode(rEncoder, INPUT);
  pinMode(lEncoder, INPUT);
 
  // Enable the pull up resistors for the encoders
  digitalWrite(rEncoder, HIGH);
  digitalWrite(lEncoder, HIGH);
  //Enable interrupts 0 and 1 on pins D2 and D3
  attachInterrupt(1, rightEncoderISR, RISING);
  attachInterrupt(0, leftEncoderISR, RISING);
  
  //ULTRASON 0 - LEFT
  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, LOW); // La broche TRIGGER doit être à LOW au repos
  pinMode(ECHO_PIN, INPUT);
  //ULTRASON 1 -RIGHT
 pinMode(TRIGGER_PIN1, OUTPUT);
 digitalWrite(TRIGGER_PIN1, LOW); // La broche TRIGGER doit être à LOW au repos
  pinMode(ECHO_PIN1, INPUT);
  //ULTRASON 2 - CENTER
  pinMode(TRIGGER_PIN2, OUTPUT);
  digitalWrite(TRIGGER_PIN2, LOW); // La broche TRIGGER doit être à LOW au repos
  pinMode(ECHO_PIN2, INPUT);
  
}// End Setup

//============================================= Loop ==============================================
void loop(){


    unsigned long currentMillis=millis();
    if ( currentMillis-previousMillis>= interval){ // boucle appellé toutes les 250 ms
       previousMillis=currentMillis;  
       calculPid(rotation,lineaire);//setPointRot[rad/s] et  setPointLin [mm/s]  
       distance_left=ultrason0(); 
       distance_right=ultrason1();
       distance_center= ultrason2();
       /*Serial.println(distance_left); // Uncoment pour l'envoi au raspery by
       Serial.println(distance_right);
       Serial.println(distance_center);*/   
    }
    
     new_inner_command = get_serial_command_from_pi(); // récuperation de la commande reçue sur le serial
    
    if (stopM==2 && i>=72){ // si "b" recu ET la roue gauche à fait >= 72 ticks(=90°) alors:
       stopM=0;
       rotation=0;   // mis à l'arret du moteur et reset des erreurs de l'integrateur
       lineaire=0;
       integralLin=0;
       integralRot=0;
       LastErrorRot=0;
       LastErrorLin=0;
       i=0; // reset du nombre de ticks à 0
       }
    if (stopM==3 && j>=72){ // si "c" recu ET la roue droite à fait >= 72 ticks(=90°) alors:
        stopM=0;// mis à l'arret du moteur et reset des erreurs de l'integrateur
        rotation=0;
        lineaire=0;
        integralLin=0;
        integralRot=0;
        LastErrorRot=0;
        LastErrorLin=0;
        j=0;  
       }
    if(stopM==4){ // si "e" reçu :
          diffPid=lineaire+10; // E : faible déviation vers la droite,addition dans lePWM du moteur gauche
          stopM=1;
      }
    if(stopM==5){//si "f" reçu :
        somPid=lineaire+10;// F: faible déviation vers la gauche, addition dans lePWM du moteur droit
        stopM=1;
      }
    
    refresh(); // fonction qui met les valeurs calculés par le PID sur les moteurs
    
    if( new_inner_command != ""){ //si le port série contient qlq chose
      messageDecode(new_inner_command); 
    }
    new_inner_command=""; // on remet le buffer à vide
    
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
   if(cmd=="a"){ 
     rotation=0;
     lineaire=140;//Setpoint à atteindre= vers l'avant
     stopM=1;
  }
  else if(cmd=="b"){
    rotation=3;// si signe +, tourne à gauche
    lineaire=0;
    stopM=2;
  }
  else if(cmd=="c"){
   rotation=-3;// si signe -, tourne à droite
   lineaire=0;
   stopM=3;
  }
  else if(cmd=="d"){
   stopM=0;
   rotation=0;
   lineaire=0;
   integralLin=0;
   integralRot=0;
   LastErrorRot=0;
   LastErrorLin=0;
  }
  else if(cmd=="e"){
   rotation=0;
   lineaire=140;
   stopM=4;
  }
  else if(cmd=="f"){
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
        inputD= rightWheel;
        double vitG= ((inputG*4*20.7)/20); //inputG = le nombre de ticks pour 0.2s  et vitG la vitesse en tr/s car *4 pour 1sec et /20 pr 1tr et *20.7 pour passer de tr/s à mm/s
        double vitD= ((inputD*4*20.7)/20); // vit en mm/s
        
        leftWheel = 0; // remise à zero du nombre de ticks chaque 250ms
        rightWheel=0;
      
        double mesureVitRot= (vitD-vitG)/150;// 15cm= D roue
        double mesureVitLin= (vitD+vitG)/2;
      
  
        // Calculate the PID values Rotation
        proportionalRot = setPointRot - mesureVitRot; //Erreur vit desire et vit reele
        derivativeRot =  mesureVitRot- LastErrorRot;
        integralRot = (integralRot + proportionalRot);
        // Scale the PID values and save total as output
        outputsPid[0] = kpRot * proportionalRot + kdRot * derivativeRot + kiRot * integralRot;
        outputsPid[0]= outputsPid[0]*(150/2);
        // Save last measured error
        LastErrorRot =mesureVitRot;
        
      ///////////////////////////////////////////////////////////////////////////////////////////////
      
         // Calculate the PID values Linear
        proportionalLin = setPointLin- mesureVitLin;//Erreur vit desire et vit reele
        derivativeLin = mesureVitLin - LastErrorLin;
        integralLin = (integralLin + proportionalLin);
        // Scale the PID values and save total as output
        outputsPid[1] = kpLin * proportionalLin + kdLin * derivativeLin + kiLin * integralLin;      
        //Save last measured error
        LastErrorLin = mesureVitLin;
        //////OUTPUTS PID//////
        somPid= (outputsPid[0]+outputsPid[1] ); // Rot+Lin  => M1=Droite
        diffPid= (outputsPid[1]-outputsPid[0]); // Lin -Rot => M2=Gauche
}


void refresh(){// directives envoyés au moteur
    if(somPid>255){ // overflow treatment
      somPid=255;
    }
    if (diffPid>255){
      diffPid=255;
    }
    if(somPid<0){ // underflow treatment
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
        }
    if(somPid>0) {
        myMotorD->setSpeed(somPid);// set speed of motor (0-255)
        myMotorD->run(FORWARD);
      }
     if(diffPid<0){
       myMotorG->setSpeed(abs(diffPid));// set speed of motor (0-255)
       myMotorG->run(BACKWARD);
      }
     if(diffPid>0){
      myMotorG->setSpeed(diffPid);// set speed of motor (0-255)
      myMotorG->run(FORWARD);
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
