#include <SoftwareSerial.h>
#include <NewPing.h>
 
#define TRIGGER_PIN  5
#define ECHO_PIN     8
#define MAX_DISTANCE 2000

#define RIGHT_SPEED 10
#define LEFT_SPEED 11
#define IN1 2
#define IN2 3
#define IN3 7
#define IN4 9
#define TEMP A5
#define LIGHT A2
#define GAS A3
#define FRONT_DIST A3
#define LEFT_DIST A1
#define RIGHT_DIST A0
#define BACK_DIST A4
#define FRONT 0
#define LEFT 1
#define RIGHT 2
#define BACK 3
#define BUZZ_PIN 12
#define MOVE_DET 13
#define echoPin 8 // Echo Pin
#define trigPin 5 // Trigger Pin
#define LEDPin 13 // Onboard LED
#define EchoSensor 21 // Read echo sensor
#define readRequest 20 //Request Serial.read();


int spd=0;
unsigned int previousRequest;
unsigned int ADC_buff[7];
unsigned long currentMillis;
unsigned long previousMillis;
int dist=0;

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
//SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

void setup()
{
  //Setup usb serial connection to computer
  Serial.begin(115200);
  currentMillis = 0;
  previousMillis = 0;
  previousRequest=255;

  //Setup Bluetooth serial connection to android
//  bluetooth.begin(115200);
 // bluetooth.print("$$$");
 // delay(100);
 // bluetooth.println("U,9600,N");
 // bluetooth.begin(9600);
 Serial.println("am terminat initilizarea");
 
 //motor init
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT); 
  pinMode(IN3, OUTPUT); 
  pinMode(IN4, OUTPUT);
  pinMode(RIGHT_SPEED, OUTPUT); 
  pinMode(LEFT_SPEED, OUTPUT);  
  pinMode(BUZZ_PIN, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(MOVE_DET, INPUT);
  
 
}
void echo2(){
  delay(50);
  int uS = sonar.ping();
  Serial.print("Ping: ");
  Serial.print(uS / US_ROUNDTRIP_CM);
  Serial.println("cm");
}

void wait(int interval, int sensor)
{
  currentMillis = millis();
  int ADC ;
  if(currentMillis - previousMillis > interval)
  {
    previousMillis = currentMillis;
    ADC = analogRead(sensor);
    if(sensor == A0)
    {
      
      ADC_buff[0] = ADC;
      Serial.print("Right distance: ");
      Serial.println(ADC);
    }
    else if(sensor == A1)
    {
      ADC_buff[1] = ADC;
      Serial.print("Left distance: ");
      Serial.println(ADC);
    }
    else if(sensor == A2)
    {
     
      ADC_buff[2] = ADC;
      Serial.print("Light intensity: ");
      Serial.println(ADC);
    }
    else if(sensor == A3)
    {
     
      ADC_buff[3] = ADC;
      Serial.print("Metan concentration: ");
      Serial.println(ADC);
    }
    else if(sensor == A4)
    {
     
      ADC_buff[4] = ADC;
      Serial.print("Back distance: ");
      Serial.println(ADC);
    }
    else if(sensor == A5)
    {
      ADC_buff[5] = ReadTemp();
      Serial.print("Room temperature: ");
      Serial.println(ReadTemp());
    }
    else if (sensor == EchoSensor)
    {
     // ADC_buff[6] =ADC;
     
     echo2();
     // Serial.print("Front distance: ");
      //Serial.println(a);
      delay(500);
    }
    else if(sensor == readRequest)
    {
      int a;
      //Buzz(1);
     // delay(5000);
      //Buzz(0);
      a =(int) Serial.read()-48;
      //Serial.println(a);
      decodeRequest(a);
    }
  }
}

void decodeRequest(int request)
{
static int c;

int gas;
      //  Buzz(1);
    
  if(previousRequest != request)
  {
     //Buzz(1);
    previousRequest = request;
    
    switch(request)
    {
      case 1:
     
    
      //Buzz(1);
      LeftMotor(0,254);
      RightMotor(0,254);
      break;
      case 2:
     

      LeftMotor(1,254);
      RightMotor(1,254);
      break;
      case 3:
      
                  
      LeftMotor(1,254);
      RightMotor(0,254);
      break;
      case 4:
      

      LeftMotor(0,254);
      RightMotor(1,254);
      break;
      
      case 5:
      
      LeftMotor(0, 0);
      RightMotor(0, 0);
      
      break;
      case 6:
      Serial.print("Light intensiti: ");
      Serial.println(ReadLight());
      break;
      case 9:
      Serial.print("Room temperature: ");
      Serial.println(ReadTemp());
      break;
      case 7:
      gas = ReadGas();
      Serial.print("Gas concentration: ");
      Serial.println(gas);
      if(gas > 600){
      Buzz(1);
      }else {
      Buzz(0);
      }
      break;
      
      default:
       
      // Buzz(1);
    //   delay(2000);
    //  LeftMotor(1,254);
    //  RightMotor(0,254);
    //           Serial.println("Am pornit motoarele");
      break; 
   }
  }
}
    
    

int maximumRange = 200; // Maximum range needed
int minimumRange = 0; // Minimum range needed
long duration, distance; // Duration used to calculate distance


long echo() 
{
   digitalWrite(BUZZ_PIN, LOW); 
  // delay(2000);
  //  digitalWrite(BUZZ_PIN, LOW); 
/* The following trigPin/echoPin cycle is used to determine the
 distance of the nearest object by bouncing soundwaves off of it. */ 
 digitalWrite(trigPin, LOW); 
 
 digitalWrite(trigPin, HIGH);
 digitalWrite(trigPin, LOW);
 duration = pulseIn(echoPin, HIGH);
 
 //Calculate the distance (in cm) based on the speed of sound.
 distance = (duration/2) /29.1;
 
//  if (distance < 4) {  // This is where the LED On/Off happens
//   Serial.print(distance);
//      Serial.println(" cm");
//}
//  else {
//   // digitalWrite(led,LOW);
//   // digitalWrite(led2,HIGH);
//  }
//  if (distance >= 20 || distance <= 0){
//    Serial.println("Out of range");
//  }
//  else {
    Serial.print(distance);
    Serial.println(" cm");
  
  delay(1000);
 }





void LeftMotor(char dir, unsigned char speed)
{
  if(dir == 0)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(LEFT_SPEED, speed);
  }
  else
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(LEFT_SPEED, speed);
  }     
}


void RightMotor(char dir, unsigned char speed)
{

  if(dir == 0)
  {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(RIGHT_SPEED, speed);
  }
  else
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(RIGHT_SPEED, speed);
  }     
}

int ReadTemp()
{
  int t = analogRead(A5);
  int j;
  j = (((5*t)/1024)*100)-274;

  return j;

}


int ReadLight()
{
    int sensorValue = 0;
    
    sensorValue = analogRead(LIGHT);
    
    return (sensorValue*5)/1024;
}

int ReadGas()
{
    int sensorValue = 0;
    
    sensorValue = analogRead(GAS);
    
    return sensorValue;
}
/*
int ReadDistance(char dir)
{
  int sensorValue = 0;
  switch(dir)
  {
    case FRONT:
    {
      //sensorValue = analogRead(FRONT_DIST);
      break;
    }
        case BACK:
    {
      sensorValue = analogRead(BACK_DIST);
      break;
    }
        case RIGHT:
    {
      sensorValue = analogRead(RIGHT_DIST);
      break;
    }
        case LEFT:
    {
      sensorValue = analogRead(LEFT_DIST);
      break;
    }
  }
  
  return sensorValue;
}*/

void Buzz(char c)
{
  if(c == 1)
      digitalWrite(BUZZ_PIN, HIGH);
      else
      digitalWrite(BUZZ_PIN, LOW);
      
}


void loop()
{
/*  //Read from bluetooth and write to usb serial
  if(bluetooth.available())
  {
    char toSend = (char)bluetooth.read();
    bluetooth.print(toSend);
 //   Serial.print('a');
  }
dist=dist+random(5);
spd=random(10,20);
bluetooth.print("S:");
bluetooth.print(spd);
bluetooth.print(",D:");
bluetooth.print(dist);
bluetooth.println("|");
Serial.println("Gata");
delay(2000);

*/
  
  
//    int c;
//  //  char request = (char)Serial.read();
//    c = ReadLight();
//    Serial.println(c);
//  
//  delay(1000);

//wait(1000,A5);
//wait(70,A1);
//wait(80,A4);
//wait(20,A5);
//wait(90,A2);
//wait(15,20);
wait(10,20);
/*echo();
 //Delay 50ms before next reading.
 delay(50);
 */
 /* LeftMotor( 1, 254);
  RightMotor( 1, 254);
  delay(10000);*/
 // LeftMotor( 0, 254);
 // RightMotor( 0, 254);
 // delay(10000);
 /* LeftMotor( 1, 254);
  RightMotor( 0, 254);
  delay(10000);*/
/*  LeftMotor( 0, 254);
  RightMotor( 1, 254);
  delay(10000);*/
  /*LeftMotor( 0, 254);
  RightMotor( 0, 254);
 delay(2000);
   LeftMotor( 0, 0);
  RightMotor( 0, 0);
  delay(2000);*/
  /*int c;
  for(c = 0; c <255; c++)
  {
    LeftMotor( 0, c);
    RightMotor( 0, c);
    delay(100);
  }
  
  for(c = 254; c >= 0; c--)
  {
    LeftMotor( 0, c);
    RightMotor( 0, c);
    delay(100);
  }
  */
 
 


}
