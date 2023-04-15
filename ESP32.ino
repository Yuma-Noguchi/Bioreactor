// I2C

#include <Wire.h>
float desired_Temp, desired_pH, desired_RPM;

// Heating
#define beta 4510 // beta of thermistor
float Temp;
int target_Temp = 30;

// pH
int PH_Max = 7;
float target_pH = 5;
int PH_Min = 3;
int PH_N_pin = A0;
int PH_P_pin = A1;
float F=96485.309;
float R=8.314510;
float C=2.30258509299;
const int basePin = 10;
const int acidPin = 11;
float PH;

// Stirring
int Time = 10;                          
volatile int pulse = 0;                
int frequency = 1000;                    
int PWM_cycles = 3000;                  
float rotation_rate = 0.0;      
float target_rotation_rate = 1000;  // this is the input rotation rate
float r1 = 0.0;
float r2 = 0.0;
float error = 0.0;                  
float error_last = 0.0;              
float integeration = 0.0;              
float integeration_last = 0.0;      
float p = 0.0;                          
float KP = 0.50;                        
float KD = 0.03;
float KI = 0.50;
// // Change this unique address for each I2C slave node

// #define NODE_ADDRESS 1

// // matches values on master side.

// #define TO_MASER_SIZE 3

// #define TO_SLAVE_SIZE 4

// #define NODE_READ_DELAY 100

// byte messageToMaster[TO_MASTER_SIZE];

// byte nodeReceive[TO_SLAVE_SIZE];

#define SLAVE_ADDR 9

void setup()

{

  // I2C

  Wire.begin(SLAVE_ADDR);  // Activate I2C network

  Wire.onRequest(sendToMaster);

  Wire.onReceive(readFromMaster);

  Serial.begin(9600);

  // Heating

  pinMode(A4, INPUT);

  pinMode(9, OUTPUT);

  // pH

  pinMode(basePin, OUTPUT);

  pinMode(acidPin, OUTPUT);

  // Stirring

  pinMode(2, INPUT);                     
  Serial.begin(9600);                   
  attachInterrupt(0, pulse_count, HIGH);

  pinMode(13, OUTPUT);

}

void getTemp(){

  float SensorValue = analogRead(A4);

  float Voltage = SensorValue/1023.0*5.0;

  // float Temp = -0.091*SensorValue + 69.73;

  Temp = beta /(log((1025.0*10/SensorValue-10)/10) + beta / 298.0) - 273.0;

  Serial.print("SensorValue:");

  Serial.print(SensorValue);

  // Serial.print(", Voltage: ");

  // Serial.print(Voltage);

  Serial.print(", temperature: ");

  Serial.print(Temp);

  Serial.print("\n");

  delay(200);

}

void controlTemp(){

  if (Temp <= target_Temp)

  {

    digitalWrite(9, HIGH);

    Serial.print("Heater ON");

  }

  else

  {

    digitalWrite(9, LOW);

    Serial.print("Heater OFF");

  }

}

void getpH(){

  int PH_N=analogRead(PH_N_pin);

  int PH_P=analogRead(PH_P_pin);

  float Voltage_N=PH_N*(5.0/1023.0);

  float Voltage_P=PH_P*(5.0/1023.0);

  PH = 7.0+((F*(Voltage_P-Voltage_N))/(R*(Temp + 273)*C));

}

void controlpH(){

  getpH();

  if (PH > target_pH)

  {

    analogWrite(basePin, 250);

    analogWrite(acidPin, 0);

  }

  else if (PH < target_pH)

  {

    analogWrite(acidPin, 250);

    analogWrite(basePin, 0);

  }

  else

  {

    analogWrite(basePin, 0);

    analogWrite(acidPin, 0);

  }

  Serial.println("PH:");

  Serial.println(PH);

  delay(500);

}

void pulse_count(){

  ++pulse;

}

float PID() {

  float p1 = 0.0, p2 = 0.0, p3 = 0.0, p4 = 0.0, p5 = 0.0;
  // p = KP*e + KD(e2-e1)/t + KIet（integeration）
  p1 = KP*error;
  p2 = KD*1000*(error-error_last)/Time;
  integeration = integeration + error*Time/1000;
  p3 = KI*(integeration + error);
  p4 = p1 + p2 + p3;
  p5 = 1 / (1+exp(-p4));
  return p5;
}

void PWM() {
  int high_time = p*1000000/frequency;         
  digitalWrite(13, HIGH);                     
  delayMicroseconds(high_time);                 
  digitalWrite(13, LOW);                       
  delayMicroseconds(1000000/frequency - high_time);
}

void getRPM(){
  interrupts();                               
  int time1 = millis();
  rotation_rate = 60 * 1000 * float(pulse) / (128.0 * Time);   
  pulse = 0;
  int time2 = millis();
  Time = time2 - time1;
  noInterrupts();                             
  //Serial.println(p);    for test                     
  Serial.println(r2);   //output rotation rate
  r2 = (r1 + rotation_rate)/2;
  r1 = rotation_rate;
}

void controlRPM(){
  getRPM();
  interrupts();                               
  int time1 = millis();
  int i, j = 50;
  rotation_rate = 1000 * float(pulse) / (128.0 * Time);   
  pulse = 0;                                                 
  error = (target_rotation_rate - rotation_rate) / target_rotation_rate;
  p = PID();                                     
  for(i=1;i<=j;i++) {
    PWM();                                     
    error_last = error;                       
  }
  int time2 = millis();
  Time = time2 - time1;

  noInterrupts();                             
  //Serial.println(p);    for test                     
  Serial.println(r2);   //output rotation rate

  r2 = 60*(r1 + rotation_rate)/2;

  r1 = 60*rotation_rate;

}

void sendToMaster()

{

  getTemp();

  getpH();

  getRPM();

  String Temp_message = "T";

  String pH_message = "P";

  String RPM_message = "R";

  String Temp_String = String(Temp,2);
  Temp_message.concat(Temp_String);

  String pH_String = String(PH,2);
  pH_message.concat(pH_String);

  String RPM_String = String(rotation_rate,2);
  RPM_message.concat(RPM_String);

  Wire.beginTransmission(9);

  Wire.write(Temp_message.c_str());
  Wire.write(pH_message.c_str());
  Wire.write(RPM_message.c_str());

  Wire.endTransmission();

  // messageToMaster[0] = NODE_ADDRESS;

  // messageToMaster[1] = (x0>>8) & 0xff;  // the top byte of x

  // messageToMaster[2] = (x0   ) & 0xff;  // the bottom byte of x

  // Wire.write(messageToMaster,TO_MASTER_SIZE);

  Serial.print(Temp_message);

  Serial.print(pH_message);

  Serial.print(RPM_message);

  Serial.print("\n");

}

void readFromMaster(int howMany)

{

  String received_string = "";

  while (Wire.available())

  {

    char c = Wire.read();

    received_string += c;

  }

  String string_value = "";
  int i;

  for (int i = 1; i <= strlen(received_string.c_str()); i++){

    char c = received_string[i];

    string_value += c;

  }

  if (received_string[0] == "T"){

    desired_Temp = string_value.toFloat();

    target_Temp = desired_Temp;

    Serial.print(desired_Temp);

  }

  else if (received_string[0] == "P"){

    desired_pH = string_value.toFloat();

    target_pH = desired_pH;

    Serial.print(desired_pH);

  }

  else if (received_string[0] == "R"){

    desired_RPM = string_value.toFloat();

    target_rotation_rate = desired_RPM;

    Serial.print(desired_RPM);

  }

  delay(100);

  // for(int i = 0; i < TO_SLAVE_SIZE; i ++){

  //   nodeReceive[i] = Wire.read();

  // }

  // Serial.print("Master says ");

  // for(int i = 0; i < TO_SLAVE_SIZE; i ++){

  //   Serial.print(nodeReceive[i]);

  // }

  // Serial.println();

}

void loop() {

  delay(100);

  if (!Wire.available()){

  controlTemp();

  controlpH();

  controlRPM();

  }

}
