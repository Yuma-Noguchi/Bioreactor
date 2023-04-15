// For I2C

#include <Wire.h>
#define SLAVE_ADDR 9
#define I2C_SDA 21
#define I2C_SCL 22

float Temp, PH, RPM;
float desired_Temp, desired_pH, desired_RPM;


#include <WiFiClient.h>
#include <WiFi.h>
#include "esp_wpa2.h"
const char* essid = "eduroam";
#define EAP_IDENTITY "zcabyno@ucl.ac.uk"              
#define EAP_PASSWORD "Kokusai-N39@jp"

// For thingsboard
#include "ThingsBoard.h"
#define TOKEN               "JRnZUQl1VwcA1VCqiW0g"
#define THINGSBOARD_SERVER  "engf0001.cs.ucl.ac.uk"

// Initialize the WiFi client object
WiFiClient wifiClient;

// Initialize ThingsBoard instance
ThingsBoard tb(wifiClient);

void setup() {
  Serial.begin(115200);

  // I2C
  // Serial.begin(9600);
  Wire.begin (I2C_SDA, I2C_SCL);

  // Wifi
  bool eduroamFound = false;
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(); 

  while (!eduroamFound) {
    Serial.println("scan start");
    int n = WiFi.scanNetworks();
    Serial.println("scan done");   

    if (n == 0) {
        Serial.println("no networks found");
    } else {
        Serial.print(n);
        Serial.println(" networks found");
        for (int i = 0; i < n; ++i) {
            String ssid = WiFi.SSID(i);
            int    rssi = WiFi.RSSI(i);
            Serial.print(i + 1);
            Serial.print(": ");
            Serial.print(ssid);
            Serial.print(" (");
            Serial.print(rssi);
            Serial.print(")");
            Serial.print((WiFi.encryptionType(i) == WIFI_AUTH_OPEN)?" ":"*");
            delay(10);
            ssid.trim();
            if (ssid == essid) {
              Serial.print(" <==== eduroam found");
              eduroamFound = true;
            }
            Serial.println("");
        }
    }
    Serial.println("");
    if (!eduroamFound)
      delay(5000);
  }

  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(essid);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_sta_wpa2_ent_set_username((uint8_t *)EAP_IDENTITY, strlen(EAP_IDENTITY));
  esp_wifi_sta_wpa2_ent_set_password((uint8_t *)EAP_PASSWORD, strlen(EAP_PASSWORD));
  esp_wifi_sta_wpa2_ent_enable();
  WiFi.begin(essid);    
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("WiFi is connected to ");
  Serial.println(essid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

}

 

RPC_Response getTemp(const RPC_Data &data)
{
  Serial.println("Received the get temperature RPC method");
  return RPC_Response(NULL, Temp);
}

RPC_Response setTemp(const RPC_Data &data)
{ 
  Serial.println("Received the set temperature RPC method");
  // Process data
  desired_Temp = data;
  // Just a response example
  return RPC_Response(NULL, desired_Temp);
}

RPC_Response getpH(const RPC_Data &data)
{
  Serial.println("Received the get temperature RPC method");
  return RPC_Response(NULL, PH);
}

RPC_Response setpH(const RPC_Data &data)
{ 
  Serial.println("Received the set temperature RPC method");
  // Process data
  desired_pH = data;
  // Just a response example
  return RPC_Response(NULL, desired_pH);
} 

RPC_Response getRPM(const RPC_Data &data)
{
  Serial.println("Received the get temperature RPC method");
  return RPC_Response(NULL, RPM);
}

RPC_Response setRPM(const RPC_Data &data)

{ Serial.println("Received the set temperature RPC method");
  // Process data
  desired_RPM = data;
  // Just a response example
  return RPC_Response(NULL, desired_RPM);
}

const size_t callbacks_size = 2;

RPC_Callback callbacks[callbacks_size] = {
  { "getTemperature", getTemp },
  { "setTemperature", setTemp }
};

bool subscribed = false;

void sendToSlave()

{
  Wire.beginTransmission(SLAVE_ADDR);
  String Temp_message = "T";
  String pH_message = "P";
  String RPM_message = "R";
 
  String Temp_String = String(desired_Temp,2);
  Temp_message.concat(Temp_String);

  String pH_String = String(desired_pH,2);
  pH_message.concat(pH_String);

  String RPM_String = String(desired_RPM,2);
  RPM_message.concat(RPM_String);

  Wire.beginTransmission(9);

  Wire.write(Temp_message.c_str());
  Wire.write(pH_message.c_str());
  Wire.write(RPM_message.c_str());

  // messageToMaster[0] = NODE_ADDRESS;
  // messageToMaster[1] = (x0>>8) & 0xff;  // the top byte of x
  // messageToMaster[2] = (x0   ) & 0xff;  // the bottom byte of x
  // Wire.write(messageToMaster,TO_MASTER_SIZE);

 // Serial.print(Temp_message.c_str(), pH_message.c_str(), RPM_message.c_str());

  Serial.print("\n");
  Wire.endTransmission();
  delay(500);
}

void readFromSlave() {
  // if data size is available from nodes
  Wire.requestFrom(SLAVE_ADDR, 7);
  String received_string = "";
  String string_value = "";
  while (Wire.available())
  {
    char c = Wire.read();
    received_string += c;
  }

  for (int i = 1; i <= strlen(received_string.c_str()); i++){
    char c = received_string[i];
    string_value += c;
  }

  if (received_string[0] == 'T'){
    Temp = string_value.toFloat();
    Serial.print(Temp);
  } 

  else if (received_string[0] == 'P'){
    PH = string_value.toFloat();
    Serial.print(PH);
  }

  else if (received_string[0] == 'R'){
    desired_RPM = string_value.toFloat();
    Serial.print(RPM);
  }
  delay(100);

  // if(Wire.available() == TO_MASTER_SIZE) {
  //   for (int i = 0; i < TO_MASTER_SIZE; i++) {
  //     messageToMaster[i] = Wire.read();  // get data
  //   }
  //   int fromAddress = messageToMaster[0];
  //   int value = ((int)messageToMaster[1] << 8 ) | (int)messageToMaster[2];
  //   Serial.print("Slave ");
  //   Serial.print(fromAddress);
  //   Serial.print(" says ");
  //   Serial.print(value);
  //}
}

void loop() {
  delay(100);
  // Wire.requestFrom(SLAVE_ADDR, 7);
  readFromSlave();

  // Thingsboard
  if (WiFi.status() == WL_IDLE_STATUS) {
    // Still connecting
    return;
  }

  if (!tb.connected()) {
    subscribed = false;

    // Connect to the ThingsBoard
    Serial.print("Connecting to: ");
    Serial.print(THINGSBOARD_SERVER);
    Serial.print(" with token ");
    Serial.println(TOKEN);
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN)) {
      Serial.println("Failed to connect, retrying ...");
      return;
    }
  } 

  if (!subscribed) {
    Serial.println("Subscribing for RPC...");
    // Perform a subscription. All consequent data processing will happen in
    // processTemperatureChange() and processSwitchChange() functions,
    // as denoted by callbacks[] array.
    if (!tb.RPC_Subscribe(callbacks, callbacks_size)) {
      Serial.println("Failed to subscribe for RPC");
      return;
    }
    Serial.println("Subscribe done");
    subscribed = true;
  }
  //Serial.println("Waiting for data...");

  tb.loop();

}
