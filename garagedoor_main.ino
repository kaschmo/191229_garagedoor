/*
 * Garage Door Controller
 * Sends and receives values via MQTT
 * BME280 sensor for P,H,T Measurements
 * Relay for switching Garage Door
 * GPIO In for status relay (open/close) on garage door
 * Copyright K.Schmolders 12/2019
 */

// wifi credentials stored externally and .gitignore
 //all wifi credential and MQTT Server importet through wifi_credential.h
 #include "wifi_credentials.h"

 //required for MQTT
 #include <ESP8266WiFi.h>
 //required for OTA updater
 #include <WiFiClient.h>
 #include <ESP8266WebServer.h>
 #include <ESP8266mDNS.h>
 #include <ESP8266HTTPUpdateServer.h>
 //end OTA requirements
 #include <PubSubClient.h>
 #include <Adafruit_BME280.h>
 
 
 // GPIO Relay out for switching
 uint16_t RELAY_OUT_PIN = 14; //D5 on NodeMCU
 // GPIO Relay in for status of door
 uint16_t RELAY_IN_PIN = 12; //D6 on NodeMCU
 // I2C for BME Sensor module
 uint16_t BME_SCL = 5; //NodeMCU D1
 uint16_t BME_SDA = 4; //NodeMCU D2
 
 //timer
 int timer_update_state_count;
 int timer_update_state = 60000; //update status via MQTT every minute
  
 //MQTT (see also wifi_credentials)
 WiFiClient espClient;
 PubSubClient client(espClient);
 
 const char* inTopic = "cmnd/relay_garage/#";
 const char* outTopic = "stat/relay_garage/";
 const char* mqtt_id = "relay_garage";
 
 //BME280
 #define SEALEVELPRESSURE_HPA (1013.25)
 Adafruit_BME280 bme280; // I2C
 float bme280_temperature, bme280_pressure, bme280_humidity, bme280_height;
 //WHY?
 float bme280_temp_offset = 1.5;

//GarageDoor Status
int garage_door_status; //1 closed, 0 open

 //OTA
 ESP8266WebServer httpServer(80);
 ESP8266HTTPUpdateServer httpUpdater;
 
 void setup_wifi() {
   delay(10);
   // We start by connecting to a WiFi network
   Serial.println();
   Serial.print("Connecting to ");
   Serial.println(ssid);
   WiFi.persistent(false);
   WiFi.mode(WIFI_OFF);
   WiFi.mode(WIFI_STA);
   WiFi.begin(ssid, password);
   while (WiFi.status() != WL_CONNECTED) {
     delay(500);
     Serial.print(".");
   }
     
   Serial.println("");
   Serial.println("WiFi connected");
   Serial.println("IP address: ");
   Serial.println(WiFi.localIP());
 
   httpUpdater.setup(&httpServer);
   httpServer.begin();
 }
 
 
 //callback function for MQTT client
 void callback(char* topic, byte* payload, unsigned int length) {
   payload[length]='\0'; // Null terminator used to terminate the char array
   String message = (char*)payload;
 
   Serial.print("Message arrived on topic: [");
   Serial.print(topic);
   Serial.print("]: ");
   Serial.println(message);
   
   //get last part of topic 
   char* cmnd = "test";
   char* cmnd_tmp=strtok(topic, "/");
 
   while(cmnd_tmp !=NULL) {
     cmnd=cmnd_tmp; //take over last not NULL string
     cmnd_tmp=strtok(NULL, "/"); //passing Null continues on string
     //Serial.println(cmnd_tmp);    
   }
   
   
 
    if (!strcmp(cmnd, "status")) {
        Serial.print("Received status request. sending status");
        send_status();
    }
    else if (!strcmp(cmnd, "reset")) {
        Serial.print(F("Reset requested. Resetting..."));
        //software_Reset();
    }
    else if (!strcmp(cmnd, "door")) {
        Serial.print("Received Open/Close Door command.");
        relay_impulse();
    }
 }

//closes relay for XX seconds to trigger door
void relay_impulse()
{
    //Relay requires LOW on input to toggle 
    digitalWrite(RELAY_OUT_PIN, LOW);
    delay(1000);
    digitalWrite(RELAY_OUT_PIN, HIGH);
}

//sends module status via MQTT
void send_status()
 {
   char outTopic_status[50];
   char msg[50];
   //IP Address
   strcpy(outTopic_status,outTopic);
   strcat(outTopic_status,"ip_address");
   
   //ESP IP
   WiFi.localIP().toString().toCharArray(msg,50);
   client.publish(outTopic_status,msg ); 
 }
 
 //send Sensor Values via MQTT
 void sendSensorValues(){
    
    char outTopic_status[50];
    char msg[50];
 
    
   //roomtemp from BME280
    strcpy(outTopic_status,outTopic);
    dtostrf(bme280_temperature,2,2,msg); 
    strcat(outTopic_status,"temperature");
    client.publish(outTopic_status, msg);
 
   //BME280 Humidity
    strcpy(outTopic_status,outTopic);
    dtostrf(bme280_humidity,2,2,msg); 
    strcat(outTopic_status,"humidity");
    client.publish(outTopic_status, msg);
 
    //BME280 Pressure
    strcpy(outTopic_status,outTopic);
    dtostrf(bme280_pressure,2,2,msg); 
    strcat(outTopic_status,"pressure");
    client.publish(outTopic_status, msg);
 
     //IP Address
    strcpy(outTopic_status,outTopic);
    strcat(outTopic_status,"ip_address");
    WiFi.localIP().toString().toCharArray(msg,50);
    client.publish(outTopic_status,msg ); 

    //GARAGE Door status
    strcpy(outTopic_status,outTopic);
    strcat(outTopic_status,"door");
    dtostrf(garage_door_status,1,0,msg); //1 = zu, 0=auf
    client.publish(outTopic_status,msg ); 
 
 }
 
 void reconnect() {
   // Loop until we're reconnected
   
   while (!client.connected()) {
     Serial.print("Attempting MQTT connection...");
     // Attempt to connect
     if (client.connect(mqtt_id)) {
       Serial.println("connected");
       
       client.publish(outTopic, "garagedoor station booted");
       
       //send current Status via MQTT to world
       sendSensorValues();
       // ... and resubscribe
       client.subscribe(inTopic);
 
     } else {
       Serial.print("failed, rc=");
       Serial.print(client.state());
       Serial.println(" try again in 5 seconds");      
       delay(5000);
     }
   }
 }
 
 void update_sensors() {
   bme280_temperature=bme280.readTemperature()-bme280_temp_offset; //C
   bme280_pressure=bme280.readPressure() / 100.0F; //in hPA
   bme280_humidity=bme280.readHumidity(); //%
   bme280_height=bme280.readAltitude(SEALEVELPRESSURE_HPA); //m
    garage_door_status=digitalRead(RELAY_IN_PIN); //1 geschlossenes relay = zu
 }

 //interrupt function for garage door relay change
 void on_openclose() {
     Serial.println("Garage Door Status changed - update sensors and send values");
     update_sensors();
     sendSensorValues();
 }
 
 void setup() {
   // Status message will be sent to the PC at 115200 baud
   Serial.begin(115200, SERIAL_8N1, SERIAL_TX_ONLY);
   Serial.println("Garage Door Opener Module");
   //INIT TIMERS
   timer_update_state_count=millis();

   

   //INIT BME280
   //SDA, SCL
   Serial.println("BME280 Init");
   Wire.begin(BME_SDA, BME_SCL);
   bool status;
   status = bme280.begin();
   if (!status) {
       Serial.println("Could not find a valid BME280 sensor, check wiring!");
       delay(1000);
       while (1);
   }

   //initializing Pins
    Serial.println("Relay Init");
   pinMode(RELAY_OUT_PIN, OUTPUT);
   digitalWrite(RELAY_OUT_PIN, HIGH); //relay open - default
   pinMode(RELAY_IN_PIN, INPUT);
   //attachInterrupt(digitalPinToInterrupt(RELAY_IN_PIN), on_openclose, RISING);

   update_sensors(); 
 
   //WIFI and MQTT
   setup_wifi();                   // Connect to wifi 
   client.setServer(mqtt_server, 1883);
   client.setCallback(callback);
 
  
 }
 
 
 void loop() {
   if (!client.connected()) {
     reconnect();
   }
   client.loop();
 
   update_sensors(); 
 
   //http Updater for OTA
   httpServer.handleClient(); 
 
   //send status update via MQTT every minute
   if(millis()-timer_update_state_count > timer_update_state) {
    //addLog_P(LOG_LEVEL_INFO, PSTR("Serial Timer triggerd."));
    timer_update_state_count=millis();
    sendSensorValues();
    
   }
   
 }
 