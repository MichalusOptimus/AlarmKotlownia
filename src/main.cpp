#include <ESP8266WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PubSubClient.h>
#include <LiquidCrystal_PCF8574.h>
#include <Wire.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <FS.h>       // Filesystem


#define oneWireBus D3         // GPIO where the DS18B20 is connected 
#define BUILTIN_LED D4        // GPIO where the LED is connected 
#define buttonAdd D6          // GPIO where the add pushbutton is connected 
#define buttonSubstract D5    // GPIO where the substract pushbutton is connected 
#define buttonOk D7           // GPIO where the confirm pushbutton is connected
#define buzzer D0             // GPIO where the buzzer is connected 
#define smokeDetector A0      // GPIO where the smoke detector is connected

// SSID and password local network
const char* ssid     = "INTERQ-dom675a";
const char* password = "Domek12345";
// Settings MQTT - RASPBERRY
const char* mqtt_server = "192.168.100.120";
const char* mqtt_topic = "domoticz/in";
const int   mqtt_port = 1883;
const int   idxTempHeater = 12;
const int   idxTempRoom = 13;
const int   idxSmokeSensor = 17;
// Setting static address IP on the local network
IPAddress addressIP(192,168,100,127);
IPAddress gateway(192,168,100,1);
IPAddress subnet(255,255,255,0);
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);
// Set the LCD I2C address
LiquidCrystal_PCF8574 lcd(0x27);
//Define NTP client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 7200, 60000);

WiFiClient espClient;
PubSubClient client(espClient);

// Time variable in miliseconds
unsigned long currentTime = millis();
unsigned long previousTimeMeasure = 0,timeBetweenMeasure = 1000;
unsigned long previousTimeDisplay = 0,timeBetweenDisplay = 60000; // 1 min
unsigned long previousTimeSending = 0,timeBetweenSending = 2000;
unsigned long previousTimeAlarm = 0, timeBetweenAlarm = 5000;    // 30 sec
unsigned long previousTimeBeep = 0, timeBetweenBeep = 200;
unsigned long previousBacklight = 0, timeBetweenBacklight = 15000; // 10 min
// Var to display time first time after change screeng
bool displayTimeFirstTime = 0;
// Temperature variable
int tempHeater,tempAlarm,tempRoom;
// Buffer msg to MQTT server
char msg[50];
// Signals from pushbuttons
bool buttonAddSignal,buttonSubstractSignal,buttonOkSignal;
// Value from smoke detector (analog)
int smokeDetectorValue;
// Ratio to calculate concetration of gas (measure in clear air)
float R0 = 1.35;
// Rs/R0 -> factor to reading concentration of gas
float ratio;
// State of alarm temp. configuration
int stateSetAlarmTemp = 1;
// State of alarm
bool stateAlarm = 0;
// State of beep
bool stateBeep = 0;
// Letter Ł in binary form
byte letterL[8] =
{                
B10000,
B10000,
B10100,
B11000,
B10000,
B10000,
B11111,
B00000
};
// degree sign in binary form
byte degree[8] ={
B00111,
B00101,
B00111,
B00000,
B00000,
B00000,
B00000,
B00000
};

//Functions
void reconnect();
void readSignals();
void setTempAlarm();
void soundAlarm();
bool checkWiFi();
void sendToDomoticz();
void measureTemp();
void buzzerBeep();
void backlightLCD();
void displayTime();

void setup() {
  Serial.begin(115200);
  pinMode(BUILTIN_LED, OUTPUT);
  pinMode(buzzer,OUTPUT);
  lcd.begin(20,4);
  lcd.createChar(0, letterL);
  lcd.createChar(1, degree);  
  lcd.setBacklight(255);
  lcd.clear();
  delay(1000);
  lcd.setCursor(4,0);
  lcd.print("URUCHAMIANIE");
  delay(1000);
  // Connecting to WiFi
  lcd.setCursor(0,1);
  lcd.write(char(0));         //Ł
  lcd.print("ACZENIE Z WIFI");
  Serial.println("Connecting to: ");
  Serial.println(ssid);
  WiFi.hostname("KotlowniaESP");
  //WiFi.config(addressIP,gateway,subnet);
  WiFi.begin(ssid, password);
  int cnt = 0; // var to change position of dot on screen
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    lcd.setCursor(cnt,2);
    lcd.print(".");
    cnt++;
    if (cnt == 20) ESP.restart();
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  lcd.setCursor(0,3);
  lcd.print("PO");
  lcd.write(char(0));         //Ł
  lcd.print("ACZONO");
  delay(1000);
  lcd.setCursor(0,3);
  lcd.print("         ");
  // Connecting to MQTT server
  lcd.setCursor(0,1);
  lcd.write(char(0));         //Ł
  lcd.print("ACZENIE Z SERWEREM");
  Serial.println("Set up MQTT server");
  client.setServer(mqtt_server, mqtt_port);
  lcd.setCursor(0,3);
  if(client.connect("ESPHeater")){
  lcd.print("PO");
  lcd.write(char(0));         //Ł
  lcd.print("ACZONO");}
  else lcd.print("NIE POLACZONO");
  delay(1000);
  // Start the DS18B20 sensor and set resolution (9bit -> 0,5 degree)
  sensors.begin();
  sensors.setResolution(9);
  // Start NTP client
  timeClient.begin();
  //Start SPIFFS
  SPIFFS.begin();
  File file2 = SPIFFS.open("/alarmTemp.txt", "r");
  if (!file2) {
    Serial.println("Failed to open file for reading");
    return;}
  Serial.println("File Content:"); 
  char buffer[5];
  while (file2.available()) {
  int l = file2.readBytesUntil('\n', buffer, sizeof(buffer));
  buffer[l] = 0;
  Serial.println(buffer);}
  tempAlarm = (buffer[0] - '0')*10 + (buffer[1] - '0'); //convert ascii -> numbers and calculate alarmTemp from SPI memory
  Serial.println("Tempalarmowa");
  Serial.println(tempAlarm);
  file2.close();
  lcd.clear();
  currentTime = millis();
}

void loop() {
  backlightLCD();
  measureTemp();
  if(checkWiFi()) sendToDomoticz();
  if(stateAlarm) soundAlarm();
  else setTempAlarm();
  if(stateSetAlarmTemp == 1) displayTime();
  currentTime = millis();
  // TEST, DEBUGGING
}

// reconnect MQTT
void reconnect() {
  // Loop until we're reconnected
  for(int i=0; i<=2; i++){
  if(!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    lcd.setCursor(0,3);
    lcd.print("LACZENIE Z MQTT");
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      lcd.setCursor(0,3);
      lcd.print("POLACZONO");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      lcd.setCursor(0,3);
      lcd.print("NIE POLACZONO");
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 1 second");
      // Wait 1 second before retrying
      delay(1000);
    }
  }
}
lcd.setCursor(0,3);
lcd.print("               ");
}

// Set inputs and read signals from pushbuttons
void readSignals(){ 
  pinMode(smokeDetector,INPUT);
  pinMode(buttonAdd,INPUT_PULLUP);           
  pinMode(buttonSubstract,INPUT_PULLUP);
  pinMode(buttonOk,INPUT_PULLUP);
  buttonAddSignal=digitalRead(buttonAdd);
  buttonSubstractSignal=digitalRead(buttonSubstract);
  buttonOkSignal=digitalRead(buttonOk);  
}

// Set alarm temperature by pushbuttons
// State machine to operation push and pull pushbuttons
void setTempAlarm(){
  readSignals();
  switch(stateSetAlarmTemp){
    case 1: if(!buttonOkSignal) stateSetAlarmTemp = 2;
            break;
    case 2: if(buttonOkSignal) stateSetAlarmTemp = 3;
            break;
    case 3: lcd.setCursor(9,2);
            lcd.print("T.ALARM:");
            lcd.printf("%i",tempAlarm);
            if(!buttonAddSignal)  {tempAlarm = tempAlarm + 1.00; stateSetAlarmTemp = 4;}            // add degree
            else if(!buttonSubstractSignal) {tempAlarm = tempAlarm - 1.00; stateSetAlarmTemp = 5;}  // substract degree 
            else if(!buttonOkSignal) stateSetAlarmTemp = 6;
            if(tempAlarm>80) tempAlarm = 80.0;              // limit max temp.
            if(tempAlarm<20) tempAlarm = 20.0;              // limit min temp.
            break;
    case 4: if(buttonAddSignal) stateSetAlarmTemp = 3;
            break;
    case 5: if(buttonSubstractSignal) stateSetAlarmTemp = 3;
            break;        
    case 6: lcd.setCursor(9,2);
            lcd.print("          ");
             File file = SPIFFS.open("/alarmTemp.txt", "w");
            if (!file) {
            Serial.println("Error opening file for writing");}
            int bytesWritten = file.print(tempAlarm);
            if (bytesWritten > 0) {
            Serial.println("File was written");
            Serial.println(bytesWritten);} 
            else {
            Serial.println("File write failed");}
            file.close();
            if(buttonOkSignal) stateSetAlarmTemp = 1;   // change state
            displayTimeFirstTime = 0;                   // set var -> displayTime
            break;
  }
  delay(1);
}

// Make sound alarm (turn on buzzer)
void soundAlarm(){
  readSignals();
  buzzerBeep();
  lcd.setCursor(0,3);
  lcd.print("ZA WYSOKA TEMP!!!");
  Serial.println("ALARM!");
  if(!buttonOkSignal){
    delay(200);
    previousTimeAlarm = currentTime;
    stateAlarm = 0;
    lcd.setCursor(0,3);
    lcd.print("                 ");
  }
  }

// Check connecting WiFi network and MQTT
bool checkWiFi(){
  lcd.setCursor(0,2);
  lcd.print("WiFi ");
  if((WiFi.status() != WL_CONNECTED)) { lcd.print("NIE "); return 0;}
  else if (!client.connected()) { lcd.print("MQTT "); reconnect(); return 0;}
  else { lcd.print("TAK "); return 1;}
}

// Send data to server via MQTT (Domoticz)
void sendToDomoticz(){
  //client.loop();
  if(currentTime - previousTimeSending >= timeBetweenSending){
  // Message format and publish on topic
  // idx - number of element in Domoticz, nvalue - default 0, svalue - sent value
  //Publish heater temperature
  snprintf(msg,50,"{\"idx\":%i,\"nvalue\":0,\"svalue\":\"%i\"}",idxTempHeater,tempHeater);
  client.publish(mqtt_topic, msg);
  Serial.println(msg);
  //Publish temperature in boiler room
  snprintf(msg,50,"{\"idx\":%i,\"nvalue\":0,\"svalue\":\"%i\"}",idxTempRoom,tempRoom);
  client.publish(mqtt_topic, msg);
  Serial.println(msg);
  //Publish value from smoke sensor
  snprintf(msg,50,"{\"idx\":%i,\"nvalue\":0,\"svalue\":\"%.2f\"}",idxSmokeSensor,ratio);
  client.publish(mqtt_topic, msg);
  Serial.println(msg);
  previousTimeSending = currentTime;
  }
}

// Temperature measurement
void measureTemp(){
  if(currentTime - previousTimeMeasure >= timeBetweenMeasure){
  sensors.requestTemperatures(); 
  tempHeater = int(sensors.getTempCByIndex(0));
  tempRoom = int(sensors.getTempCByIndex(1)); 
  smokeDetectorValue = analogRead(smokeDetector);
  float sensor_volt;
  float RS_gas;   // Get value of RS in a GAS
  sensor_volt=(float)smokeDetectorValue/1024*5.0;
  RS_gas = (5.0-sensor_volt)/sensor_volt;
  ratio = RS_gas/R0;                  // ratio = RS/R0
  Serial.println("RS/R0=");
  Serial.println(ratio);
  //Display on LCD
  lcd.setCursor(0,0);
  lcd.print("PIEC |");
  lcd.setCursor(0,1);
  lcd.print("       ");
  lcd.setCursor(0,1);           // temp. heater
  lcd.printf("%i ",tempHeater);
  lcd.write(char(1));           // degree sign
  lcd.print("C");
  lcd.setCursor(5,1);           // |
  lcd.print("|");
  lcd.setCursor(8,0);
  lcd.print("KOT");
  lcd.write(char(0));
  lcd.print(" |");
  lcd.setCursor(8,1);
  lcd.print("       ");
  lcd.setCursor(8,1);            // temp. room
  lcd.printf("%i ",tempRoom);
  lcd.write(char(1));            // degree sign
  lcd.print("C");
  lcd.setCursor(13,1);           // |
  lcd.print("|");
  lcd.setCursor(16,0);
  lcd.print("DYM");
  lcd.setCursor(15,1);
  lcd.printf("%.2f ",ratio);
  //lcd.print(smokeDetectorValue);
  //Serial.print(tempHeater);
  //Serial.println("ºC");
  if((currentTime - previousTimeAlarm >= timeBetweenAlarm) && (tempHeater >= tempAlarm)) stateAlarm = 1;
  else digitalWrite(buzzer,LOW);                // Turn off buzzer
  previousTimeMeasure = currentTime;
  }
}

// Make specified sound from buzzer
void buzzerBeep(){
  if(currentTime - previousTimeBeep >= timeBetweenBeep){
    stateBeep = !stateBeep;
    digitalWrite(buzzer,stateBeep);
    previousTimeBeep = currentTime;
  }
}

// Check pushbutton and turn on/off backlight
// Default 10 min = 600 sec
void backlightLCD(){
  readSignals();
  if(!buttonAddSignal || !buttonSubstractSignal || !buttonOkSignal || stateAlarm) {previousBacklight = millis(); lcd.setBacklight(255);}
  else if(currentTime - previousBacklight >= timeBetweenBacklight ) lcd.setBacklight(0);
}

// Display time on LCD (hours and minutes) every minute
void displayTime(){
  if((currentTime - previousTimeDisplay >= timeBetweenDisplay) && displayTimeFirstTime){
  timeClient.update();
  lcd.setCursor(11,2);
  if(timeClient.getHours()<10) lcd.print("0"); // when hours are less than 10 
  if(timeClient.getMinutes()<10) {             // when minutes are less than 10
  lcd.printf("%i:",timeClient.getHours());
  lcd.print("0");
  lcd.printf("%i",timeClient.getMinutes());}
  else{                                        // when minutes are more than 10
  lcd.printf("%i:%i",timeClient.getHours(),timeClient.getMinutes());}
  previousTimeDisplay = currentTime;
  }
  else if(!displayTimeFirstTime){              // <- "else" execute one time after change screen
  timeClient.update();
  lcd.setCursor(11,2);
  if(timeClient.getHours()<10) lcd.print("0"); // when hours are less than 10 
  if(timeClient.getMinutes()<10) {             // when minutes are less than 10
  lcd.printf("%i:",timeClient.getHours());
  lcd.print("0");
  lcd.printf("%i",timeClient.getMinutes());}
  else{                                        // when minutes are more than 10
  lcd.printf("%i:%i",timeClient.getHours(),timeClient.getMinutes());}
  displayTimeFirstTime = 1;
  }
  
}