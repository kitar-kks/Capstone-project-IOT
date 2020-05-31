#include <ESP8266WIFI.h>
#include <PubSubClient.h>
#include <LiquidCrystal_I2C.h>
#include <TridentTD_LineNotify.h>

#define LINE_TOKEN "OwUl3VQexfCSAKxAmF5WL6KFYkNSEOmxoiDIzmMMZKP"

LiquidCrystal_I2C lcd(0x27, 16, 2);   //Module IIC/I2C Interface บางรุ่นอาจจะใช้ 0x3f

const char* ssid = "true_home2G_754";
const char* password = "QF9BNRR7";
const char* mqtt_server = "broker.netpie.io";
const int mqtt_port = 1883;
const char* mqtt_Client = "a806ceb3-261a-4431-8511-174ea292fce0";  /// Client_ID
const char* mqtt_username = "wedbQC1BCiKL3izJoYET19A6MHMVLhnJ" ;  //Token
const char* mqtt_password =  ")#j$PUm2slpE#oBbc7OhnKcUe0AVGOeT"; //Secret
WiFiClient espClient;
PubSubClient client(espClient);

int measurePin = A0;
int v_speed = 0;    //Pin LED
const int motor = 15,en = 12,ledPower = 0; /// 15 D8, 12 D6, 0 D3;

bool power_on = false, loop_check = true;

char msg[100];

int samplingTime = 280;
int deltaTime = 40;
int sleepTime = 9680;
int i=0,AQI=0;

float voMeasured = 0,Vo = 0,vp_speed =0;
float calcVoltage = 0;
float dustDensity = 0;

// char *aqi_text[] = {"Good","Moderate","LowUnhealthy","MidUnhealthy","VeryUnhealthy","Hazardous"};

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection…");
    if (client.connect(mqtt_Client, mqtt_username, mqtt_password)) {
      Serial.println("connected");
      client.subscribe("@msg/ToggleStatus");
    }
    else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println("try again in 1 seconds");
      delay(1000);
    }
  }
}

void callback(char* topic,byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  String msgs;
  for (int i = 0; i < length; i++) {
    msgs = msgs + (char)payload[i];
  }
   Serial.println(msgs);
  if (String(topic) == "@msg/ToggleStatus") {
    if (msgs == "on"){
      power_on = true;
    } 
    else if (msgs == "off") {
      power_on = false;
    } 
  }
}

void setup() {
  //Serial.begin(9600);
  pinMode(ledPower, OUTPUT);
  pinMode(measurePin,INPUT);
  pinMode(motor,OUTPUT);
  pinMode(en,OUTPUT);
  LINE.setToken(LINE_TOKEN);
  Serial.begin(115200);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  init_LCD();
}
void init_LCD()
{
  lcd.init();                                                                 
  lcd.backlight();
  lcd.clear();
  lcd.home();
}

void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  if(power_on == true)
  {
  digitalWrite(ledPower, LOW); // power on the LED
  delayMicroseconds(samplingTime);

  voMeasured = analogRead(measurePin); // read the dust value
  vp_speed = map(voMeasured,0,1024,0,100);
  digitalWrite(en,HIGH);
  analogWrite(motor,voMeasured);

  delayMicroseconds(deltaTime);
  digitalWrite(ledPower, HIGH); // turn the LED off
  delayMicroseconds(sleepTime);

  // 0 - 3.3V mapped to 0 - 1023 integer values
  // recover voltage
  calcVoltage = voMeasured * (3.3 / 1024);

  // linear eqaution taken from http://www.howmuchsnow.com/arduino/airquality/
  // Chris Nafis (c) 2012
  dustDensity = (0.17 * calcVoltage) - 0.1;

  Serial.print("Raw Signal Value (0-1023): ");
  Serial.print(voMeasured);

  Serial.print(" - Voltage: ");
  Serial.print(calcVoltage);

  if (dustDensity <= 0.00) {
    dustDensity = 0.00;
  }
  
  dustDensity = (dustDensity * 1000);
  Serial.print(" - Dust Density: ");
  Serial.print(dustDensity);
  Serial.println(" µg/m³");
  
  String data = "{\"data\": {\"dustDensity\":" + String(dustDensity) + ", \"speed\":" + String(vp_speed) + ", \"AQI\":" + String(AQI) + "}}";
  data.toCharArray(msg, (data.length() + 1));
  client.publish("@shadow/data/update", msg);

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("PM2.5: ");
    lcd.setCursor(6,0);
    //lcd.print(voMeasured);
    lcd.print(dustDensity);
    if(dustDensity<12)
    {
      lcd.setCursor(0,1);
      lcd.print("AQI:Good");
      AQI = map(dustDensity,0,12,0,50);
     // client.publish("@msg/AQI","Good");
      
      //client.publish("@shadow/data/update", "{\"data\" : {\"AQI\" : \"Good\"}}");
      //i=0;
    }
    if(dustDensity<35 & dustDensity>11)
    {
      lcd.setCursor(0,1);
      lcd.print("AQI:Moderate");
      AQI = map(dustDensity,12,35,51,100);
      //client.publish("@shadow/data/update", "{\"data\" : {\"AQI\" : \"Moderate\"}}");
      //i=1;
    }
    if(dustDensity<55 & dustDensity>34)
    {
      lcd.setCursor(0,1);
      lcd.print("AQI:LowUnhealthy");
      AQI = map(dustDensity,33,55,101,150);
      //client.publish("@shadow/data/update", "{\"data\" : {\"AQI\" : \"LowUnhealthy\"}}");
      //i=2;
    }
    if(dustDensity<150 & dustDensity>54)
    {
      lcd.setCursor(0,1);
      lcd.print("AQI:MidUnhealthy");
      AQI = map(dustDensity,55,150,151,200);
      //client.publish("@shadow/data/update", "{\"data\" : {\"AQI\" : \"MidUnhealthy\"}}");
      //i=3;
    }
    if(dustDensity<250 & dustDensity>149)
    {
      lcd.setCursor(0,1);
      lcd.print("AQI:VeryUnhealthy");
      AQI = map(dustDensity,150,250,201,300);
      //client.publish("@shadow/data/update", "{\"data\" : {\"AQI\" : \"VeryUnhealthy\"}}");
      //i=4;
    }
    if(dustDensity>249)
    {
      lcd.setCursor(0,1);
      lcd.print("AQI:Hazardous");
      AQI = dustDensity-63;
      //client.publish("@msg/AQI","Hazardous");
      //i=5;
    }

    if(AQI >= 100 && loop_check == true)
    {
       LINE.notify("Get the fuck out of here. Dust Density is fucking High.");
       loop_check = false;
    }
    if(AQI <= 100 && loop_check == false)
    {
       LINE.notify("Come back please my love. I miss you Dust Density is low now");
       loop_check = true;
    }
    delay(1000);
  }

   if(power_on == false)
  {
    client.loop();
    dustDensity = 0;
    AQI = 0;
    vp_speed = 0;
    analogWrite(motor,vp_speed);
    lcd.clear();
    lcd.setCursor(2,0);
    lcd.print("POWER OFF");
    String data = "{\"data\": {\"dustDensity\":" + String(dustDensity) + ", \"speed\":" + String(vp_speed) + ", \"AQI\":" + String(AQI) + "}}";
    data.toCharArray(msg, (data.length() + 1));
    client.publish("@shadow/data/update", msg);
    delay(1000);
  }
 }
