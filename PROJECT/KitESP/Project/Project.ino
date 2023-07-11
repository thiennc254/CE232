#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "NCT";
const char* password = "00000000";

#define MQTT_SERVER "mqtt.flespi.io"
#define MQTT_PORT 1883
#define MQTT_USER "7qhnUejGlaRwMhWs5DXGbDuUOlOwYK4V1b9LegfneUJQ1w8UXWoWnGGZSkmGTepK"
#define MQTT_PASSWORD ""
#define MQTT_LDP_TOPIC "abc"

WiFiClient wifiClient;
PubSubClient client(wifiClient);

unsigned long long timeStart = 0;
unsigned long long timeTarget = 0;
char prevMsg = '0';
char currMsg = '0';
#define MAXCHAIN 10
int chain = 0;

enum state {
  S0,
  S1,
  S2,
  S3,
  S4,
  S5,
  S6,
};

int led20 = 18;
int led40 = 5;
int led60 = 17;
int led80 = 16;
int led100 = 4;
state currState = S0;

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  // for (int i = 0; i < length; i++) {
  //   Serial.print((char)payload[i]);
  //   Serial.print(length);
  // }
  // if ((char)payload[0] == '1') {
  //   Serial.println("Close");
  // } else {
  //   Serial.println("Open");
  // }

  currMsg = (char)payload[0];
  // updateChain(currMsg);
  updateState();
  Serial.print(currMsg);
  Serial.print(" / ");
  Serial.print(currState);
  Serial.print(" / ");
  Serial.println(chain);
  prevMsg = currMsg;
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD)) {
      Serial.println("connected");
      client.subscribe(MQTT_LDP_TOPIC);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");
      delay(2000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(callback);
  reconnect();

  pinMode(led20, OUTPUT);
  pinMode(led40, OUTPUT);
  pinMode(led60, OUTPUT);
  pinMode(led80, OUTPUT);
  pinMode(led100, OUTPUT);
  AllON();
}

void startTimer(unsigned long long timeWait) {
  timeStart = millis();
  timeTarget = timeWait;
}

bool isTimerDone() {
  return (millis() - timeStart) >= timeTarget;
}

int updateChain(char Msg) {
  if (Msg == prevMsg) {
    chain++;
  } else {
    chain = 0;
  }
  return chain;
}

void updateState() {
  switch (currState) {
    case S0:
      AllON();
      if (currMsg == '1' && updateChain(currMsg) >= MAXCHAIN) {
        currState = S1;
        // startTimer(15000);
        startTimer(5000);
      }
      break;
    case S1:
      AllON();
      if (currMsg == '0' && updateChain(currMsg) == MAXCHAIN) {
        currState = S0;
        startTimer(0);
      } else if (isTimerDone()){
        currState = S2;
        startTimer(5000);
      } 
      break;
    case S2:
      AllON();
      if (currMsg == '0' && updateChain(currMsg) == MAXCHAIN) {
        currState = S0;
      } else if (isTimerDone()){
        currState = S3;
        startTimer(5000);
      } 
      break;
    case S3:
      digitalWrite(led100, LOW);
      if (currMsg == '0' && updateChain(currMsg) == MAXCHAIN) {
        currState = S2;
        startTimer(5000);
        digitalWrite(led100, HIGH);
      } else if (isTimerDone()){
        currState = S4;
        startTimer(5000);
      } 
      break;
    case S4:
      digitalWrite(led80, LOW);
      if (currMsg == '0' && updateChain(currMsg) == MAXCHAIN) {
        currState = S3;
        startTimer(5000);
        digitalWrite(led80, HIGH);
      } else if (isTimerDone()){
        currState = S5;
        startTimer(5000);
      } 
      break;
    case S5:
      digitalWrite(led60, LOW);
      if (currMsg == '0' && updateChain(currMsg) == MAXCHAIN) {
        currState = S4;
        startTimer(5000);
        digitalWrite(led60, HIGH);
      } else if (isTimerDone()){
        currState = S6;
      } 
      break;
    case S6:
      digitalWrite(led40, LOW);
      if (currMsg == '0' && updateChain(currMsg) == MAXCHAIN) {
        currState = S5;
        startTimer(5000);
        digitalWrite(led40, HIGH);
      }
      break;
    default:
      AllON();
      break;
  }
}

void loop() {
  client.loop();
  if (!client.connected()) {
    reconnect();
  }
}


void AllOFF() {
  digitalWrite(led20, LOW);
  digitalWrite(led40, LOW);
  digitalWrite(led60, LOW);
  digitalWrite(led80, LOW);
  digitalWrite(led100, LOW);
}

void AllON() {
  digitalWrite(led20, HIGH);
  digitalWrite(led40, HIGH);
  digitalWrite(led60, HIGH);
  digitalWrite(led80, HIGH);
  digitalWrite(led100, HIGH);
  
}
