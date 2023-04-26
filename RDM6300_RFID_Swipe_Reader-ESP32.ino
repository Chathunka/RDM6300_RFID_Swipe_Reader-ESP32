#include <HardwareSerial.h>

#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "ssid";
const char* password = "password";
const char* mqttServer = "demo.thingsboard.io";
const int mqttPort = 1883;
const char* mqttUser = "mqttuser";
const char* mqttPassword = "";

WiFiClient espClient;
PubSubClient client(espClient);

const int BUFFER_SIZE = 14; // RFID DATA FRAME FORMAT: 1byte head (value: 2), 10byte data (2byte version + 8byte tag), 2byte checksum, 1byte tail (value: 3)
const int DATA_SIZE = 10; // 10byte data (2byte version + 8byte tag)
const int DATA_VERSION_SIZE = 2; // 2byte version (actual meaning of these two bytes may vary)
const int DATA_TAG_SIZE = 8; // 8byte tag
const int CHECKSUM_SIZE = 2; // 2byte checksum

bool rdyUpdate = false;
int lastUpdatedAdd = 0;

uint8_t buffer[BUFFER_SIZE]; // used to store an incoming data frame 
int buffer_index = 0;

unsigned tags[20];
uint8_t tagindex = 0;
int waitingtimer = 0;
int waitingtime = 2000;

HardwareSerial ssrfid(1); // Using HardwareSerial on ESP32
const int RX_PIN = 4; // Change to appropriate pin for your board
const int TX_PIN = 5; // Change to appropriate pin for your board

const int LED = 12;
const int PLED = 13;

void setup() {
  Serial.begin(115200);
  Serial.println("RFID INIT");
  ssrfid.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN); // Initializing HardwareSerial on ESP32
  pinMode(LED,OUTPUT);
  pinMode(PLED,OUTPUT);
  digitalWrite(PLED,HIGH);
  digitalWrite(LED,LOW);
  Serial.println("RFID INIT - DONE");
  Serial.println("WIFI INIT");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("WIFI INIT - CONNECTED");
  Serial.println("MQTT INIT");
  client.setServer(mqttServer, mqttPort);
  reconnect();
  Serial.println("MQTT INIT - CONNECTED");
  waitingtimer = millis();
}

void loop() {
  if((millis() - waitingtimer) < waitingtime){
    digitalWrite(LED,HIGH);
    unsigned ctag = readTag();
    digitalWrite(LED,LOW);
    if(ctag > 0){
      if(tagindex == 0){
        rdyUpdate = true;
        tags[0]=ctag;
        tagindex++;
      }else{
        if(matchTag(ctag)){
          rdyUpdate = true;
          tags[tagindex] = ctag;  
          tagindex++;
        }else{
        }
      }
      waitingtimer = millis();
    }
  }else{
    digitalWrite(LED,LOW);
    Serial.println("update:");
    Serial.print("ItemsCountes : ");
    Serial.println(tagindex);
    showAllTags();
    if (!client.connected()) {
      reconnect();
    }
    client.loop();
    if(rdyUpdate){
      Serial.print("Sending data to Thingspeak : ");
      sendData(lastUpdatedAdd);
      rdyUpdate = false;
    }
    lastUpdatedAdd = tagindex;
    Serial.println("update: finished");
    delay(1000);
    waitingtimer = millis();
    delay(2);
//update
  }
}

void sendData(int lastUpdatedAdd) {
  for(int i =lastUpdatedAdd; i<tagindex; i++){
    char data[50];
    sprintf(data, "{\"Tag UID\": %d, \"Occurance\": %d}", tags[i], 1);
    client.publish("v1/devices/me/telemetry", data);
    Serial.print("updating: ");
    Serial.println(tags[i]);
    delay(500);
  }
  char data[25];
  sprintf(data, "{\"Tag Count\": %d}", tagindex);
  client.publish("v1/devices/me/telemetry", data);
}

void reconnect() {
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    if (client.connect("ESP32Client", mqttUser, mqttPassword )) {
      Serial.println("Connected to MQTT");
    } else {
      Serial.print("Failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }
}

void showAllTags(){
  for(int i =0; i<tagindex; i++){
    Serial.println(tags[i]);
  }
}

bool matchTag(unsigned tag){
  bool res = true;
  for(int i =0; i<tagindex; i++){
    if(tags[i] == tag){
      res = false;
      break;
    }
  }
  return res;
}

unsigned readTag(){
  unsigned tag=0;
  while (ssrfid.available() > 0){
    bool call_extract_tag = false;
    int ssvalue = ssrfid.read(); // read 
    if (ssvalue == -1) { // no data was read
      break;
    }
    if (ssvalue == 2) { // RDM630/RDM6300 found a tag => tag incoming 
      buffer_index = 0;
    } else if (ssvalue == 3) { // tag has been fully transmitted       
      call_extract_tag = true; // extract tag at the end of the function call
    }
    if (buffer_index >= BUFFER_SIZE) { // checking for a buffer overflow (It's very unlikely that an buffer overflow comes up!)
      Serial.println("Error: Buffer overflow detected! ");
      break;
    }
    buffer[buffer_index++] = ssvalue; // everything is alright => copy current value to buffer
    if (call_extract_tag == true) {
      if (buffer_index == BUFFER_SIZE) {
        tag = extract_tag();
        break;
      } else { // something is wrong... start again looking for preamble (value: 2)
        buffer_index = 0;
        break;
      }
    }    
  }
  return tag;
}

unsigned extract_tag() {
    uint8_t msg_head = buffer[0];
    uint8_t *msg_data = buffer + 1; // 10 byte => data contains 2byte version + 8byte tag
    uint8_t *msg_data_version = msg_data;
    char *msg_data_tag = (char*) (msg_data + 2);  // Change type to char*
    uint8_t *msg_checksum = buffer + 11; // 2 byte
    uint8_t msg_tail = buffer[13];

    long tag = hexstr_to_value(msg_data_tag, DATA_TAG_SIZE);
    Serial.print("Extracted Tag: ");
    Serial.println(tag);

    return tag;
}

long hexstr_to_value(char *str, unsigned int length) { // converts a hexadecimal value (encoded as ASCII string) to a numeric value
  char* copy = (char*)heap_caps_malloc((sizeof(char) * length) + 1, MALLOC_CAP_8BIT);
  memcpy(copy, str, sizeof(char) * length);
  copy[length] = '\0'; 
  // the variable "copy" is a copy of the parameter "str". "copy" has an additional '\0' element to make sure that "str" is null-terminated.
  long value = strtol(copy, NULL, 16);  // strtol converts a null-terminated string to a long value
  heap_caps_free(copy); // clean up 
  return value;
}
