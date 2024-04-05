
/*
      Controller Type: 4 Relay
      Controller id  : CON1867
      EXtra support  : modbus interface RS485
*/


/*********controller name*******************/
#define ControllerID "CON1867"  //make sure evrery controller is unique

/*********library header files**************/
#include <WiFi.h>
#include "define.h"
#include <WebServer.h>
#include <WiFiMulti.h>
#include "WiFiClientSecure.h"
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include "credentials.h"
#include "GitHubOTA.h"
#include <ArduinoJson.h>

/**************WiFi credentials***********************************************/
const char* ssid = "Vaibhav1";      //wifi ssid
const char* password = "12345678";  //wifi password



/************ MQTT Broker credentials*****************************************/
const char* mqtt_broker = "broker.emqx.io";  //broker url
const int mqtt_port = 8883;                  //mqtt port



/**********  Set ssl certificate***********************************************/
const char* root_ca = CA_CRT;  //ca crt
//const char* server_cert = SERVER_CERT;       //server cart
//const char* server_key  = SERVER_KEY;        //KEY



/************mqtt IN_out_alive topics******************************************/
char Topic_INPUT[50] = "controller/CON1867/INPUT/";
char Topic_OUTPUT[50] = "controller/CON1867/OUTPUT/";
char Alive[50] = "controller/CON1867/alive/";
/*************mqtt input output ************************************/
char input_message[50];
char output_message[50];

/*****************watchdog timer******************************************/
const int button = 35;  //gpio to use to trigger delay
const int button1 = 34;
const int wdtTimeout_Task_1 = 5000;  //time in ms to trigger the watchdog
const int wdtTimeout_Task_2 = 5000;  //time in ms to trigger the watchdog
hw_timer_t* timer_Task_1 = NULL;
hw_timer_t* timer_Task_2 = NULL;

/************************Interval Delay***************************************/
unsigned long int current_time = 0;
unsigned long int previous_time = 0;
uint16_t Interval = 1000;
bool flag_data_error = false;


void ARDUINO_ISR_ATTR resetModule() {
  ets_printf("reboot\n");
  esp_restart();
}

// WiFiClient espClient;
WiFiClientSecure espClient;
PubSubClient client(espClient);

TaskHandle_t Task1;
TaskHandle_t Task2;

volatile bool flag_mqtt=false;

void setup() {
  /******************testing debug UART buad rate**********************************/
  Serial.begin(115200);        // Set software serial baud to 115200;
  WiFi.begin(ssid, password);  // Connecting to a WiFi network
  while (!Serial) { delay(10); }
  /***********************realy pin config*************************************/
  pinMode(relay_1, OUTPUT);
  pinMode(relay_2, OUTPUT);
  pinMode(relay_3, OUTPUT);
  pinMode(relay_4, OUTPUT);
  /*****************************attacheted 1task timer 1**********************/
  timer_Task_1 = timerBegin(1, 80, true);                          //timer 0, div 80
  timerAttachInterrupt(timer_Task_1, &resetModule, true);          //attach callback
  timerAlarmWrite(timer_Task_1, wdtTimeout_Task_1 * 1000, false);  //set time in us
  timerAlarmEnable(timer_Task_1);
  /*****************************attacheted 2task timer 2**********************/
  timer_Task_2 = timerBegin(2, 80, true);                          //timer 0, div 80
  timerAttachInterrupt(timer_Task_2, &resetModule, true);          //attach callback
  timerAlarmWrite(timer_Task_2, wdtTimeout_Task_2 * 1000, false);  //set time in us
  timerAlarmEnable(timer_Task_2);
  /******************************input button *************************************/
  // pinMode(button, INPUT_PULLUP);
  // pinMode(button1, INPUT_PULLUP);
  //pinMode(button1, INPUT_PULLUP);
  // pinMode(button1, INPUT_PULLUP);


  /******************************start dual core mode**********************/
  xTaskCreatePinnedToCore(Task1code, "Task1", 10000, NULL, 1, &Task1, 0);
  delay(500);
  xTaskCreatePinnedToCore(Task2code, "Task2", 10000, NULL, 1, &Task2, 1);
  delay(500);

  /****************************wifi ***********************************/
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  /**************OTA Update***************************/
  if (newFirmwareAvailable()) {
    updateFirmware();
  }
  /****************Connecting to a mqtt broker width ssl certification******************/
  espClient.setCACert(root_ca);
  //espClient.setCertificate(server_cert);  // for client verification
  // espClient.setPrivateKey(server_key);    // for client verification

  /*********** Connect to the MQTT Broker remotely***************************************/
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);

  while (!client.connected()) {
    String client_id = "esp32-client";
    client_id += String(WiFi.macAddress());
    /**********Print the Name and the id of the esp32 controller********************/
    Serial.printf("The client %s connects to the public mqtt broker\n", client_id.c_str());
    if (client.connect(client_id.c_str())) {
      Serial.println("Public emqx mqtt broker connected");
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }
  /*****************Test meseeage Publish and Subscribe****************************/
  client.publish(Topic_OUTPUT, "Controller is online");
  client.subscribe(Topic_INPUT);
}



void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived in topic: ");
  Serial.println(Topic_OUTPUT);
  Serial.print("Message:");
  memcpy(input_message, payload, length);
  input_message[length] = '\0';
  flag_mqtt =true;
  Serial.print(input_message);
  Serial.println();
  Serial.println("-----------------------");
}

/*****************************Core 1 Start*****************************************************************************************************************************/
void Task1code(void* parameter) {
  Serial.print("Task1 is running on core ");
  Serial.println(xPortGetCoreID());

  for (;;) {


    timerWrite(timer_Task_2, 0);  //reset timer (feed watchdog)
                                  // long loopTime = millis();
    current_time = millis();
    //while button is pressed, delay up to 3 seconds to trigger the timer
    // while (!digitalRead(button1)) {
    //   Serial.println("button pressed");
    //   vTaskDelay(10 / portTICK_PERIOD_MS);
    // }
    /*******************************************code logic Area************************************************/
    //all_Blink();
    client.loop();
    client.publish(Alive, "A");
    // Send a message every 10 seconds

    /*******************************************code logic Area************************************************/

    // loopTime = millis() - loopTime;
    // Serial.print("loop time is = ");
    // Serial.println(loopTime);  //should be under 3000
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}
/*************************core 1 End***********************************************************************************************************************************/
/*XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX*/
/*************************Core 2 Start*********************************************************************************************************************************/
void Task2code(void* parameter) {
  Serial.print("Task2 is running on core ");
  Serial.println(xPortGetCoreID());

  for (;;) {
    // Serial.println("running main loop");
    timerWrite(timer_Task_1, 0);  //reset timer (feed watchdog)
                                  //long loopTime = millis();

    //while button is pressed, delay up to 3 seconds to trigger the timer
    // while (!digitalRead(button)) {
    //   Serial.println("button pressed");
    //   vTaskDelay(500 / portTICK_PERIOD_MS);
    // }
    // delay(1000);  //simulate work
    /************************************code logic Area********************************/
    // Create a JSON object
    // Parse JSON

  //   if (flag_mqtt) {
  //     DynamicJsonDocument doc(512);
  //     DeserializationError error = deserializeJson(doc, input_message);
  //     if (error) {
  //       Serial.print("deserializeJson() failed: ");
  //       Serial.println(error.c_str());
  //       return;
  //     }
  //     // Extract command for relay i from JSON
  //     String relayCommand = "relay" + String(0 + 1);
  //     const char* command = doc[relayCommand.c_str()];

  //     // Check if command exists for relay i
  //     if (command != nullptr) {
  //       if (strcmp(command, "on") == 0) {
  //         relayStates[i] = true;
  //         digitalWrite(relay_1, HIGH);
  //         Serial.println(relayCommand + " turned ON");
  //       } else if (strcmp(command, "off") == 0) {
  //         relayStates[i] = false;
  //         digitalWrite(relay_1, LOW);
  //         Serial.println(relayCommand + " turned OFF");
  //       } else {
  //         Serial.println("Invalid command");
  //       }
  //     }
  //   }
  //   flag_mqtt = false;


  // DynamicJsonDocument doc(200);
  // // Add relay states to the JSON object
  //   String relayKey = "relay1" 
  //   doc[relayKey] = relayStates[i] ? "on" : "off";
  // }
  // // Serialize the JSON object to a string
  // String jsonString;
  // serializeJson(doc, jsonString);
  // // Convert the string to a char array
  // char payload[jsonString.length() + 1];
  // jsonString.toCharArray(payload, jsonString.length() + 1);
  // // Publish the JSON string
  // client.publish(Topic_OUTPUT, payload);

  /************************************Code area End*****************************************/
  // loopTime = millis() - loopTime;
  // Serial.print("loop time is = ");
  // Serial.println(loopTime);  //should be under 3000
  vTaskDelay(2000 / portTICK_PERIOD_MS);
}
}
/**************************Core 2 End ********************************************************/
void loop() {
}
