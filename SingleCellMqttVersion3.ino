////when start button of charging/discharging is pressed ,only once reading of (starting_voltage , initial_ah , bat_state )of a single LFP cell are sent to MQTT server
////when stop button is pressed , only once reading of (ending_voltage, final_ah, bat_state )of a single LFP cell are sent to Mqtt server
////in between start and stop buttons continuous reading of cell_voltage, current , accumulated_charge of LFP cell are sent to MQTT  server
///through MQTT apps or server charging, discharging , stopping of the system can be done automatically by sending commands["01","10","00"] to (start charging, start discharging , stop)  [ here Stop is not working by command from mqtt server]
////esp32_id = "0x0000F4512BB3A3A0", from MQTT server by sending and verifying ESP_id then ON/OFF commands will work accordingly
////to eliminate less drawn current issue , INA219 is used
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <Adafruit_INA219.h>
Adafruit_INA219 ina219;
Adafruit_ADS1115 ads;                // Create an ADS1115 object
const char* ssid = "Asterix_Ace";
const char* password = "83524200";
const char* mqttServer = "122.166.84.234";
const int mqttPort = 1883;
const char* clientId = "d2s";
const char* clientId2 = "s2d";
const char* topic1 = "cycle_data";    // Topic for receiving data of start, stop button
const char* topic2 = "battery_data";  // Topic for continuous data

const char* username = "publisher";
const char* passwordd = "password@publisher1";

WiFiClient espClient;
PubSubClient client(espClient);

// Define pin connections
#define BUTTON_PIN_ch 18
#define BUTTON_PIN_dh 19
#define LED_PIN_ch 16
#define LED_PIN_dh 17
#define relay_PIN_ch 14
#define relay_PIN_dh 23
#define BUTTON_PIN_stop 13


// Define variables to store the values
float starting_voltage = 0;
float ending_voltage = 0;
float initial_mAh = 0;
float final_mAh = 0;

float voltage1 = 0.0;
float B_total = 0.0;

//float results;
float current;

//single cell parameters
//int16_t adc1;
float voltage_cell;
static int callCount = 0;  // Static variable to store the count
float gain = 0;              //These are two internal factory set values.
//int offset = 0;              //We read them once at boot up and use them in many functions
long totalCoulombCount = 0;  //Keeps track of overall pack fuel gauage

const float shunt_resistance = 0.7;  // Shunt resistor value in ohms (change as per your resistor)
const int sample_rate = 50;          // Sample rate in milliseconds
unsigned long last_time = 0;
// Variables to store the last reading of data before stopping
float last_accumulated_mAh = 0;
unsigned long last_stop_time = 0;
float last_voltage_cell = 0;  // To store the last voltage reading
float last_current = 0;       // To store the last current reading

// Global variable
String esp32_id = "0x0000F4512BB3A3A0"; // Unique identifier for this ESP32
// Debounce parameters
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 200;  // 50 ms debounce time
bool ledState_ch = false;           // Current state of LED
bool ledState_stop = false;
bool ledState_dh = false;
bool lastButtonState_ch = false;  // Previous state of button
bool lastButtonState_dh = false;
bool lastButtonState_stop = false;

bool reading_dh_butt;
bool reading_stop_butt;
bool reading_ch_butt;

bool ch2_state = false;
bool dh2_state = false;
// Global variable to track if charging should start
bool startCharging = false;
bool startDischarging = false;
bool turnOffSystem = false;
bool id_verified = true; // Flag to check if the ID is verified

float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;
float accumulated_mAh = 0; // Variable to store accumulated charge in mAh
unsigned long previousMillis = 0; // Variable to store the previous time

void setup() {
  Serial.begin(9600); 
  
  // Retrieve ESP32 Chip ID (based on the MAC address)
  uint64_t chipId = ESP.getEfuseMac(); // Get the unique 64-bit MAC
  esp32_id = String((uint32_t)(chipId >> 32), HEX) + String((uint32_t)chipId, HEX);
  esp32_id.toUpperCase(); // Convert to uppercase for a consistent ID format

  Serial.print("ESP32 ID (Chip ID): 0x");
  Serial.println(esp32_id);

  setup_wifi();  //setup communication interfaces(Serial , MQTT)
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

  Wire.begin();

  delay(1000);
  while (!Serial) {
      // will pause Zero, Leonardo, etc until serial console opens
      delay(1);
  }

  uint32_t currentFrequency;
    
  Serial.println("Hello!");

  if (! ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }
   // Set calibration
  ina219.setCalibration_32V_2A();  // Use this or adjust for your nee
  Serial.println("Measuring voltage and current with INA219 ...");
  ads.begin(); //initialize ADC module
  pinMode(BUTTON_PIN_ch, INPUT);
  pinMode(BUTTON_PIN_stop, INPUT);
  pinMode(BUTTON_PIN_dh, INPUT);
 
  pinMode(relay_PIN_ch, OUTPUT);
  pinMode(relay_PIN_dh, OUTPUT);
  pinMode(LED_PIN_ch, OUTPUT);
  pinMode(LED_PIN_dh, OUTPUT);
  digitalWrite(LED_PIN_ch, LOW);
  digitalWrite(LED_PIN_dh, LOW);
  digitalWrite(relay_PIN_ch, HIGH);
  digitalWrite(relay_PIN_dh, HIGH);
  
  lastButtonState_ch = false;
  lastButtonState_dh = false;
  lastButtonState_stop = false;
  ledState_ch = false;
  ledState_dh = false;
  ledState_stop = false;
}

void captureStartingConditions() {
  starting_voltage = ina219.getBusVoltage_V();  // Read the cell voltage
  initial_mAh= accumulated_mAh;                                      // Store the current accumulated Ah
  Serial.println(" Starting Conditions...");
  Serial.print("Starting Voltage (V): ");
  Serial.println(starting_voltage, 2);
}

void captureEndingConditions() {
  ending_voltage = ina219.getBusVoltage_V();     // Read the cell voltage at stop
  final_mAh = accumulated_mAh;                                         // Store the final accumulated Ah
  //last_accumulated_charge = accumulated_charge;                      // Store the final accumulated charge
  last_accumulated_mAh = accumulated_mAh;                              // Store the final Ah reading
  last_voltage_cell =ina219.getBusVoltage_V();  // Store the last cell voltage reading                                          // Store the last current reading
  last_stop_time = millis();                                         // Capture the time when stop was pressed

  Serial.println("Ending Conditions...");
  Serial.print("Ending Voltage (V): ");
  Serial.println(ending_voltage, 2);
  Serial.print("Final Ampere-hours (mAh): ");
  Serial.println(final_mAh, 2);
  Serial.print("Last Cell Voltage (V): ");
  Serial.println(last_voltage_cell, 2);
  Serial.print("Last Current (A): ");
  Serial.println(last_current, 6);
}


void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();


  // Check if charging needs to be started
  if (startCharging) {
    // Turn on the relay and LED for charging
    captureStartingConditions();                        // Capture starting conditions for charging
    displayVoltages1(starting_voltage, initial_mAh, 1);  // Display starting conditions
    digitalWrite(LED_PIN_ch, HIGH);                     // Turn on charging LED
    digitalWrite(relay_PIN_ch, LOW);                    // Activate relay for charging
    ch2_state = true;
    startCharging = false;  // Reset the flag after charging has started
  }
   

  // Check if discharging needs to be started
  if (startDischarging) {
    // Start discharging
    captureStartingConditions();
    displayVoltages1(starting_voltage, initial_mAh, 2);  // Display starting conditions for discharging
    digitalWrite(LED_PIN_dh, HIGH);  // Turn on discharging LED
    digitalWrite(relay_PIN_dh, LOW); // Activate relay for discharging
    dh2_state = true;
    startDischarging = false;  // Reset flag after discharging has started
  }
  
 


  // Check if Turn Off is triggered
  if (turnOffSystem) {
    Serial.println("System turning off...");
    
    // Implement turn off logic
    digitalWrite(LED_PIN_ch, LOW);  // Turn off charging LED
    digitalWrite(relay_PIN_ch, HIGH);  // Turn off charging relay
    digitalWrite(LED_PIN_dh, LOW);  // Turn off discharging LED
    digitalWrite(relay_PIN_dh, HIGH);  // Turn off discharging relay

    // Reset states and flags
    startCharging = false;
    startDischarging = false;
    //ledState_stop = !ledState_stop;
    ch2_state = false;
    dh2_state = false;
    
    turnOffSystem = false;  // Reset the turn off flag after processing
  }

  // Check if the system should be turned off
 
  reading_ch_butt = digitalRead(BUTTON_PIN_ch);
  reading_stop_butt = digitalRead(BUTTON_PIN_stop);
  reading_dh_butt = digitalRead(BUTTON_PIN_dh);

  // Handle button press for charging
  if (reading_ch_butt != lastButtonState_ch) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) < debounceDelay && reading_ch_butt == HIGH && lastButtonState_ch != reading_ch_butt) {
    ledState_ch = !ledState_ch;
    if (ledState_ch) {
      captureStartingConditions();
      displayVoltages1(starting_voltage, initial_mAh, 1);
    }
  }

  // Handle button press for discharging
  if (reading_dh_butt != lastButtonState_dh) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) < debounceDelay && reading_dh_butt == HIGH && lastButtonState_dh != reading_dh_butt) {
    ledState_dh = !ledState_dh;
    if (ledState_dh) {
      captureStartingConditions();
      displayVoltages1(starting_voltage, initial_mAh, 2);
    }
  }

  // Handle stop button
  if (reading_stop_butt != lastButtonState_stop) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) < debounceDelay && reading_stop_butt == HIGH) {
    ledState_stop = !ledState_stop;
    if (ledState_stop) {
      captureEndingConditions();
      displayVoltages2(ending_voltage, final_mAh, 0);
      // displayVoltages(starting_voltage, ending_voltage, initial_ah, final_ah, 2); // 2 for stopping state
    }
  }


  // Save the reading for the next loop

  if (ledState_ch == true && dh2_state == false) {

    captureStartingConditions();


    digitalWrite(LED_PIN_ch, HIGH);
    digitalWrite(relay_PIN_ch, LOW);
    ledState_ch = !ledState_ch;
    ch2_state = true;
    Serial.println("ch//////////");
    Serial.println(ch2_state);
    Serial.println(dh2_state);
    Serial.println(ledState_ch);
    Serial.println("//////////");
  }

  if (ledState_stop == true) {

    captureEndingConditions();
    digitalWrite(LED_PIN_ch, LOW);
    digitalWrite(relay_PIN_ch, HIGH);
    digitalWrite(LED_PIN_dh, LOW);
    digitalWrite(relay_PIN_dh, HIGH);
    ledState_stop = !ledState_stop;
    ch2_state = false;
    dh2_state = false;
  }
  if (ledState_dh == true && ch2_state == false) {

    captureStartingConditions();
    Serial.print("Starting Voltage(V) : ");
    Serial.println(starting_voltage, 2);
    Serial.print("Initial_ah : ");
    Serial.println(initial_mAh, 2);

    Serial.println("dh//////////");
    Serial.println(ch2_state);
    Serial.println(dh2_state);

    digitalWrite(LED_PIN_dh, HIGH);
    digitalWrite(relay_PIN_dh, LOW);
    ledState_dh = !ledState_dh;
    dh2_state = true;
  }


  if (ch2_state == true) {
      shuntvoltage = ina219.getShuntVoltage_mV();
      busvoltage = ina219.getBusVoltage_V();
      current_mA = ina219.getCurrent_mA();
      
      if (current_mA < 0) {
            current_mA = -current_mA; // Force positive current
      }
    //  current_mA = abs(ina219.getCurrent_mA()); // Ensure current is positive current_mA = abs(ina219.getCurrent_mA()) This ensures that the current value is always a positive number, regardless of its original sign.
      loadvoltage = busvoltage + (shuntvoltage / 1000);
     
      unsigned long currentMillis = millis(); // Get the current time
      float elapsedHours = (currentMillis - previousMillis) / 3600000.0; // Convert elapsed time to hours
      previousMillis = currentMillis; // Update the previous time

     // Accumulate the charge (mAh)
      accumulated_mAh += current_mA * elapsedHours;
  
      Serial.print("cell_Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
      Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
      Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
      Serial.print("Current:       "); Serial.print(current_mA,4); Serial.println(" mA");
      Serial.print("Accumulated Charge: "); Serial.print(accumulated_mAh); Serial.println(" mAh");
      Serial.println("");
       int bat_state = 1;  // Set charging state
        
       delay(2000);
        // Call your display function
      displayVoltages(bat_state, current_mA, accumulated_mAh);

        // Check if charging should stop
      check_st_status();
           if (ledState_stop == true || voltage_cell >= 3.7 || turnOffSystem) {
            Serial.println("Charge threshold reached, stopping charging.");

            // Store the current values before stopping
            last_accumulated_mAh = accumulated_mAh;
            last_stop_time = last_time;

            // Capture the voltage before stopping
            starting_voltage = ina219.getBusVoltage_V();
            final_mAh = accumulated_mAh;

            // Display final values
            Serial.print("Starting Voltage(V) : ");
            Serial.println(starting_voltage, 2);
            Serial.print("final_ah : ");
            Serial.println(final_mAh, 2);

            // Turn off charging and relays
            digitalWrite(LED_PIN_ch, LOW);
            digitalWrite(relay_PIN_ch, HIGH);
            digitalWrite(LED_PIN_dh, LOW);
            digitalWrite(relay_PIN_dh, HIGH);

            // Reset states
            ledState_stop = !ledState_stop;
            ch2_state = false;
            dh2_state = false;
          //  break;
          return;
        }
    }

  
if (dh2_state == true) {

      shuntvoltage = ina219.getShuntVoltage_mV();
      busvoltage = ina219.getBusVoltage_V();
      current_mA = ina219.getCurrent_mA();
       // Ensure current is negative during discharging (bat_state == 2)
      int bat_state = 2;  // Set discharging state
     // if (bat_state == 2)
     // {
      //  current = -abs(current);  // Make sure current is negative during discharging
     // }
    //  current_mA = abs(ina219.getCurrent_mA()); // Ensure current is positive current_mA = abs(ina219.getCurrent_mA()) This ensures that the current value is always a positive number, regardless of its original sign.
      loadvoltage = busvoltage + (shuntvoltage / 1000);
     
      unsigned long currentMillis = millis(); // Get the current time
      float elapsedHours = (currentMillis - previousMillis) / 3600000.0; // Convert elapsed time to hours
      previousMillis = currentMillis; // Update the previous time

     // Accumulate the charge (mAh)
      accumulated_mAh += current_mA * elapsedHours;
  
      Serial.print("cell_Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
      Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
      Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
      Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
      Serial.print("Accumulated Charge: "); Serial.print(accumulated_mAh); Serial.println(" mAh");
      Serial.println("");
     
        
      delay(2000);
        // Call your display function
      displayVoltages(bat_state, current_mA, accumulated_mAh);

    delay(sample_rate);

    // Check if discharging should stop
    check_st_status();
    if (ledState_stop == true || voltage_cell <= 2.5 || turnOffSystem) {
      Serial.println("Discharge threshold reached, breaking the loop.");
      
      // Store the current values before stopping
      last_stop_time = last_time;

      // Capture the voltage before stopping
      ending_voltage = ina219.getBusVoltage_V() ;
      final_mAh = accumulated_mAh;

      // Turn off discharging and relays
      digitalWrite(LED_PIN_ch, LOW);
      digitalWrite(relay_PIN_ch, HIGH);
      digitalWrite(LED_PIN_dh, LOW);
      digitalWrite(relay_PIN_dh, HIGH);
      
      // Reset states
      ledState_stop = !ledState_stop;
      ch2_state = false;
      dh2_state = false;
      return;
    //  break;  // Exit the while loop when condition is met
    }
}

  lastButtonState_ch = reading_ch_butt;
  lastButtonState_stop = reading_stop_butt;
  lastButtonState_dh = reading_dh_butt;
}


void check_st_status() {
  bool reading_stop_butt = digitalRead(BUTTON_PIN_stop);

  if (reading_stop_butt != lastButtonState_stop) {
    lastDebounceTime = millis();  // Reset the debounce timer
  }
  if ((millis() - lastDebounceTime) < debounceDelay) {
    // If the button state has changed from its previous state
    if (reading_stop_butt == HIGH) {
      ledState_stop = !ledState_stop;
    }
  }
  if (ledState_stop == true) {
    digitalWrite(LED_PIN_ch, LOW);
    digitalWrite(relay_PIN_ch, HIGH);
    digitalWrite(LED_PIN_dh, LOW);
    digitalWrite(relay_PIN_dh, HIGH);
    ledState_stop = !ledState_stop;
    ch2_state = false;
    dh2_state = false;
  }
}
void displayVoltages(int bat_state, float current_mA, float accumulated_mAh) {
 
  countFunctionCalls();

   voltage_cell= ina219.getBusVoltage_V();
 
  // voltage_cell = (adc1 * 0.1875)/1000;   
  Serial.println("voltage_cell..........");
  Serial.println(voltage_cell);
  Serial.println();



  // Create JSON object
  StaticJsonDocument<2000> jsonDocument1;

  char voltage_cell1[8];  // Buffer to hold the formatted string
  dtostrf(voltage_cell, 1, 5, voltage_cell1);

  char accumulated_mAh_rr[8];  // Buffer to hold the formatted string
  dtostrf(accumulated_mAh, 1, 5, accumulated_mAh_rr);
  char current_rr[8];  // Buffer to hold the formatted string
  dtostrf(current_mA, 1, 5, current_rr);

  jsonDocument1["cc"] = callCount;
  jsonDocument1["vcell"] = voltage_cell1;
  jsonDocument1["I"] = current_rr;
  jsonDocument1["mAh"] = accumulated_mAh_rr;
  jsonDocument1["bat_state"] = bat_state;

  // Serialize JSON to string
  String jsonString1;
  serializeJson(jsonDocument1, jsonString1);

  // Publish JSON string to MQTT topic
  client.publish("battery_data", jsonString1.c_str());

  Serial.println("Published sensor data:");
  Serial.println(jsonString1);
}


void displayVoltages1(float starting_voltage, float initial_mAh, int bat_state) {
  char starting_voltage_str[20];
  char initial_mAh_str[20];
  dtostrf(starting_voltage, 1, 6, starting_voltage_str);
  dtostrf(initial_mAh, 1, 6, initial_mAh_str);

  // Creating JSON document for starting voltage and initial Ah
  StaticJsonDocument<512> jsonDocument;  // Adjust buffer size
  jsonDocument["starting_voltage"] = starting_voltage_str;
  jsonDocument["initial_mAh"] = initial_mAh_str;
  jsonDocument["batstate"] = bat_state;

  String jsonString;
  serializeJson(jsonDocument, jsonString);
  // Publishing the JSON data via MQTT
  bool result = client.publish("cycle_data", jsonString.c_str(), true);  // Retain message
 
  // Logging the result
  Serial.println("Publishing starting voltage data: ");
  Serial.println(jsonString);

  if (result) {
    Serial.println("MQTT publish (starting voltage) successful.");
  } else {
    Serial.println("MQTT publish (starting voltage) failed.");
  }
}

void displayVoltages2(float ending_voltage, float final_mAh, int bat_state) {
  char ending_voltage_str[20];
  char final_mAh_str[20];
  dtostrf(ending_voltage, 1, 6, ending_voltage_str);
  dtostrf(final_mAh, 1, 6, final_mAh_str);

  // Creating JSON document for ending voltage and final Ah
  StaticJsonDocument<512> jsonDocument;  // Adjust buffer size
  jsonDocument["ending_voltage"] = ending_voltage_str;
  jsonDocument["final_mAh"] = final_mAh_str;
  jsonDocument["batstate"] = bat_state;

  String jsonString;
  serializeJson(jsonDocument, jsonString);

  // Publishing the JSON data via MQTT
  bool result = client.publish("cycle_data", jsonString.c_str(), true);  // Retain message

  // Logging the result
  Serial.println("Publishing ending voltage data: ");
  Serial.println(jsonString);

  if (result) {
    Serial.println("MQTT publish (ending voltage) successful.");
  } else {
    Serial.println("MQTT publish (ending voltage) failed.");
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(clientId, username, passwordd)) {
      Serial.println("connected");
      client.subscribe("cycle_data");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}



void setup_wifi() {
  delay(10);
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
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);

  // Parse the JSON payload
  StaticJsonDocument<200> doc;
  char payloadStr[length + 1];
  memcpy(payloadStr, payload, length);
  payloadStr[length] = '\0';

  Serial.print("Received payload: ");
  Serial.println(payloadStr);

  DeserializationError error = deserializeJson(doc, payloadStr);
  if (error) {
    Serial.print("JSON parsing failed: ");
    Serial.println(error.f_str());
    return;
  }

  // Handle ID verification
  if (doc.containsKey("verify_id")) {
    String received_id = doc["verify_id"];
    received_id.trim(); // Remove spaces

    Serial.print("Expected ID: ");
    Serial.println(esp32_id);
    Serial.print("Received ID for verification: ");
    Serial.println(received_id);

    if (received_id.equals(esp32_id)) { // Use equals() for String comparison
      Serial.println("ID verified successfully.");
      id_verified = true;

      // Send acknowledgment to the server
      StaticJsonDocument<128> ackDoc;
      ackDoc["esp_id"] = esp32_id;
      ackDoc["status"] = "ID Verified";
      String ackJson;
      serializeJson(ackDoc, ackJson);
      if (client.publish("id_verification_response", ackJson.c_str())) {
        Serial.println("ID verification acknowledgment sent.");
      } else {
        Serial.println("Failed to send ID verification acknowledgment.");
      }
    } else {
      Serial.println("ID verification failed. Ignoring commands.");
      id_verified = false;

      // Send failure acknowledgment
      StaticJsonDocument<128> nackDoc;
      nackDoc["esp_id"] = esp32_id;
      nackDoc["status"] = "ID Verification Failed";
      String nackJson;
      serializeJson(nackDoc, nackJson);
      if (client.publish("id_verification_response", nackJson.c_str())) {
        Serial.println("ID verification failure acknowledgment sent.");
      } else {
        Serial.println("Failed to send ID verification failure acknowledgment.");
      }
    }
    return; // Exit after ID verification
  }

  // Process commands only if ID is verified
  if (id_verified) {
    if (doc.containsKey("command")) {
      String command = doc["command"]; // Use String for ease of comparison
      command.trim(); // Remove spaces

      Serial.print("Received command: ");
      Serial.println(command);

      if (command == "01") {
        Serial.println("Processing Start Charging command.");
        startCharging = true;
        startDischarging = false;
        turnOffSystem = false;
      } else if (command == "10") {
        Serial.println("Processing Start Discharging command.");
        startCharging = false;
        startDischarging = true;
        turnOffSystem = false;
      } else if (command == "00") {
        Serial.println("Processing Turn Off command.");
        startCharging = false;
        startDischarging = false;
        turnOffSystem = true;
      } else {
        Serial.println("Unknown command received.");
      }
    } else {
      Serial.println("No command found in the payload.");
    }
  } else {
    Serial.println("Command ignored: ID not verified.");
  }
}



void countFunctionCalls() {
  // static int callCount = 0; // Static variable to store the count

  callCount++;  // Increment the count each time the function is called

  // Check if callCount is greater than or equal to 10000
  if (callCount >= 1000000) {
    callCount = 0;  // Reset the count to 0
    Serial.println("Call count has been reset.");
  }

  Serial.print("Function has been called ");
  Serial.print(callCount);
  Serial.println(" times");
}

