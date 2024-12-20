////when start button of charging/discharging is pressed ,only once reading of (starting_voltage , initial_ah , bat_state )of a single LFP cell are sent to MQTT server
////when stop button is pressed , only once reading of (ending_voltage, final_ah, bat_state )of a single LFP cell are sent to Mqtt server
////in between start and stop buttons continuous reading of cell_voltage, current , accumulated_charge of LFP cell are sent to MQTT  server
///through MQTT apps or server charging, discharging , stopping of the system can be done automatically by sending commands["01","10","00"] to (start charging, start discharging , stop)  [ here Stop is not working by command from mqtt server]
////esp32_id = "0x0000F4512BB3A3A0", from MQTT server by sending and verifying ESP_id then ON/OFF commands will work accordingly

///disadvantage here : while charging this cell, current (I) A drawn is very less

#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFi.h>


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

float battery1;
float I_total;

// Define variables to store the values
float starting_voltage = 0;
float ending_voltage = 0;
float initial_ah = 0;
float final_ah = 0;

float voltage1 = 0.0;
float B_total = 0.0;
float vtotal;
float results;
float current;

//single cell parameters
int16_t adc1;
float voltage_cell;
void displayVoltages(float starting_voltage, float ending_voltage, float initial_ah, float final_ah, int bat_state);
static int callCount = 0;  // Static variable to store the count

//void displayVoltages(float starting_voltage, float ending_voltage, float initial_ah, float final_ah, int bat_state);

Adafruit_ADS1115 ads;                // Create an ADS1115 object
const float shunt_resistance = 0.7;  // Shunt resistor value in ohms (change as per your resistor)
const int sample_rate = 50;          // Sample rate in milliseconds

float accumulated_charge = 0;  // To store accumulated charge in Coulombs
float accumulated_ah = 0;      // To store accumulated Ampere-hours
unsigned long last_time = 0;
// Variables to store the last reading of data before stopping
float last_accumulated_charge = 0;
float last_accumulated_ah = 0;
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
  starting_voltage = ads.computeVolts(ads.readADC_SingleEnded(1));  // Read the cell voltage
  initial_ah = accumulated_ah;                                      // Store the current accumulated Ah
  Serial.println(" Starting Conditions...");
  Serial.print("Starting Voltage (V): ");
  Serial.println(starting_voltage, 2);
}

void captureEndingConditions() {
  ending_voltage = ads.computeVolts(ads.readADC_SingleEnded(1));     // Read the cell voltage at stop
  final_ah = accumulated_ah;                                         // Store the final accumulated Ah
  last_accumulated_charge = accumulated_charge;                      // Store the final accumulated charge
  last_accumulated_ah = accumulated_ah;                              // Store the final Ah reading
  last_voltage_cell = ads.computeVolts(ads.readADC_SingleEnded(1));  // Store the last cell voltage reading
  last_current = current;                                            // Store the last current reading
  last_stop_time = millis();                                         // Capture the time when stop was pressed

  Serial.println("Ending Conditions...");
  Serial.print("Ending Voltage (V): ");
  Serial.println(ending_voltage, 2);
  Serial.print("Final Ampere-hours (Ah): ");
  Serial.println(final_ah, 2);
  Serial.print("Final Accumulated Charge (C): ");
  Serial.println(last_accumulated_charge, 6);
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
    displayVoltages1(starting_voltage, initial_ah, 1);  // Display starting conditions
    digitalWrite(LED_PIN_ch, HIGH);                     // Turn on charging LED
    digitalWrite(relay_PIN_ch, LOW);                    // Activate relay for charging
    ch2_state = true;
    startCharging = false;  // Reset the flag after charging has started
  }
   

  // Check if discharging needs to be started
  if (startDischarging) {
    // Start discharging
    captureStartingConditions();
    displayVoltages1(starting_voltage, initial_ah, 2);  // Display starting conditions for discharging
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
      displayVoltages1(starting_voltage, initial_ah, 1);
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
      displayVoltages1(starting_voltage, initial_ah, 2);
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
      displayVoltages2(ending_voltage, final_ah, 0);
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


    // Calculate SoC at the time of stopping
    //calculateSoC();

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
    Serial.println(initial_ah, 2);

    Serial.println("dh//////////");
    Serial.println(ch2_state);
    Serial.println(dh2_state);

    digitalWrite(LED_PIN_dh, HIGH);
    digitalWrite(relay_PIN_dh, LOW);
    ledState_dh = !ledState_dh;
    dh2_state = true;
  }


    if (ch2_state == true) {
        unsigned long current_time = millis();

        // Read the shunt voltage from A0 (In millivolts)
        int16_t adc0 = ads.readADC_SingleEnded(0); //shunt
        Serial.print("adc value: ");
        Serial.println(adc0);
        float voltage_shunt = ads.computeVolts(adc0);
        Serial.println("sh_voltage:");
        Serial.println(voltage_shunt,5);
        // Convert the ADC value to the corresponding shunt voltage
         float voltage_shunt = adc0 * 0.1616 / 1000.0;  // Conversion factor based on your calibration

        // Calculate the current through the shunt resistor (I = V * calibration_factor)
       
         current = voltage_shunt * 3.45;  // Current in Amps (4.32 is your calibration factor)
        // Ensure current is positive during charging (bat_state == 1)
        int bat_state = 1;  // Set charging state
        if (bat_state == 1) {
            current = abs(current);  // Make sure current is positive during charging
        }

        // Calculate the time difference since the last reading
        float time_diff = (current_time - last_time) / 1000.0;  // Time in seconds


        // Calculate Ampere-hours (Ah = I * t (in hours)) and accumulate
        accumulated_ah += current * (time_diff / 3600.0);  // Convert seconds to hours

        // Update last_time
        last_time = current_time;

        // Print the results
        Serial.print("Shunt Voltage (V): ");
        Serial.print(voltage_shunt, 6);
        Serial.print(" V, Current (A): ");
        Serial.print(current, 6);  // This should now always print positive current during charging
        Serial.print(" C, Accumulated Ampere-hours (Ah): ");
        Serial.println(accumulated_ah, 6);  // This should now always print positive Ah during charging

        // Call your display function
        displayVoltages(bat_state, current, accumulated_ah);

        // Check if charging should stop
        check_st_status();
        if (ledState_stop == true || voltage_cell >= 3.7 || turnOffSystem) {
            Serial.println("Charge threshold reached, stopping charging.");

            // Store the current values before stopping
        
            last_accumulated_ah = accumulated_ah;
            last_stop_time = last_time;

            // Capture the voltage before stopping
            starting_voltage = ads.computeVolts(ads.readADC_SingleEnded(1));
            final_ah = accumulated_ah;

            // Display final values
            Serial.print("Starting Voltage(V) : ");
            Serial.println(starting_voltage, 2);
            Serial.print("final_ah : ");
            Serial.println(final_ah, 2);

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
    unsigned long current_time = millis();

    // Read the shunt voltage from A0 (In millivolts)
    int16_t adc0 = ads.readADC_SingleEnded(0);
    Serial.print("adc value: ");
    Serial.println(adc0);
    float voltage_shunt = ads.computeVolts(adc0);
    Serial.println("sh_voltage:");
   Serial.println(voltage_shunt,6);
    // Convert the ADC value to the corresponding shunt voltage
    float voltage_shunt = adc0 * 0.1760 / 1000.0;  // Conversion factor based on your calibration

    // Calculate the current through the shunt resistor (I = V * calibration_factor)
 
    float current = voltage_shunt * 3.42;  // Current in Amps (4.48 is your calibration factor)

    // Ensure current is negative during discharging (bat_state == 2)
    int bat_state = 2;  // Set discharging state
    if (bat_state == 2) {
        current = -abs(current);  // Make sure current is negative during discharging
    }

    // Calculate the time difference since the last reading
    float time_diff = (current_time - last_time) / 1000.0;  // Time in seconds

    // Calculate charge (C = I * t) and accumulate
    //
    // Calculate Ampere-hours (Ah = I * t (in hours)) and accumulate
    accumulated_ah += current * (time_diff / 3600.0);  // Convert seconds to hours

    // Update last_time
    last_time = current_time;

    // Print the results
    Serial.print("Shunt Voltage (V): ");
    Serial.print(voltage_shunt, 6);
    Serial.print(" V, Current (A): ");
    Serial.print(current, 6);  // This should now print negative current during discharging
    Serial.print(" A, Accumulated Charge (Coulombs): ");
    Serial.print(accumulated_charge, 6);
    Serial.print(" C, Accumulated Ampere-hours (Ah): ");
    Serial.println(accumulated_ah, 6);  // This should now print negative Ah during discharging
    Serial.print("showing BQ76940000 voltages");

    displayVoltages(bat_state, current, accumulated_ah);

    delay(sample_rate);

    // Check if discharging should stop
    check_st_status();
    if (ledState_stop == true || voltage_cell <= 2.5 || turnOffSystem) {
      Serial.println("Discharge threshold reached, breaking the loop.");
      
      // Store the current values before stopping
     
      last_accumulated_ah = accumulated_ah;
      last_stop_time = current_time;

      // Capture the voltage before stopping
      ending_voltage = ads.computeVolts(ads.readADC_SingleEnded(1));
      final_ah = accumulated_ah;

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
void displayVoltages(int bat_state, float current, float accumulated_ah) {
 
  countFunctionCalls();

  adc1 = ads.readADC_SingleEnded(1);
  Serial.print("adc1 value: ");
  Serial.println(adc1);

 voltage_cell = ads.computeVolts(adc1);
  // voltage_cell = (adc1 * 0.1875)/1000;   
  Serial.println("voltage_cell..........");
  Serial.println(voltage_cell);



  // Create JSON object
  StaticJsonDocument<2000> jsonDocument1;

  char voltage_cell1[8];  // Buffer to hold the formatted string
  dtostrf(voltage_cell, 1, 5, voltage_cell1);

  char accumulated_ah_rr[8];  // Buffer to hold the formatted string
  dtostrf(accumulated_ah, 1, 5, accumulated_ah_rr);
  char current_rr[8];  // Buffer to hold the formatted string
  dtostrf(current, 1, 5, current_rr);

  jsonDocument1["cc"] = callCount;
  jsonDocument1["vcell"] = voltage_cell1;
  jsonDocument1["I"] = current_rr;
  jsonDocument1["AH"] = accumulated_ah_rr;
  jsonDocument1["bat_state"] = bat_state;

  // Serialize JSON to string
  String jsonString1;
  serializeJson(jsonDocument1, jsonString1);

  // Publish JSON string to MQTT topic
  client.publish("battery_data", jsonString1.c_str());

  Serial.println("Published sensor data:");
  Serial.println(jsonString1);
}


void displayVoltages1(float starting_voltage, float initial_ah, int bat_state) {

  char starting_voltage_str[20];
  char initial_ah_str[20];
  dtostrf(starting_voltage, 1, 6, starting_voltage_str);
  dtostrf(initial_ah, 1, 6, initial_ah_str);

  // Creating JSON document for starting voltage and initial Ah
  StaticJsonDocument<512> jsonDocument;  // Adjust buffer size
  jsonDocument["starting_voltage"] = starting_voltage_str;
  jsonDocument["initial_ah"] = initial_ah_str;
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

void displayVoltages2(float ending_voltage, float final_ah, int bat_state) {
  char ending_voltage_str[20];
  char final_ah_str[20];
  dtostrf(ending_voltage, 1, 6, ending_voltage_str);
  dtostrf(final_ah, 1, 6, final_ah_str);

  // Creating JSON document for ending voltage and final Ah
  StaticJsonDocument<512> jsonDocument;  // Adjust buffer size
  jsonDocument["ending_voltage"] = ending_voltage_str;
  jsonDocument["final_ah"] = final_ah_str;
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
  if (callCount >= 100000) {
    callCount = 0;  // Reset the count to 0
    Serial.println("Call count has been reset.");
  }

  Serial.print("Function has been called ");
  Serial.print(callCount);
  Serial.println(" times");
}

