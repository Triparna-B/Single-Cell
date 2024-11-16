////when start button of charging/discharging is pressed ,only once reading of starting_voltage , initial_ah , bat_state of a single LFP cell are sent to MQTT server
////when stop button is pressed manually , only once reading of (ending_voltage, final_ah, bat_state )of single LFP cell are sent to Mqtt server
//in between start and stop buttons continuous reading of cell_voltage, current , accumulated_charge, accumulated_Ah of LFP cell are sent to MQTT  server
///through MQTT apps/server charging or discharging can be started automatically, just by sending commands "01"=start charging ,"10"=start discharging, "00"== stop

////In this code ,command from MQTT server/Apps "00" = OFF is not working(working on this issue)

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

//float gain = 0;              //These are two internal factory set values.
//int offset = 0;              //We read them once at boot up and use them in many functions
long totalCoulombCount = 0;  //Keeps track of overall pack fuel gauage


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

void sendMessage() {
  // Create a JSON document
  StaticJsonDocument<500> jsonDocument;
  jsonDocument["battery1"] = battery1;

  String jsonString;
  serializeJson(jsonDocument, jsonString);
}

void setup() {
  Serial.begin(9600);
  setup_wifi();
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

  Wire.begin();

  delay(1000);
  ads.begin();
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
    captureStartingConditions();  // Capture starting conditions for charging
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
    ch2_state = false;
    dh2_state = false;
    turnOffSystem = false;  // Reset the turn off flag after processing
  }

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

  
while (dh2_state == true) {
    unsigned long current_time = millis();

    // Read the shunt voltage from A0 (In millivolts)
    int16_t adc0 = ads.readADC_SingleEnded(0);
    Serial.print("adc value: ");
    Serial.println(adc0);

    // Convert the ADC value to the corresponding shunt voltage
    float voltage_shunt = adc0 * 0.1760 / 1000.0;  // Conversion factor based on your calibration

    // Calculate the current through the shunt resistor (I = V * calibration_factor)
  // float current = voltage_shunt * 4.10;  // Current in Amps (4.48 is your calibration factor)
     float current = voltage_shunt * 4.48;  // Current in Amps (4.48 is your calibration factor)
    // Ensure current is negative during discharging (bat_state == 2)
    int bat_state = 2;  // Set discharging state
    if (bat_state == 2) {
        current = -abs(current);  // Make sure current is negative during discharging
    }

    // Calculate the time difference since the last reading
    float time_diff = (current_time - last_time) / 1000.0;  // Time in seconds

    // Calculate charge (C = I * t) and accumulate
    accumulated_charge += current * time_diff;

    // Calculate Ampere-hours (Ah = I * t (in hours)) and accumulate
    accumulated_ah += current * (time_diff / 3600.0);  // Convert seconds to hours

    // Update last_time
    last_time = current_time;

    // Print the results
    Serial.print("Shunt Voltage (V): ");
    Serial.print(voltage_shunt, 2);
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
    if (ledState_stop == true || voltage_cell <= 2.5) {
      Serial.println("Discharge threshold reached, breaking the loop.");
      
      // Store the current values before stopping
      last_accumulated_charge = accumulated_charge;
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
      break;  // Exit the while loop when condition is met
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

  Serial.println("voltage_cell..........");
  Serial.println(voltage_cell);

  // Create JSON object
  StaticJsonDocument<2000> jsonDocument1;

  char voltage_cell1[8];  // Buffer to hold the formatted string
  dtostrf(voltage_cell, 1, 3, voltage_cell1);

  char accumulated_ah_rr[8];  // Buffer to hold the formatted string
  dtostrf(accumulated_ah, 1, 3, accumulated_ah_rr);
  char current_rr[8];  // Buffer to hold the formatted string
  dtostrf(current, 1, 3, current_rr);

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
  // Check network status
  // if (WiFi.status() != WL_CONNECTED) {
  //   Serial.println("WiFi not connected, retrying...");
  //   WiFi.begin(ssid, password);
  //  return;  // Exit if WiFi is not connected
  // }

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

  // Check if MQTT client is connected before publishing
  // if (!client.connected()) {
  // connectToMQTT();
  // }

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

void receivedCallback(uint32_t from, String& msg) {
  // Parse the received JSON string
  StaticJsonDocument<200> jsonDocument;
  DeserializationError error = deserializeJson(jsonDocument, msg);

  if (!error) {
    // Extract the values from the JSON document
    int receivedBat1 = jsonDocument["battery1"];
    Serial.printf("startHere: Received from %u B1 = %f\n", from, receivedBat1, 2);
  } else {
    Serial.println("Error parsing JSON");
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

  // Allocate JSON document
  StaticJsonDocument<200> doc;

  // Copy payload into a temporary buffer and add null terminator
  char payloadStr[length + 1];
  memcpy(payloadStr, payload, length);
  payloadStr[length] = '\0';

  // Print the payload to debug the incoming message
  Serial.print("Payload: ");
  Serial.println(payloadStr);

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, payloadStr);
  if (error) {
    Serial.print(F(""));
    Serial.println(error.f_str());
    return;
  }

  // Check if the "command" key exists
  if (!doc.containsKey("command")) {
    Serial.println("Command key not found in JSON");
    return;
  }

  // Extract the command from the JSON document
  const char* command = doc["command"];
  if (command == nullptr) {
    Serial.println("Command value is null");
    return;
  }

  // Control logic based on the command
  if (strcmp(command, "01") == 0) {
    Serial.println("Received Start Charging command");
    startCharging = true;
    startDischarging = false;
    turnOffSystem = false;
  } else if (strcmp(command, "10") == 0) {
    Serial.println("Received Start Discharging command");
    startCharging = false;
    startDischarging = true;
    turnOffSystem = false;
  } else if (strcmp(command, "00") == 0) {
    Serial.println("Received Turn Off command");
    startCharging = false;
    startDischarging = false;
    turnOffSystem = true;
  } else {
    Serial.println("Unknown command");
  }
}



void countFunctionCalls() {
  // static int callCount = 0; // Static variable to store the count

  callCount++;  // Increment the count each time the function is called

  // Check if callCount is greater than or equal to 10000
  if (callCount >= 10000) {
    callCount = 0;  // Reset the count to 0
    Serial.println("Call count has been reset.");
  }

  Serial.print("Function has been called ");
  Serial.print(callCount);
  Serial.println(" times");
}
