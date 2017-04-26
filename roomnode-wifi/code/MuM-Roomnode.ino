
#include <Homie.h>
#include <OneWire.h>
#include <DallasTemperature.h>

/*
 * Monitoring Mum roomnode
 * WiFi/Homie version
 * 
 * Andrew Findlay
 * November 2016
 * andrew.findlay@skills-1st.co.uk
 */

#define FIRMWARE_NAME "MuM-roomnode"
#define FIRMWARE_VERSION "0.1.4"

// FUDGE
// IPAddress ip(192, 168, 1, 7);
// IPAddress netmask(255, 255, 255, 0);
// IPAddress gateway(192, 168, 1, 254); 

// OneWire bus is plugged into D7 (GPIO 13) on the WeMos D1 Mini
// Note: does not work on GPIO 16
const int ONE_WIRE_BUS = 13;

// Internal temp sensor - one-wire bus again is on D6 (GPIO 12)
const int INTERNAL_ONE_WIRE = 12;

// PIR movement detector on D5 (GPIO14)
const int PIR_PIN = 14;
// The PIR gives false alarms for about a minute after power-up so ignore it for this long after boot (ms)
const unsigned long PIR_LOCKOUT_TIME = 70000;
bool PIRLockout = true;
// PIRState records the last-seen value from the PIR so we can detect changes
int PIRState = LOW;
// seenMovement is set true in any report period where the PIR state was high at any time
bool seenMovement = false;
// Milliseconds between recent-movement reports
const unsigned long movementInterval = 60000;
unsigned long lastMovementReportTime = 0;

// Light dependent resistor is powered from D8 (GPIO 15)
// and read from A0 (AIN)
const int LDR_POWER_PIN = 15;
const int LDR_PIN = 0;
// Milliseconds between light-level reports
const unsigned long lightInterval = 60000;
unsigned long lastLightReportTime = 0;
// Reciprocal of LDR raw reading is multiplied by this value to get approximate Lux value
const int LDRMultiplier = 5000;
// The useful range of LDR readings - about 3Lux to 1000Lux
const int LDR_SATURATE_LEVEL = 5;
const int LDR_DARK_LEVEL = 800;

// Milliseconds between temperature reports
const unsigned long temperatureInterval = 60000;

// the maximum number of external temp sensors supported
#define MAX_THERMOMETERS 10
// How many temp sensors are actually connected?
unsigned int numInternalThermometers = 0;
unsigned int numThermometers = 0;
// array to hold device address
String thermoNames[MAX_THERMOMETERS];
// Is the temp sensor converting?
bool thermoConverting = false;
// When do we expect temp reading to be ready (millis)?
unsigned long thermoReadyAt = 0;

// The millis() time when we last sent a reading
unsigned long lastTemperatureSent = 0;


HomieNode temperatureNode("temperature", "temperature");
HomieNode lightNode("light", "light");
HomieNode movementNode("movement", "movement");

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Another one for the internal temp sensor
OneWire internalOneWire(INTERNAL_ONE_WIRE);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
DallasTemperature internalSensors(&internalOneWire);

// This callback does things at the end of setup that cannot be done inside setup() itself
void setupHandler() {
  temperatureNode.setProperty("unit").send("C");
  temperatureNode.setProperty("number").send( String(numThermometers) );
}

void loopHandler() {
  unsigned long now = millis();

  // Start temperature conversion
  if (!thermoConverting && (now - lastTemperatureSent >= temperatureInterval || lastTemperatureSent == 0)) {
      // We need to request a conversion
      sensors.requestTemperatures();
      internalSensors.requestTemperatures();
      thermoConverting = true;
      // DS18B20 does 12-bit conversion in 750ms so give it 1000ms
      thermoReadyAt = now + 1000UL;
      Serial.println("Thermo converting");

  }

  // Read temperature
  if (thermoConverting && (now > thermoReadyAt)) {
    thermoConverting = false;
    Serial.println("Reading temperatures");
    
    // Read the temperature from the internal sensor
    float internalTemperature = internalSensors.getTempCByIndex(0);
    Serial.println("Internal temp: " + String(internalTemperature) );
    
    // WiFi.forceSleepWake();
    if ((numInternalThermometers != 1) || (internalTemperature < -50.0)) {
      temperatureNode.setProperty("internal").send( "none" );
    } else {
      temperatureNode.setProperty("internal").send( String(internalTemperature) );
    }
    
    // Read any external sensors
    int count;
    for (count = 0; count < numThermometers; count++) {
      float temperature = sensors.getTempCByIndex(count);
      Serial.println("External temp " + thermoNames[count] + ": " + String(temperature) );
    
      if (temperature < -50.0) {
        temperatureNode.setProperty(thermoNames[count].c_str()).send( "none" );
      } else {
        temperatureNode.setProperty(thermoNames[count].c_str()).send( String(temperature) );
      }
    }

    /* Test code for sleep mode
    Serial.println("Preparing Homie for sleep");
    Homie.prepareToSleep();
    Serial.println("Waiting 1s");
    delay(1000);
    Serial.println("Sleeping WiFi for 5000000us");
    WiFi.forceSleepBegin(5000000);
    Serial.println("Delay for 20s");
    delay(20000);
    Serial.println("Waking WiFi");
    WiFi.forceSleepWake();
    */

    // Record the time
    lastTemperatureSent = now;
  }

  // Keep PIR locked out until it has had time to settle
  if (PIRLockout && (now > PIR_LOCKOUT_TIME)) {
    PIRLockout = false;
    movementNode.setProperty("enabled").send("true");
  }

  // Watch for new movement
  if (!PIRLockout && !PIRState && (digitalRead(PIR_PIN) == HIGH)) {
    PIRState = true;
    seenMovement = true;
    Serial.println("Movement...");
    // Report movement immediately
    // WiFi.force();
    movementNode.setProperty("moving").send("true");
    
    // delay(100);
    // WiFi.forceSleepBegin();
  }

  // Watch for end of movement
  if (!PIRLockout && PIRState && (digitalRead(PIR_PIN) == LOW)) {
    PIRState = false;
    Serial.println("Movement ends");
    // Report end of movement immediately
    // WiFi.forceSleepWake();
    movementNode.setProperty("moving").send("false");
    
    // delay(100);
    // WiFi.forceSleepBegin();
  }

  // Regular movement reports
  if (!PIRLockout && ((now - lastMovementReportTime) > movementInterval)) {
    // WiFi.forceSleepWake();
    movementNode.setProperty("recent").send( seenMovement ? "true" : "false" );
    lastMovementReportTime = now;
    
    seenMovement = (digitalRead(PIR_PIN) == HIGH);
    
    // delay(100);
    // WiFi.forceSleepBegin();
  }

 
  // Regular light-level reports
  if ((now - lastLightReportTime) > lightInterval) {
    // Power up the PIR
    digitalWrite(LDR_POWER_PIN, HIGH);
    delay(2);
    int lightLevel = analogRead(LDR_PIN);
    Serial.print("Raw light level ");
    Serial.println(lightLevel);
    
    // WiFi.forceSleepWake();
    
    if (lightLevel > LDR_DARK_LEVEL) {
      lightNode.setProperty("level").send( "0" );
    }
    else if (lightLevel < LDR_SATURATE_LEVEL) {
      // LDR has saturated so clip to 1000 Lux
      lightNode.setProperty("level").send( "1000" );
    }
    else {
      // Make an estimate of the light level in Lux
      int lux = LDRMultiplier / lightLevel;
      Serial.print("Approx Lux: ");
      Serial.println( String(lux) );
      lightNode.setProperty("level").send( String(lux) );
    }

    digitalWrite(LDR_POWER_PIN, LOW);
    lastLightReportTime = now;
    
    // delay(100);
    // WiFi.forceSleepBegin();
  }
}


// function to convert a OneWire device address to a string
String addressToString(DeviceAddress deviceAddress)
{
  String result = "";
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) result += "0";
    result += String(deviceAddress[i], HEX);
  }
  return result;
}

// setup
void setup() {
  Serial.begin(115200); Serial << endl << endl;
  Homie_setFirmware(FIRMWARE_NAME, FIRMWARE_VERSION);
  Homie.setSetupFunction(setupHandler);
  Homie.setLoopFunction(loopHandler);
  Homie.onEvent(onHomieEvent);

  Serial.println("Monitoring Mum project");
  Serial.println(FIRMWARE_NAME);
  Serial.println(FIRMWARE_VERSION);
 
  temperatureNode.advertise("unit");
  temperatureNode.advertise("number");
  
  // Look for internal temperature sensors
  internalSensors.begin();
  numInternalThermometers = internalSensors.getDeviceCount();

  // Look for external temperature sensors
  sensors.begin();
  numThermometers = sensors.getDeviceCount();
  
  // Record the addresses of the external sensors
  int count = 0;
  DeviceAddress dev;
  while ((count < MAX_THERMOMETERS) && sensors.getAddress(dev, count)) {
    // Set sensor resolution to 12 bits
    // sensors.setResolution(dev, 12);
    // Get string version of name and advertise it
    String thermoName = addressToString(dev);
    Serial.print("Thermometer ");
    Serial.println(thermoName);
    thermoNames[count] = thermoName;
    temperatureNode.advertise(thermoName.c_str());
    count++;
  }
  // Run in async mode
  internalSensors.setWaitForConversion(false);
  sensors.setWaitForConversion(false);

  // Sanity check
  if (numThermometers != count) {
    Serial.println("Number of temperature sensors does not match the list of addresses found");
  }

  // Reduce power consumption
  // Homie.disableLedFeedback();
  Homie.setup();

  pinMode(PIR_PIN, INPUT);
  lastMovementReportTime = lastLightReportTime = millis();

  pinMode(LDR_POWER_PIN, OUTPUT);
  digitalWrite(LDR_POWER_PIN, LOW);

  // pinMode(INTERNAL_ONE_WIRE, INPUT_PULLUP);


}

void onHomieEvent(HomieEvent event) {
  switch(event.type) {
    case HomieEventType::WIFI_CONNECTED:
      // FUDGE to fix IP:
      // WiFi.config(ip, gateway, netmask);

      Serial.print("Network config: ");
      Serial.print(WiFi.localIP()); Serial.print(" ");
      Serial.print(WiFi.subnetMask()); Serial.print(" ");
      Serial.println(WiFi.gatewayIP());

    break;
  }
}

void loop() {
  Homie.loop();

}
