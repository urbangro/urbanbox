/***********************************
 * Arudino Mega + 8266 functions
 ***********************************
  Sensors
    Air Temp        https://randomnerdtutorials.com/complete-guide-for-dht11dht22-humidity-and-temperature-sensor-with-arduino/
    Air Humidity
    Air PPM         https://wiki.keyestudio.com/KS0457_keyestudio_CCS811_Carbon_Dioxide_Temperature_Air_Quality_Sensor
    Water Level
    Water Temp      
    Water PPM

    Water pH
  Switch on/off controlled by sensor
    *Air cooler
    *Air humidifier
    *Air dehumidifier
    *Air CO2
    *Water cooler
    *NutePump Grow
    NutePump Bloom
    NutePump Micro
    NutePump CalMag
    NutePump pH+
    NutePump pH-
  Switch on/off for other units
    Air circulation
    *Carbon Filter
    Water Air pump
    LED main
    LED side
    Camera
    Center Control
  Output
    LED
    BackEnd  
************************************/


/***  Include libraries  ***/
#include "DHT.h"    // air humid/temp
#include <CCS811.h> // air ppm

#include <OneWire.h>    // water temp
#include <DallasTemperature.h>

// I2C Commnunication
#include <Wire.h>

/***  Define pins ***/
/** Input  **/
#define PIN_AIR_HUMI_TEMP 44

#define PIN_WATER_LEVEL 48
#define PIN_WATER_TEMP 45 // Data wire is plugged into port 2 on the Arduino
#define TEMPERATURE_PRECISION 9

#define PIN_WATER_TDS A8
#define VREF 5.0 // analog reference voltage(Volt) of the ADC
#define SCOUNT 20 // sum of sample point
int analogBuffer[SCOUNT]; // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;
long averageVoltage = 0,waterTdsValue = 0,waterTemp = 25;

/** Switch Control Pins  **/
#define AIR_HUMIDIFIER_SWITCH     10
#define AIR_DEHUMIDIFIER_SWITCH   11
#define AIR_COOLER_SWITCH         12
#define AIR_HEATER_SWITCH         13
#define AIR_CO2GENERATOR_SWITCH   14

#define WATER_AIRPUMP_SWITCH      15
#define WATER_COOLER_SWITCH    16
#define WATER_HEATER_SWITCH    17


/*** Define Sensors ***/
#define DHTTYPE DHT11     // AIR humid/temp
DHT dht(PIN_AIR_HUMI_TEMP, DHTTYPE); // Initialize DHT sensor for normal 16mhz Arduino

CCS811 airPPMSensor; // AIR PPM CCS811 airPPMSensor(&Wire, /*IIC_ADDRESS=*/0x5A);

OneWire oneWire(PIN_WATER_TEMP);  // Water temp: Setup a oneWire instance to communicate with any OneWire devices(not just Maxim/Dallas temperature ICs)
DallasTemperature waterTempSensors(&oneWire);  // Pass our oneWire reference to Dallas Temperature.
int waterTempSensorCount; // Number of temperature devices found
DeviceAddress tempDeviceAddress;  // We'll use this variable to store a found device DeviceAddress



/**  Sensor status  **/
boolean isAirHumiditySensorReady;
// boolean isAirTemperatureSensorReady;
boolean isAirPPMSensorReady;
boolean isWaterTemperatureSensorReady;
boolean isWaterPPMSensorReady;
boolean isWaterPHSensorReady;
boolean isWaterLevelSensorReady;

/** Define Environment Auto Control ON / OFF **/
boolean airTemperatureAuto = false;
boolean airHumidityAuto = false;
boolean airCO2PPMAuto = false;
boolean waterTemperatureAuto = false;
boolean waterPPMAuto = false;
boolean waterPHAuto = false;

/** Define Environment Auto Control Default Idea Range **/
long airTemperatureMax = 33.0;  // Degree C
long airTemperatureMin = 18.0;
long airHumidityMax = 85.0;     // Relative Humidity Percentage
long airHumidityMin = 18.0;
long airCO2PPMMax = 2000;             // PPM
long airCO2PPMMin = 500;

long waterPPMMax = 1800;
long waterPPMMin = 0;
long waterTemperatureMax = 25.0;  // Degree C
long waterTemperatureMin = 14.0;
long waterPHMax = 8;
long waterPHMin = 5;

/** Track Current Environment Parameters **/
long airHumidity = 0;
long airTemperatureC = 0;
long airTemperatureF = 0;
long airHeatIndex = 0;
long airCO2PPM = 0;
long airTVOC = 0;

long waterPPM = 0;
long waterPH = 0;
long waterLevel = 0; // Low is 0
long waterTemperature = 0;


/** Device switch ON/OFF **/
boolean isAirCoolerOn = false;
boolean isAirHeaterOn = false;
boolean isAirDryerOn = false;
boolean isAirHumidifierOn = false;
boolean isAirCO2GeneratorOn = false;
boolean isWaterCoolerOn = false;
boolean isWaterHeaterOn = false;


/***  Init environments and connect sensors  ***/
void setup() {
  // TODO: keep track of login credential?
  // TODO: Sync time when power on

  Serial.begin(9600);
  delay(500);

  /* Init Sensor */
  dht.begin();
  initAirPPMSensor();  
  initWaterTempSensors();    // now just one sensor
  // initWaterPPMSensor(); check if it's reading
  pinMode(PIN_WATER_TDS, INPUT);

  /* Init environment switch */
  pinMode(AIR_HUMIDIFIER_SWITCH, OUTPUT);
  digitalWrite(AIR_HUMIDIFIER_SWITCH, LOW); 
  pinMode(AIR_DEHUMIDIFIER_SWITCH, OUTPUT);
  digitalWrite(AIR_DEHUMIDIFIER_SWITCH, LOW); 
  pinMode(AIR_COOLER_SWITCH, OUTPUT);
  digitalWrite(AIR_COOLER_SWITCH, LOW); 
  pinMode(AIR_HEATER_SWITCH, OUTPUT);
  digitalWrite(AIR_HEATER_SWITCH, LOW); 
  pinMode(AIR_CO2GENERATOR_SWITCH, OUTPUT);
  digitalWrite(AIR_CO2GENERATOR_SWITCH, LOW); 

  pinMode(WATER_AIRPUMP_SWITCH, OUTPUT);
  digitalWrite(WATER_AIRPUMP_SWITCH, LOW); 
  pinMode(WATER_COOLER_SWITCH, OUTPUT);
  digitalWrite(WATER_COOLER_SWITCH, LOW); 
  pinMode(WATER_HEATER_SWITCH, OUTPUT);
  digitalWrite(WATER_HEATER_SWITCH, LOW); 

  // send data to ESP8266 through I2C
  Wire.begin(8);                /* join i2c bus with address 8 */
  Wire.onReceive(receiveEvent); /* register receive event */
  Wire.onRequest(requestEvent); /* register request event */

  delay(500);
}

void loop() {
    static unsigned long updateEnvironmentParamTimePoint = millis();
    static unsigned long waterQualitySamplingTimePoint = millis();

    if(millis()-waterQualitySamplingTimePoint > 100U) {
        doWaterTdsSampling();
        waterQualitySamplingTimePoint = millis();
    }

    if(updateEnvironmentParamTimePoint > 2000U) {
        readEnvironmentSensorsData();
        delay(100); 
        updateEnvironment();

        updateEnvironmentParamTimePoint = millis();
    }

    /* 
      if(sensorErrorCode > 0) {
        sendErrorCodeNotification();
      }

    */
}

/*** Air PPM ***/
void initAirPPMSensor() {

    /*Wait for the chip to be initialized completely, and then exit*/
    static int tryTimeCount = 0;

    while (!isAirPPMSensorReady && tryTimeCount < 10) {
        if(airPPMSensor.begin() != 0) {
            tryTimeCount ++;
        } else {
            isAirPPMSensorReady = true;
        }
    } 

    
    if(isAirPPMSensorReady) {
        /**
         * @brief Set measurement cycle
         * @param cycle:in typedef enum{
         *                  eClosed,      //Idle (Measurements are disabled in this mode)
         *                  eCycle_1s,    //Constant power mode, IAQ measurement every second
         *                  eCycle_10s,   //Pulse heating mode IAQ measurement every 10 seconds
         *                  eCycle_60s,   //Low power pulse heating mode IAQ measurement every 60 seconds
         *                  eCycle_250ms  //Constant power mode, sensor measurement every 250ms
         *                  }eCycle_t;
         */
        airPPMSensor.setMeasCycle(airPPMSensor.eCycle_250ms);
        // Serial.println("Init Air PPM Sensor completed");
    }
}

/*** Air HumidityTemperature ***/
void updateAirHumidityTemperature() {
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    airHumidity = dht.readHumidity();
    airTemperatureC = dht.readTemperature(); // Read temperature as Celsius
    airTemperatureF = dht.readTemperature(true); // Read temperature as Fahrenheit
    
    if (isnan(airHumidity) || isnan(airTemperatureC) || isnan(airTemperatureF)) {
      // Serial.println("Failed to read from DHT sensor!");
      // isAirHumiditySensorReady = false;
      return;
    }
    isAirHumiditySensorReady = true;
    airHeatIndex = dht.computeHeatIndex(airTemperatureF, airHumidity);
}

/*** Air CO2 PPM ***/
void updateAirPPM() {
    
    if(airPPMSensor.checkDataReady() == true){
        airCO2PPM = airPPMSensor.getCO2PPM();    //Unit: ppm  
        airTVOC = airPPMSensor.getTVOCPPB();  //Unit: ppb
    } else {
        isAirPPMSensorReady = false;
        Serial.println("Data is not ready!");
    }
    /*!
     * @brief Set baseline
     * @param get from getBaseline.ino
     */
    airPPMSensor.writeBaseLine(0x847B);
}

/*** Water Level ***/
void updateWaterLevel() {
    waterLevel = digitalRead(PIN_WATER_LEVEL);
}

///*** Water Temp ***/
void initWaterTempSensors () {
   // Start up the library
   waterTempSensors.begin();
   
   // Grab a count of devices on the wire
   waterTempSensorCount = waterTempSensors.getDeviceCount();
   
   //waterTempSensors.isParasitePowerMode()

   // Loop through each device, print out address
   for(int i=0;i<waterTempSensorCount; i++)
   {
     // Search the wire for address
     if(waterTempSensors.getAddress(tempDeviceAddress, i))    
    {
      // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
      waterTempSensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);
      isWaterTemperatureSensorReady = true;
      // Serial.print("Resolution actually set to: ");
      // Serial.print(waterTempSensors.getResolution(tempDeviceAddress), DEC);
      // Serial.println();
    }else{
      // isWaterTemperatureSensorReady = false;
      // Serial.print("Could not detect address. Check power and cabling");
    }
   } 
}

void updateWaterTemp () {
   waterTempSensors.requestTemperatures(); 
  
   for(int i=0;i < waterTempSensorCount; i++)
   {
      if(waterTempSensors.getAddress(tempDeviceAddress, i))
      {
        waterTemp = waterTempSensors.getTempC(tempDeviceAddress);
        // Get Degree F from Degree C: DallasTemperature::toFahrenheit(waterTemp)
        // Read again from sensor: waterTempSensors.getTempF(tempDeviceAddress);
      }
    
   }
}

/*** Water TDS ***/
void doWaterTdsSampling() {
    analogBuffer[analogBufferIndex] = analogRead(PIN_WATER_TDS);
    analogBufferIndex++;

    if(analogBufferIndex == SCOUNT)
        analogBufferIndex = 0;
}

void updateWaterTds() {
    static long compensationCoefficient = 0;
    static long compensationVolatge = 0; 
        
    for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
        analogBufferTemp[copyIndex] = analogBuffer[copyIndex];

    averageVoltage = getMedianNum( analogBufferTemp, SCOUNT ) * ( long ) VREF / 1024.0; 
    // read the analog value more stable by the median filtering algorithm, and convert to voltage value
    compensationCoefficient = 1.0 + 0.02 * (waterTemp - 25.3); 
    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    compensationVolatge = averageVoltage / compensationCoefficient; 
    //temperature compensation
    
    waterPPM = 
        ( 133.42 * compensationVolatge * compensationVolatge * compensationVolatge 
        - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge ) * 0.5; 
        //convert voltage value to tds value(UNIT: PPM)    
}

int getMedianNum(int bArray[], int iFilterLen)
{
    int bTab[iFilterLen];
    for (byte i = 0; i<iFilterLen; i++)
    bTab[i] = bArray[i];
    int i, j, bTemp;
    for (j = 0; j < iFilterLen - 1; j++)
    {
        for (i = 0; i < iFilterLen - j - 1; i++)
        {
          if (bTab[i] > bTab[i + 1])
          {
              bTemp = bTab[i];
              bTab[i] = bTab[i + 1];
              bTab[i + 1] = bTemp;
          }
        }
    }
    if ((iFilterLen & 1) > 0)
        bTemp = bTab[(iFilterLen - 1) / 2];
    else
        bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
    return bTemp;
}


// https://forum.arduino.cc/index.php?topic=165243.0
/***  I2C commnication  ****/
// Controller ask Mega to provide environment information

  // To reduce transmittion time we send 8 sensor's data in one round
  // First 6 sensor data use 4 bytes; last 2 use 2 bytes
  // If we need to add more sensors, we'll need to send data in two round.
  // Or use 2 bytes for each sensor, we can use two bytes for each data.
const int I2C_TRANSMISSION_SIZE = 32;
unsigned char sensorDataToServer[I2C_TRANSMISSION_SIZE];
void requestEvent() {
  // I2C transfer buffer size is 32 bytes so we can only transfer 32 bytes at a time.

  copyFourByteSensorDataToArray(0, airHumidity);
  copyFourByteSensorDataToArray(1, airTemperatureC);
  copyFourByteSensorDataToArray(2, airHeatIndex);
  copyFourByteSensorDataToArray(3, airCO2PPM);
  copyFourByteSensorDataToArray(4, airTVOC);
  copyFourByteSensorDataToArray(5, waterPH);
  copyFourByteSensorDataToArray(6, waterTemp);
  copyTwoByteSensorDataToArray(7, waterPPM);  // use 2 bytes
  copyTwoByteSensorDataToArray(8, waterLevel);   // use 2 bytes

  // copyLongDataToCharArray(sensorDataToServer, 9, waterPH);
  // copyLongDataToCharArray(sensorDataToServer, 7, averageVoltage);// ?


    // char dataChar[I2C_TRANSMISSION_SIZE];
    // for(int i =0; i<I2C_TRANSMISSION_SIZE; i++) {
    //     dataChar[i] = (i >= sensorData.length()) ? '_' : sensorData[i];
    // }
    
    // strncpy(dataChar, sensorData.c_str(), sensorData.length());
    Wire.write(sensorDataToServer, I2C_TRANSMISSION_SIZE);
    // sensorIndex = (sensorIndex + 1) % 10;
}

void copyFourByteSensorDataToArray(int dataIndex, long sensorData) {
    sensorDataToServer[dataIndex * 4 + 0] = (sensorData >> 24) & 0xFF;
    sensorDataToServer[dataIndex * 4 + 1] = (sensorData >> 16) & 0xFF;
    sensorDataToServer[dataIndex * 4 + 2] = (sensorData >> 8 ) & 0xFF;
    sensorDataToServer[dataIndex * 4 + 3] = sensorData & 0xFF;
}

void copyTwoByteSensorDataToArray(int dataIndex, long sensorData) {
    sensorDataToServer[dataIndex * 4 + 0] = (sensorData >> 8) & 0xFF;
    sensorDataToServer[dataIndex * 4 + 1] = sensorData & 0xFF;
}

// Controller send request evnrironment update request
void receiveEvent() {

    String dataFromController = "";
    while (0 <Wire.available()) {
        dataFromController += Wire.read();      /* receive byte as a character */
    }
    parseUpdateRequest(dataFromController);
    delay(200);
}

void parseUpdateRequest(String params) {

    // AIR temp transmitted in degree C, not Fahrenheit
    if( params.indexOf("ATEMA") >=0 ) waterTemperatureAuto = getParamBoolean(params, "ATEMA"); // Auto Air Temp
    if( params.indexOf("ATEMX") >=0 ) airTemperatureMax = getParamLong(params, "ATEMX"); // Air Temp Max
    if( params.indexOf("ATEMN") >=0 ) airTemperatureMin = getParamLong(params, "ATEMN"); // Air Temp Min

    if( params.indexOf("AHUMA") >=0 ) airHumidityAuto = getParamBoolean(params, "AHUMA"); // Auto Air Humidity
    if( params.indexOf("AHUMX") >=0 ) airHumidityMax = getParamLong(params, "AHUMX"); // Air Humi Max
    if( params.indexOf("AHUMN") >=0 ) airHumidityMin = getParamLong(params, "AHUMN"); // Air Humi Min

    if( params.indexOf("ACO2A") >=0 ) airCO2PPMAuto= getParamBoolean(params, "ACO2A"); // Auto Air CO2
    if( params.indexOf("ACO2X") >=0 ) airCO2PPMMax = getParamLong(params, "ACO2X"); // Air CO2 Max 
    if( params.indexOf("ACO2N") >=0 ) airCO2PPMMin = getParamLong(params, "ACO2N"); // Air CO2 Min

    if( params.indexOf("WTEMA") >=0 ) waterTemperatureAuto = getParamBoolean(params, "WTEMA"); // Auto Water Temp
    if( params.indexOf("WTEMX") >=0 ) waterTemperatureMax = getParamLong(params, "WTEMX"); // Water Temp Max
    if( params.indexOf("WTEMN") >=0 ) waterTemperatureMin = getParamLong(params, "WTEMN"); // Water Temp Min

    if( params.indexOf("WPPMA") >=0 ) waterPPMAuto = getParamBoolean(params, "WPPMA"); // Auto Water PPM
    if( params.indexOf("WPPMX") >=0 ) waterPPMMax = getParamLong(params, "WPPMX"); // Water PPM Max
    if( params.indexOf("WPPMN") >=0 ) waterPPMMin = getParamLong(params, "WPPMN"); // Water PPM Min

    if( params.indexOf("WPHVA") >=0 ) waterPHAuto = getParamBoolean(params, "WPHVA"); // Auto Water PH Value
    if( params.indexOf("WPHVX") >=0 ) waterPHMax = getParamLong(params, "WPHVX"); // Water pH Max
    if( params.indexOf("WPHVN") >=0 ) waterPHMin = getParamLong(params, "WPHVN"); // Water pH Min

    readEnvironmentSensorsData();
    updateEnvironment();
}

boolean getParamBoolean(String paramStr, String param) {
    int start = paramStr.indexOf(param) + param.length() + 1; // + 1 for string separater '-'
    int end = paramStr.indexOf("-", start);
    return paramStr.substring(start, end).equals("true");
}

long getParamLong(String paramStr, String param) { 
    int start = paramStr.indexOf(param) + param.length() + 1;
    int end = paramStr.indexOf("_", start);
    return paramStr.substring(start, end).toInt();

}

void readEnvironmentSensorsData() {
    updateAirHumidityTemperature();
    updateAirPPM();
    updateWaterLevel();
    updateWaterTds();
    //updateAirPH();
}

void turnOn(int environmentControlSwitch) {
    digitalWrite(environmentControlSwitch, HIGH);
}

void turnOff(int environmentControlSwitch) {
    digitalWrite(environmentControlSwitch, LOW);
}

void updateEnvironment() {
  // Air
  if(airTemperatureAuto)  {
    if(airTemperatureF > airTemperatureMax) {
        turnOn(AIR_COOLER_SWITCH);
        turnOff(AIR_HEATER_SWITCH);
    } 
    else if ( airTemperatureF < airTemperatureMin ) {
        turnOn(AIR_HEATER_SWITCH);
        turnOff(AIR_COOLER_SWITCH);
    }
  }

  if(airHumidityAuto)  {
    if(airHumidity > airHumidityMax) { 
        turnOn(AIR_DEHUMIDIFIER_SWITCH);
        turnOff(AIR_HUMIDIFIER_SWITCH);
    } 
    else if ( airHumidity < airHumidityMin ) {
        turnOn(AIR_HUMIDIFIER_SWITCH);
        turnOff(AIR_DEHUMIDIFIER_SWITCH);
    }
  }

  if(airCO2PPMAuto)  {
      if(airCO2PPM > airCO2PPMMax) {
          turnOff(AIR_CO2GENERATOR_SWITCH);
      } 
      else if ( airCO2PPM < airCO2PPMMin ) {
          turnOn(AIR_CO2GENERATOR_SWITCH);
      }
  }

  // Water / nutrition resovior
  if(waterTemperatureAuto) {
      if(waterTemperature > waterTemperatureMax) {
          turnOn(WATER_COOLER_SWITCH);
          turnOff(WATER_HEATER_SWITCH);
      } 
      else if(waterTemperature < waterTemperatureMin) {
          turnOn(WATER_HEATER_SWITCH);
          turnOff(WATER_COOLER_SWITCH);
      }
  }
  
  if(waterPPMAuto) {
      if(waterPPM > waterPPMMax || waterPPM < waterPPMMin) {
          adjustNutritionWater();
      } 
  }

  if(waterPHAuto) {
      if(waterPH > waterPHMax || waterPH < waterPHMin) {
          adjustNutritionWater();
      } 
  }
}

/***  Environment Control  ***/
void adjustNutritionWater() {
  // Send water change warning
  if(waterPPM > waterPPMMax) {
      // addWaterToSafeZone
  } 
  else if (waterPPM < waterPPMMin) {
      // addNute
  }

  if(waterPH > waterPH) {
      // addPHMinus
  } else if(waterPH < waterPH) {
      // addPHPlus
  }
}