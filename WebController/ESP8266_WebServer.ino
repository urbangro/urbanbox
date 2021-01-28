#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>

#include <Wire.h>


#ifndef STASSID
#define STASSID "PuTao"
#define STAPSK  "6262024229"
#endif

const char *ssid = STASSID;
const char *password = STAPSK;

ESP8266WebServer server(2020);

String SENSOR_VALUES = "";
boolean WIFI_CONNECTED = false;

const int I2C_TRANSMISSION_SIZE = 32;  // 32 Byte is I2C transfer buffer size so we can only transfer 32 bytes at a time. 
const int sensorNumber = 10;

const int PIN_AIR_PUMP = 6;
/*
Setup
    . connect to wifi
    . connect to sensors board
    
Loop
    . read data from sensors board
    . output to serial port
    . receive request
      . App request environment values
      . App request update values
    . send request
      . Send environment values to App(Secheduled/Reply)
      
*/

void setup(void) {

  Serial.begin(9600);
  
  Serial.println("Master Wire ESP8266 init");
  
  if(tryConnectWifi()) {
      trySetUpServer();
  }
}

boolean tryConnectWifi() {
    Wire.begin(D1, D2); /* join i2c bus with SDA=D1 and SCL=D2 of NodeMCU */
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
  
    // Wait for connection
    int maxTry = 20;
    int alreadyTried = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        if(maxTry == alreadyTried) {
          return false;
        }
        alreadyTried++;
    }

    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    return true;
}

boolean trySetUpServer() {

  if (MDNS.begin("esp8266")) {
    Serial.println("MDNS responder started");
  }

  serverStartRouting();
  Serial.println("HTTP server started");
}

void serverStartRouting() {
    server.on("/", handleRoot);
    server.on("/test.svg", drawGraph);
    
    server.on("/env", []() {
      server.send(200, "text/plain", SENSOR_VALUES);
    });
    server.on("/updateRoomParam", sendDataToSensorBoard);
    
    server.onNotFound(handleNotFound);
    server.begin();  
}

static unsigned long lastReadTime = millis();
void loop(void) {
    server.handleClient();
    MDNS.update();
  
    if(millis()-lastReadTime > 5000) {
       readDataFromSensorBoard();
       lastReadTime = millis();
    }
}



void handleRoot() {
    
    Serial.println("Start heandleRoot");
    
    char temp[400];
    int sec = millis() / 1000;
    int min = sec / 60;
    int hr = min / 60;
  
    snprintf(temp, 400,
             "<html>\
                <head>\
                  <meta http-equiv='refresh' content='5'/>\
                  <title>ESP8266 Demo</title>\
                  <style>\
                    body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\
                  </style>\
                </head>\
                <body>\
                  <h1>Hello from ESP8266!</h1>\
                  <p>Data from Sensors: </p>\
                  <p>Uptime: %02d:%02d:%02d</p>\
                  <img src=\"/test.svg\" />\
                </body>\
              </html>",
             hr, min % 60, sec % 60
            );
    server.send(200, "text/html", temp);
    Serial.println("End heandleRoot");
}

void handleNotFound() {
    Serial.println("Start handleNotFound");
    String message = "File Not Found\n\n";
    message += "URI: ";
    message += server.uri();
    message += "\nMethod: ";
    message += (server.method() == HTTP_GET) ? "GET" : "POST";
    message += "\nArguments: ";
    message += server.args();
    message += "\n";
    message += "Arg 0: ";
    message += server.arg(0);
    message += "\n";

    for (uint8_t i = 0; i < server.args(); i++) {
      message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
    }
  
    server.send(404, "text/plain", message);

    if(server.argName(0).equals("on")) {
      digitalWrite(PIN_AIR_PUMP, HIGH);
      Serial.println("Turn airpump high ");
    }
    if(server.argName(0).equals("off")) {
      digitalWrite(PIN_AIR_PUMP, LOW);
      Serial.println("Turn airpump low ");
    }
    
    Serial.println("End handleNotFound");
}

void drawGraph() {
    String out;
    out.reserve(2600);
    char temp[70];
    out += "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" width=\"400\" height=\"150\">\n";
    out += "<rect width=\"400\" height=\"150\" fill=\"rgb(250, 230, 210)\" stroke-width=\"1\" stroke=\"rgb(0, 0, 0)\" />\n";
    out += "<g stroke=\"black\">\n";
    int y = rand() % 130;
    for (int x = 10; x < 390; x += 10) {
      int y2 = rand() % 130;
      sprintf(temp, "<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" stroke-width=\"1\" />\n", x, 140 - y, x + 10, 140 - y2);
      out += temp;
      y = y2;
    }
    out += "</g>\n</svg>\n";
  
    server.send(200, "image/svg+xml", out);
}

/* Read data from sensor board and forward to server */
void readDataFromSensorBoard() {
    // Send GET request to backend controller

    char buf[I2C_TRANSMISSION_SIZE];
    int index = 0;
    String sensorData = "";
    
    Serial.println(" --- Read data from sensor board --- ");
    for(int sensorIndex = 0; sensorIndex < sensorNumber; sensorIndex++) {
        Wire.requestFrom(8, I2C_TRANSMISSION_SIZE); 
        
        while(Wire.available()){
            buf[sensorIndex] = Wire.read();
        }
    } 
    
    SENSOR_VALUES = sensorData;
    Serial.print("From sensor board : ");
    Serial.println(buf);
    Serial.println(" --- read end --- ");
}

/* Forward server's update request to Mega */
void sendDataToSensorBoard() {
    static String envUpdateRequest;

    if (server.method() != HTTP_POST) {
        server.send(405, "text/plain", "Method Not Allowed");
    } 
    if (!server.hasArg("envParam")) {
        server.send(400, "text/plain", "Wrong Request Parameters");
    }
    envUpdateRequest = server.arg("envParam");

    // ESP8266 only do forwarding
    // TODO: check string size <= I2C_TRANSMISSION_SIZE & checksum
    /* 
    if(isValid(envUpdateRequest)) {
      send...
      double check if necessary??
    } 
    */
    Wire.beginTransmission(44); // transmit to device #44 (0x2c)
                                // device address is specified in datasheet
    
    Wire.write(SENSOR_VALUES.c_str());  // sends value byte  
    Wire.endTransmission();
}
