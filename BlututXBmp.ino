#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif// Bluetooth Serial object
BluetoothSerial SerialBT;
//BMP280 object;
Adafruit_BMP280 bmp;//GPIO of LED
const int ledPin = 23;// Handle received and sent messages
String message = "";
char incomingChar;
String temperatureString = "";// Timer: auxiliar variables
unsigned long previousMillis = 0;    // Stores last time temperature was published
const long interval = 10000;         // interval at which to publish sensor readings
void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  Serial.begin(115200);
  // Bluetooth device name
  SerialBT.begin("ESP32");
  Serial.println("The device started, now you can pair it with bluetooth!");
  
  if (!bmp.begin(0x76)) {
  Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
  while (1);
  }
  delay(2000);
}void loop() {
 unsigned long currentMillis = millis();
  // Send temperature readings
  if (currentMillis - previousMillis >= interval){
    previousMillis = currentMillis;
    temperatureString = bmp.readTemperature(); 
    SerialBT.println(temperatureString); 
  }
  // Read received messages (LED control command)
  if (SerialBT.available()){
    char incomingChar = SerialBT.read();
    if (incomingChar != '\n'){
      message += String(incomingChar);
    }
    else{
      message = "";
    }
    Serial.write(incomingChar);  
  }
  // Check received message and control output accordingly
  if (message =="led_on"){
    digitalWrite(ledPin, HIGH);
  }
  else if (message =="led_off"){
    digitalWrite(ledPin, LOW);
  }
  delay(20);
}
