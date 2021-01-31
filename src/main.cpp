#include <Arduino.h>
#include <Adafruit_ADS1015.h>
#include <WiFiManager.h>
#include <WiFiUdp.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>





Adafruit_ADS1015 ads = Adafruit_ADS1015(0x48); 
WiFiManager wm;
WiFiUDP udp;
WiFiUDP udp_server;


void TaskBlink(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  // initialize digital pin 2 as an output.
  pinMode(2, OUTPUT);

  for (;;) // A Task shall never return or exit.
  {
    digitalWrite(2, HIGH);   // turn the LED on (HIGH is the voltage level)
    vTaskDelay( 1000 / portTICK_PERIOD_MS ); // wait for one second
    digitalWrite(2, LOW);    // turn the LED off by making the voltage LOW
    vTaskDelay( 1000 / portTICK_PERIOD_MS ); // wait for one second
  }
}
void TaskReceiveCommands(void *pvPArameters){
  char packetBuffer[255]; 
  int packageSize=0;
  int len = 0;
  udp_server.begin(5006);
  for(;;){
    packageSize = udp_server.parsePacket();

    if(packageSize){
      len = udp_server.read(packetBuffer, 255);
      if (len > 0) {

        packetBuffer[len] = 0;

      }

      Serial.println("Contents:");

      Serial.println(packetBuffer);
    }
    else{
      vTaskDelay( 1 / portTICK_PERIOD_MS ); // wait for one seconds
    }
    
    
  }

}
void TaskSendChannelData(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  

  for (;;) // A Task shall never return or exit.
  {
    int16_t adc0, adc1, adc2;
    int8_t chars_read, result;
    adc0 = ads.readADC_SingleEnded(0);
    adc1 = ads.readADC_SingleEnded(1);
    adc2 = ads.readADC_SingleEnded(2);
    Serial.println("Sending Channel data");
    result = udp.beginPacket(IPAddress(192,168,5,173),5005);
    Serial.println(result);
    udp.write('c');
    udp.write((uint8_t)(adc0 >> 8));
    udp.write((uint8_t)(adc0));
    udp.write((uint8_t)(adc1 >> 8));
    udp.write((uint8_t)(adc1));
    udp.write((uint8_t)(adc2 >> 8));
    udp.write((uint8_t)(adc2));
    result = udp.endPacket();
    vTaskDelay( 1000 / portTICK_PERIOD_MS ); // wait for one second
  }
}

void setup() {
  
  WiFi.mode(WIFI_STA);
  Serial.begin(115200);
  bool res;
  res = wm.autoConnect("Wall-E");
  if(!res) {
      Serial.println("Failed to connect");
      // ESP.restart();
  } 
  else {
      //if you get here you have connected to the WiFi    
      Serial.println("connected...yeey :)");
  }
  

 
  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
   ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
  
  ads.begin();
  
  xTaskCreate(
    TaskBlink
    ,  "Blink"   // A name just for humans
    ,  4096  // Stack size
    ,  NULL
    ,  tskIDLE_PRIORITY  // priority
    ,  NULL );

    xTaskCreate(
    TaskSendChannelData
    ,  "SendChannelData"   // A name just for humans
    ,  4096  // Stack size
    ,  NULL
    ,  tskIDLE_PRIORITY  // priority
    ,  NULL );
    xTaskCreate(
    TaskReceiveCommands
    ,  "ReceiveCommands"   // A name just for humans
    ,  4096  // Stack size
    ,  NULL
    ,  5  // priority
    ,  NULL );
}


void loop() {
  
}

