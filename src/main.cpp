#include <Arduino.h>
#include <Adafruit_ADS1015.h>
#include <WiFiUdp.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <WiFiManager.h>
#include <Tone32.h>
#include <ESPmDNS.h>

#define BUZZER 12

Adafruit_ADS1015 ads_rails = Adafruit_ADS1015(0x48);
Adafruit_ADS1015 ads_battery = Adafruit_ADS1015(0x49);
WiFiManager wm;
WiFiUDP udp;
WiFiUDP udp_server;
SemaphoreHandle_t lock;
boolean serverAddressKnown = false;
IPAddress server_addr;
uint16_t server_port;
int servoPos[9];


void playHello() {
 tone(BUZZER, 262, 250,1); // plays C4
 delay(300);
 tone(BUZZER, 330, 250,1); // plays E4
 delay(300);
 tone(BUZZER, 392, 250,1); // plays G4
 delay(300);
 tone(BUZZER, 523, 500,1); // plays C5
 delay(550);
}


void TaskUpdateServos(void *pvParameters)
{
  (void)pvParameters;
  Serial.println("starting servo update task");
  for (;;)
  {
    if (xSemaphoreTake(lock, (10 / portTICK_PERIOD_MS)) == pdTRUE)
    {
      Wire.beginTransmission(0x03);
      Wire.write('S');
      for (int i = 0; i < 9; i++)
      {
        Wire.write(servoPos[i]);
      }
      Wire.endTransmission();
      xSemaphoreGive(lock);
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void TaskBlink(void *pvParameters) // This is a task.
{
  (void)pvParameters;
  // initialize digital pin 2 as an output.
  pinMode(2, OUTPUT);

  for (;;) // A Task shall never return or exit.
  {
    digitalWrite(2, HIGH);                 // turn the LED on (HIGH is the voltage level)
    vTaskDelay(1000 / portTICK_PERIOD_MS); // wait for one second
    digitalWrite(2, LOW);                  // turn the LED off by making the voltage LOW
    vTaskDelay(1000 / portTICK_PERIOD_MS); // wait for one second
  }
}
void TaskReceiveCommands(void *pvParameters)
{
  (void)pvParameters;
  Serial.println("starting receive task");
  char packetBuffer[255];
  int packageSize = 0;
  int len = 0;
  udp_server.begin(5006);
  for (;;)
  {
    packageSize = udp_server.parsePacket();

    if (packageSize)
    {
      len = udp_server.read(packetBuffer, 255);
      if (len > 0)
      {
        packetBuffer[len] = 0;
      }
      Serial.print("The control chaaracter is ");
      Serial.println(packetBuffer[0]);
      switch (packetBuffer[0])
      {
        case 'S':
          if (len != 10)
          {
            Serial.println("invallid servo command received");
          }
          else
          {
            for (int i = 1; i < 10; i++)
            {
              servoPos[i - 1] = packetBuffer[i];
            }
          }
          break;
        case 'C':
          Serial.println("connection string received");
          uint8_t first,second,third,fourth;
          first = packetBuffer[1];
          second = packetBuffer[2];
          third = packetBuffer[3];
          fourth = packetBuffer[4];
          server_port = 0;
          server_addr = IPAddress(first,second,third,fourth);
          memcpy(&server_port, &packetBuffer[5],sizeof(uint16_t));
          Serial.print("the port is ");
          Serial.println(server_port);
          serverAddressKnown= true;
      }
      vTaskDelay(50 / portTICK_PERIOD_MS); // wait for one seconds
    }
    else
    {
      vTaskDelay(50 / portTICK_PERIOD_MS); // wait for one seconds
    }
  }
}
void TaskSendChannelData(void *pvParameters) // This is a task.
{
  (void)pvParameters;

  Serial.println("starting send channel data task");
  for (;;) // A Task shall never return or exit.
  {

    int16_t adc0, adc1, adc2,cell_1,cell_2,cell_3;
    if (xSemaphoreTake(lock, (10 / portTICK_PERIOD_MS)) == pdTRUE)
    {

      adc0 = ads_rails.readADC_SingleEnded(0);
      adc1 = ads_rails.readADC_SingleEnded(1);
      adc2 = ads_rails.readADC_SingleEnded(2);
      cell_1 = ads_battery.readADC_SingleEnded(0);
      cell_2 = ads_battery.readADC_SingleEnded(1);
      cell_3 = ads_battery.readADC_SingleEnded(2);
      Serial.println(cell_1);
      xSemaphoreGive(lock);
    }
    else
    {
      continue;
    }
    if(serverAddressKnown){
      udp.beginPacket(server_addr, server_port);
      udp.write('c');
      udp.write((uint8_t)(adc0 >> 8));
      udp.write((uint8_t)(adc0));
      udp.write((uint8_t)(adc1 >> 8));
      udp.write((uint8_t)(adc1));
      udp.write((uint8_t)(adc2 >> 8));
      udp.write((uint8_t)(adc2));
      udp.write((uint8_t)(cell_1 >> 8));
      udp.write((uint8_t)(cell_1));
      udp.write((uint8_t)(cell_2 >> 8));
      udp.write((uint8_t)(cell_2));
      udp.write((uint8_t)(cell_3 >> 8));
      udp.write((uint8_t)(cell_3));
      udp.endPacket();
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS); // wait for one second
  }
}

void setup()
{
  Serial.begin(115200);
  ads_rails.setGain(GAIN_TWOTHIRDS); // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  ads_rails.begin();
  ads_battery.setGain(GAIN_TWOTHIRDS); // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  ads_battery.begin();
  /* start the wifi*/
  wm.autoConnect("WALL-E", "walleconfig");
  if(!MDNS.begin("walle")){
    Serial.println("Error while starting mdns");
  }
  MDNS.addService("walle","udp",5006);
  /*set all servopos to 128 aka the middle*/ //TODO: kan dit niet in de init ?
  for (int i = 0; i < 9; i++)
  {
    servoPos[i] = 128;
  }
  Serial.println("Play hello");
  //playHello();
  lock = xSemaphoreCreateBinary();
  xSemaphoreGive(lock);
  xTaskCreate(
      TaskBlink, "Blink" // A name just for humans
      ,
      4096 // Stack size
      ,
      NULL, tskIDLE_PRIORITY // priority
      ,
      NULL);

  xTaskCreate(
      TaskSendChannelData, "SendChannelData" // A name just for humans
      ,
      4096 // Stack size
      ,
      NULL, tskIDLE_PRIORITY // priority
      ,
      NULL);
  xTaskCreate(
      TaskReceiveCommands, "ReceiveCommands" // A name just for humans
      ,
      4096 // Stack size
      ,
      NULL, tskIDLE_PRIORITY // priority
      ,
      NULL);
  xTaskCreate(
      TaskUpdateServos, "TaskUpdateServos" // A name just for humans
      ,
      4096 // Stack size
      ,
      NULL, tskIDLE_PRIORITY // priority
      ,
      NULL);
}

void loop()
{
}

