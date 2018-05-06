#include <TinyGPS.h>
#include <Wire.h>
#include <avr/wdt.h>
#include "Kalman.h"
#include "Thread.h"
#include "StaticThreadController.h"
#include "DFRobot_sim808.h"
#include "SerialCommand.h"
#include <EEPROM.h>
#include "EepromUtil.h"

#define WAIT_MS(ms) for(unsigned long xx = millis(); (millis() - xx) < ms;)
#define WAIT_US(us) for(unsigned long xx = micros(); (micros() - xx) < us;)
#define EVERYMS(ms) static uint16_t __CONCAT(_t,__LINE__); for(uint16_t _m = millis(); _m - __CONCAT(_t,__LINE__) >= ms; __CONCAT(_t,__LINE__) = _m)
#define LENOF(x)  (sizeof(x) / sizeof((x)[0]))
#define CMD_SERIAL          Serial    //CMD Serial
#define GPS_SERIAL          Serial1   //GPS Serial
#define SIM_SERIAL          Serial2   //Sim Serial
#define BLT_SERIAL          Serial3   //Bluetooth Serial
  
#define LED_PIN             13

#define GPS_INTERVAL        500
#define CONSOLE_INTERVAL    2000
#define MPU_INTERVAL        100
#define SIM_INTERVAL        500
#define BG_INTERVAL         100    // Background thread, doing some general stuff with HIGH Freq

#define PHONE_NUMBER        "01685428880"
#define MESSAGE_LENGTH      160

//Eeprom
const int MAXSIZE = 50; //maximum password length
String globalPassWord("default");

char message[MESSAGE_LENGTH];
int messageIndex = 0;
char phone[16];
char datetime[24];

//LoopHandle
char gprsBuffer[64];
char *s = NULL;

SerialCommand command(&BLT_SERIAL);
DFRobot_SIM808 sim808(&SIM_SERIAL);

TinyGPS gps;
float globalFlat, globalFlon;
int kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

Thread gpsThread = Thread();
Thread consoleThread = Thread();
Thread mpuThread = Thread();
Thread simThread = Thread();
Thread bgThread = Thread();

StaticThreadController<5> threadPool ( &gpsThread, &consoleThread, &mpuThread, &simThread, &bgThread);

void gpsDataThread()
{
    bool newData = false;
    static bool isDataReceived = false;
    static byte counter = 0;
    byte times = 5000/GPS_INTERVAL;
    if(++counter > times)
    {
        isDataReceived = false;
        counter = 0;
    }
    unsigned int loop = isDataReceived?100:200;
    if((int)globalFlat == 0 && (int)globalFlon == 0) loop = 1000; //Longer time to delay due to weak GPS signal 
    for (unsigned long start = millis(); millis() - start < loop;)
    {
        while (GPS_SERIAL.available())
        {
            char c = GPS_SERIAL.read();
            if (gps.encode(c)) // Did a new valid sentence come in?
              newData = true;
        }
    }
    if (newData)
    {
        float flat, flon;
        unsigned long age;
        gps.f_get_position(&globalFlat, &globalFlon, &age);
        isDataReceived = true;
    }
}

void simCallBackLoopHandle()
{
   if(sim808.readable())
   {
      sim808_read_buffer(gprsBuffer, 32, DEFAULT_TIMEOUT);
      s = strstr(gprsBuffer,"+CMTI: \"SM\"");
      if(s)
      {
          Serial.println("Sim808 received a new message");
          messageIndex = atoi(s+12);
          char phone[16];
          char datetime[24];
          if(sim808.readSMS(messageIndex, message, MESSAGE_LENGTH, phone, datetime))
          {
              Serial.print("MSG: ");
              Serial.println(message);
              Serial.print("From: ");
              Serial.println(phone);

              if(!isPhoneNumberValid(phone))
              {
                  Serial.println("Unauthorized user - DO NOT PROVIDE GPS LOCATION");
              }
              else if(strstr(message,"?") && globalFlat != 0 && globalFlon != 0)
              {
                  String mapUrl("http://maps.google.com/?q=");
                  String lat(globalFlat, 6);
                  String lon(globalFlon, 6);
                  mapUrl += lat;
                  mapUrl += ",";
                  mapUrl += lon;
                  char mapUrlArray[50];
                  sprintf(mapUrlArray, "%s", mapUrl.c_str());
                  sim808.sendSMS(phone, mapUrlArray);
                  Serial.println(mapUrlArray);
              }
          }
          sim808.deleteSMS(messageIndex);
     }
     sim808_clean_buffer(gprsBuffer,32);
   }
}

bool isPhoneNumberValid(const char* phone)
{
    if(strstr(phone, "8880") ||
       strstr(phone, "1865") )
    {
        return true;
    }
    else
    {
        return false;
    }
}

void backGroundCallBack()
{
//    static bool ledStatus = false;
//    ledStatus = !ledStatus;
//    digitalWrite(13, ledStatus);
  
    command.readSerial();
    wdt_reset();
}
void printOutGps()
{
    Serial.print(globalFlat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : globalFlat, 6);
    Serial.print(",");
    Serial.print(globalFlon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : globalFlon, 6);
    Serial.print("__X=");
    Serial.print(kalAngleX);
    Serial.print("__Y=");
    Serial.print(kalAngleY);
    Serial.println();
}

void setUpEepromPassword()
{
   // Serial.println("Saving string to eeprom...");
    //EepromUtil::eeprom_write_string(100, globalPassWord.c_str());
  
    char buf[MAXSIZE];
    EepromUtil::eeprom_read_string(100, buf, MAXSIZE);
    globalPassWord = String(buf);
}

void setup()
{
  MCUSR = 0;
  wdt_disable();
  //readEpr();
  Serial.begin(115200);   //Concolse Serial
  GPS_SERIAL.begin(9600);     //NeoGPS
  SIM_SERIAL.begin(9600);     //SIM
  BLT_SERIAL.begin(38400);     //Bluetooth Serial
  Serial.print("Identifying SIM808...");
  while(!sim808.init())
  {
    Serial.print(".");
    delay(1000);
  }
  Serial.println();
  Serial.println("SIM808 Init Successful");

  pinMode(LED_PIN, OUTPUT);

  init_command();

  setupMpu6050();

  gpsThread.onRun(gpsDataThread);
  gpsThread.setInterval(GPS_INTERVAL);

  consoleThread.onRun(printOutGps);
  consoleThread.setInterval(CONSOLE_INTERVAL);

  mpuThread.onRun(mpuUpdateValue);
  mpuThread.setInterval(MPU_INTERVAL);

  simThread.onRun(simCallBackLoopHandle);
  simThread.setInterval(SIM_INTERVAL);

  bgThread.onRun(backGroundCallBack);
  bgThread.setInterval(BG_INTERVAL);

  setUpEepromPassword();
  
  Serial.println("===Huy KMT Begin===");
}


void loop()
{
    threadPool.run();
}
