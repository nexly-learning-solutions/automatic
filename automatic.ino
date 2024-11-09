#include <dht.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Timer.h>
SoftwareSerial btSerial(11,12);

 int light = 4;
 int wifi = 5;
 int tv = 6;
 int air = 7;
 int door = 8;
 int buzzer = 9;
 int sensor_in = A0;

 int ac_fan = 3;   
#define SPEED 20
#define GATE_IMPULSE 5
#define FREQ 101
unsigned int  CH1;
unsigned int  buf_CH1;
unsigned char clock_cn;    
unsigned int  clock_tick;   
unsigned char i = 255;

 int timer_temp = 0;
 int timer_humi = 0;

 int fan_address = 0;
 int notifi_fun_address = 1;
 int ai_fun_address = 2;
 int sensor_fun_address = 3;

  int notifi_condition = 1;
  int ai_condition = 1;
  int sensor_condition = 1;

int light_address = 4, wifi_address = 5, tv_address = 6, air_address = 7, door_address = 8;
int light_fc = 0, wifi_fc = 0, tv_fc = 0, air_fc = 0, door_fc = 0;
int fan_condition = 0;
 
dht DHT;
#define DHT11_PIN 10

 int readData = 0; 
 int temp_data = 0; 
 int humi_data = 0;

 String myValue = "null";

void setup() {

 Serial.begin(9600);
 Serial.println("dib Soft iOT");
 btSerial.begin(9600);
 pinMode(13,OUTPUT);
 pinMode(light, OUTPUT);
 pinMode(wifi, OUTPUT);
 pinMode(tv, OUTPUT);
 pinMode(air, OUTPUT);
 pinMode(door, OUTPUT);
 pinMode(buzzer, OUTPUT);
 pinMode(sensor_in, INPUT);

 pinMode(ac_fan, OUTPUT);    
 attachInterrupt(0, zero_crosss_int, RISING);
 Timer1.initialize(10); 
 Timer1.attachInterrupt( timerIsr );

  tone(buzzer, 1000);
  delay(2000);        
  noTone(buzzer);    
 starting();
}


void loop() {


  if(timer_temp > 600)
  {
    timer_temp = 0;
    int temp = temp_data + 1100;
    String tempString = String(temp);
    btSerial.print(tempString);
  }

  if(timer_humi > 500)
  {
    timer_humi = 0;
    int humidi = humi_data + 1200;
    String humdiString = String(humidi);
    btSerial.print(humdiString);
  }

  readData = DHT.read11(DHT11_PIN);
  if (readData !=-2){
  temp_data = DHT.temperature;
  humi_data = DHT.humidity;
  }
  
  if (btSerial.available())
  {

    myValue = btSerial.readString();
    Serial.println(myValue);
    notifi_condition = EEPROM.read(notifi_fun_address);
    if(myValue == "START DEVICE")
    {
      starting();
    }
    on_click();
    
    }
    
  if (Serial.available())

  {
    btSerial.write(Serial.read());
  }

   timer_temp ++;
   timer_humi ++;
   delay(1);
   output_control ();
   sensor_back_door();
   ai_control();
   buf_CH1=DIMM_VALUE(i);
}

int on_click()
{

  if(myValue == "LIGHT ON")
  {
    digitalWrite(13,1);
    digitalWrite(light, 1);
    light_fc = 1;
    EEPROM.write(light_address, 1);
    if(notifi_condition == 0)
    {
      sound_tone();
    }
  }else if(myValue == "LIGHT OFF")
  {
    digitalWrite(13,0);
    digitalWrite(light, 0);
    light_fc = 0;
    EEPROM.write(light_address, 0);
    if(notifi_condition == 0)
    {
      sound_tone();
    }
  }else if(myValue == "WIFI ON")
  {
    digitalWrite(wifi, 1);
    wifi_fc = 1;
    EEPROM.write(wifi_address, 1);
    if(notifi_condition == 0)
    {
      sound_tone();
    }
  }else if(myValue == "WIFI OFF")
  {
    digitalWrite(wifi, 0);
    wifi_fc = 0;
    EEPROM.write(wifi_address, 0);
    if(notifi_condition == 0)
    {
      sound_tone();
    }
  }else if(myValue == "TV ON")
  {
    digitalWrite(tv, 1);
    tv_fc = 1;
    EEPROM.write(tv_address, 1);
    if(notifi_condition == 0)
    {
      sound_tone();
    }
  }else if(myValue == "TV OFF")
  {
    digitalWrite(tv, 0);
    tv_fc = 0;
    EEPROM.write(tv_address, 0);
    if(notifi_condition == 0)
    {
      sound_tone();
    }
  }else if(myValue == "DOOR ON")
  {
    digitalWrite(door, 1);
    door_fc = 1;
    EEPROM.write(door_address, 1);
    if(notifi_condition == 0)
    {
      sound_tone();
    }
  }else if(myValue == "DOOR OFF")
  {
    digitalWrite(door, 0);
    door_fc = 0;
    EEPROM.write(door_address, 0);
    if(notifi_condition == 0)
    {
      sound_tone();
    }
  }else if(myValue == "AIR ON")
  {
    digitalWrite(air, 1);
    air_fc = 1;
    EEPROM.write(air_address, 1);
    if(notifi_condition == 0)
    {
      sound_tone();
    }
  }else if(myValue == "AIR OFF")
  {
    digitalWrite(air, 0);
    air_fc = 0;
    EEPROM.write(air_address, 0);
    if(notifi_condition == 0)
    {
      sound_tone();
    }
  }

  else if(myValue == "NOTIFI ON")
  {
    notifi_condition = 0;
    EEPROM.write(notifi_fun_address, 0);
  }else if(myValue == "NOTIFI OFF")
  {
    notifi_condition = 1;
    EEPROM.write(notifi_fun_address, 1);
  }else if(myValue == "AI ON")
  {
    ai_condition = 0;
    EEPROM.write(ai_fun_address, 0);
  }else if(myValue == "AI OFF")
  {
    ai_condition = 1;
    EEPROM.write(ai_fun_address, 1);
  }else if(myValue == "SENSOR ON")
  {
    sensor_condition = 0;
    EEPROM.write(sensor_fun_address, 0);
  }else if(myValue == "SENSOR OFF")
  {
    sensor_condition = 1;
    EEPROM.write(sensor_fun_address, 1);
  }

  if(myValue == "FAN OFF" || myValue == "FAN OFFFAN OFF")
  {
    i = 255;
    EEPROM.write(fan_address, 0);
    if(notifi_condition == 0)
    {
      sound_tone();
    }
  }else if(myValue == "ONE")
  {
    i = 220;
    EEPROM.write(fan_address, 1);
    if(notifi_condition == 0)
    {
      sound_tone();
    }
  }else if(myValue == "TWO")
  {
    i = 205;
    EEPROM.write(fan_address, 2);
    if(notifi_condition == 0)
    {
      sound_tone();
    }
  }else if(myValue == "THREE")
  {
    i = 180;
    EEPROM.write(fan_address, 3);
    if(notifi_condition == 0)
    {
      sound_tone();
    }
  }else if(myValue == "FOUR")
  {
    i = 155;
    EEPROM.write(fan_address, 4);
    if(notifi_condition == 0)
    {
      sound_tone();
    }
  }else if(myValue == "FIVE")
  {
    i = 130;
    EEPROM.write(fan_address, 5);
    if(notifi_condition == 0)
    {
      sound_tone();
    }
  }else if(myValue == "SIX")
  {
    i = 105;
    EEPROM.write(fan_address, 6);
    if(notifi_condition == 0)
    {
      sound_tone();
    }
  }else if(myValue == "SEVEN")
  {
    i = 80;
    EEPROM.write(fan_address, 7);
    if(notifi_condition == 0)
    {
      sound_tone();
    }
  }else if(myValue == "EIGHT")
  {
    i = 55;
    EEPROM.write(fan_address, 8);
    if(notifi_condition == 0)
    {
      sound_tone();
    }
  }else if(myValue == "NINE")
  {
    i = 30;
    EEPROM.write(fan_address, 9);
    if(notifi_condition == 0)
    {
      sound_tone();
    }
  }else if(myValue == "TEN")
  {
    i = 0;
    EEPROM.write(fan_address, 10);
    if(notifi_condition == 0)
    {
      sound_tone();
    }
  }

  else if(myValue == "POWER OFF")
  {    
    if(notifi_condition == 0)
    {
      sound_tone();
    }
    digitalWrite(13, 0);
    digitalWrite(light, 0);
    digitalWrite(wifi, 0);
    digitalWrite(tv, 0);
    digitalWrite(air, 0);
    digitalWrite(door, 0);
    i = 0;
    EEPROM.write(light_address, 0);
    EEPROM.write(wifi_address, 0);
    EEPROM.write(tv_address, 0);
    EEPROM.write(air_address, 0);
    EEPROM.write(door_address, 0);
    EEPROM.write(fan_address, 0);
  }

  notifi_condition = EEPROM.read(notifi_fun_address);
  ai_condition = EEPROM.read(ai_fun_address);
  sensor_condition = EEPROM.read(sensor_fun_address);

}

int starting()
{
  delay(100);
  int light_condition = EEPROM.read(light_address);
  int wifi_condition = EEPROM.read(wifi_address);
  int tv_condition = EEPROM.read(tv_address);
  int air_condition = EEPROM.read(air_address);
  int door_condition = EEPROM.read(door_address);
  
  if(light_condition == 1)
  {
    light_fc = 1;
    btSerial.write("1310");
  }else
  {
    light_fc = 0;
    btSerial.write("1311");
  }

  delay(100);

  if(wifi_condition == 1)
  {
    wifi_fc = 1;
    btSerial.write("1312");
  }else
  {
    wifi_fc = 0;
    btSerial.write("1313");
  }

  delay(100);

  if(tv_condition == 1)
  {
    tv_fc = 1;
    btSerial.write("1314");
  }else
  {
    tv_fc = 1;
    btSerial.write("1315");
  }

  delay(100);

  if(air_condition == 1)
  {
    air_fc = 1;
    btSerial.write("1316");
  }else
  {
    air_fc = 0;
    btSerial.write("1317");
  }

  delay(100);

  if(door_condition == 1)
  {
    door_fc = 1;
    btSerial.write("1318");
  }else
  {
    door_fc = 0;
    btSerial.write("1319");
  }

  delay(100);

  notifi_condition = EEPROM.read(notifi_fun_address);
  ai_condition = EEPROM.read(ai_fun_address);
  sensor_condition = EEPROM.read(sensor_fun_address);

  if(notifi_condition == 0)
  {
    btSerial.write("1320");
  }else if(notifi_condition == 1)
  {
    btSerial.write("1321");
  }

  delay(100);

  if(ai_condition == 0)
  {
    btSerial.write("1322");
  }else if(ai_condition == 1)
  {
    btSerial.write("1323");
  }

  delay(100);

  if(sensor_condition == 0)
  {
    btSerial.write("1324");
  }else if(sensor_condition == 1)
  {
    btSerial.write("1325");
  }

  delay(100);
  
  fan_condition = EEPROM.read(fan_address);

  if(fan_condition == 0)
  {
    i = 255;
    btSerial.write("1340");
  }else if(fan_condition == 1)
  {
    i = 220;
    btSerial.write("1341");
  }else if(fan_condition == 2)
  {
    i = 205;
    btSerial.write("1342");
  }else if(fan_condition == 3)
  {
    i = 180;
    btSerial.write("1343");
  }else if(fan_condition == 4)
  {
    i = 155;
    btSerial.write("1344");
  }else if(fan_condition == 5)
  {
    i = 130;
    btSerial.write("1345");
  }else if(fan_condition == 6)
  {
    i = 105;
    btSerial.write("1346");
  }else if(fan_condition == 7)
  {
    i = 80;
    btSerial.write("1347");
  }else if(fan_condition == 8)
  {
    i = 55;
    btSerial.write("1348");
  }else if(fan_condition == 9)
  {
    i = 30;
    btSerial.write("1349");
  }else if(fan_condition == 10)
  {
    i = 0;
    btSerial.write("1350");
  }

  delay(200);

  btSerial.write("1335");
}


int output_control ()
{
  if(light_fc == 1)
  {
    digitalWrite(13, 1);
    digitalWrite(light, 1);
  }else if(light_fc == 0)
  {
    digitalWrite(13, 0);
    digitalWrite(light, 0);
  }

  if(wifi_fc == 1)
  {
    digitalWrite(wifi, 1);
  }else if(wifi_fc == 0)
  {
    digitalWrite(wifi, 0);
  }

  if(tv_fc == 1)
  {
    digitalWrite(tv, 1);
  }else if(tv_fc == 0)
  {
    digitalWrite(tv, 0);
  }

  if(air_fc == 1)
  {
    digitalWrite(air, 1);
  }else if(air_fc == 0)
  {
    digitalWrite(air, 0);
  }

  if(door_fc == 1)
  {
    digitalWrite(door, 1);
  }else if(door_fc == 0)
  {
    digitalWrite(door, 0);
  }
  
}

int sound_tone()
{
  tone(buzzer, 1000);
  delay(200);        
  noTone(buzzer);
}

int sensor_back_door()
{
  int sen_value = digitalRead(sensor_in);

  if(sensor_condition == 0)
  {
    if(sen_value == HIGH)
    {
      digitalWrite(door, 1);
    }else{
      digitalWrite(door, 0);
    }
  }
}

int ai_control()
{
  if(ai_condition == 0)
  {
    if(temp_data > 20 && temp_data < 25)
    {
    }

    if(temp_data > 25 && temp_data < 30)
    {
    }

    if(temp_data > 30)
    {
    }

    if(temp_data < 16)
    {
    }
  }
}

void timerIsr()
{    
    clock_tick++;

    if (clock_cn) 
     {
      clock_cn++;
      
       if (clock_cn==GATE_IMPULSE)
       {
        digitalWrite(ac_fan, LOW); 
        clock_cn=0;
       }
     }
   
        if (CH1==clock_tick)
         {
          digitalWrite(ac_fan, HIGH);
          clock_cn=1;
         }  
                    
}

void zero_crosss_int()
{
  CH1=buf_CH1;
  clock_tick=0;        
}

unsigned int DIMM_VALUE (unsigned char level)
{
 unsigned int buf_level;

 if (level < 26)  {level=26;}
 if (level > 229) {level=229;}
 return ((level*(FREQ))/256)*10;  
}
