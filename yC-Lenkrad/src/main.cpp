#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
 
const char *ssid = "bobbyCarV1";
const char *pass = "12345678"; 
 
unsigned int localPort = 8888; // local port to listen for UDP packets

int torquePedal = 0;
int breakPedal = 0;
int breakPedalToSend = 0;

byte torquePedal_byte1 = 0;
byte torquePedal_byte2 = 0;

byte breakPedal_byte1 = 0;
byte breakPedal_byte2 = 0;

bool highspeedEnable = false;
bool reversedEnable = false;
 
IPAddress ServerIP(192,168,4,1);
IPAddress ClientIP(192,168,4,2);
 
// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;
 
char packetBuffer[9];   //Where we get the UDP data

//Prototype fuctions

ICACHE_RAM_ATTR void interruptSpeed();
ICACHE_RAM_ATTR void interruptReverse();

//=======================================================================
//                Setup
//=======================================================================
void setup()
{
    Serial.begin(115200);
    Serial.println("initialise Accespoint");
    
    delay(1000);
    //Start UDP
    Serial.println("Starting UDP");
    udp.begin(localPort);
    Serial.print("Local port: ");
    Serial.println(udp.localPort());

    Serial.println("Server available");
    delay(500);

    torquePedal = 0;
    breakPedal = 0;
    breakPedalToSend = 0;

    //WiFi.persistent(false);
    WiFi.softAP(ssid, pass);    //Create Access point

    pinMode(D0, OUTPUT);
    pinMode(D1, OUTPUT);
    pinMode(D6,INPUT);
    pinMode(D5,INPUT);
    attachInterrupt(digitalPinToInterrupt(D6),interruptSpeed,CHANGE);
    attachInterrupt(digitalPinToInterrupt(D5),interruptReverse,CHANGE);
    
}
//======================================================================
//                MAIN LOOP
//======================================================================
void loop()
{
    digitalWrite(D1, HIGH);
    delay(5);
    if(highspeedEnable)
    {
      breakPedal = (analogRead(A0) -262)*2;
    }
    else
    {
      breakPedal = (analogRead(A0) -262)/2;
    }
    digitalWrite(D1, LOW);
    
    digitalWrite(D0, HIGH);
    delay(5);
    if(highspeedEnable)
    {
      torquePedal = (analogRead(A0)-258)*2;
    }
    else
    {
      torquePedal = (analogRead(A0)-258)/2;
    }
    digitalWrite(D0, LOW);
  
    if(breakPedal < 0)
    {
      breakPedal = 0;
    }
    if(torquePedal < 0)
    {
      torquePedal = 0;
    }

    breakPedalToSend = breakPedal;
    //for testing purpose
    //breakPedal = 0;
        
    int cb = udp.parsePacket();
    if (!cb) 
    {
      //If serial data is recived send it to UDP
      if(true)     //only if the potiValue changes
        {
        udp.beginPacket(ClientIP, 8888);
        //Send UDP requests are to port 2000

        torquePedal_byte1 = (byte)torquePedal;
        torquePedal_byte2 = (byte)(torquePedal >>8);

        breakPedal_byte1 = (byte)breakPedalToSend;
        breakPedal_byte2 = (byte)(breakPedalToSend >>8);


        udp.write(torquePedal_byte1); //Send one byte to ESP8266
        udp.write(torquePedal_byte2); //Send one byte to ESP8266
        udp.write(breakPedal_byte1); //Send one byte to ESP8266
        udp.write(breakPedal_byte2); //Send one byte to ESP8266
        udp.write(reversedEnable);
                
        Serial.print("sent torque Pedal: ");
        Serial.print(torquePedal);
        Serial.print(" break Pedal: ");
        Serial.println(breakPedalToSend);

        udp.endPacket();
        delay(10);
        }
    }
    /*
    else {
      // We've received a UDP packet, send it to serial
      udp.read(packetBuffer, 1); // read the packet into the buffer, we are reading only one byte
      Serial.print(packetBuffer);
      delay(20);
    }
    */
}

ICACHE_RAM_ATTR void interruptSpeed(){
  if(digitalRead(D6) == 1)
  {
    Serial.println("Highspeed Mode Enable");
    highspeedEnable = true;
  }
  else
  {
    Serial.println("Highspeed Mode Disable");
    highspeedEnable = false;
  }
}

ICACHE_RAM_ATTR void interruptReverse(){
  if(digitalRead(D5) == 1)
  {
    Serial.println("Reverse Mode Enable");
    reversedEnable = true;
  }
  else
  {
    Serial.println("Reverse Mode Disable");
    reversedEnable = false;
  }
}