#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <SoftwareSerial.h>
#include <Ticker.h>

 
#define POTI_TORQUE_MIN 0
#define POTI_BREAK_MIN 0
#define POTI_TORQUE_MAX 1000
#define POTI_BREAK_MAX 1000
#define TORQUE_DIV 1
#define MAX_TORQUE 1500

//MotorController
#define HOVER_SERIAL_BAUD   38400       // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD         115200      // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME         0xAAAA     	// [-] Start frme definition for reliable serial communication
#define TIME_SEND           100         // [ms] Sending time interval
#define SPEED_MAX_TEST      200         // [-] Maximum speed for testing
//#define DEBUG_RX 
// [-] Debug received data. Prints all bytes to serial (comment-out to disable)

#define DEBUG_SIGNALS_STEERING
#define DEBUG_MOTORCONTROLLER
//#define FLOATING_AVERAGE

//Prototyp Functions
boolean updatePedalValues(int& para_torquePedal, int& para_breakPedal, WiFiUDP& para_udp);
void Send(int16_t uSteer, int16_t uSpeed);
void changeState();
ICACHE_RAM_ATTR void interruptR2D();
void Receive();
int16_t break_Motors_abs(int16_t breakTorque);
bool reverseEnable = false;



//WIFI Variables
const char *ssid = "bobbyCarV1";
const char *pass = "12345678"; 
 
unsigned int localPort = 8888; // local port to listen for UDP packets
 
IPAddress ServerIP(192,168,4,1);
IPAddress ClientIP(192,168,4,2);

WiFiUDP udp;

// Global variables
SoftwareSerial HoverSerial(D1,D0); 		    // RX, TX
int torquePedal = 0;
int torquePedalBuffer[4] = {0,0,0,0};
int torquePedalToSend = 0;

int breakPedal = 0;
int valueToSend = 0;

boolean newValuesAvailable = false;
boolean ready2Drive = false;
int connectionInactivityCounter = 0;

bool doOnce = false;

char packetBuffer[9];   //Where we get the UDP data

Ticker timerSend;

uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;
bool sendFlag = false;

typedef struct{
   uint16_t	start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct{
   uint16_t start;
   int16_t 	cmd1;
   int16_t 	cmd2;
   int16_t 	speedR;
   int16_t 	speedL;
   int16_t 	speedR_meas;
   int16_t 	speedL_meas;
   int16_t 	batVoltage;
   int16_t 	boardTemp;
   int16_t  currentMotorDC;
   int16_t  currentPhaseAB;
   int16_t  currentPhaseBC;
   uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;




//======================================================================
//                Setup
//======================================================================
void setup()
{
    Serial.begin(SERIAL_BAUD);
    Serial.println("Hoverboard Serial v1.0");
    delay(1000);
    //WiFi.persistent(false);
    WiFi.begin(ssid, pass);   //Connect to access point
    Serial.println("Client");
    delay(1000);
    HoverSerial.begin(HOVER_SERIAL_BAUD);
    pinMode(LED_BUILTIN, OUTPUT);

    int k = 0;
    while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    k++;
    if(k>100) ESP.reset();
  }


    
    //Serial.begin(9600);
    //Serial.println();
 

 
    pinMode(D7,OUTPUT);
    pinMode(D6,OUTPUT);
    pinMode(D5,INPUT);

    attachInterrupt(digitalPinToInterrupt(D5),interruptR2D,RISING);
    timerSend.attach_ms(10,changeState);

    digitalWrite(D7, LOW);

  // Wait for connection
  
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
    
    //Start UDP
    Serial.println("Starting UDP");
    udp.begin(localPort);
    Serial.print("Local port: ");
    Serial.println(udp.localPort());
}
//======================================================================
//                MAIN LOOP
//======================================================================
void loop()
{
    unsigned long timeNow = millis();
    
    Receive();                 //get Data from Motorcontroller

    if(sendFlag == true)        //start send routine every 20 ms
    {
      //receive new values
      newValuesAvailable = updatePedalValues(torquePedal,breakPedal,udp);
      
      

      //eliminate poti noise
      if(breakPedal < POTI_BREAK_MIN )
      {
        breakPedal = 0;
      }
      if(torquePedal < POTI_TORQUE_MIN )
      {
        torquePedal = 0;
      }

      //floatingAverage
      
      torquePedalToSend = torquePedal;

      #ifdef FLOATING_AVERAGE
      torquePedalBuffer[3] = torquePedalBuffer[2];
      torquePedalBuffer[2] = torquePedalBuffer[1];
      torquePedalBuffer[1] = torquePedalBuffer[0];
      torquePedalBuffer[0] = torquePedal;
      torquePedalToSend = ((float)(torquePedalBuffer[0]+torquePedalBuffer[1]+torquePedalBuffer[2]+torquePedalBuffer[3]))/4;
      #endif



      //evaluate signals
      if(abs(breakPedal) > 20 + POTI_BREAK_MIN)
      {
        if((abs(Feedback.speedL_meas) < 100 || abs(Feedback.speedR_meas)<100) && reverseEnable)
        {
          valueToSend = -breakPedal;
          
        }
        else{
          valueToSend = break_Motors_abs(((float)breakPedal/POTI_BREAK_MAX)*MAX_TORQUE/TORQUE_DIV);
        }
      }
      else{
        valueToSend = ((float)torquePedalToSend/POTI_TORQUE_MAX)*MAX_TORQUE/TORQUE_DIV;
      }

      //Safety functions
      //Check if the values updated
      if(newValuesAvailable == true)
      {
        connectionInactivityCounter = 0;
      }
      else
      {
        connectionInactivityCounter += 1;
      }

      if(connectionInactivityCounter > 50){
        valueToSend = 0;                                  //if the values are not updating anymore, the value to send is set to 0 -> no torque
        digitalWrite(LED_BUILTIN, LOW);
        Serial.print("WARNING: Connection Lost");
      }
      else{
        digitalWrite(D6, (timeNow%2000)<1000);  
        digitalWrite(LED_BUILTIN, (timeNow%2000)<1000);
      }

      if(!ready2Drive)
      {
        valueToSend = 0;
        digitalWrite(D7, LOW);
      }
      else
      {
        digitalWrite(D7, HIGH);
      }
    
    #ifdef DEBUG_SIGNALS_STEERING
      Serial.print("ready2Drive: ");
      Serial.print(ready2Drive);
      Serial.print(" torquePedal: ");
      Serial.print(torquePedal);
      Serial.print(" torquePedalToSend: ");
      Serial.print(torquePedalToSend);
      Serial.print(" breakPedal: ");
      Serial.print(breakPedal);
      Serial.print(" valueToSend: ");
      Serial.println(valueToSend);
    #endif

      Send(0, valueToSend);   //Send data to motorcontroller
      sendFlag = false;
    }
}

//======================================================================
//                FUNCTIONS
//======================================================================

int16_t break_Motors_abs(int16_t breakTorque)
{
	int16_t breakTorqueToSend = 0;
  if(Feedback.speedL_meas >= 1 || Feedback.speedR_meas<=1)
  {
    return 0;
  }

	if(abs(Feedback.speedL_meas) < 50 || abs(Feedback.speedR_meas)<50)
	{
		breakTorqueToSend = -breakTorque/10;
	}
	else
	{
		breakTorqueToSend = -breakTorque;
	}
	return breakTorqueToSend;
}

ICACHE_RAM_ATTR void interruptR2D()
{
  if(doOnce == false)
  {
    ready2Drive = true; //toggle r2d
    doOnce = true;
  }
}

void changeState()
{
  sendFlag = true;
}

boolean updatePedalValues(int& para_torquePedal, int& para_breakPedal, WiFiUDP& para_udp)
{
  int cb = para_udp.parsePacket();
    if (cb) 
    {
      // We've received a UDP packet, send it to serial
      para_udp.read(packetBuffer, 5); // read the packet into the buffer, we are reading only one byte
      
      byte torquePedal_byte1 = (byte)packetBuffer[0];
      byte torquePedal_byte2 = (byte)packetBuffer[1];
      byte breakPedal_byte1 = (byte)packetBuffer[2];
      byte breakPedal_byte2 = (byte)packetBuffer[3];
      reverseEnable = (bool)packetBuffer[4];
      
      para_torquePedal = torquePedal_byte1 + torquePedal_byte2 *256;
      para_breakPedal = breakPedal_byte1 + breakPedal_byte2 *256; 
      return true;
    }
    else
    {
      return false;
    }
}

void Send(int16_t uSteer, int16_t uSpeed)
{
  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  HoverSerial.write((uint8_t *) &Command, sizeof(Command)); 
}

void Receive()
{
	// Check for new data availability in the Serial buffer
	if (HoverSerial.available()) {
		incomingByte 	  = HoverSerial.read();		                              // Read the incoming byte
		bufStartFrame	= ((uint16_t)(incomingBytePrev) << 8) +  incomingByte;	// Construct the start frame
		
	}
	else {
		return;
	}

  // If DEBUG_RX is defined print all incoming bytes
  #ifdef DEBUG_RX
		Serial.print(incomingByte);
		return;
	#endif    	
	
	// Copy received data
	if (bufStartFrame == START_FRAME) {	                    // Initialize if new data is detected
		p 		= (byte *)&NewFeedback;
		*p++ 	= incomingBytePrev;
		*p++ 	= incomingByte;		
		idx 	= 2;		
	} else if (idx >= 2 && idx < sizeof(SerialFeedback)) {	// Save the new received data
		*p++ 	= incomingByte; 
		idx++;
	}	
	
	// Check if we reached the end of the package
	if (idx == sizeof(SerialFeedback)) {  	
		uint16_t checksum;
		checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR ^ NewFeedback.speedL
					^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.currentMotorDC ^ NewFeedback.currentPhaseAB ^ NewFeedback.currentPhaseBC);
	
		// Check validity of the new data
		if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
			// Copy the new data
			memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));
		  
			// Print data to built-in Serial
      #ifdef DEBUG_MOTORCONTROLLER
        Serial.print("1: ");   Serial.print(Feedback.cmd1);
        Serial.print(" 2: ");  Serial.print(Feedback.cmd2);
        Serial.print(" 3: ");  Serial.print(Feedback.speedR);
        Serial.print(" 4: ");  Serial.print(Feedback.speedL);
        Serial.print(" 5: ");  Serial.print(Feedback.speedR_meas);
        Serial.print(" 6: ");  Serial.print(Feedback.speedL_meas);
        Serial.print(" 7: ");  Serial.print(Feedback.batVoltage);
        Serial.print(" 8: ");  Serial.print(Feedback.boardTemp);
        Serial.print(" 9: ");  Serial.println(Feedback.currentMotorDC*100/40);
        //Serial.print(" 10: ");  Serial.print(Feedback.currentPhaseAB);
        //Serial.print(" 11: ");  Serial.println(Feedback.currentPhaseBC);
      #endif
			
		} else {
		  Serial.println("Non-valid data skipped");		  
		}
		idx = 0;	// Reset the index (it prevents to enter in this if condition in the next cycle)
	}
 	
	// Update previous states
	incomingBytePrev 	= incomingByte;
}

