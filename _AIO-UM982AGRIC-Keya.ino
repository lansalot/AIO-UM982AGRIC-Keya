/*
 * UDP Autosteer code for Teensy 4.1
 * For Andy's Keya/CANBUS board
 * Nov 2025
 * Like all Arduino code - copied from somewhere else :)
 * So don't claim it as your own
 */

// Serial Ports
#define SerialAOG Serial               // AgIO USB conection
#include "UM982Parser.h"
UM982Parser um982parser;
UM982PAOGIData PAOGIData;
String PAOGISentence;
HardwareSerial& SerialGPS = Serial2; // I hope, AIO4.5

constexpr int serial_buffer_size = 1023;

const int32_t baudGPS = 460800;
const int32_t baudRTK = 115200; // most are using Xbee radios with default of 115200

#define RAD_TO_DEG_X_10 572.95779513082320876798154814105
// Status LED's
// #define GGAReceivedLED 13         //Teensy onboard LED
#define Power_on_LED 5            //Red
#define Ethernet_Active_LED 6     //Green
#define GPSRED_LED 9              //Red (Flashing = NO IMU or Dual, ON = GPS fix with IMU)
#define GPSGREEN_LED 10           //Green (Flashing = Dual bad, ON = Dual good)
#define AUTOSTEER_STANDBY_LED 11  //Red
#define AUTOSTEER_ACTIVE_LED 12   //Green
uint32_t gpsReadyTime = 0;        //Used for GGA timeout
// 
#define TeensyLED 13
// Apart from that work it out yourself :D
/*****************************************************************/

#include <Wire.h>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include <FlexCAN_T4.h>


// Keya Support
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_256> canbus1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_256> canbus2;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_256> canbus3;

elapsedMillis lastKeyaHeatbeat;
bool keyaDetected = false;
bool keyaIntendToSteer;
int16_t keyaSteeringPosition;
int16_t keyaCurrentSetSpeed;
int16_t keyaCurrentActualSpeed;
float pwmDriveF = 0;

#define wheelBase 3.20
int8_t workingDir;
float wheelAngleGPS;

struct ConfigIP
{
	uint8_t ipOne = 192;
	uint8_t ipTwo = 168;
	uint8_t ipThree = 5;
};  ConfigIP networkAddress;   //3 bytes

// IP & MAC address of this module of this module
byte Eth_myip[4] = { 0, 0, 0, 0 }; // This is now set via AgIO
byte mac[] = { 0x00, 0x00, 0x56, 0x00, 0x00, 0x78 };

unsigned int portMy = 5120;                      // port of this module
unsigned int AOGNtripPort = 2233;                // port NTRIP data from AOG comes in
unsigned int AOGAutoSteerPort = 8888;            // port Autosteer data from AOG comes in
unsigned int portDestination = 9999;             // Port of AOG that listens
char Eth_NTRIP_packetBuffer[serial_buffer_size]; // buffer for receiving ntrip data

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Eth_udpPAOGI;     // Out port 5544, for GPS
EthernetUDP Eth_udpNtrip;     // In port 2233
EthernetUDP Eth_udpAutoSteer; // In & Out Port 8888

IPAddress Eth_ipDestination;

byte CK_A = 0;
byte CK_B = 0;
int relposnedByteCount = 0;

// Speed pulse output
elapsedMillis speedPulseUpdateTimer = 0;
byte velocityPWM_Pin = 36; // Velocity (MPH speed) PWM pin

// Used to set CPU speed
extern "C" uint32_t set_arm_clock(uint32_t frequency); // required prototype

bool useDual = false;
bool dualReadyGGA = false;
bool dualReadyRelPos = false;

elapsedMillis GGAReadyTime = 10000;
elapsedMillis ethernetLinkCheck = 1000;

// Dual
double headingcorr = 900; // 90deg heading correction (90deg*10)

double baseline = 0;
double rollDual = 0;
double relPosD = 0;
double heading = 0;
double headingVTG = 0;
double headingRate = 0;
int8_t GPS_Hz = 10;
int  solQualityHPR;

byte ackPacket[72] = { 0xB5, 0x62, 0x01, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

uint8_t GPSrxbuffer[serial_buffer_size];  // Extra serial rx buffer
uint8_t GPStxbuffer[serial_buffer_size];  // Extra serial tx buffer
uint8_t RTKrxbuffer[serial_buffer_size];  // Extra serial rx buffer

bool isTriggered = false;
bool blink = false;

bool Ethernet_running = false; //Auto set on in ethernet setup
bool Autosteer_running = true; // Auto set off in autosteer setup

float roll = 0;
float pitch = 0;
float yaw = 0;

const uint8_t WAS_SENSOR_PIN = A15;

const uint16_t LOOP_TIME = 40; // 25Hz
uint32_t lastTime = LOOP_TIME;
uint32_t currentTime = LOOP_TIME;
uint32_t autosteerLastTime = LOOP_TIME;

// Setup procedure ------------------------
void setup()
{
	delay(500);               // Small delay so serial can monitor start up
	set_arm_clock(450000000); // Set CPU speed to 150mhz
	Serial.println("\r\n** AGRICB experimental **\r\n");
	Serial.print("CPU speed set to: ");
	Serial.println(F_CPU_ACTUAL);
	pinMode(TeensyLED, OUTPUT);

	SerialGPS.begin(460800);
	delay(100);
	um982parser.begin(SerialGPS, 3); // 3 metres
	Serial.println(um982parser.isRxBufferEnabled() ? "GPS RX buffer: 1024 bytes" : "GPS RX buffer not enabled");
	Serial.println("UM982 AGRICB initialised!");

	Serial.println("\r\nStarting AutoSteer...");
	autosteerSetup();

	Serial.println("\r\nStarting Ethernet...");
	EthernetStart();

	Serial.println("\r\nStarting Keya CANBUS...");
	CAN_Setup();

	Serial.println("\r\nEnd setup, waiting for GPS...\r\n");
}

void loop()
{
	KeyaBus_Receive();
	currentTime = millis();
	const UM982Message& msg = um982parser.message();
	const char* paogiBuffer = PAOGISentence.c_str();
	const size_t paogiLen = PAOGISentence.length();

	if (um982parser.update())
	{
		if (um982parser.decodeAgricToPAOGI(msg, PAOGIData))
		{
			um982parser.formatPAOGISentence(PAOGIData, PAOGISentence);
			//Serial.print(PAOGISentence);
		}
		else
		{
			Serial.println("Failed to decode UM982 message");
		}
		um982parser.clearMessage();
	}


	// Check for RTK via UDP
	unsigned int packetLength = Eth_udpNtrip.parsePacket();

	if (packetLength > 0)
	{
		if (packetLength > serial_buffer_size) packetLength = serial_buffer_size;
		Eth_udpNtrip.read(Eth_NTRIP_packetBuffer, packetLength);
		SerialGPS.write(Eth_NTRIP_packetBuffer, packetLength);
	}

	if (Autosteer_running) autosteerLoop();
	else ReceiveUdp();


	if (ethernetLinkCheck > 10000)
	{
		if (Ethernet.linkStatus() == LinkON)
		{
			ethernetLinkCheck = 0;
		}
	}

	if (currentTime - lastTime >= LOOP_TIME)
	{
		lastTime = currentTime;
		if (PAOGIData.dgpsAgeSeconds > 100 || PAOGIData.dgpsAgeSeconds < 0 || PAOGIData.headingDegrees > 360 ||
			PAOGIData.speedKnots > 100 || PAOGIData.headingDegrees < 0 || PAOGIData.speedKnots < 0 || PAOGIData.altitudeMeters < 0) {
			Serial.println("Dropped bad data!");
		}
		else {
			Eth_udpPAOGI.beginPacket(Eth_ipDestination, 9999);
			Eth_udpPAOGI.write(reinterpret_cast<const uint8_t*>(paogiBuffer), paogiLen);
			Eth_udpPAOGI.endPacket();
		}
	}




} // End Loop
//**************************************************************************

bool calcChecksum()
{
	CK_A = 0;
	CK_B = 0;

	for (int i = 2; i < 70; i++)
	{
		CK_A = CK_A + ackPacket[i];
		CK_B = CK_B + CK_A;
	}

	return (CK_A == ackPacket[70] && CK_B == ackPacket[71]);
}
