/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules
  
 Modified to drive a 3-wheeled BLE Robot Rover! by http://james.devi.to

 Pick one up today in the Adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution.

 16-Jul-2016: Modified extensively by geekguy@hybotics.org 
    Added speed control - slow down is button 1, speed up is button
      2, Stop is button 4. The rover now moves until stopped.
*********************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#include <Adafruit_MotorShield.h>
// #include "utility/Adafruit_PWMServoDriver.h"
// #include <Servo.h> 
#include <string.h>

#define ONBOARD_LED   13
#define BLINKTIME     500
#define BLINKVBAT     150
#define VBAT_ENABLED  false
#define VBAT_PIN      A7
#define VBAT_MIN      3.6

#define MINUTEMS      60000
#define WAITMIN       5

#define MINSPEED      20
#define MAXSPEED      250
#define SPEEDINCR     20

#define STARTDELAYMS  6000

#define DEBUG         true

#if DEBUG
#define VERBOSE_MODE  true  // If set to 'true' enables Bluefruit debug output
#else
#define VERBOSE_MODE  false
#endif

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// And connect 2 DC motors to port M3 & M4 !
Adafruit_DCMotor *L_MOTOR = AFMS.getMotor(4);
Adafruit_DCMotor *R_MOTOR = AFMS.getMotor(3);

//not used, testing acceleration
// int accelTime = 200;

//Name your RC here
String BROADCAST_NAME = "Little Red Rover 2";

String BROADCAST_CMD = String("AT+GAPDEVNAME=" + BROADCAST_NAME);

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// Global variables
int velocity = 0;

float x, y;

int L_restrict = 0;
int R_restrict = 0;

unsigned long lastAccelPacket = 0;

bool isMoving = false;

bool modeToggle = false;

byte leftMSpeed = MAXSPEED / 4;
byte rightMSpeed = MAXSPEED / 4;

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];

char buf[60];

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void) {
  Serial.begin(115200);
  delay(2000);
  Serial.println(F("Hybotics Little Red Rover WIP"));

#if DEBUG
  Serial.println();
#endif

  AFMS.begin();  // Create with the default frequency 1.6KHz

  // Turn on motors
  L_MOTOR->setSpeed(0);
  L_MOTOR->run(RELEASE);

  R_MOTOR->setSpeed(0);
  R_MOTOR->run(RELEASE);
    
  /* Initialize the module */
  BLEsetup();

  // Allow some time to place the rover on the floor
  delay(STARTDELAYMS);
}

void loop(void) {
  // Read new packet data
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  // if (len == 0) return;

  // Read from Accelerometer input
  if ( accelMode() ) {
    lastAccelPacket = millis();
    modeToggle = true;
    return;
  }

  // Stop motors if accelerometer data is turned off (100ms timeout)
  if ( millis() - lastAccelPacket > 100 & modeToggle) {
    L_MOTOR->run(RELEASE);
    R_MOTOR->run(RELEASE);
    modeToggle = false;
    return;
  }

  // If no accelerometer, use control pad
  if ( !modeToggle ) {
    buttonMode();
  }
}

bool accelMode(){
  if (packetbuffer[1] == 'A') {
    x = parsefloat( packetbuffer + 2 );
    y = parsefloat( packetbuffer + 6 );

    if ( x <= -0.55 ) {
      x += 0.55;
      x *= -100.0;
      L_MOTOR->run( BACKWARD );
      R_MOTOR->run( BACKWARD );

      if ( x >= 45 ) {
        x = 45;
      }

      if ( x <= 0 ) {
        x = 0;
      }

      velocity = map( x, 0, 45, 0 ,255 );
#if DEBUG
      Serial.println(F("Accel: Moving REVERSE"));
#endif
    } else if ( x >= -0.25 ) {
      x+= 0.25;
      x *= 100;
      L_MOTOR->run( FORWARD );
      R_MOTOR->run( FORWARD );

      if( x>= 45 ) {
        x = 45;
      }

      if ( x<= 0 ) {
        x = 0;
      }

      velocity = map( x, 0, 45, 0, 255 );
#if DEBUG
      Serial.println(F("Accel: Moving FORWARD"));
#endif
    } else {
      L_MOTOR->run( RELEASE );
      R_MOTOR->run( RELEASE );
      velocity = 0;
#if DEBUG
      Serial.println(F("Accel: ALL STOP!"));
#endif
    }

    // Account for L / R accel

    if ( y >= 0.1 ) {
      y -= 0.1;
      y *= 100;
      if( y >= 50 ) y = 50;
      if( y <= 0 ) y = 0;

      L_restrict = fscale( y, 0.0, 50.0, 0.0, 100.0, -4.0 );
    } else if ( y <= -0.1 ) {
      y += 0.1;
      y *= -100;
      if( y>= 50 ) y = 50;
      if( y<= 0 ) y = 0;

      R_restrict = fscale( y, 0.0, 50.0, 0.0, 100.0, -4.0 );
    } else {
      L_restrict = 0;
      R_restrict = 0;
    }

    float Lpercent = ( 100.0 - L_restrict ) / 100.00 ;
    float Rpercent = ( 100.0 - R_restrict ) / 100.00 ;

    // Serial.print( x ); 
    // Serial.print( "\t" ); 
    // Serial.print( Lpercent ); 
    // Serial.print( "\t" ); 
    // Serial.print( velocity ); 
    // Serial.print( "\t" ); 
    // Serial.println( Rpercent ); 

    L_MOTOR->setSpeed( velocity * Lpercent ); 
    R_MOTOR->setSpeed( velocity * Rpercent ); 

    return true;
  }
    
  return false;
}

bool buttonMode() {
  uint8_t checkSpeed = 0;
  uint8_t buttonNumber;
  boolean buttonPressed;
  static unsigned long lastPress = 0;

  // Buttons
  if (packetbuffer[1] == 'B') {
    buttonNumber = packetbuffer[2] - '0';
    buttonPressed = packetbuffer[3] - '0';

#if DEBUG
    Serial.print(F("Button: "));
    Serial.println(buttonNumber);

    Serial.print(F("Moving: "));
    Serial.println(isMoving);
#endif

    if (buttonPressed) {
      switch(buttonNumber) {
        case 1:
          checkSpeed = MINSPEED + SPEEDINCR;

          if ((leftMSpeed > checkSpeed) and (rightMSpeed > checkSpeed)) {
            leftMSpeed -= SPEEDINCR;
            rightMSpeed -= SPEEDINCR;
          }

#if DEBUG
          Serial.println(F("Slowing Down"));
#endif
          break;
          
        case 2:
          checkSpeed = MAXSPEED - SPEEDINCR;

          if ((leftMSpeed < checkSpeed) and (rightMSpeed < checkSpeed)) {
            leftMSpeed += SPEEDINCR;
            rightMSpeed += SPEEDINCR;
          }

#if DEBUG
          Serial.println(F("Speeding Up"));
#endif
          break;
          
        case 3:
#if DEBUG
          Serial.println(F("Unassigned"));
#endif
          break;
          
        case 4:
          L_MOTOR->run(RELEASE);
          R_MOTOR->run(RELEASE);

          isMoving = false;
#if DEBUG
          Serial.println(F("ALL STOP!"));
#endif
          break;
          
        case 5:
          L_MOTOR->run(FORWARD);
          R_MOTOR->run(FORWARD);

          isMoving = true;
#if DEBUG
          Serial.println(F("Moving FORWARD"));
#endif
          break;

        case 6:
          L_MOTOR->run(BACKWARD);
          R_MOTOR->run(BACKWARD);

          isMoving = true;
#if DEBUG
          Serial.println(F("Moving REVERSE"));
#endif
          break;
          
        case 7:
          L_MOTOR->run(RELEASE);
          R_MOTOR->run(FORWARD);

          isMoving = true;
#if DEBUG
          Serial.println(F("Turning LEFT"));
#endif
          break;

        case 8:
          L_MOTOR->run(FORWARD);
          R_MOTOR->run(RELEASE);

          isMoving = true;
#if DEBUG
          Serial.println(F("Turning RIGHT"));
#endif
          break;
      }

      lastPress = millis();
      
#if DEBUG
      Serial.print(F("Speed: Left = "));
      Serial.print(leftMSpeed);
      Serial.print(F(", Right = "));
      Serial.println(rightMSpeed);
#endif

      L_MOTOR->setSpeed(leftMSpeed); 
      R_MOTOR->setSpeed(rightMSpeed);  
    }

    return true; 
  }

  // if(isMoving){
  // unsigned long timeSincePress = millis() - lastPress;
  // if(timeSincePress <= accelTime){

  //   Serial.println( timeSincePress ) ;
        
  //   int motorSpeed = map( timeSincePress, 0, accelTime, 0, 255 );
        
  //   L_MOTOR->setSpeed(motorSpeed); 
  //   R_MOTOR->setSpeed(motorSpeed); 
  // }
      
  // else{
  // // full speed ahead!
  // L_MOTOR->setSpeed(255); 
  // R_MOTOR->setSpeed(255);  
  // }
  // }

  return false;
}

void BLEsetup(){
#if DEBUG
  Serial.print(F("Initializing the Bluefruit LE module.."));
#endif

  if ( !ble.begin(VERBOSE_MODE) ) {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }

#if DEBUG
  Serial.println( F("OK!") );

  // Perform a factory reset to make sure everything is in a known state
  Serial.println(F("Performing a factory reset.."));
#endif

  if (! ble.factoryReset() ){
       error(F("Couldn't factory reset"));
  }

  // Convert the name change command to a char array
  BROADCAST_CMD.toCharArray(buf, 60);

  // Change the broadcast device name here!
  if(ble.sendCommandCheckOK(buf)){
#if DEBUG
    Serial.println("Name changed");
#endif
  }

  delay(250);

  //reset to take effect
  if(ble.sendCommandCheckOK("ATZ")){
#if DEBUG
    Serial.println("Resetting..");
#endif
  }

  delay(250);

  // Confirm name change
  ble.sendCommandCheckOK("AT+GAPDEVNAME");

  /* Disable command echo from Bluefruit */
  ble.echo(false);

#if DEBUG
  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();
#endif

  ble.verbose(false);  // Debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
    delay(500);
  }

#if DEBUG
  Serial.println(F("*****************"));

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
#endif

  ble.setMode(BLUEFRUIT_MODE_DATA);

#if DEBUG
  Serial.println(F("*****************"));
#endif
}

//Logarithmic mapping function from http://playground.arduino.cc/Main/Fscale
float fscale( float inputValue,  float originalMin, float originalMax, float newBegin, float newEnd, float curve){
  float OriginalRange = 0;
  float NewRange = 0;
  float zeroRefCurVal = 0;
  float normalizedCurVal = 0;
  float rangedValue = 0;
  boolean invFlag = 0;

  // condition curve parameter
  // limit range

  if (curve > 10) {
    curve = 10;
  }

  if (curve < -10) {
    curve = -10;
  }

  curve = (curve * -.1) ; // - invert and scale - this seems more intuitive - postive numbers give more weight to high end on output 
  curve = pow(10, curve); // convert linear scale into lograthimic exponent for other pow function

  /*
   Serial.println(curve * 100, DEC);   // multply by 100 to preserve resolution  
   Serial.println(); 
   */

  // Check for out of range inputValues
  if (inputValue < originalMin) {
    inputValue = originalMin;
  }

  if (inputValue > originalMax) {
    inputValue = originalMax;
  }

  // Zero Refference the values
  OriginalRange = originalMax - originalMin;

  if (newEnd > newBegin){ 
    NewRange = newEnd - newBegin;
  } else {
    NewRange = newBegin - newEnd; 
    invFlag = 1;
  }

  zeroRefCurVal = inputValue - originalMin;
  normalizedCurVal  =  zeroRefCurVal / OriginalRange;   // normalize to 0 - 1 float

  /*
  Serial.print(OriginalRange, DEC);  
   Serial.print("   ");  
   Serial.print(NewRange, DEC);  
   Serial.print("   ");  
   Serial.println(zeroRefCurVal, DEC);  
   Serial.println();  
   */

  // Check for originalMin > originalMax  - the math for all other cases i.e. negative numbers seems to work out fine 
  if (originalMin > originalMax ) {
    return 0;
  }

  if (invFlag == 0) {
    rangedValue =  (pow(normalizedCurVal, curve) * NewRange) + newBegin;
  } else {    // invert the ranges   
    rangedValue =  newBegin - (pow(normalizedCurVal, curve) * NewRange); 
  }

  return rangedValue;
}

