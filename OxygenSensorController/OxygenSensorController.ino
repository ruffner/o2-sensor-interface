#include <SoftPWM.h>
#include <SoftPWM_timer.h>

/*
  Read oxygen sensor output voltage from opamp and control oxygen sensor heater with
  PID control using a thermocouple.
  Log data serial as csv.
  Simple serial commands to control heater on/off and setpoint and logging on/off

  Matt Ruffner, April 2020
  https://github.com/ruffner/o2-sensor-interface
 */

#include <SPI.h>
#include <PID_v1.h>
#include <SoftPWM.h>
#include "Adafruit_MAX31855.h"

#define SERIAL_BAUD     115200

#define ADC_REF         3.3
#define ADC_RES_INC     4095
#define ADC_RES_BITS    12

// this is the voltage applied to the black lead of the oxygen sensor
// V_bais = 3.3v * (2 kohm/502 kohm) volts
#define SENSOR_BIAS     0.01314741035 //volts

// non-inverting opamp gain 
// Gain = Vout / Vin 
//      = 1 + Rf/Rin 
//      = 1 + ( 100 kohm / 2 kohm )
#define OPAMP_GAIN      51

#define HEATER_PIN      20
#define ADC_PIN_OP_AMP  A1

// different intervals used. Log interval is the most useful to change.
#define COMMAND_CHECK_INTERVAL  50
#define HEATER_UPDATE_INTERVAL  500
#define TC_UPDATE_INTERVAL      500
#define LOG_INTERVAL            100

// inital setpoint temperature of sensor heater
#define INTIAL_SETPOINT         500 // deg c

// create a thermocouple to digital interface over SPI
#define MAXIMUM_TC_TEMP         1000 // deg c
#define MAXCS   10
Adafruit_MAX31855 thermocouple(MAXCS);

// TODO: improve tuning params
double Kp=10, Ki=1, Kd=1;

// create the PID control object and link variables
double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// sliding window for PID heater control
int WindowSize = 1000;
unsigned long windowStartTime;

// var to hold last TC reading
double lastTCTemp = 0.0;

// change these if you want logging and/or heating to start immediately upon startup
bool loggingActive = true;
bool heating = false;
int heaterStatus = 0;

// timestamp holders
unsigned long lastCommandCheck, lastHeaterUpdate, lastLogTime;
unsigned long lastTCUpdate=0;


//////////////////////////////////////////////////////////////////////////////////////////
// store TC temp reading in deg celcius
void updateTC() {
   double c = thermocouple.readCelsius();
   if (isnan(c)) {
     //Serial.println("Something wrong with thermocouple!");
     lastTCTemp = 0;
   } else {
     lastTCTemp = c;
   }
}


//////////////////////////////////////////////////////////////////////////////////////////
// parse simple command strings
// log <on/off>: turn serial csv output on/off
//    heater <on/off>: turn relay toggle pin for heating element on/off
//    heater period <1-2^16>: in milliseconds, the period of the 50% duty cycle toggle pin
void checkForCommand() {
  if( Serial.available() ){
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if( cmd.length() == 0 ) return;

    int ind1 = cmd.indexOf(' ');  //finds location of first space
    if( ind1 == -1 ) return;
    String op1 = cmd.substring(0, ind1);   //captures first data String
    
    int ind2 = cmd.indexOf(' ', ind1+1 );   //finds location of second space
    if( ind2 == -1 ){
      ind2 = cmd.length()-1;
    }
    String op2 = cmd.substring(ind1+1, ind2+1);   //captures second data String;  
    int ind3 = cmd.indexOf(' ', ind2+1 );
    String op3;
    if( ind3 < 0 ){
      ind3 = cmd.length()-1;
    }
    op3 = cmd.substring(ind2+1, ind3+1);

    if( op1.toLowerCase().equals("log") ){
      if( op2.toLowerCase().equals("on") ){
        loggingActive = true;
      } else if( op2.toLowerCase().equals("off") ){
        loggingActive = false;
      }
    } else if( op1.toLowerCase().startsWith("heater") ){
      if( op2.toLowerCase().startsWith("on") ){
        heating = true;
        if( op3.length() > 0 ){
          heaterStatus = min(max(op3.toInt(), 0), 255);
        }
      } else if( op2.toLowerCase().startsWith("off") ){
        heating = false;
      }
    } 
  }
}


//////////////////////////////////////////////////////////////////////////////////////////
// read an adc pin, averaging the reading k times with period ms between readings
unsigned int averageADC(int pin, int k, unsigned short period) {  
  unsigned int a = 0;
  
  for( int i=0; i<k; i++ ){
    a = a + analogRead(pin);
    delayMicroseconds(period);
  }
  
  a = a / k;
  return a;
}


//////////////////////////////////////////////////////////////////////////////////////////
// rescale adc reading to volts, taking into account opamp gain and adc 
// resolution
float rescaleToVolts(unsigned int raw) {
  return ((float)(raw) / (float)ADC_RES_INC)*(float)ADC_REF/(float)OPAMP_GAIN;
}


//////////////////////////////////////////////////////////////////////////////////////////
// setup serial gpio etc
void setup() {
  // initialize the digital pin as an output.
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(ADC_PIN_OP_AMP, INPUT);
  
  Serial.begin(SERIAL_BAUD);

  // 0-4095
  analogReadResolution(ADC_RES_BITS);

  SoftPWMBegin(SOFTPWM_NORMAL);
  
  delay(1000);

  analogWrite(HEATER_PIN, 200);
}


//////////////////////////////////////////////////////////////////////////////////////////
// main loop foeva
void loop() {
  unsigned long now = millis();

  if( now - lastTCUpdate > TC_UPDATE_INTERVAL ){
    lastTCUpdate = now;
    updateTC();
    Input = map(lastTCTemp, 0.0, MAXIMUM_TC_TEMP, 0.0, WindowSize);
  }
  
  if( now - lastCommandCheck > COMMAND_CHECK_INTERVAL ){
    lastCommandCheck = now;
    checkForCommand();
  }

//  if( now - lastHeaterUpdate > HEATER_UPDATE_INTERVAL ){
//    lastHeaterUpdate = now;
//    if( heaterStatus  < 2){
//      digitalWrite(HEATER_PIN, HIGH);
//      heaterStatus++;
//    } else {
//      digitalWrite(HEATER_PIN, LOW);
//      heaterStatus = 0;
//    }
//  }
  
  if( heating ){
    
    //digitalWrite(HEATER_PIN, HIGH);
    //SoftPWMSet(HEATER_PIN, heaterStatus);
    //analogWrite(HEATER_PIN, heaterStatus);
  } else {
    //analogWrite(HEATER_PIN, 0);
    //SoftPWMSet(HEATER_PIN, 0);
    //if( heaterStatus != 0 ){
     // heaterStatus = 0;
    //}
  }

  if( !loggingActive ) return;

  if( now - lastLogTime > LOG_INTERVAL ){
  //if( now - lastLogTime > LOG_INTERVAL && heaterStatus == 1 ){  
    lastLogTime = now;
    unsigned int rawOp  = averageADC(ADC_PIN_OP_AMP, 50, 1000);
    float voltsOp  = (rescaleToVolts(rawOp));
  
    Serial.print(now);
    Serial.print(",");
    Serial.print(heaterStatus);
    Serial.print(",");
    Serial.print(rawOp);
    Serial.print(",");
    Serial.print(voltsOp, DEC);
    Serial.print(",");
    Serial.println(lastTCTemp,DEC);
  }
}
