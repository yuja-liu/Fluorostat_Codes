#include <SPI.h>
#include "Adafruit_MAX31855.h"

#define DO   3
#define CS   4
#define CLK  5
Adafruit_MAX31855 thermocouple(CLK, CS, DO);

#define HeaterPin 14
#define AirPin 21
#define OD_LED 19
#define LevelLEDPin 17
#define PumpInPin 26
#define PumpOutPin 2
#define ODSensorPin A15
#define LevelSensorPin A13
#define ODLEDSensorPin A12
#define LevelLEDSensorPin A14


//color 1&2 pins
#define ExciteLEDRefPin A6

//-------------------------------------------------------
// Manually set variables
//-------------------------------------------------------
float temperature_SetTo = 30;//37; Temperature Thresh
int target_OD = 700; //OD voltage set point for turbidostat
int Fluorostat_target_channel = 0;
int Fluorostat_target_gain = 36; //26,31,36,41,46,51
//int Fluorostat_target=200;
int Target_fluoro[2] = {200, 200};

//LD Variables---------------
long delaytime = ((3L * 60L + 50L) * 1000L * 60L); //num mils to night
//(hr to night x conversion from hr to min + min to night)x conversion from mils to secs * conversion from secs to mins
long transtime = (15 * 1000 * 3600L);
//num hours for day and night x conversion froms mils to secs x conversion secs to hours
int num_skips = 4; //If you don't want to take fluoroscence measurements every time define this
int current_skip = 0;

//Chemostat Variables--------
long chem_starttime = millis();
long chem_pump_interval = (2 * 1000 * 3600L);
//-------------------------------------------------------

//for when the media runs out, counter of how many times the pumps have gone on initialize
int counter = 0;

const char PMTReadingPin[2] = {A3, A9};
const char ExciteLEDPin[2] = {A0, A7};
const char PMTSensitivityPin[2] = {A2, A5};
const int PMTGainPin[2] = {11, 9};
const int LEDGainPin[2] = {12, 8};

float thermocouple_celcius = 0;
char OneWireAddress = '0';

int InitialPMTGain[2] = {0, 0}; //25-51 should be 0.5~1.1V    when set 100-204//   // value 0-255 as 0-5V // sensitive control
int InitialLEDGain[2] = {0, 0}; //value 0-255 as 0-5V

float SensitivitySetTo[2] = {0, 0};
int ExcitationSetTo[2] = {0, 0};
int SensitivityOutput[2];
int PMTRead[2];
int ExcitationOutput[2];
float ExcitationRFRead[2];
int PMTSwitch[2] = {1, 1};
float ODRead;
float ODLEDrf;
float LiquidLevel;
float LevelODLEDrf;

int LEDGain[2] = {InitialLEDGain[0], InitialLEDGain[1]};
int PMTGain[2] = {InitialPMTGain[0], InitialPMTGain[1]};
int PMTGainCap[2] = {6, 6};
int PMTGainCurrent[2] = { -1, -1};

int PMTGainInput[2] = {0, 0};
int LEDGainInput[2] = {0, 0};
int LastPMTRead[2] = {0, 0};
int FluoroTargetRead = 0;
int LastPMTGain[2] = {0, 0};
int LastODRead = 0;
int LastTempRead = 0;
int LastLiquidLevel = 0;

int ODLEDStatus = 0;
int StirStatus = 0;
int HeaterStatus = 0;
int PumpInStatus = 0;
int PumpOutStatus = 0;
int LevelLEDStatus = 0;

//int incomingByte = 0; // for incoming serial data
bool LightsOn = true; //whether the lights are on or not
long starttime = millis();
long timeleft = 0;

// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(9600);  // initialize serial communication at 9600 bits per second:
  pinMode(LEDGainPin[0], OUTPUT);   // sets the pin as output
  pinMode(LEDGainPin[1], OUTPUT);   // sets the pin as output
  pinMode(PMTGainPin[0], OUTPUT);   // sets the pin as output
  pinMode(PMTGainPin[1], OUTPUT);   // sets the pin as output
  pinMode(HeaterPin, OUTPUT);
  pinMode(AirPin, OUTPUT);
  pinMode(PumpInPin, OUTPUT);
  pinMode(PumpOutPin, OUTPUT);
  pinMode(OD_LED, OUTPUT);
  pinMode(LevelLEDPin, OUTPUT);

  delay(50);


  ExciteLED_SET(0, InitialLEDGain[0]);
  PMT_SET(0, InitialPMTGain[0]);
  ExciteLED_SET(1, InitialLEDGain[1]);
  PMT_SET(1, InitialPMTGain[1]);
  Heater_OFF();
  //stir(5);
  wait(5);

  ExciteLED_OFF(0);
  //OD_calib();
  Heater_OFF();
}

//----------------------------------------------
void PMT_SET(int Which, int Gain) {
  analogWrite(PMTGainPin[Which], Gain);
  SensitivitySetTo[Which] = float(5 * Gain * 4) / 1023;
  send_data();
  delay(100);
}
void PMT_OFF(int Which) {
  analogWrite(PMTGainPin[Which], 0);
  delay(100);
  SensitivitySetTo[Which] = 0;
  send_data();
}
void ExciteLED_SET(int Which, int Gain) {
  analogWrite(LEDGainPin[Which], Gain);
  delay(100);
  ExcitationSetTo[Which] = Gain * 4;
  send_data();
}
void ExciteLED_OFF(int Which) {
  analogWrite(LEDGainPin[Which], 0);
  ExcitationSetTo[Which] = 0;
  send_data();
  delay(100);
}

void ODLED_ON() {
  digitalWrite(OD_LED, HIGH);
  ODLEDStatus = 1;
  send_data();
  delay(100);
}
void ODLED_OFF() {
  digitalWrite(OD_LED, LOW);
  ODLEDStatus = 0;
  send_data();
  delay(100);
}
void Stir_ON() {
  digitalWrite(AirPin, HIGH);
  StirStatus = 1;
  send_data();
  delay(100);
}
void Stir_OFF() {
  digitalWrite(AirPin, LOW);
  StirStatus = 0;
  send_data();
  delay(100);
}
void Heater_ON() {
  digitalWrite(HeaterPin, LOW);
  HeaterStatus = 1;
  send_data();
  delay(100);
}
void Heater_OFF() {
  digitalWrite(HeaterPin, HIGH);
  HeaterStatus = 0;
  send_data();
  delay(100);
}
void PumpIn_ON() {
  digitalWrite(PumpInPin, HIGH);
  PumpInStatus = 1;
  send_data();
  delay(100);
}
void PumpIn_OFF() {
  digitalWrite(PumpInPin, LOW);
  PumpInStatus = 0;
  send_data();
  delay(100);
}
void PumpOut_ON() {
  digitalWrite(PumpOutPin, HIGH);
  PumpOutStatus = 1;
  send_data();
  delay(100);
}
void PumpOut_OFF() {
  digitalWrite(PumpOutPin, LOW);
  PumpOutStatus = 0;
  send_data();
  delay(100);
}
void LevelLED_ON() {
  digitalWrite(LevelLEDPin, HIGH);
  LevelLEDStatus = 1;
  send_data();
  delay(100);
}
void LevelLED_OFF() {
  digitalWrite(LevelLEDPin, LOW);
  LevelLEDStatus = 0;
  send_data();
  delay(100);
}


void AirPin_ON() {
  digitalWrite(AirPin, HIGH);
  delay(100);
  StirStatus = 1;
  send_data();
  delay(100);

}

void AirPin_OFF() {
  digitalWrite(AirPin, LOW);
  delay(100);
  StirStatus = 0;
  send_data();
  delay(100);
}

//----------------------------------------------

void turbido_pump(int duration, int ODSet) {

  if (LastODRead < ODSet) {
    //stir(10);
    pump_out(duration); // in case the water level too high
    PumpIn_ON();

    for (int i = 0; i < duration; i++) {
      delay(1000);
    }

    PumpIn_OFF();

    //stir(10);
    delay(2000);//2s
    temperature_measurements_and_set_peltier();
    pump_out(duration * 2); // motors pump fluid out (for tudbidostat),4 times PumpOut time than PumpIn time
    pump_off();//make sure that the pump is off
  }

  else { /// just in case
    //stir(10);
    pump_out(duration);
    pump_off();
  }
}

//----------------------------------------------


void Fluoro_pump(int FluoroSet, int duration) {

  if (FluoroTargetRead > FluoroSet) {
    stir(10);
    pump_out(duration); // in case the water level too high
    PumpIn_ON();

    for (int i = 0; i < duration; i++) {
      delay(1000);
    }

    PumpIn_OFF();

    stir(10);
    delay(2000);//2s
    temperature_measurements_and_set_peltier();
    pump_out(duration * 4); // motors pump fluid out (for tudbidostat),4 times PumpOut time than PumpIn time
    pump_off();//make sure that the pump is off
  }

  else { /// just in case
    stir(10);
    pump_out(duration);
    pump_off();
  }
}


//----------------------------------------------

void pump_out(int duration) {
  PumpOut_ON();
  for (int i = 0; i < duration; i++) {
    delay(1000);
  }
  PumpOut_OFF();
}

void pump_in(int duration) {
  PumpIn_ON();
  for (int i = 0; i < duration; i++) {
    delay(1000);
  }
  PumpIn_OFF();
}

//----------------------------------------------
void pump_off() {
  PumpIn_OFF();
  PumpOut_OFF();
}

//----------------------------------------------
void wait(int duration) {
  for (int i = 0; i < duration; i++) {
    delay(1000);
  }
}
//----------------------------------------------
void temperature_measurements_and_set_peltier() { // N seconds
  /////////Temp Measurements
  thermocouple_celcius = thermocouple.readCelsius();
  LastTempRead = thermocouple_celcius;
  delay(50);
  /////////set temperature
  if (LastTempRead < temperature_SetTo) {
    Heater_ON();
  }
  else
    Heater_OFF();
}

//----------------------------------------------

void stir(int duration) {
  Stir_ON();
  for (int i = 0; i < duration; i++) {
    delay(1000);
  }
  Stir_OFF();
}

//----------------------------------------------
void  PMT_rolling_measurements(int which) {
  for (int k = 0; k < PMTGainCap[which] + 1; k++) {
    set_PMT_from_internal(which);//rolling PMT
    PMTSet_measurements(which, LEDGain[which], PMTGain[which]);
    wait(1);
    //stir(5);
    //wait(2);
  }
}

void set_PMT_from_internal(int which) {
  //////////////////////////////////////////////////////////////////////Decide what to set
  if (LastPMTRead[which] < 900) { // when signal voltage is in good range
    if (PMTGainCurrent[which] < PMTGainCap[which]) {
      PMTGainCurrent[which]++;      // increase PMT Gain
    }
    else {
      PMTGainCurrent[which] = 0;             // when hit max PMT gain, reset PMT Gain to min PMT gain
    }
  }

  if (LastPMTRead[which] > 900) {        //when signal voltage is to high
    if (PMTGainCap[which] > 0) {      // if  the max PMT gain is not yet set to 0
      PMTGainCap[which] = PMTGainCurrent[which] - 1;      // set the max PMT gain as 1 level lower than the current PMT gain
      PMTGainCurrent[which] = 0;           //reset PMT Gain to min PMT gain
    }
    else {            // if  the max PMT gain has set to 0
      PMTSwitch[which] = 0;  // stop using PMT
    }
  }
  //////////////////////////////////////////////////////////////////////SET
  if (PMTSwitch[which] != 0) { // continue using PMT
    LEDGain[which] = 255;
    PMTGain[which] = int(26 + 5 * float(PMTGainCurrent[which]));
  }
  else {              // stop using PMT
    LEDGain[which] = 0;
    PMTGain[which] = 0;
  }
}

void set_PMT_from_internal2(int which) {
  //////////////////////////////////////////////////////////////////////Decide what to set
  if (LastPMTRead[which] < 900 && LastPMTRead[which] > 200) { // when signal voltage is in good range
    if (PMTGainCurrent[which] < PMTGainCap[which]) {
      PMTGainCurrent[which]++;      // increase PMT Gain
    }
    if (PMTGainCurrent[which] == PMTGainCap[which]) {
      PMTGainCurrent[which] = 0;             // when hit max PMT gain, reset PMT Gain to min PMT gain
    }
  }

  if (LastPMTRead[which] < 200) { // when signal voltage is too weak
    if (PMTGainCurrent[which] < PMTGainCap[which]) {
      PMTGainCurrent[which]++;      // increase PMT Gain
    }
    if (PMTGainCurrent[which] == PMTGainCap[which]) {
      if (PMTGainCap[which] < 6) {
        PMTGainCap[which]++;
        PMTGainCurrent[which] = 0;
      }
      else {
        PMTGainCap[which] = 6;
        PMTGainCurrent[which] = 0;
      }
    }
  }

  if (LastPMTRead[which] > 900) {        //when signal voltage is to high
    if (PMTGainCap[which] > 0) {      // if  the max PMT gain is not yet set to 0
      PMTGainCap[which] = PMTGainCurrent[which] - 1;      // set the max PMT gain as 1 level lower than the current PMT gain
      PMTGainCurrent[which] = 0;           //reset PMT Gain to min PMT gain
    }
    else {            // if  the max PMT gain has set to 0
      PMTSwitch[which] = 0;  // stop using PMT
    }
  }
  //////////////////////////////////////////////////////////////////////SET
  if (PMTSwitch[which] != 0) { // continue using PMT
    LEDGain[which] = 255;
    PMTGain[which] = int(26 + 5 * float(PMTGainCurrent[which]));
  }
  else {              // stop using PMT
    LEDGain[which] = 0;
    PMTGain[which] = 0;
  }
}

//----------------------------------------------
void PMTSet_measurements(int Which, int LedG, int PmtG) {
  ExciteLED_SET(Which, LedG);
  PMT_SET(Which, PmtG);
  delay(10500); // cant not lower than 10000. LED and PMT needs time to be stable when start from 0
  temperature_measurements_and_set_peltier();
  //Heater_OFF();//////turn off the heater when measure
  //delay(2000);

  float sen = 0;
  float PMTr = 0;
  float excit = 0;
  float LEDr = 0;
  int i = 0;
  //int NumOfPoints=20;
  int NumOfPoints = 100;

  for (i = 0; i < NumOfPoints; i++) { /// start measuring
    sen = sen + analogRead(PMTSensitivityPin[Which]);
    PMTr = PMTr + analogRead(PMTReadingPin[Which]);
    excit = excit + analogRead(ExciteLEDPin[Which]);
    LEDr = LEDr + analogRead(ExciteLEDRefPin);
    delay(50);
  }

  PMTRead[Which] = float(float(PMTr) / float(NumOfPoints)); /// calculate average
  SensitivityOutput[Which] = float(float(sen) / float(NumOfPoints));
  ExcitationOutput[Which] = int(float(excit) / float(NumOfPoints));
  ExcitationRFRead[Which] = int(float(LEDr) / float(NumOfPoints));
  LastPMTRead[Which] = PMTRead[Which]; ////////////////////////////////////////////////
  if (PmtG == Fluorostat_target_gain && Which == Fluorostat_target_channel) {
    FluoroTargetRead = PMTRead[Which]; ////////////////////////////////////////////////
  }
  send_data();
  delay(50);

  ExciteLED_OFF(Which);
  PMT_OFF(Which);
  temperature_measurements_and_set_peltier();
  delay(2000);//wait for voltage stable

}

//----------------------------------------------
void OD_signal_read() {
  float ODsensor = 0;
  float ODLEDsensor = 0;
  int i = 0;
  int NumOfPoints = 20;
  Heater_OFF(); // for LED stablize
  delay(500);
  ODLED_ON();
  delay(1000);// wait 1s for Led stablize

  for (i = 0; i < NumOfPoints; i++) {
    delay(500);
    ODLEDsensor = ODLEDsensor + analogRead(ODLEDSensorPin);
    delay(500);
    ODsensor = ODsensor + analogRead(ODSensorPin);
    Serial.println(analogRead(ODSensorPin));
  }
  ODLEDrf = float(ODLEDsensor) / float(NumOfPoints);
  ODRead = float(ODsensor) / float(NumOfPoints);
  LastODRead = ODRead;
  send_data();
  delay(50);
  ODLED_OFF();//turn off ODLED
  delay(50);
  temperature_measurements_and_set_peltier();
}

void LevelSensing() {

  float Levelsensor = 0;
  float LevelLEDsensor = 0;
  int i = 0;
  int NumOfPoints = 50;
  //int NumOfPoints=1;
  Heater_OFF(); // for LED stablize
  delay(500);
  LevelLED_ON();
  delay(1000);

  for (i = 0; i < NumOfPoints; i++) {
    delay(500);
    Levelsensor = Levelsensor + analogRead(LevelSensorPin);
    delay(500);
    LevelLEDsensor = LevelLEDsensor + analogRead(LevelLEDSensorPin);
  }
  LevelODLEDrf = float(LevelLEDsensor) / float(NumOfPoints);
  LiquidLevel = float(Levelsensor) / float(NumOfPoints);
  LastLiquidLevel = LiquidLevel;
  send_data();
  delay(50);
  LevelLED_OFF();//turn off ODLED
  delay(50);

}

//----------------------------------------------
void CleanData() {
  PMTRead[0] = double(0);
  PMTRead[1] = double(0);
  ODRead = 0;
  SensitivityOutput[0] = double(0);
  ExcitationOutput[0] = double(0);
  ExcitationRFRead[0] = double(0);
  SensitivityOutput[1] = double(0);
  ExcitationOutput[1] = double(0);
  ExcitationRFRead[1] = double(0);
  ODLEDrf = 0;
  thermocouple_celcius = 0;
  LiquidLevel = 0;
  LevelODLEDrf = 0;
}

//----------------------------------------------
void Printdata() {
  //signal scale conversion
  float  SenV0 = SensitivityOutput[0] * (5.0 / 1023.0); // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  float  SenV1 = SensitivityOutput[1] * (5.0 / 1023.0);

  /////////print measurements

  double data[] = {SensitivitySetTo[0], SenV0, ExcitationSetTo[0], ExcitationOutput[0], ExcitationRFRead[0], Target_fluoro[0], PMTRead[0],
                   SensitivitySetTo[1], SenV1, ExcitationSetTo[1], ExcitationOutput[1], ExcitationRFRead[1], Target_fluoro[1], PMTRead[1],
                   target_OD, ODRead, temperature_SetTo, thermocouple_celcius, ODLEDStatus, StirStatus,
                   HeaterStatus, PumpInStatus, PumpOutStatus, ODLEDrf, LevelLEDStatus, LiquidLevel, LevelODLEDrf
                  };

  int data_length = 0;
  data_length = sizeof(data) / sizeof(data[0]);

  for (int j = 0; j < data_length - 1; j++) {
    Serial.print(data[j]);
    Serial.print(" ");
  }
  Serial.println(data[data_length - 1]);
}

void send_data() {
  Printdata();
  CleanData();
  delay(100);
}

//.--------------------------------------------------------------algorithms--------------------------------------------------------------


void Pump_for_Exp_Start() {

  pump_out(4); // in case the water level too high
  PumpIn_ON();
  for (int i = 0; i < 4; i++) {
    delay(1000);
  }
  PumpIn_OFF();
  stir(10);
  delay(2000);//2s
  temperature_measurements_and_set_peltier();
  pump_out(10); // motors pump fluid out (for tudbidostat),4 times PumpOut time than PumpIn time
  pump_off();//make sure that the pump is off

}


void Thorlab() {
  //temperature_measurements_and_set_peltier();
  ExcitationRFRead[0] = int(analogRead(ExciteLEDRefPin));
  send_data();
}

void turbidostat(int targetOD) {
  target_OD = targetOD;

  stir(10);
  temperature_measurements_and_set_peltier();
  PMT_rolling_measurements(0);
  stir(10);
  temperature_measurements_and_set_peltier();
  OD_signal_read();

  //stir(10);
  //temperature_measurements_and_set_peltier();
  //PMT_rolling_measurements(1);
  //stir(10);
  //temperature_measurements_and_set_peltier();
  //OD_signal_read();

  //stir(10);

  ////////for turbido
  for (int i = 0; i < 3; i++) { //3 times make sure dilute enough
    temperature_measurements_and_set_peltier();
    wait(1);
    turbido_pump(2, targetOD); // 2 secs pump in, in case of overflow
    LevelSensing();
  }
}

//Same as the turbidostat code, but turns the lights on when not taking OD or fluorescence measurements.
void turbidostat_cyano(int targetOD) {
  target_OD = targetOD;
  LevelLED_ON();
  stir(10);
  temperature_measurements_and_set_peltier();
  LevelLED_OFF();
  //PMT_rolling_measurements(0);
  LevelLED_ON();
  stir(10);
  temperature_measurements_and_set_peltier();
  LevelLED_OFF();
  OD_signal_read();

  LevelLED_ON();
  stir(10);
  temperature_measurements_and_set_peltier();
  LevelLED_OFF();
  //PMT_rolling_measurements(1);
  LevelLED_ON();
  stir(10);
  temperature_measurements_and_set_peltier();
  LevelLED_OFF();
  OD_signal_read();

  LevelLED_ON();
  stir(10);

  ////////for turbido
  for (int i = 0; i < 3; i++) { //3 times make sure dilute enough
    temperature_measurements_and_set_peltier();
    wait(1);
    turbido_pump(2, targetOD); // 2 secs pump in, in case of overflow
    LevelSensing();
  }
}

void fluorostat(int channel, int target) {

  target_OD = 0;
  Target_fluoro[0] = 1000;
  Target_fluoro[1] = 1000;
  Target_fluoro[channel] = target;

  stir(10);
  temperature_measurements_and_set_peltier();
  PMT_rolling_measurements(0);
  stir(10);
  temperature_measurements_and_set_peltier();
  OD_signal_read();

  //stir(10);
  //temperature_measurements_and_set_peltier();
  //PMT_rolling_measurements(1);
  //stir(10);
  //temperature_measurements_and_set_peltier();
  //OD_signal_read();

  stir(10);

  ////////for fluorostat
  for (int i = 0; i < 3; i++) { //3 times make sure dilute enough
    temperature_measurements_and_set_peltier();
    wait(1);
    Fluoro_pump(target, 2); //(channel, target reading, secs pump in)
    LevelSensing();
  }
}


void batch() {

  ////batch
  stir(5);
  temperature_measurements_and_set_peltier();
  PMT_rolling_measurements(0);
  stir(5);
  temperature_measurements_and_set_peltier();
  OD_signal_read();

  stir(5);
  temperature_measurements_and_set_peltier();
  PMT_rolling_measurements(1);
  stir(5);
  temperature_measurements_and_set_peltier();
  OD_signal_read();
}


void ODbatch() {

  ////batch
  stir(5);
  temperature_measurements_and_set_peltier();
  //PMT_rolling_measurements(0);
  stir(5);
  temperature_measurements_and_set_peltier();
  OD_signal_read();

  stir(5);
  temperature_measurements_and_set_peltier();
  //PMT_rolling_measurements(1);
  stir(5);
  temperature_measurements_and_set_peltier();
  OD_signal_read();
}

void ODbatch_light() {

  ////batch
  //ExciteLED_SET(0 , 255);
  LevelLED_OFF();
  stir(5);
  temperature_measurements_and_set_peltier();
  //ExciteLED_OFF(0);

  LevelLED_ON();
  if ( current_skip % num_skips == 0) {
    //PMT_rolling_measurements(0);
  }
  //ExciteLED_SET(0 , 255);
  stir(10);
  //ExciteLED_OFF(0);
  //LevelLED_ON();
  OD_signal_read();
  //ExciteLED_SET(0 , 255);
  LevelLED_OFF();

  for (int i = 0; i < 180; i++) { //15 min of stirring
    stir(5);
    temperature_measurements_and_set_peltier();
  }
  ++num_skips;
}

long timetillevent(long stime, long ttime, long dtime) {
  long offset = ttime - dtime;
  long now = millis();
  long timepassed = now - stime;
  long timetill = ttime - (timepassed + offset);
  //Serial.println("Time until switch:");
  //Serial.println(timetill);
  return timetill;
}

long countdown(long stime, long ttime) {
  long now = millis();
  long timepassed = now - stime;
  long timetill = ttime - (timepassed);
  //Serial.println("Time until switch:");
  //Serial.println(timetill);
  return timetill;
}

void ODbatch_light_LD(long stime, long ttime, long dtime) {
  starttime = stime;
  delaytime = dtime;
  //
  //  //if (Serial.available() > 0) {
  //  // read the incoming byte:
  //  //incomingByte = Serial.read();
  //
  //  // say what you got:
  //  //Serial.println(incomingByte);
  //  //49 byte means 1 (true)
  //  //48 byte means 0 (false)
  //  //if(incomingByte == 49){
  //  //    Lightson = true;
  //  //  }
  //  //}
  //  //if(incomingByte == 48){
  //  //    Lightson = false;
  //
  //  // }
  timeleft = timetillevent(stime, ttime, dtime);
  //Serial.println(timeleft);
  if (timeleft < 1000) {
    starttime = millis();
    delaytime = transtime;
    LightsOn = !LightsOn;
  }


  ////batch
  //ExciteLED_SET(0 , 255);
  if (LightsOn) {
    LevelLED_OFF();
  }
  stir(5);
  temperature_measurements_and_set_peltier();
  //ExciteLED_OFF(0);
  //
  //ExciteLED_SET(0 , 255);
  stir(10);
  //ExciteLED_OFF(0);
  LevelLED_ON();
  OD_signal_read();
  PMT_rolling_measurements(0);
  //ExciteLED_SET(0 , 255);
  if (LightsOn) {
    LevelLED_OFF();
  }
  for (int i = 0; i < 100; i++) {
    stir(5);
    temperature_measurements_and_set_peltier();
  }
}
//void toarduino(){
//  if (Serial.available() > 0) {
//    // read the incoming byte:
//    incomingByte = Serial.read();
//    Serial.print(incomingByte);
//    delay(5000);
//
//    if(incomingByte == 49){
//        LightsOn = true;
//      }
//    }
//    if(incomingByte == 48){
//        LightsOn = false;
//    }
//  if (LightsOn){
//    LevelLED_OFF();
//  }
//  else{
//    LevelLED_ON();
//  }
//  }
void ODbatch_light_air() {

  ////batch
  //ExciteLED_SET(0 , 255);
  LevelLED_OFF();
  AirPin_ON();
  //stir(5);
  temperature_measurements_and_set_peltier();
  //ExciteLED_OFF(0);

  //PMT_rolling_measurements(0);
  //ExciteLED_SET(0 , 255);
  //stir(10);
  //ExciteLED_OFF(0);

  AirPin_OFF();
  wait(30);
  LevelLED_ON();
  OD_signal_read();
  //ExciteLED_SET(0 , 255);
  LevelLED_OFF();
  AirPin_ON();

  for (int i = 0; i < 180; i++) { //15 minutes between readings
    wait(5);
    temperature_measurements_and_set_peltier();
    //AirPin_ON();
    wait(5);
  }
}

void ODbatch_light_air_PMT() {

  ////batch
  //ExciteLED_SET(0 , 255);
  LevelLED_OFF();
  AirPin_ON();
  //stir(5);
  temperature_measurements_and_set_peltier();
  //ExciteLED_OFF(0);


  //ExciteLED_SET(0 , 255);
  //stir(10);
  //ExciteLED_OFF(0);
  LevelLED_ON();
  AirPin_OFF();
  OD_signal_read();
  PMT_rolling_measurements(0);
  //ExciteLED_SET(0 , 255);
  LevelLED_OFF();
  AirPin_ON();

  for (int i = 0; i < 100; i++) {
    temperature_measurements_and_set_peltier();
    wait(5);
  }


}

//Same as the turbidostat code, but turns the lights on when not taking OD or fluorescence measurements.
void chemostat_cyano_air(long chem_pump_interval, long starttime) {
  chem_starttime = starttime;
  LevelLED_OFF();
  AirPin_ON();


  temperature_measurements_and_set_peltier();


  AirPin_OFF();
  wait(30);
  LevelLED_ON();

  pump_out(5);

  //wait(10);
  OD_signal_read();
  //
  //ExciteLED_SET(0 , 255);
  //PMT_rolling_measurements(0);

  LevelLED_OFF();
  ////////for turbido

  timeleft = countdown(chem_starttime, chem_pump_interval);

  Serial.println(timeleft);
  if (timeleft < 1000) {
    //Serial.println("True \n");
    chem_starttime = millis();
    for (int i = 0; i < 1; i++) { //3 times make sure dilute enough
      temperature_measurements_and_set_peltier();
      wait(1);
      pump_out(5);
      pump_in(5);
      pump_out(10);
    }
  }

  AirPin_ON();
  LevelLED_OFF();

  for (int i = 0; i < 180; i++) { //15 minutes between readings
    wait(5);
    temperature_measurements_and_set_peltier();
    //AirPin_ON();
    wait(5);
  }
  delay(3000);

}

void turbidostat_cyano_air(int targetOD, int count) {
  count = counter;
  target_OD = targetOD;
  LevelLED_OFF();
  AirPin_ON();


  temperature_measurements_and_set_peltier();


  AirPin_OFF();
  wait(45);
  LevelLED_ON();

  pump_out(5);

  //wait(10);
  OD_signal_read();
  //
  //ExciteLED_SET(0 , 255);
  //PMT_rolling_measurements(0);

  LevelLED_OFF();
  ////////for turbido
  while ( LastODRead < targetOD and count < 10) {
    for (int i = 0; i < 1; i++) { //3 times make sure dilute enough
      temperature_measurements_and_set_peltier();
      wait(1);
      turbido_pump(10, targetOD); // 2 secs pump in, in case of overflow
    }
    count = count + 1;
    LevelLED_ON();
    OD_signal_read();
    LevelLED_OFF();
  }
  if (count > 9) {
    counter = 10;
  }

  AirPin_ON();
  LevelLED_OFF();
  for (int i = 0; i < 180; i++) { //15 minutes between readings
    wait(5);
    temperature_measurements_and_set_peltier();
    //AirPin_ON();
    wait(5);
  }

}
//Same as the turbidostat code, but turns the lights on when not taking OD or fluorescence measurements.
void turbidostat_cyano_air_LD(int targetOD, long stime, long dtime, int count) {
  pump_out(5);
  starttime = stime;
  delaytime = dtime;
  target_OD = targetOD;
  count = counter;
  if (LightsOn) {
    LevelLED_OFF();
  }
  AirPin_ON();


  timeleft = timetillevent(starttime, transtime, dtime);
  //Serial.println(timeleft);
  if (timeleft < 1000) {
    starttime = millis();
    delaytime = transtime;
    LightsOn = !LightsOn;
  }
  if (LightsOn) {
    LevelLED_OFF();
  }


  temperature_measurements_and_set_peltier();


  AirPin_OFF();
  wait(45);
  LevelLED_ON();

  pump_out(5);

  //wait(10);
  OD_signal_read();
  //
  //ExciteLED_SET(0 , 255);
  PMT_rolling_measurements(0);

  if (LightsOn) {
    LevelLED_OFF();
  }
  ////////for turbido
  while ( LastODRead < targetOD and count < 10) {
    for (int i = 0; i < 1; i++) { //3 times make sure dilute enough
      temperature_measurements_and_set_peltier();
      wait(1);
      turbido_pump(10, targetOD); // 2 secs pump in, in case of overflow
    }
    count = count + 1;
    LevelLED_ON();
    OD_signal_read();
    if (LightsOn) {
      LevelLED_OFF();
    }
  }
  if (count > 9) {
    counter = 10;
  }

  AirPin_ON();
  if (LightsOn) {
    LevelLED_OFF();
  }
  for (int i = 0; i < 180; i++) { //15 minutes between readings
    wait(5);
    temperature_measurements_and_set_peltier();
    //AirPin_ON();
    wait(5);
  }

}



void rolling_measure() {

  target_OD = 0;
  Target_fluoro[0] = 1000;
  Target_fluoro[1] = 1000;

  stir(10);
  PMT_rolling_measurements(0);
  stir(10);
  OD_signal_read();
}

void OD_calib() {

  target_OD = 0;
  OD_signal_read();
  int it = 0;
  while ( it < 5 ) {
    stir(5);
    //OD_calib();
    it++;
  }

}

void hold_temp() {
  temperature_measurements_and_set_peltier();
  wait(10);
}



//--------------------------------------------------------------modes--------------------------------------------------------------


// the loop routine runs over and over again forever:
void loop() {
  //--------------------------------------------
  //Choose one function to set turbidostat mode:
  //--------------------------------------------
  //turbidostat(target_OD);
  //turbidostat_cyano(target_OD);
  //fluorostat(Fluorostat_target_channel,Target_fluoro[Fluorostat_target_channel]);//(channel (0 or 1), gain, target reading)
  //Pump_for_Exp_Start();
  //rolling_measure();
  //ODbatch();
  //Thorlab();
  //tbatch();
  //OD_calib();
  //
  //pump_out(5);
  //pump_in(5);
  //wait(10);
  //ODbatch_light();
  //ODbatch_light_LD(starttime, transtime, delaytime);
  //ODbatch_light_air();
  //OD_signal_read();
  //ODbatch_light_air_PMT();
  //AirPin_ON();
  //digitalWrite(AirPin, HIGH);
  //hold_temp();
  //turbidostat_cyano_air_LD(target_OD,starttime,delaytime,counter);
  //turbidostat_cyano_air(target_OD,counter);

  //chemostat_cyano_air(chem_pump_interval,chem_starttime);
  //--------------------------------------------

  //Test code:commented out
  ////
  //  AirPin_ON();
  //  delay(5000);
  //  AirPin_OFF();
  //  delay(5000);
  //  wait(30);
  //  LevelLED_ON();

  //pump_out(5);

  //wait(10);
  OD_signal_read();
  //AirPin_ON();
  //wait(10);
  //Serial.println();
  //Serial.println("Start Time: ");
  //Serial.println(starttime);
  //Serial.println("Photoperiod Length:");
  //Serial.println(transtime);
  //Serial.println("Time till First Night: ");
  //Serial.println(delaytime);
  //timeleft = timetillevent(starttime, transtime, delaytime);
  //
  //if (timeleft < 1000){
  //    starttime = millis();
  //    delaytime = transtime;
  //}
  //

  //timeleft = countdown(chem_starttime, chem_pump_interval);
  //Serial.print(timeleft);
  //if (timeleft < 1000){
  //  chem_starttime = millis();
  //}

  //delay(3000);

  //ExciteLED_OFF(0);

  //  AirPin_ON();
  //  delay(5000);
  //
  //  AirPin_OFF();
  //delay(60000);
  //ExciteLED_SET(0,255);
  //delay(3000);
  //digitalWrite(AirPin, HIGH);
  //PMT_rolling_measurements(0);
  //delay(3000);
  //stir(5);
  //Stir_ON();
  //delay(10000);
  //toarduino();
  //temperature_measurements_and_set_peltier();
  //Heater_ON();
  //  ODLED_ON();
  //  AirPin_ON();
  //LevelLED_OFF();
  //PumpIn_ON();
  //wait(5);
  //PumpIn_OFF();
  //PumpOut_ON();
  //wait(15);
  //PumpOut_OFF();

  //Heater_OFF();
  //LevelLED_ON();
}
