// CHANGES IN V5:
// Functions modified: turbidostat_cyano_air() and turbidostat_cyano_airLD()
// Adds a brief bubbling step after pumping in new media and
// before measuring the OD again. Ensuring the media is well mixed.

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

enum Err_flag { info, warning, error };

enum LD_mode { LD, binary, sinusoid };

//-------------------------------------------------------
// Manually set variables
//-------------------------------------------------------
float temperature_SetTo = 30; //Temperature Thresh
// int target_OD = 647; //OD voltage set point for turbidostat
int Fluorostat_target_channel = 0;
int Fluorostat_target_gain = 36; //26,31,36,41,46,51
//int Fluorostat_target=200;
int Target_fluoro[2] = {200, 200};

//LD Variables---------------
// long delaytime = ((3L * 60L + 40L) * 1000L * 60L); //num mins to night   // maybe change later
//(hr to night x conversion from hr to min + min to night)x conversion from mils to secs * conversion from secs to mins
// long daylength = (12L * 1000L * 3600L);
// long nightlength = (12L * 1000L * 3600L);
// long transtime = 0; // if the lights are on initially then this should be equal to the daylength. I made a boolean check set this in setup, so the value here doesn't really matter.
//num hours for day and night x conversion froms mils to secs x conversion secs to hours
// int num_skips = 4; //If you don't want to take fluoroscence measurements every time define this
// int current_skip = 0;

//Chemostat Variables--------
// long chem_starttime = millis();
// long chem_pump_interval = (2 * 1000 * 3600L);
//-------------------------------------------------------

//for when the media runs out, counter of how many times the pumps have gone on initialize
// int counter = 0;

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
float LastODRead = 0;
int LastTempRead = 0;
int LastLiquidLevel = 0;

int ODLEDStatus = 0;
int StirStatus = 0;
int HeaterStatus = 0;
int PumpInStatus = 0;
int PumpOutStatus = 0;
int LevelLEDStatus = 1;    // default to dark

//int incomingByte = 0; // for incoming serial data
//bool LightsOn = true; //whether the lights are on or not
long starttime = millis();
long timeleft = 0;

const float TARGET_OD_VOLT_RATIO = 380.0;
const int AIR_PUMP_OFF_DELAY = 10;    // seconds
const int AIR_PUMP_MIXING_DELAY = 60;    //seconds
const int MEASURE_INTERVAL = 900;    // seconds
const int PUMP_OUT_DURATION = 20;    // seconds
const int PUMP_IN_DURATION = 10;    // seconds
const float OD_CHANGE_THRESHOLD = 0.05;

unsigned long last_timestamp = 0;
unsigned long last_pump_in = -1000000000;

LD_mode mode = binary;
// The two sets of parameters are mutually exclusive
// E.g. if mode == LD, LD_VEC will be ignored

// After a light period of $first_day_length,
// it'll go through DL cycles with the durations set by
// $night_length and $day_length
const int DAY_LENGTH = 5;    // minutes
const int NIGHT_LENGTH = 5;    // minutes
const int FIRST_DAY_LENGTH = 5;    // minutes

const int N_STEPS = 5;
const char LD_VEC[N_STEPS + 1] = "LLDLD";
const int LENGTH_VEC[N_STEPS] = {5*60, 24*60, 12*60, 12*60, 12*60};    // minutes
// how long does it have to wait to begin LD step i?
int wait_vec[N_STEPS];

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
  //Heater_OFF();
  //stir(5);
  //delay(5000);

  ExciteLED_OFF(0);
  //OD_calib();
  //Heater_OFF();

  /*if (LightsOn) { // Only relevant for turbidostat_cyano_airLD
    transtime = daylength; // There is probably a better way to do this but here is a temporary fix for specifying photoperiod - AFS 23/04/25
  }
  else {
    transtime = nightlength;
  }*/

  // fill wait_vec
  if (mode == binary) {
    wait_vec[0] = LENGTH_VEC[0];
    for (int i = 1; i < N_STEPS; i++) {
      wait_vec[i] = wait_vec[i - 1] + LENGTH_VEC[i];
    }
  }

  msg("Machine online", info);
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

void Heater_ON() {
  if (!HeaterStatus) {
    //send_data();
  }
  digitalWrite(HeaterPin, LOW);
  HeaterStatus = 1;
  delay(100);
}
void Heater_OFF() {
  if (HeaterStatus) {
    //send_data();
  }
  digitalWrite(HeaterPin, HIGH);
  HeaterStatus = 0;
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
  /*if (!LevelLEDStatus) {
    //msg("LED switched Off", info);
    //send_data();
  }
  LevelLEDStatus = 1;*/
  delay(100);
}
void LevelLED_OFF() {
  digitalWrite(LevelLEDPin, LOW);
  /*if (LevelLEDStatus) {
    msg("LED switched on", info);
    //send_data();
  }
  LevelLEDStatus = 0;*/
  delay(100);
}


void AirPin_ON() {
  digitalWrite(AirPin, HIGH);
  delay(100);
  StirStatus = 1;
  //send_data();
  delay(100);

}

void AirPin_OFF() {
  digitalWrite(AirPin, LOW);
  delay(100);
  StirStatus = 0;
  //send_data();
  delay(100);
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
void PMT_rolling_measurements(int which) {
  for (int k = 0; k < PMTGainCap[which] + 1; k++) {
    set_PMT_from_internal(which);//rolling PMT
    PMTSet_measurements(which, LEDGain[which], PMTGain[which]);
    delay(1000);
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
                   TARGET_OD_VOLT_RATIO, ODRead, temperature_SetTo, thermocouple_celcius, ODLEDStatus, StirStatus,
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

void msg(const char* message, Err_flag flag) {
  Serial.print(-1);    // flag for message
  Serial.print(" ");
  Serial.print(flag);
  Serial.print(" ");
  Serial.print(message);
  Serial.println();
}

float get_light_intensity() {
  return 0.0;
}

bool is_light() {
  unsigned long cur_t = millis();

  if (mode == LD) {
    if (cur_t <= FIRST_DAY_LENGTH*60000) {
      return true;
    } else {
      unsigned long remainder = (cur_t - FIRST_DAY_LENGTH*6000)%(DAY_LENGTH*60000 + NIGHT_LENGTH*60000);
      return remainder > NIGHT_LENGTH*60000;
    }
  } else if (mode == binary) {
    for (int i = 0; i < N_STEPS; i++) {
      if (cur_t < wait_vec[i]*60000) {
        return LD_VEC[i] == 'L';
      }
    }
    return true;     // keep lights on after the program
  } else {
    // this function will not be called in analog mode
    return false;
  }
}

void update_light() {
  // set the status of the LED ring
  if (mode == sinusoid) {
    // TODO: "continuously" tuning the brightness
  } else {
    bool light_status = is_light();
    if (light_status) {
      if (LevelLEDStatus) {
        msg("LED switched on", info);
        send_data();
      }
      LevelLED_OFF();    // turn lights on
      LevelLEDStatus = false;
    } else {
      if (!LevelLEDStatus) {
        msg("LED switched off", info);
        send_data();
      }
      LevelLED_ON();    // turn lights off
      LevelLEDStatus = true;
    }
  }
}

void turbidostat(int PMT_id) {
  // PMT_id == -1 means off

  // housekeeping
  update_light();
  AirPin_ON();
  temperature_measurements_and_set_peltier();

  unsigned long cur_time = millis();
  if (cur_time - last_timestamp > MEASURE_INTERVAL*1000L) {
    // update timestamp
    last_timestamp += MEASURE_INTERVAL*1000L;

    // measure OD and fluorescence
    AirPin_OFF();    // turn off bubbling
    delay(AIR_PUMP_OFF_DELAY*1000L);
    LevelLED_ON();    // turn off LED ring

    if (PMT_id >= 0) {
      PMT_rolling_measurements(PMT_id);
    }

    OD_signal_read();

    // Higher OD gives lower volt ratio
    if (LastODRead < TARGET_OD_VOLT_RATIO) {
      update_light();

      // back-to-back dilutions?
      if (cur_time - last_pump_in < 2*MEASURE_INTERVAL*1000L) {
        msg("Consecutive dilutions. You may want to increase "
        "pump-in duration or shortern measure interval.", warning);
      } else {
        msg("Being diluted", info);
      }
      last_pump_in = cur_time;    //update

      // dilute
      pump_out(PUMP_OUT_DURATION);    // make sure brings back the total vol
      pump_in(PUMP_IN_DURATION);
      pump_off();    // turn off all pumps

      // Get the OD immediately after the dilution
      // will be useful in estimating the dilution factor
      // Bubble to mix
      AirPin_ON();
      delay(AIR_PUMP_MIXING_DELAY*1000L);
      AirPin_OFF();

      float previous_OD = LastODRead;
      LevelLED_ON();    // turn off lights
      OD_signal_read();
      float frac_change = abs(LastODRead - previous_OD)/previous_OD;
      if (frac_change < OD_CHANGE_THRESHOLD) {
        msg("Dilution failed. Immediately check for pump failure, liquid overflow, "
        "sensor failure, or biofilm build-up", error);
      }
    }
  }
}

void pump_test() {
  update_light();
  temperature_measurements_and_set_peltier();

  AirPin_ON();
  delay(10000);
  AirPin_OFF();

  pump_out(PUMP_OUT_DURATION);    // make sure brings back the total vol
  pump_in(PUMP_IN_DURATION);
}

void sensor_test(int PMT_id) {
  AirPin_OFF();    // turn off bubbling
  LevelLED_ON();    // turn off LED ring

  if (PMT_id >= 0) {
    PMT_rolling_measurements(PMT_id);
  }

  OD_signal_read();
}


//--------------------------------------------------------------modes--------------------------------------------------------------


// the loop routine runs over and over again forever:
void loop() {
  
  turbidostat(0);
}
