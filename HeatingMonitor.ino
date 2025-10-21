#include <DS3232RTC.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <I2C_eeprom.h>

#define FW_VERSION                "V2.05"
// declaration of input pin for the sensor module
#define PIN_TEMP_SENSORS_HEAT     6
#define PIN_TEMP_SENSOR_OUT       3
#define PIN_GPIO_BURNER           4
#define PIN_GPIO_VALVE            5 // not used in this version
#define PIN_GPIO_WATER            2

// declaration for loop interval
#define LOOP_INTERVAL             250
#define OP_DIVIDER                4

// constants declarations
#define UINT_MAX_VAL              0xFFFFu
#define PULSES_PER_LITER          75

#define NO_OF_EE_BLOCKS           32
#define CALC_BLOCK_ADR(b)         (b*256)
#define CALC_RECORD_ADR(b, r)     ((b*256)+sizeof(pageHdr)+(r*10))
#define CALC_DATE_VALUE(y,m,d)    (((y-2020)<<9) + (m<<5) + d)
#define INCR_BLOCK_IDX(idx)       {idx=(idx+1)%NO_OF_EE_BLOCKS;}
#define CALC_TEMP_AVR(temp)       (int8_t)((temp.cntVal>0) ? ((temp.tempCum*2) + temp.cntVal) / (2 * temp.cntVal) : 0);

#define SIZE_SERIAL_READ_BUFFER   0x20
#define SBP_INCR(pos)             {pos=(pos+1)%SIZE_SERIAL_READ_BUFFER;}

// global objects
OneWire   HMON_oneWireHeat(PIN_TEMP_SENSORS_HEAT);
OneWire   HMON_oneWireOut(PIN_TEMP_SENSOR_OUT); 
DS3232RTC HMON_rtc;
I2C_eeprom HMON_ee(0x50, I2C_DEVICESIZE_24LC64);
DallasTemperature HMON_tSensorCtrl(&HMON_oneWireHeat);

const struct SensorConfig {
  OneWire *onewire;
  int     index;
} HMON_tSensorConfig[] = {
  { &HMON_oneWireHeat, 0 },
  { &HMON_oneWireHeat, 1 },
  { &HMON_oneWireOut , 0 }
};
#define CNT_OF_TEMP_SENSORS       (sizeof(HMON_tSensorConfig)/sizeof(struct SensorConfig))

// type definitions
typedef union {
  struct {
    uint16_t  year;
    uint8_t   month;
    uint8_t   day;
    uint16_t  burnerStarts;
    uint16_t  burningTime;
    uint16_t  waterVolume;
  } s;
  struct {
    int8_t    dateVal[4];
    uint16_t  countVal[3];
  } t;
  struct {
    uint16_t  year;
    uint8_t   month;
    uint8_t   day;
  } dt;
  uint8_t   data[16];
} pageHdr;

typedef union {
  struct {
    int8_t    tempOutside;
    int8_t    tempAvrOut;
    int8_t    tempAvrIn;
    int8_t    tempMaxOut;
    int8_t    tempMaxIn;
    int8_t    tempMinOut;
    int8_t    tempMinIn;
    uint8_t   burnerStarts;
    uint8_t   burningTime;
    uint8_t   waterVolume;
  } s;
  struct {
    int8_t    tempVal[7];
    uint8_t   countVal[3];
  } t;
  uint8_t   data[10];
} dataRec;

/////////////////////////////////////////////////////////////
// global control structures
struct {
  uint16_t      currBlockIdx;
  time_t        tsNow         = 0;
  time_t        tsLastProc    = 0;
  bool          logEnabled = false;
  bool          readValues = false;
  pageHdr       hdr;
} HMON_ctrl;

// global variable for water counter updated by interrupt
volatile unsigned int HMON_waterPulses = 0;

/////////////////////////////////////////////////////////////
// interrupt service routine for water counter
void isrIncrWater() {
  HMON_waterPulses++;
}

/////////////////////////////////////////////////////////////
// UI helper routines
String formatNumber(int num, unsigned int digits = 2) {
  const String strPrefix = "00000000";
  String strNum = String(num);
  int offset = ((strNum.length() >= digits) || digits>strPrefix.length())
                ? 0
                : strPrefix.length()-(digits-strNum.length());
  return (offset > 0) ? String(strPrefix.substring(offset) + strNum)
                      : strNum;
}

void printDateAndTime(const char* title, time_t ts) {
  Serial.print(String(title + formatNumber(day(ts)) + "." + formatNumber(month(ts)) + "." + String(year(ts))));
  Serial.println(String(" " + formatNumber(hour(ts)) + ":" + formatNumber(minute(ts)) + ":" + formatNumber(second(ts))));
}

bool scanChar(char& scannedChar, const char *serialBuffer, unsigned int& readPos, const char *charList) {
  bool retCode = false;
  const char* ch = charList;
  
  while(*ch != '\0') {
    if(serialBuffer[readPos] == *ch) {
      scannedChar = *ch;
      SBP_INCR(readPos);
      retCode = true;
      break;
    }
    ch++;
  };
  return retCode;
}

bool scanDelimiter(char& delimiter, const char *serialBuffer, unsigned int& readPos) {
  if(!isprint(serialBuffer[readPos])) {
    delimiter = serialBuffer[readPos];
    SBP_INCR(readPos);
    return true;
  }
  return scanChar(delimiter, serialBuffer, readPos, ";.");
}

String readOpCmd(const char *serialBuffer, unsigned int& readPos, unsigned int& writePos) {
  String strOpCmd;
  while(readPos != writePos) {
    const char ch = serialBuffer[readPos];
    SBP_INCR(readPos)
    if(isprint(ch)) {
      if(ch == ';') {
        break;
      } else if(ch == '.') {
        readPos = writePos;
        break;
      }
      strOpCmd += ch;
    }
  }
  return strOpCmd;
}

bool checkTimeParameter(tmElements_t& newTime, String strTimeParam) {
  if(strTimeParam.length() != 14) {
    return false;
  }
  for(int i=0; i<14; i++) {
    if(!isdigit(strTimeParam.charAt(i))) {
      return false;
    }
  }
  newTime.Year   = strTimeParam.substring( 0, 4).toInt() - 1970;
  newTime.Month  = strTimeParam.substring( 4, 6).toInt();
  newTime.Day    = strTimeParam.substring( 6, 8).toInt();
  newTime.Hour   = strTimeParam.substring( 8,10).toInt();
  newTime.Minute = strTimeParam.substring(10,12).toInt();
  newTime.Second = strTimeParam.substring(12,14).toInt();
  return ((newTime.Month>=1) && (newTime.Month<=12)  && (newTime.Day>=1) && (newTime.Day<=31) &&
          (newTime.Hour<=23) && (newTime.Minute<=59) && (newTime.Second<=59));
}

unsigned int processOpCmdTime(const char *serialBuffer, unsigned int readPos, unsigned int writePos) {
  String strTimeCmd = readOpCmd(serialBuffer, readPos, writePos);
  tmElements_t newTime;

  if(strTimeCmd.length() == 1) {
    printDateAndTime("date and time: ", now());
  } else if(checkTimeParameter(newTime, strTimeCmd.substring(1))) {
    time_t tsNew = makeTime(newTime);
    setTime(tsNew);
    tsNew = now();
    HMON_rtc.set(tsNew);
    printDateAndTime("new time: ", tsNew);
  } else {
    Serial.println(F("syntax error"));
  }
  return readPos;
}

void processOperatorCommand() {
  static char serialBuffer[SIZE_SERIAL_READ_BUFFER];
  static unsigned int writePos=0;
  static unsigned int readPos=0;
  bool terminatorReceived = false;

  // read pending bytes on serial interface
  while(Serial.available() > 0) {
    char ch = Serial.read();
    if(!isprint(ch)) {
      ch = '.';
    }
    if(!terminatorReceived) {
      serialBuffer[writePos] = ch;
      if(ch != '.') {
        Serial.print(String(ch));
      } else {
        terminatorReceived = true;
        if(writePos != readPos) {
          Serial.println("");
        }
      }
      SBP_INCR(writePos)
    }
    if(writePos == readPos) {
      // buffer overflow, move read pointer
      SBP_INCR(readPos)
    }
  }

  // process all received bytes
  while((readPos != writePos) && terminatorReceived) {
    char delimiter = '\0';
    if(scanDelimiter(delimiter, serialBuffer, readPos)) {
      continue;
    }
    switch(serialBuffer[readPos]) {
      case 't':
        // process time command
        readPos = processOpCmdTime(serialBuffer, readPos, writePos);
        break;
      case 'l':
        // enable/disable screen logging
        HMON_ctrl.logEnabled = (HMON_ctrl.logEnabled) ? false : true;
        SBP_INCR(readPos)
        break;
      case 'r':
      case 'd':
        // set flag to read stored values
        HMON_ctrl.readValues = true;
        SBP_INCR(readPos)
        break;
      default:
        SBP_INCR(readPos)
        break;
    }
  }
}

/////////////////////////////////////////////////////////////
// temperature sensor read routine
void readTemperatures(int *tbuffer, const struct SensorConfig *sensors, int numOfSensors) {
  OneWire *onewire = NULL;

  memset(tbuffer, 0, numOfSensors);
  for(int i=0; i<numOfSensors; i++) {
    if(sensors[i].onewire != onewire) {
      HMON_tSensorCtrl.setOneWire(sensors[i].onewire);
      HMON_tSensorCtrl.requestTemperatures();
    }
    tbuffer[i] = (int)(HMON_tSensorCtrl.getTempCByIndex(sensors[i].index)+0.5);
  }
}

/////////////////////////////////////////////////////////////
// EEPROM read and write routines
void printEepromContent() {
  pageHdr hdr;
  dataRec rec;
  for(uint16_t b=0; b<NO_OF_EE_BLOCKS; b++) {
    // read header
    HMON_ee.readBlock(CALC_BLOCK_ADR(b), hdr.data, sizeof(hdr));
    // print header
    Serial.print(F("-----------------------------\r\nRecording Date: "));
    Serial.println(formatNumber(hdr.s.day) + "." + formatNumber(hdr.s.month) + "." + String(hdr.s.year));

    Serial.println(F("Starts;BurningTime;Water;"));
    for(int i=0; i<sizeof(hdr.t.countVal)/sizeof(hdr.t.countVal[0]); i++) {
      Serial.print(hdr.t.countVal[i]);
      Serial.print(";");
    }

    Serial.println(F("\r\ntempOutside;tempAvrOut;tempAvrIn;tempMaxOut;tempMaxIn;tempMinOut;tempMinIn;burnerStarts;burningTime;waterVolume;"));
    // read record
    for(uint16_t r=0; r<24; r++) {
      // read data record
      HMON_ee.readBlock(CALC_RECORD_ADR(b, r), rec.data, sizeof(rec));
      // print hour temperature value
      for(int i=0; i<sizeof(rec.t.tempVal)/sizeof(rec.t.tempVal[0]); i++) {
        Serial.print(rec.t.tempVal[i]);
        Serial.print(";");
      }
      // print hour counter value
      for(int i=0; i<sizeof(rec.t.countVal)/sizeof(rec.t.countVal[0]); i++) {
        Serial.print(rec.t.countVal[i]);
        Serial.print(";");
      }
      Serial.print("\r\n");
    }
  }
}

void cleanupBlockAndSetDate(pageHdr hdr, uint16_t idx, uint16_t year, uint8_t month, uint8_t day) {
  dataRec rec;
  // cleanup header  and set current date
  memset(hdr.data, 0, sizeof(hdr));
  hdr.s.year  = year;
  hdr.s.month = month;
  hdr.s.day   = day;
  // write header to EEPROM
  HMON_ee.writeBlock(CALC_BLOCK_ADR(idx), hdr.data, sizeof(hdr.data));
  
  // clean all 24 records in this block
  memset(rec.data, 0, sizeof(rec.data));
  for(uint16_t r=0; r<24; r++) {
    // clean data record
    HMON_ee.writeBlock(CALC_RECORD_ADR(idx, r), rec.data, sizeof(rec));
  }
}

uint16_t evalPageIndexAndInitBlock(uint16_t year, uint8_t month, uint8_t day) {
  pageHdr hdr;
  uint16_t idx = 0;
  uint16_t dateVal = 0xFFFF;
  uint16_t currDateVal = CALC_DATE_VALUE(year, month, day);

  for(uint16_t b=0; b<NO_OF_EE_BLOCKS; b++) {
    // read header
    HMON_ee.readBlock(CALC_BLOCK_ADR(b), hdr.data, sizeof(hdr.dt));
    if((hdr.dt.year>=2020) && (hdr.dt.year<=2099) &&
       (hdr.dt.month>=1) && (hdr.dt.month<=12) &&
       (hdr.dt.day>=1) && (hdr.dt.day<32)) {
      if(CALC_DATE_VALUE(hdr.dt.year, hdr.dt.month, hdr.dt.day) == currDateVal) {
        // current day already exists, return block index without cleaning up the block
        return b;
      } else if(CALC_DATE_VALUE(hdr.dt.year, hdr.dt.month, hdr.dt.day) < dateVal) {
        dateVal = CALC_DATE_VALUE(hdr.dt.year, hdr.dt.month, hdr.dt.day);
        idx = b;
      }
    } else {
      // block date is invalid ==> use this block
      idx = b;
      break;
    }
  }
  cleanupBlockAndSetDate(HMON_ctrl.hdr, idx, year, month, day);
  return idx;
}

/*******************************************************************************
 * procedure setup()
 */
void setup() {
  // initialisation of serial port
  Serial.begin(57600);
  Serial.print("Heating Monitor ");
  Serial.println(FW_VERSION);

  HMON_ee.begin();
  if (!HMON_ee.isConnected())
  {
    Serial.println("ERROR: Can't find eeprom ...");
  }
  
  // set provider for time syncing from RTC to local timer
  setSyncProvider(HMON_rtc.get);
  setSyncInterval(600);  // synchronize every 10 minutes

  // initialisation of sensor
  HMON_tSensorCtrl.begin();
  // initialisation of GPIOs
  pinMode(LED_BUILTIN,     OUTPUT);
  pinMode(PIN_GPIO_BURNER, INPUT_PULLUP);
  pinMode(PIN_GPIO_VALVE,  INPUT_PULLUP);
  pinMode(PIN_GPIO_WATER, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_GPIO_WATER), isrIncrWater, FALLING);

  // initialize control structure
  HMON_ctrl.tsNow = now();
  if(HMON_ctrl.tsNow< 1672574400) {
    // date and time not set, set to 01.01.2023 12:00:00
    processOpCmdTime("t20230101120000", 0, 15);
  }
  HMON_ctrl.tsLastProc = HMON_ctrl.tsNow;
  HMON_ctrl.currBlockIdx = evalPageIndexAndInitBlock(year(HMON_ctrl.tsNow), month(HMON_ctrl.tsNow), day(HMON_ctrl.tsNow));
}

/*******************************************************************************
 * procedure loop
 */
void loop() {
  static int divider = OP_DIVIDER - 1;
  static int waterCounter = 0;
  static int measures[CNT_OF_TEMP_SENSORS];
  static struct {
    bool cleanuptemp     = true;
    int burnerStarts     = 0;
    int burningTime      = 0;
    int burnerStartsHour = 0;
    int burningTimeHour  = 0;
    int burningTimeCurr  = 0;
    int burningState     = digitalRead(PIN_GPIO_BURNER);
    unsigned int waterPulsesLast = 0;
    unsigned int waterPulsesCurr = 0;
    unsigned int waterVolumeHour = 0;
    unsigned int waterVolumeDay  = 0;
  } mctrl;

  // check for operator command over serial interface (Arduino serial monitor)
  processOperatorCommand();

  if(HMON_ctrl.readValues) {
    // print EEPROM content
    printDateAndTime("Dump from ", HMON_ctrl.tsNow);
    printEepromContent();
    HMON_ctrl.readValues = false;
  }

  divider = (divider + 1) % OP_DIVIDER;
  if(divider == 0) {
    static bool logEnabledLast = false;
    //////////////////////////////////////////////////////////////////////
    // second handler
    if(HMON_ctrl.logEnabled && !logEnabledLast) {
      Serial.println("t1,t2,t3,burner,valve,water,bStartH,bTimeH,PLast,PFrag,WPulses");
    }
    logEnabledLast = HMON_ctrl.logEnabled;
    if(HMON_ctrl.logEnabled) {
      // start temperature measurement
      readTemperatures(measures, HMON_tSensorConfig, CNT_OF_TEMP_SENSORS);
      for(int i=0; i<CNT_OF_TEMP_SENSORS; i++) {
        Serial.print(measures[i]);
        Serial.print(", ");
      }
      Serial.print(digitalRead(PIN_GPIO_BURNER));
      Serial.print(", ");
      Serial.print(digitalRead(PIN_GPIO_VALVE));
      Serial.print(", ");
      Serial.print(digitalRead(PIN_GPIO_WATER));
      Serial.print(", ");
      Serial.print(mctrl.burnerStartsHour);
      Serial.print(", ");
      Serial.print(mctrl.burningTimeHour);
      Serial.print(", ");
      Serial.print(mctrl.waterPulsesLast);
      Serial.print(", ");
      Serial.print(mctrl.waterPulsesCurr);
      Serial.print(", ");
      Serial.println(HMON_waterPulses);
    }
    
    // animate heartbeat
    static int blinkState = LOW;
    blinkState = (blinkState == HIGH) ? LOW : HIGH;
    digitalWrite(LED_BUILTIN, blinkState);
    // get current timestamp
    HMON_ctrl.tsNow = now();
 
    //////////////////////////////////////////////////////////////////////
    // action handler for every minute
    if(minute(HMON_ctrl.tsNow) != minute(HMON_ctrl.tsLastProc)) {
      static struct {
        int  tempCum;
        int  cntVal;
        char tMin;
        char tMax;
      } temp[CNT_OF_TEMP_SENSORS];
      
      if(mctrl.cleanuptemp) {
        // initialize static structure
        memset(temp, 0, sizeof(temp));
        mctrl.cleanuptemp = false;
      }
      // read the temparature for all sensors
      readTemperatures(measures, HMON_tSensorConfig, CNT_OF_TEMP_SENSORS);
      for(int i=0; i<CNT_OF_TEMP_SENSORS; i++) {
        if(abs(measures[i]) <= 120) {
          temp[i].tMin = ((temp[i].tMin > (char)measures[i]) || (temp[i].cntVal==0)) ? (char)measures[i] : temp[i].tMin;
          temp[i].tMax = ((temp[i].tMax < (char)measures[i]) || (temp[i].cntVal==0)) ? (char)measures[i] : temp[i].tMax;
          temp[i].tempCum += measures[i];
          temp[i].cntVal++;
        }
      }

      //////////////////////////////////////////////////////////////////////
      // get burning state
      int bs = digitalRead(PIN_GPIO_BURNER);
      mctrl.burningTime     += ((bs == mctrl.burningState  ) && (bs == LOW )) ? 1 : 0;
      mctrl.burningTimeCurr += ((bs == mctrl.burningState  ) && (bs == LOW )) ? 1 : 0;
      mctrl.burnerStarts    += ((mctrl.burningState == HIGH) && (bs == LOW )) ? 1 : 0;
      if((mctrl.burningState == LOW ) && (bs == HIGH)) {
        mctrl.burnerStartsHour++;
        mctrl.burningTimeHour += mctrl.burningTimeCurr;
        mctrl.burningTimeCurr = 0;
      }
      if(HMON_ctrl.logEnabled && (bs != mctrl.burningState)) {
        Serial.print("burner: time=");
        Serial.print(mctrl.burningTime);
        Serial.print(", starts=");  
        Serial.println(mctrl.burnerStarts);  
      }
      mctrl.burningState = bs;
      //////////////////////////////////////////////////////////////////////
      // get water pulses
      unsigned int waterPulses[2] = { 0, 1 };
      unsigned int waterPulseDiff = 0;
      for(int i=0; (i<3) && (waterPulses[0]!=waterPulses[1]); i++) {
        waterPulses[0] = HMON_waterPulses;
        waterPulses[1] = HMON_waterPulses;
      }
      if(waterPulses[0] == waterPulses[1]) {
        if(waterPulses[0] >= mctrl.waterPulsesLast) {
          waterPulseDiff = waterPulses[0] - mctrl.waterPulsesLast;
        } else {
          waterPulseDiff = (UINT_MAX_VAL - mctrl.waterPulsesLast) + waterPulses[0] + 1;
        }
        mctrl.waterPulsesLast = waterPulses[0];
        mctrl.waterPulsesCurr += waterPulseDiff;
        mctrl.waterVolumeHour += (mctrl.waterPulsesCurr / PULSES_PER_LITER);
        // limit water volume 
        if(mctrl.waterVolumeHour > 255) {
          mctrl.waterVolumeHour = 255;
        }
        mctrl.waterVolumeDay  += mctrl.waterPulsesCurr / PULSES_PER_LITER;
        // calculate liter fragment
        mctrl.waterPulsesCurr %= PULSES_PER_LITER;
      }
      //////////////////////////////////////////////////////////////////////
      // action handler for every hour
      if(hour(HMON_ctrl.tsNow) != hour(HMON_ctrl.tsLastProc)) {
        dataRec rec;
        rec.s.tempOutside = CALC_TEMP_AVR(temp[2]);
        rec.s.tempAvrOut  = CALC_TEMP_AVR(temp[0]);
        rec.s.tempAvrIn   = CALC_TEMP_AVR(temp[1]);
        rec.s.tempMaxOut  = temp[0].tMax;
        rec.s.tempMinOut  = temp[0].tMin;
        rec.s.tempMaxIn   = temp[1].tMax;
        rec.s.tempMinIn   = temp[1].tMin;
        rec.s.burnerStarts = (uint8_t)mctrl.burnerStartsHour;
        rec.s.burningTime  = (uint8_t)mctrl.burningTimeHour;
        rec.s.waterVolume  = (uint8_t)mctrl.waterVolumeHour;
        HMON_ee.writeBlock(CALC_RECORD_ADR(HMON_ctrl.currBlockIdx, hour(HMON_ctrl.tsLastProc)), rec.data, sizeof(rec));
        // clear hour counter
        mctrl.burnerStartsHour = 0;
        mctrl.burningTimeHour  = 0;
        mctrl.waterVolumeHour  = 0;
        
        // action handler for midnight
        if(day(HMON_ctrl.tsNow) != day(HMON_ctrl.tsLastProc)) {
          HMON_ctrl.hdr.s.year  = year(HMON_ctrl.tsLastProc);
          HMON_ctrl.hdr.s.month = month(HMON_ctrl.tsLastProc);
          HMON_ctrl.hdr.s.day   = day(HMON_ctrl.tsLastProc);
          HMON_ctrl.hdr.s.burnerStarts = mctrl.burnerStarts;
          HMON_ctrl.hdr.s.burningTime  = mctrl.burningTime;
          HMON_ctrl.hdr.s.waterVolume  = mctrl.waterVolumeDay;
          // write header to EEPROM
          HMON_ee.writeBlock(CALC_BLOCK_ADR(HMON_ctrl.currBlockIdx), HMON_ctrl.hdr.data, sizeof(HMON_ctrl.hdr.data));
          INCR_BLOCK_IDX(HMON_ctrl.currBlockIdx);
          cleanupBlockAndSetDate(HMON_ctrl.hdr, HMON_ctrl.currBlockIdx, year(HMON_ctrl.tsNow), month(HMON_ctrl.tsNow), day(HMON_ctrl.tsNow));

          // cleanup local counter
          mctrl.burnerStarts = (mctrl.burningState == LOW) ? 1 : 0;
          mctrl.burningTime  = 0;
          mctrl.waterVolumeDay = 0;
        } // if(day(HMON_ctrl.tsNow) != day(HMON_ctrl.tsLastProc))
        // cleanup local temparature array
        mctrl.burningTimeHour  = 0;
        mctrl.burnerStartsHour = 0;
        mctrl.cleanuptemp = true;
      } // if(hour(HMON_ctrl.tsNow) != hour(HMON_ctrl.tsLastProc))

      // mark minute as processed
      HMON_ctrl.tsLastProc = HMON_ctrl.tsNow;
      Serial.flush();
    } // if(minute(HMON_ctrl.tsNow) != minute(HMON_ctrl.tsLastProc))
  }

  delay(LOOP_INTERVAL);
}
