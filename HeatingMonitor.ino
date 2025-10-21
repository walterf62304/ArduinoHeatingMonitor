#include <DS3232RTC.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define FW_VERSION                "1.05"
// declaration of input pin for the sensor module
#define PIN_TEMP_SENSORS_HEAT     6
#define PIN_TEMP_SENSOR_OUT       3
#define PIN_GPIO_BURNER           4
#define PIN_GPIO_VALVE            5
#define PIN_GPIO_WATER            2

// declaration for loop interval
#define LOOP_INTERVAL             250
#define OP_DIVIDER                4

// constants declarations
#define UINT_MAX_VAL              0xFFFFu
#define PULSES_PER_LITER          75

#define NO_OF_HOUR_BLOCKS         32
#define NO_OF_WATER_BLOCKS        16
#define NO_OF_DAYS                2
#define INCR_IDX_HBLOCK(idx)      {idx=(idx+1)%NO_OF_HOUR_BLOCKS;}
#define INCR_IDX_WBLOCK(idx)      {idx=(idx+1)%NO_OF_WATER_BLOCKS;}
#define INCR_IDX_DBLOCK(idx)      {idx=(idx+1)%NO_OF_DAYS;}

#define SIZE_SERIAL_READ_BUFFER   0x20
#define SBP_INCR(pos)             {pos=(pos+1)%SIZE_SERIAL_READ_BUFFER;}

// global objects
OneWire   HMON_oneWireHeat(PIN_TEMP_SENSORS_HEAT);
OneWire   HMON_oneWireOut(PIN_TEMP_SENSOR_OUT); 
DS3232RTC HMON_rtc;
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
typedef struct {
  struct {
    char tMin;
    char tMax;
    char tAvr;
    char tCnt;
  } tempVal[CNT_OF_TEMP_SENSORS];
  int  burnerStartsHour;
  int  burningTimeHour;
  int  waterVolumeHour;
} hourBlock;

typedef struct {
  int  start;
  int  duration;
} waterHeating;

typedef struct {
  int  dayOfMonth;
  int  idxHourBStart;
  int  idxHourBEnd;
  int  idxWHeatStart;
  int  idxWHeatEnd;
  int  burnerStarts;
  int  burningTime;
  int  waterVolume;
} dayControl;

// global control structure
struct {
  int           idxCurrentDay = 0;
  time_t        tsNow         = 0;
  time_t        tsLastProc    = 0;
  bool          logEnabled = false;
  bool          readValues = false;

  hourBlock     hourBlocks[NO_OF_HOUR_BLOCKS];
  waterHeating  waterHeat[NO_OF_WATER_BLOCKS];
  dayControl    dayCtrl[NO_OF_DAYS];
} HMON_ctrl;
// global variable for water counter updated by interrupt
volatile unsigned int HMON_waterPulses = 0;

// interrupt service routine for water counter
void isrIncrWater() {
  HMON_waterPulses++;
}

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

/*******************************************************************************
 * procedure setup()
 */
void setup() {
  // clear memory blocks
  memset(HMON_ctrl.hourBlocks, 0, sizeof(HMON_ctrl.hourBlocks));
  memset(HMON_ctrl.waterHeat , 0, sizeof(HMON_ctrl.waterHeat ));
  memset(HMON_ctrl.dayCtrl   , 0, sizeof(HMON_ctrl.dayCtrl   ));

  // initialize control structure
  HMON_ctrl.tsNow = now();
  HMON_ctrl.tsLastProc = HMON_ctrl.tsNow;
  HMON_ctrl.dayCtrl[HMON_ctrl.idxCurrentDay].dayOfMonth  = day(HMON_ctrl.tsNow);
  HMON_ctrl.dayCtrl[HMON_ctrl.idxCurrentDay].idxHourBEnd = hour(HMON_ctrl.tsNow);

  // initialisation of serial port
  Serial.begin(57600);
  Serial.print("Heating Monitor V");
  Serial.println(FW_VERSION);

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
    int hotWaterTime     = 0;
    int hotWaterStart    = 0;
    int hotWaterState    = digitalRead(PIN_GPIO_VALVE);
    unsigned int waterPulsesLast = 0;
    unsigned int waterPulsesFrag = 0;
  } mctrl;

  // check for operator command over serial interface (Arduino serial monitor)
  processOperatorCommand();

  if(HMON_ctrl.readValues) {
    // transfer all stored data
    Serial.println(F("day control\ndayOfMon;idxHourBStart;idxHourBEnd;idxWHeatStart;idxWHeatEnd;burnerStarts;burningTime;waterVol;"));
    for(int i=0; i<NO_OF_DAYS; i++) {
      Serial.print(HMON_ctrl.dayCtrl[i].dayOfMonth);
      Serial.print(";");
      Serial.print(HMON_ctrl.dayCtrl[i].idxHourBStart);
      Serial.print(";");
      Serial.print(HMON_ctrl.dayCtrl[i].idxHourBEnd);
      Serial.print(";");
      Serial.print(HMON_ctrl.dayCtrl[i].idxWHeatStart);
      Serial.print(";");
      Serial.print(HMON_ctrl.dayCtrl[i].idxWHeatEnd);
      Serial.print(";");
      Serial.print(HMON_ctrl.dayCtrl[i].burnerStarts);
      Serial.print(";");
      Serial.print(HMON_ctrl.dayCtrl[i].burningTime);
      Serial.print(";");
      Serial.print(HMON_ctrl.dayCtrl[i].waterVolume);
      Serial.println(";");
    }
    Serial.println(F("\nwater heating\nidx;start;duration;"));
    for(int i=0; i<NO_OF_WATER_BLOCKS; i++) {
      Serial.print(i);
      Serial.print(";");
      Serial.print(HMON_ctrl.waterHeat[i].start);
      Serial.print(";");
      Serial.print(HMON_ctrl.waterHeat[i].duration);
      Serial.println(";");
    }
    Serial.println(F("\nhour blocks\nidx;tMin[n];tMax[n];tAvr[n];tCnt[n];bStartsHour;bTimeHour;waterVol"));
    for(int i=0; i<NO_OF_HOUR_BLOCKS; i++) {
      Serial.print(i);
      Serial.print(";");
      for(int j=0; j<CNT_OF_TEMP_SENSORS; j++) {
        Serial.print(int(HMON_ctrl.hourBlocks[i].tempVal[j].tMin));
        Serial.print((j<CNT_OF_TEMP_SENSORS-1) ? "," : ";");
      }
      for(int j=0; j<CNT_OF_TEMP_SENSORS; j++) {
        Serial.print(int(HMON_ctrl.hourBlocks[i].tempVal[j].tMax));
        Serial.print((j<CNT_OF_TEMP_SENSORS-1) ? "," : ";");
      }
      for(int j=0; j<CNT_OF_TEMP_SENSORS; j++) {
        Serial.print(int(HMON_ctrl.hourBlocks[i].tempVal[j].tAvr));
        Serial.print((j<CNT_OF_TEMP_SENSORS-1) ? "," : ";");
      }
      for(int j=0; j<CNT_OF_TEMP_SENSORS; j++) {
        Serial.print(int(HMON_ctrl.hourBlocks[i].tempVal[j].tCnt));
        Serial.print((j<CNT_OF_TEMP_SENSORS-1) ? "," : ";");
      }
      Serial.print(int(HMON_ctrl.hourBlocks[i].burnerStartsHour));
      Serial.print(";");
      Serial.print(HMON_ctrl.hourBlocks[i].burningTimeHour);
      Serial.print(";");
      Serial.print(HMON_ctrl.hourBlocks[i].waterVolumeHour);
      Serial.println(";");
    }
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
      Serial.print(mctrl.waterPulsesFrag);
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
      int dblkidx = HMON_ctrl.idxCurrentDay;
      int hblkidx = HMON_ctrl.dayCtrl[dblkidx].idxHourBEnd;
      int wblkidx = HMON_ctrl.dayCtrl[dblkidx].idxWHeatEnd;
      
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
      if(bs != mctrl.burningState) {
        Serial.print("burner: time=");
        Serial.print(mctrl.burningTime);
        Serial.print(", starts=");  
        Serial.println(mctrl.burnerStarts);  
      }
      mctrl.burningState = bs;
      //////////////////////////////////////////////////////////////////////
      // get water heating state
      int hs = digitalRead(PIN_GPIO_VALVE);
      if((hs == LOW) && (hs == mctrl.hotWaterState)) {
        mctrl.hotWaterTime++;
        //Serial.print("incr hotWaterTime: ");
        //Serial.println(mctrl.hotWaterTime);
      } else if((hs == LOW) && (mctrl.hotWaterState == HIGH)) {
        mctrl.hotWaterStart = hour(HMON_ctrl.tsNow) * 60 + minute(HMON_ctrl.tsNow);
        mctrl.hotWaterTime = 0;
        mctrl.hotWaterState = hs;
        Serial.print("hotWaterStart: ");
        Serial.println(mctrl.hotWaterStart);
      } else if((hs == HIGH) && (mctrl.hotWaterState == LOW)) {
        // the water heating is finished write entry to global structure
        HMON_ctrl.waterHeat[wblkidx].start    = mctrl.hotWaterStart;
        HMON_ctrl.waterHeat[wblkidx].duration = mctrl.hotWaterTime;
        INCR_IDX_WBLOCK(HMON_ctrl.dayCtrl[dblkidx].idxWHeatEnd);
        // check for wrapping
        if(HMON_ctrl.dayCtrl[dblkidx].idxWHeatStart == HMON_ctrl.dayCtrl[dblkidx].idxWHeatEnd) {
          // protect overwriting
          INCR_IDX_WBLOCK(HMON_ctrl.dayCtrl[dblkidx].idxWHeatStart);
        }
        Serial.print("hotWaterEnd, endidx: ");
        Serial.println(HMON_ctrl.dayCtrl[dblkidx].idxWHeatEnd);
        mctrl.hotWaterState = hs;
      }
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
        mctrl.waterPulsesFrag += waterPulseDiff;
        HMON_ctrl.hourBlocks[hblkidx].waterVolumeHour += mctrl.waterPulsesFrag / PULSES_PER_LITER;
        HMON_ctrl.dayCtrl[dblkidx].waterVolume        += mctrl.waterPulsesFrag / PULSES_PER_LITER;
        mctrl.waterPulsesFrag %= PULSES_PER_LITER;
      }
      //////////////////////////////////////////////////////////////////////
      // transfer hour and day values into global structure to get the current values for read
      HMON_ctrl.hourBlocks[hblkidx].burnerStartsHour = mctrl.burnerStartsHour;
      HMON_ctrl.hourBlocks[hblkidx].burningTimeHour  = mctrl.burningTimeHour;
      HMON_ctrl.dayCtrl[dblkidx].burnerStarts        = mctrl.burnerStarts;
      HMON_ctrl.dayCtrl[dblkidx].burningTime         = mctrl.burningTime;

      //////////////////////////////////////////////////////////////////////
      // action handler for every hour
      if(hour(HMON_ctrl.tsNow) != hour(HMON_ctrl.tsLastProc)) {
        // transfer local temperatures into global structure
        for(int i=0; i<CNT_OF_TEMP_SENSORS; i++) {
          HMON_ctrl.hourBlocks[hblkidx].tempVal[i].tMin = temp[i].tMin;
          HMON_ctrl.hourBlocks[hblkidx].tempVal[i].tMax = temp[i].tMax;
          HMON_ctrl.hourBlocks[hblkidx].tempVal[i].tAvr = (char)((temp[i].cntVal>0) ? ((temp[i].tempCum*2) + temp[i].cntVal) / (2 * temp[i].cntVal) : 0);
          HMON_ctrl.hourBlocks[hblkidx].tempVal[i].tCnt = temp[i].cntVal;
        }
        INCR_IDX_HBLOCK(HMON_ctrl.dayCtrl[dblkidx].idxHourBEnd);
        hblkidx = HMON_ctrl.dayCtrl[HMON_ctrl.idxCurrentDay].idxHourBEnd;
        // clear hour block buffer
        memset(&HMON_ctrl.hourBlocks[hblkidx], 0, sizeof(hourBlock));
        
        // action handler for midnight
        if(day(HMON_ctrl.tsNow) != day(HMON_ctrl.tsLastProc)) {
          int idxHour = HMON_ctrl.dayCtrl[dblkidx].idxHourBEnd;
          int idxHeat = HMON_ctrl.dayCtrl[dblkidx].idxWHeatEnd;
          // finalize current time block
          // move to next block and initialize it
          INCR_IDX_DBLOCK(HMON_ctrl.idxCurrentDay);
          dblkidx = HMON_ctrl.idxCurrentDay;
          HMON_ctrl.dayCtrl[dblkidx].dayOfMonth    = day(HMON_ctrl.tsNow);
          HMON_ctrl.dayCtrl[dblkidx].idxHourBStart = idxHour;
          HMON_ctrl.dayCtrl[dblkidx].idxHourBEnd   = idxHour;
          HMON_ctrl.dayCtrl[dblkidx].idxWHeatStart = idxHeat;
          HMON_ctrl.dayCtrl[dblkidx].idxWHeatEnd   = idxHeat;
          HMON_ctrl.dayCtrl[dblkidx].burnerStarts  = 0;
          HMON_ctrl.dayCtrl[dblkidx].burningTime   = 0;
          HMON_ctrl.dayCtrl[dblkidx].waterVolume   = 0;
          // cleanup local counter
          mctrl.burnerStarts = (mctrl.burningState == LOW) ? 1 : 0;
          mctrl.burningTime  = 0;
        }
        // cleanup local temparature array
        mctrl.burningTimeHour  = 0;
        mctrl.burnerStartsHour = 0;
        mctrl.cleanuptemp = true;
        // show current status every eight hours
        if((hour(HMON_ctrl.tsNow) & 0x03) == 0) {
          HMON_ctrl.readValues = true;
        }
      }
      // mark minute as processed
      HMON_ctrl.tsLastProc = HMON_ctrl.tsNow;
      Serial.flush();
    }
  }

  delay(LOOP_INTERVAL);
}
