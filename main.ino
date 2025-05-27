#include "stm32f4xx_hal.h"
#include <DWIN_Arduino.h>
#include <JQ6500_Serial.h>
#include "STM32_CAN.h"

#include <ModbusRtu.h>





STM32_CAN Can(CAN1, DEF);  // Use PA11/12 pins for CAN1
static CAN_message_t CAN_RX_msg;
#define DGUS_BAUD 115200
#define DGUS_SERIAL Serial2
DWIN hmi(DGUS_SERIAL);

bool isTripLatched = false;
#define RESET_DEBOUNCE_DELAY 100  // ms
bool lastResetState = LOW;
unsigned long lastResetChange = 0;

bool resetTriggered = false;
unsigned long resetTimestamp = 0;

bool alarmMuted = false;
bool lastMuteButtonState = LOW;
// === I/O Pins ===

#define AUTO_PIN PA15 //IN1 
#define MANUAL_PIN PC10 //IN2 
#define MAINTENANCE_PIN PC11 //IN3
#define RESET_BUTTON_PIN PC12 //IN4
#define AUDIOPLAY PC1
#define MUTE_BUTTON PD0 //IN5 
#define FIRE PD1//IN6 
#define CP   PD2 //IN7 
#define DETECTOR_FAULT PD3 //IN8
#define MANUAL_OPERATION PD4 //IN9

#define TCIV_OPEN_PIN   PE0 //IN10
#define TCIV_CLOSE_PIN  PD6 //IN11

#define ODV_OPEN_PIN    PE2 //IN12
#define ODV_CLOSE_PIN   PB9 //IN13

#define NIV_OPEN_PIN    PE3 //IN14
#define NIV_CLOSE_PIN   PC2 //IN15
#define RELAYMOTORTCIV PA5 //RELAY FOR CONTROLLING MOTOR SHORT
#define RELAYMOTORODV PB1 //RELAY FOR CONTROLLING MOTOR SHORT


// === I/O Pins ===

//PWM CONFIGURATIONS
#define TCIV_PWM1 PA8   // TCIV Forward
#define TCIV_PWM2 PC9   // TCIV Reverse

#define ODV_PWM1 PC8    // ODV Forward
#define ODV_PWM2 PC7    // ODV Reverse
//PWM COMFIGURATIONS



bool systemHealthy = true;


bool lastButtonState = LOW;


// Valve label addresses on HMI
#define TCIV_ADDR  0x3001
#define ODV_ADDR   0x3010
#define NIV_ADDR   0x3020

// Valve status pins

// END
//Relays 
#define TCIV PE10 //RELAY1
#define ODV  PA4  //RELAY2
#define NIV  PE12  //RELAY3

// DIGITAL OUTPUTS 
#define SYSTEMHEALTHY PD10 //OUT2
#define MAINTENANCELIGHT PD15 //OUT3
#define SYSTEMON PD14 //OUT4
#define ALARM PD13 //OUT5

// === Status Variables from CAN ===
int Differential   = 0;
int BuchholzRelay  = 0;
int MasterTrip     = 0;
int REF            = 0;
int OverCurrent    = 0;
int PRV            = 0;
int RPRR           = 0;



// VP For Text And COLOURS 
const uint16_t VP_TEXT[7] = {0x400, 0x1210, 0x1220, 0x1230, 0x1240, 0x1250, 0x1260}; // Text display VP
const uint16_t VP_COLOR[7] = {0x5000, 0x5500, 0x6000, 0x6500, 0x7000, 0x7500, 0x8000}; // Color VP


int lastPWM_TCIV1 = -1, lastPWM_TCIV2 = -1;
int lastPWM_ODV1 = -1, lastPWM_ODV2 = -1;


enum ValveState {
  IDLE,
  TCIV_CLOSING,
  ODV_OPENING,
  NIV_ON,
  TCIV_OPENING,
  ODV_CLOSING,
  NIV_OFF
};


ValveState valveState = IDLE;
unsigned long valveTimer = 0;
const unsigned long MOTOR_TIMEOUT = 3000;  // 3 sec
const unsigned long MOTOR_RUN_TIME = 1000; // 1 sec

const char* activeCommand = "";

// END 


// === Flash Configuration ===
#define FLASH_SECTOR         FLASH_SECTOR_7



#define FLASH_START_ADDRESS  0x08060000

#define TRIP_FLAG_ADDR       (FLASH_START_ADDRESS + 0x00)

char lastValidStatus[8] = {'x','x','x','x','x','x','x','\0'};

bool lastfirestat = false;
// === Serial Definitions ===
HardwareSerial Serial1(PA10, PA9);
HardwareSerial Serial3(PB11, PB10);
HardwareSerial Serial2(PA3, PA2);

#define VAL5_PIN PE5  // Replace with your actual input pin
#define AUDIO_RISE_INDEX 11  // File index for rising edge (HIGH)
#define AUDIO_FALL_INDEX 12  // File index for falling edge (LOW)


bool tripFlagWritten = false;
unsigned long lastTripCheck = 0;
unsigned long lastFlashWriteTime = 0;
const unsigned long TRIP_CHECK_INTERVAL = 100;     // Check trip every 100ms
const unsigned long FLASH_WRITE_INTERVAL = 5000;   // Write to flash every 5 sec max



// === Serial1 Buffering ===
String receivedString = "";
unsigned long lastSerialRead = 0;
const unsigned long SERIAL_TIMEOUT = 50;

bool fire;


#define FLASH_LOG_START_ADDRESS  0x08080000  // Sector 8
#define MAX_LOGS                 5
#define LOG_ENTRY_SIZE           32          // 32 bytes per log (aligned to 4-byte boundary)
#define TOTAL_LOG_BYTES          (MAX_LOGS * LOG_ENTRY_SIZE)

#define LCD_LOG_ADDRESS          0x5600      // LCD memory-mapped log display address

// Mode ENUM Definition 
JQ6500_Serial mp3(Serial3);
Modbus slave(1, Serial1, -1);     


enum Mode {
  AUTO_MODE,
  MANUAL_MODE,
  MAINTENANCE_MODE,
  UNKNOWN_MODE
};


// Mode Variable Definition 

Mode currentMode = UNKNOWN_MODE;

uint16_t au16data[18];  // 18 holding registers
// 


// === Flash Helpers ===
void unlockFlash() {
  HAL_FLASH_Unlock();
}

void lockFlash() {
  HAL_FLASH_Lock();
}

void eraseSector() {
  FLASH_EraseInitTypeDef eraseInit;
  uint32_t sectorError = 0;

  eraseInit.TypeErase    = FLASH_TYPEERASE_SECTORS;
  eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  eraseInit.Sector       = FLASH_SECTOR;
  eraseInit.NbSectors    = 1;

  HAL_FLASH_Unlock();
  HAL_FLASHEx_Erase(&eraseInit, &sectorError);
  HAL_FLASH_Lock();
}

// === Your Custom Flash Writer ===
void writeFlash(uint32_t address, const char* str, size_t len) {
  HAL_FLASH_Unlock();

  uint32_t paddedLen = ((len + 3) / 4) * 4;
  for (uint32_t i = 0; i < paddedLen; i += 4) {
    uint32_t word = 0xFFFFFFFF;
    for (int j = 0; j < 4; j++) {
      if (i + j < len) {
        word &= ~(0xFF << (8 * j));
        word |= ((uint8_t)str[i + j]) << (8 * j);
      }
    }
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + i, word);
  }

  HAL_FLASH_Lock();
}

void readFlash(uint32_t address, char* buffer, size_t len) {
  for (size_t i = 0; i < len; i++) {
    buffer[i] = (__IO uint8_t)(address + i);
  }
  buffer[len - 1] = '\0'; // Ensure null terminator
}

// === Trip Flag Write/Read ===
void writeTripFlagToFlash(uint8_t state) {
  char existing[2] = {0};
  readFlash(TRIP_FLAG_ADDR, existing, 2);

  // Don't write if it's already same
  if ((state == 1 && existing[0] == '1') || (state == 0 && existing[0] == '0')) {
    return;  // No need to erase/write
  }

  eraseSector();  // Only if value actually changed
  char val[2] = { (state == 1 ? '1' : '0'), '\0' };
  writeFlash(TRIP_FLAG_ADDR, val, 2);
}


uint8_t readTripFlagFromFlash() {
  char buffer[2] = {0};
  readFlash(TRIP_FLAG_ADDR, buffer, 2);
  return (buffer[0] == '1') ? 1 : 0;
}


void writeLogToFlash(uint32_t address, const char* log) {
  HAL_FLASH_Unlock();

  for (int i = 0; i < LOG_ENTRY_SIZE; i += 4) {
    uint32_t word = 0xFFFFFFFF;
    for (int j = 0; j < 4; j++) {
      if (i + j < LOG_ENTRY_SIZE && log[i + j] != '\0') {
        word &= ~(0xFF << (8 * j));
        word |= ((uint8_t)log[i + j]) << (8 * j);
      }
    }
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + i, word);
  }

  HAL_FLASH_Lock();
}




void readLogs(char logs[MAX_LOGS][LOG_ENTRY_SIZE]) {
  for (int i = 0; i < MAX_LOGS; i++) {
    for (int j = 0; j < LOG_ENTRY_SIZE; j++) {
      logs[i][j] = *(__IO uint8_t*)(FLASH_LOG_START_ADDRESS + i * LOG_ENTRY_SIZE + j);
    }
    logs[i][LOG_ENTRY_SIZE - 1] = '\0'; // null-terminate
  }
}


void displayAllLogsOnLCD(char logs[5][32]);


void setLog(const char* newLog) {
  char logs[MAX_LOGS][LOG_ENTRY_SIZE];
  readLogs(logs);  // Step 1: Load existing logs from flash

  // Step 2: Shift logs up (oldest is removed)
  for (int i = 0; i < MAX_LOGS - 1; i++) {
    strncpy(logs[i], logs[i + 1], LOG_ENTRY_SIZE);
  }

  // Step 3: Add new log at the end
  strncpy(logs[MAX_LOGS - 1], newLog, LOG_ENTRY_SIZE);
  logs[MAX_LOGS - 1][LOG_ENTRY_SIZE - 1] = '\0';  // Ensure null-termination

  // Step 4: Erase sector and re-write all logs
  FLASH_EraseInitTypeDef eraseInit;
  uint32_t sectorError = 0;
  eraseInit.TypeErase    = FLASH_TYPEERASE_SECTORS;
  eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  eraseInit.Sector       = FLASH_SECTOR_8;
  eraseInit.NbSectors    = 1;

  HAL_FLASH_Unlock();
  HAL_FLASHEx_Erase(&eraseInit, &sectorError);
  HAL_FLASH_Lock();

  for (int i = 0; i < MAX_LOGS; i++) {
    writeLogToFlash(FLASH_LOG_START_ADDRESS + i * LOG_ENTRY_SIZE, logs[i]);
  }

  // Step 5: Update all logs on DWIN screen
  displayAllLogsOnLCD(logs);
}



void displayAllLogsOnLCD(char logs[MAX_LOGS][LOG_ENTRY_SIZE]) {
  String combined = "";

  for (int i = 0; i < MAX_LOGS; i++) {
    if (strlen(logs[i]) > 0) {
      combined += String(i + 1) + "-" + logs[i];
      if (i < MAX_LOGS - 1) combined += "\n";
    }
  }

  hmi.setText(0x5600, combined.c_str());
}






void handleVal5AudioTrigger() {
  static bool lastVal5State = LOW;
  bool val5 = digitalRead(AUDIOPLAY);

  if (val5 == HIGH && lastVal5State == LOW) {
    mp3.playFileByIndexNumber(AUDIO_RISE_INDEX);  // Rising edge
  } else if (val5 == LOW && lastVal5State == HIGH) {
    mp3.playFileByIndexNumber(AUDIO_FALL_INDEX);  // Falling edge
  }

  lastVal5State = val5;
}


// === Setup ===
void setup() {
  Serial1.begin(9600);
  slave.start();         // Start Modbus RTU slave
  Can.begin();
  Can.setBaudRate(500000);
  Serial3.begin(9600);

  DGUS_SERIAL.begin(DGUS_BAUD);
  hmi.echoEnabled(false);                    
  
  hmi.setPage(0);
  hmi.setBrightness(0x20);       // see the docs 0x82 LED_Config



  pinMode(TCIV_PWM1, OUTPUT);
  pinMode(TCIV_PWM2, OUTPUT);
  pinMode(ODV_PWM1, OUTPUT);
  pinMode(ODV_PWM2, OUTPUT);

  analogWriteFrequency(20000);  // 20 kHz for all PWM-capable timers
  

  pinMode(FIRE, INPUT_PULLUP);
  pinMode(CP, INPUT_PULLUP);
  pinMode(TCIV, OUTPUT);
  pinMode(NIV, OUTPUT);
  pinMode(ODV, OUTPUT);
  pinMode(RELAYMOTORTCIV,OUTPUT);
  pinMode(RELAYMOTORODV,OUTPUT);
  pinMode(AUTO_PIN, INPUT_PULLUP);
  pinMode(MANUAL_PIN, INPUT_PULLUP);
  pinMode(MAINTENANCE_PIN, INPUT_PULLUP);
  pinMode(TCIV_OPEN_PIN, INPUT_PULLUP);
  pinMode(TCIV_CLOSE_PIN, INPUT_PULLUP);
  pinMode(AUDIOPLAY, INPUT_PULLUP);

  pinMode(ODV_OPEN_PIN, INPUT_PULLUP);
  pinMode(ODV_CLOSE_PIN, INPUT_PULLUP);
  pinMode(VAL5_PIN,INPUT_PULLUP);

  pinMode(NIV_OPEN_PIN, INPUT_PULLUP);
  pinMode(NIV_CLOSE_PIN, INPUT_PULLUP);

  pinMode(MANUAL_OPERATION, INPUT_PULLUP);
  pinMode(RESET_BUTTON_PIN,INPUT_PULLUP);
  pinMode(SYSTEMHEALTHY,OUTPUT);
  pinMode(MAINTENANCELIGHT,OUTPUT);
  pinMode(SYSTEMON,OUTPUT);
  pinMode(ALARM,OUTPUT);
  pinMode(MUTE_BUTTON,INPUT_PULLUP);
  pinMode(DETECTOR_FAULT,INPUT_PULLDOWN);

  // Read and print flash value
  char flagBuffer[2] = {0};
  readFlash(TRIP_FLAG_ADDR, flagBuffer, 2);

  //Serial3.print("🧠 Trip flag from flash: '");
  //Serial3.print(flagBuffer[0]);
  //Serial3.println("'");

  if (flagBuffer[0] == '1') {
    


    controlValvesByCommand("TRIP");
    //Serial3.println("🔁 Trip state recovered on boot");
  } else {
    
    controlValvesByCommand("NORMAL");
    //Serial3.println("✅ System is normal on boot");
  }

  digitalWrite(SYSTEMON,HIGH);
  digitalWrite(SYSTEMHEALTHY,HIGH);
 // digitalWrite(ALARM,HIGH);

  mp3.reset();
  mp3.setVolume(500);
  mp3.setLoopMode(MP3_LOOP_NONE);


  char logs[MAX_LOGS][LOG_ENTRY_SIZE];
  readLogs(logs);
  displayAllLogsOnLCD(logs);

  setLog("HELLO");

  setLog("ANSH");
  setLog("YADAV");
  

  // Initialize Independent Watchdog (approx 2 seconds timeout)
   


}

// === Main Loop ===
void loop() {
  // Always handle incoming CAN data immediately
  processCANData();
  //hmi.setText(0x5600, "ANSH YADAV");


   au16data[0] = 1;  // POWER ON
        au16data[1] = 1;  // IN SERVICE
        au16data[2] = 0;  // MAINTENANCE  
        au16data[3] = 1;  // AUTO
        au16data[4] = 0;  // MANUAL

  checkTripCondition(fire);
  updateTripStatuses();
  //Schedule-based logic execution
  static unsigned long lastValveUpdate = 0;
  static unsigned long lastModeUpdate = 0;
  static unsigned long lastDigitalCheck = 0;
  static unsigned long lastManualCheck = 0;
  static unsigned long lastAudioCheck = 0;
  static unsigned long lastSystemHealthUpdate = 0;
  static unsigned long lastResetCheck = 0;

  unsigned long now = millis();

  // Every 100ms: Update Valve Status
  if (now - lastValveUpdate >= 100) {
    updateValveStatuses();
    lastValveUpdate = now;
  }

  // Every 200ms: Update Mode Status
  if (now - lastModeUpdate >= 200) {
    updateMode();
    lastModeUpdate = now;
  }

  // Every 150ms: Check digital inputs
  if (now - lastDigitalCheck >= 150) {
    checkDigitalInputs();
    lastDigitalCheck = now;
  }

  // Every 300ms: Check manual trip condition
  if (now - lastManualCheck >= 300) {
    checkManualTripCondition();
    lastManualCheck = now;
  }

  



  // Every 250ms: Handle audio
  if (now - lastAudioCheck >= 250) {
    handleVal5AudioTrigger();
    lastAudioCheck = now;
  }

  // Every 500ms: Update system health indicators
  if (now - lastSystemHealthUpdate >= 50) {
    updateSystemHealthOutput();
    lastSystemHealthUpdate = now;
  }

  // Every 50ms: Reset button check (debounced)
  if (now - lastResetCheck >= 50) {
    if (digitalRead(RESET_BUTTON_PIN) == HIGH &&
    (Differential == 0 || Differential == 2) &&
    (BuchholzRelay == 0 || BuchholzRelay == 2) &&
    (MasterTrip == 0 || MasterTrip == 2) &&
    (REF == 0 || REF == 2) &&
    (OverCurrent == 0 || OverCurrent == 2) &&
    (PRV == 0 || PRV == 2) &&
    (RPRR == 0 || RPRR == 2)) {

    isTripLatched = false;
    MasterTrip = 0;
    resetTriggered = true;
    resetTimestamp = millis();
    writeTripFlagToFlash(0);
   }


    lastResetCheck = now;
  }
  slave.poll(au16data, 18);

  // Optional debug
  // Serial3.println(systemHealthy);
}


void updateValveStatuses() {
  static String lastTCIVStatus = "";
  static String lastODVStatus = "";
  static String lastNIVStatus = "";

  // === TCIV ===
  String tcivStatus;
  if (digitalRead(TCIV_OPEN_PIN) == HIGH && digitalRead(TCIV_CLOSE_PIN) == LOW) {
    tcivStatus = "OPEN";
  } else if (digitalRead(TCIV_OPEN_PIN) == LOW && digitalRead(TCIV_CLOSE_PIN) == HIGH) {
    tcivStatus = "CLOSE";
  } else {
    tcivStatus = "---";
  }
  if (tcivStatus != lastTCIVStatus) {
    hmi.setText(TCIV_ADDR, tcivStatus);
    lastTCIVStatus = tcivStatus;
  }

  // === ODV ===
  String odvStatus;
  if (digitalRead(ODV_OPEN_PIN) == HIGH && digitalRead(ODV_CLOSE_PIN) == LOW) {
    odvStatus = "OPEN";
  } else if (digitalRead(ODV_OPEN_PIN) == LOW && digitalRead(ODV_CLOSE_PIN) == HIGH) {
    odvStatus = "CLOSE";
  } else {
    odvStatus = "---";
  }
  if (odvStatus != lastODVStatus) {
    hmi.setText(ODV_ADDR, odvStatus);
    lastODVStatus = odvStatus;
  }

  // === NIV ===
  String nivStatus;
  if (digitalRead(NIV_OPEN_PIN) == HIGH && digitalRead(NIV_CLOSE_PIN) == LOW) {
    nivStatus = "OPEN";
  } else if (digitalRead(NIV_OPEN_PIN) == LOW && digitalRead(NIV_CLOSE_PIN) == HIGH) {
    nivStatus = "CLOSE";
  } else {
    nivStatus = "---";
  }
  if (nivStatus != lastNIVStatus) {
    hmi.setText(NIV_ADDR, nivStatus);
    lastNIVStatus = nivStatus;
  }
}

// === Trip Condition Evaluation ===
  // Keep track of the last good status


void checkDigitalInputs() {
  static String lastFireText = "";
  static String lastCPText = "";
  static String lastDetectorText = "";
  static String lastNIVLeakText = "";

  int fireStatus = digitalRead(FIRE);
  int cpStatus = digitalRead(CP);
  int detectorFault = digitalRead(DETECTOR_FAULT);

  // === CP Status ===
  String cpText = (cpStatus == 1) ? "LOW" : "NORMAL";
  if (cpText != lastCPText) {
    hmi.setText(0x3030, cpText);
    lastCPText = cpText;
  }
  if (cpStatus == 1) setSystemUnhealthy();

  // === FIRE Status ===
  String fireText = (fireStatus == 1) ? "FIRE" : "NORMAL";
  if (fireText != lastFireText) {
    hmi.setText(0x3040, fireText);
    lastFireText = fireText;
  }
  fire = (fireStatus == 1);
  if (fire) setSystemUnhealthy();

  // === DETECTOR FAULT ===
  String detectorText = (detectorFault == 1) ? "NORMAL" : "FAULT";
  if (detectorText != lastDetectorText) {
    hmi.setText(0x3060, detectorText);
    lastDetectorText = detectorText;
  }
  if (detectorFault == 1) setSystemUnhealthy();

  // === NIV Leak Detection ===
  bool nivOpen = digitalRead(NIV_OPEN_PIN);
  bool nivClose = digitalRead(NIV_CLOSE_PIN);
  bool odvOpen = digitalRead(ODV_OPEN_PIN);
  bool odvClose = digitalRead(ODV_CLOSE_PIN);

  String nivLeakText = "NORMAL";
  if (odvOpen == LOW && odvClose == HIGH && nivOpen == HIGH && nivClose == LOW) {
    nivLeakText = "NIV LEAK";
    setSystemUnhealthy();
  }

  if (nivLeakText != lastNIVLeakText) {
    hmi.setText(0x3050, nivLeakText);
    lastNIVLeakText = nivLeakText;
  }
}


void setSystemUnhealthy() {
  systemHealthy = false;
}



// Keep these as global if not already
void checkTripCondition(bool firestat) {
  static unsigned long lastTripChangeTime = 0;
  static bool lastTripNow = false;

  if (resetTriggered && (millis() - resetTimestamp < 2000)) {
    MasterTrip = 0;
  } else {
    resetTriggered = false;
  }

  bool condition1 = ((Differential == 1) || (REF == 1) || (OverCurrent == 1)) &&
                    ((BuchholzRelay == 1) || (PRV == 1) || (RPRR == 1)) &&
                    (MasterTrip == 1);

  bool condition2 = firestat &&
                    ((BuchholzRelay == 1) || (PRV == 1) || (RPRR == 1)) &&
                    (MasterTrip == 1);

  bool tripNow = (condition1 || condition2) && (currentMode == AUTO_MODE);

  if (tripNow != lastTripNow) {
    lastTripChangeTime = millis();
    lastTripNow = tripNow;
  }

  if ((millis() - lastTripChangeTime) > 100) {
    if (tripNow && !isTripLatched) {
      isTripLatched = true;
      tripFlagWritten = false;
    }
  }

  if (isTripLatched) {
    digitalWrite(TCIV, LOW);
    controlValvesByCommand("TRIP");
    // Throttled flash write
    if (!tripFlagWritten && (millis() - lastFlashWriteTime > FLASH_WRITE_INTERVAL)) {
      writeTripFlagToFlash(1);
      lastFlashWriteTime = millis();
      tripFlagWritten = true;
    }
  } else {


    controlValvesByCommand("NORMAL");
    if (tripFlagWritten && (millis() - lastFlashWriteTime > FLASH_WRITE_INTERVAL)) {
      writeTripFlagToFlash(0);
      lastFlashWriteTime = millis();
      tripFlagWritten = false;
    }
  }
}


void checkManualTripCondition() {
  if (currentMode == MANUAL_MODE) {
    bool currentButtonState = digitalRead(MANUAL_OPERATION);

    // ✅ Check if MasterTrip is active (1)
    if (currentButtonState == HIGH &&
        lastButtonState == LOW &&
        MasterTrip == 1) {

      domanualoperation();
      isTripLatched = true;
      // Serial3.println("⚠️ Trip Latched due to Manual + MasterTrip");
    }

    lastButtonState = currentButtonState;
  } else {
    lastButtonState = LOW;
  }
}


void updateSystemHealthOutput() {
  bool allClear = digitalRead(FIRE) == LOW &&
                  digitalRead(CP) == LOW &&
                  digitalRead(DETECTOR_FAULT) == HIGH &&  // HIGH means OK
                  (Differential == 0 &&
                   BuchholzRelay == 0 &&
                   REF == 0 &&
                   OverCurrent == 0 &&
                   PRV == 0 &&
                   RPRR == 0 &&
                   MasterTrip == 0);

  static bool alarmActive = false;
  static bool lastMuteButtonState = LOW;
  static bool latchedUnhealthy = false;
  static bool lastResetState = LOW;

  // === Check if any fault occurs → latch unhealthy
  if ((Differential != 0) || (BuchholzRelay != 0) || (REF != 0) ||
      (OverCurrent != 0) || (PRV != 0) || (RPRR != 0) || (MasterTrip != 0) ||
      digitalRead(FIRE) == HIGH || digitalRead(CP) == HIGH) {
    alarmActive = true;
    alarmMuted = false;
    latchedUnhealthy = true;
  }

  // === Force alarm if Detector Fault or Maintenance Mode
  bool detectorFault = (digitalRead(DETECTOR_FAULT) == LOW);  // LOW = FAULT
  bool forceAlarm = (currentMode == MAINTENANCE_MODE || detectorFault);
  if (forceAlarm) {
    alarmActive = true;          // force alarm
    alarmMuted = false;
    latchedUnhealthy = true;     // also force unhealthy
  }

  // === Mute Button Handling
  bool currentMuteButtonState = digitalRead(MUTE_BUTTON);
  if (currentMuteButtonState == HIGH && lastMuteButtonState == LOW) {
    alarmMuted = true;
    alarmActive = false;
    digitalWrite(ALARM, LOW);
  }
  lastMuteButtonState = currentMuteButtonState;

  // === Reset Button Logic (Rising Edge + allClear check)
  bool currentResetState = digitalRead(RESET_BUTTON_PIN);
  if (currentResetState == HIGH && lastResetState == LOW) {
    if (allClear) {
      latchedUnhealthy = false;
    }
  }
  lastResetState = currentResetState;

  // === Final ALARM output logic
  if ((alarmActive && !alarmMuted) || (forceAlarm && !alarmMuted)) {
    digitalWrite(ALARM, HIGH);
  } else {
    digitalWrite(ALARM, LOW);
  }

  // === SYSTEM HEALTHY light (latched logic)
  if (!latchedUnhealthy) {
    digitalWrite(SYSTEMHEALTHY, HIGH);
    systemHealthy = true;
  } else {
    digitalWrite(SYSTEMHEALTHY, LOW);
    systemHealthy = false;
  }

  // === Cutoff Motors in Maintenance Mode
  if (currentMode == MAINTENANCE_MODE) {
    digitalWrite(RELAYMOTORTCIV, LOW);
    digitalWrite(RELAYMOTORODV, LOW);
  }
}




void updateMode() {
  static Mode lastMode = UNKNOWN_MODE;

  bool autoState = digitalRead(AUTO_PIN);
  bool manualState = digitalRead(MANUAL_PIN);
  bool maintenanceState = digitalRead(MAINTENANCE_PIN);

  Mode newMode = UNKNOWN_MODE;

  if (autoState && !manualState && !maintenanceState) {
    newMode = AUTO_MODE;
  } else if (!autoState && manualState && !maintenanceState) {
    newMode = MANUAL_MODE;
  } else if (!autoState && !manualState && maintenanceState) {
    newMode = MAINTENANCE_MODE;
  }

  if (newMode != lastMode) {
    lastMode = newMode;
    currentMode = newMode;

    switch (newMode) {
      case AUTO_MODE:
        hmi.setText(0x2005, "AUTO");
        digitalWrite(MAINTENANCELIGHT, LOW);
        digitalWrite(RELAYMOTORTCIV,HIGH);
        digitalWrite(RELAYMOTORODV,HIGH);
        break;
      case MANUAL_MODE:
        hmi.setText(0x2005, "MANUAL");
        digitalWrite(MAINTENANCELIGHT, LOW);
        digitalWrite(RELAYMOTORTCIV,HIGH);
        digitalWrite(RELAYMOTORODV,HIGH);
        break;
      case MAINTENANCE_MODE:
        hmi.setText(0x2005, "MAINTENANCE");
        digitalWrite(MAINTENANCELIGHT, HIGH);
        digitalWrite(RELAYMOTORTCIV,LOW);
        digitalWrite(RELAYMOTORODV,LOW);
        break;
      default:
        // Optionally clear the text or indicate unknown
        break;
    }
  }
}


void domanualoperation(){


// Step 1: Close TCIV unconditionally
controlValvesByCommand("TRIP");

}


void updateTripStatuses() {
  static int lastTripState[7] = { -1, -1, -1, -1, -1, -1, -1 };
  static bool firstUpdateDone = false;

  const int currentStatus[7] = {
    Differential,
    BuchholzRelay,
    MasterTrip,
    REF,
    OverCurrent,
    PRV,
    RPRR
  };

  for (uint8_t i = 0; i < 7; i++) {
    int state = currentStatus[i];


    if (!firstUpdateDone || state != lastTripState[i]) {
      lastTripState[i] = state;

      const char* text;
      uint16_t color;

      switch (state) {
        case 2:
          text = "DISCONNECTED";
          color = 0xF800;  // RED
          break;
        case 1:
          text = "TRIP";
          color = 0xF800;  // RED
          break;
        case 0:
          text = "NORMAL";
          color = 0x07E0;  // GREEN
          break;
        default:
          text = "---";
          color = 0xF800;  // RED fallback
          break;
      }

      hmi.setText(VP_TEXT[i], text);
      hmi.setTextColor(VP_COLOR[i], 0x03, color);
    }
  }

  firstUpdateDone = true;
}






void setMasterTrip(bool value) {
    MasterTrip = value;

    // Optional: Log changes to MasterTrip
    
}

void controlValvesByCommand(const char* command) {
  // Start only if IDLE
  if (valveState == IDLE && (strcmp(command, "TRIP") == 0 || strcmp(command, "NORMAL") == 0)) {
    activeCommand = command;
    valveTimer = millis();

    if (strcmp(activeCommand, "TRIP") == 0) {
      // Start TCIV Close
      safeAnalogWrite(TCIV_PWM1, 0, lastPWM_TCIV1);
      safeAnalogWrite(TCIV_PWM2, 200, lastPWM_TCIV2);
      valveState = TCIV_CLOSING;
    } else {
      // Start TCIV Open
      safeAnalogWrite(TCIV_PWM1, 200, lastPWM_TCIV1);
      safeAnalogWrite(TCIV_PWM2, 0, lastPWM_TCIV2);
      valveState = TCIV_OPENING;
    }
  }

  // === Process TRIP ===
  if (strcmp(activeCommand, "TRIP") == 0) {
    static bool tcivClosedConfirmed = false;

    if (valveState == TCIV_CLOSING) {
      if (digitalRead(TCIV_CLOSE_PIN) == HIGH) {
        // Confirm TCIV closed
        safeAnalogWrite(TCIV_PWM1, 0, lastPWM_TCIV1);
        safeAnalogWrite(TCIV_PWM2, 0, lastPWM_TCIV2);
        tcivClosedConfirmed = true;
        valveTimer = millis();

        // Start ODV open only now
        safeAnalogWrite(ODV_PWM1, 200, lastPWM_ODV1);
        safeAnalogWrite(ODV_PWM2, 0, lastPWM_ODV2);
        valveState = ODV_OPENING;
      } else if (millis() - valveTimer >= MOTOR_TIMEOUT) {
        // Still timeout fallback
        safeAnalogWrite(TCIV_PWM1, 0, lastPWM_TCIV1);
        safeAnalogWrite(TCIV_PWM2, 0, lastPWM_TCIV2);
        valveState = IDLE;  // Cancel operation
        activeCommand = ""; // Abort TRIP if not safe
      }
    } 
    else if (valveState == ODV_OPENING && tcivClosedConfirmed) {
      if (millis() - valveTimer >= MOTOR_RUN_TIME) {
        safeAnalogWrite(ODV_PWM1, 0, lastPWM_ODV1);
        safeAnalogWrite(ODV_PWM2, 0, lastPWM_ODV2);
        digitalWrite(NIV, HIGH);
        valveState = NIV_ON;
      }
    } 
    else if (valveState == NIV_ON) {
      valveState = IDLE;
      activeCommand = "";
      tcivClosedConfirmed = false;
    }
  }

  // === Process NORMAL ===
   // === Process NORMAL ===
  else if (strcmp(activeCommand, "NORMAL") == 0) {
    static bool tcivOpenedConfirmed = false;

    if (valveState == TCIV_OPENING) {
      if (digitalRead(TCIV_OPEN_PIN) == HIGH) {
        // Confirm TCIV open
        safeAnalogWrite(TCIV_PWM1, 0, lastPWM_TCIV1);
        safeAnalogWrite(TCIV_PWM2, 0, lastPWM_TCIV2);
        tcivOpenedConfirmed = true;
        valveTimer = millis();

        // Now start ODV close
        safeAnalogWrite(ODV_PWM1, 0, lastPWM_ODV1);
        safeAnalogWrite(ODV_PWM2, 200, lastPWM_ODV2);
        valveState = ODV_CLOSING;
      } else if (millis() - valveTimer >= MOTOR_TIMEOUT) {
        // Fail-safe: abort if TCIV doesn't open in time
        safeAnalogWrite(TCIV_PWM1, 0, lastPWM_TCIV1);
        safeAnalogWrite(TCIV_PWM2, 0, lastPWM_TCIV2);
        valveState = IDLE;
        activeCommand = "";
      }
    }

    else if (valveState == ODV_CLOSING && tcivOpenedConfirmed) {
      if (millis() - valveTimer >= MOTOR_RUN_TIME) {
        safeAnalogWrite(ODV_PWM1, 0, lastPWM_ODV1);
        safeAnalogWrite(ODV_PWM2, 0, lastPWM_ODV2);
        digitalWrite(NIV, LOW);
        valveState = NIV_OFF;
      }
    }

    else if (valveState == NIV_OFF) {
      valveState = IDLE;
      activeCommand = "";
      tcivOpenedConfirmed = false;
    }
  }

}







void safeAnalogWrite(int pin, int value, int &lastValue) {
  if (value != lastValue) {
    analogWrite(pin, value);
    lastValue = value;
  }
}

bool getMasterTrip(){

  return MasterTrip;
}

// === Serial1 Packet Reader ===
void processCANData() {
  if (!Can.read(CAN_RX_msg)) return;
  if (CAN_RX_msg.len < 7 || CAN_RX_msg.flags.remote) return;

  // Directly assign to global status variables
  Differential   = (CAN_RX_msg.buf[0] <= 2) ? CAN_RX_msg.buf[0] : 9;
  BuchholzRelay  = (CAN_RX_msg.buf[1] <= 2) ? CAN_RX_msg.buf[1] : 9;
  MasterTrip     = (CAN_RX_msg.buf[2] <= 2) ? CAN_RX_msg.buf[2] : 9;
  REF            = (CAN_RX_msg.buf[3] <= 2) ? CAN_RX_msg.buf[3] : 9;
  OverCurrent    = (CAN_RX_msg.buf[4] <= 2) ? CAN_RX_msg.buf[4] : 9;
  PRV            = (CAN_RX_msg.buf[5] <= 2) ? CAN_RX_msg.buf[5] : 9;
  RPRR           = (CAN_RX_msg.buf[6] <= 2) ? CAN_RX_msg.buf[6] : 9;

  // Now use these updated values
 // 
 //   // if implemented with globals

  // Optional debug
  
}



