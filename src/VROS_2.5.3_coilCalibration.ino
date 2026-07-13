#include <FastLED.h>
#include <EEPROM.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>

// heightmaps
//https://tangrams.github.io/heightmapper

const char FIRMWARE_PRODUCT_NAME[] = "VROS_caseController";
const char FIRMWARE_SEMVER[] = "2.5.10";
const char FIRMWARE_COMPAT_ID[] = "VROS_2.5.10_caseController";


// pin definitions
#define DIR_LEFT_PIN 6
#define STEP_LEFT_PIN 7
#define DIR_RIGHT_PIN 8
#define STEP_RIGHT_PIN 9
#define FAULTLEFT 0
#define FAULTRIGHT 1
#define MICROSTEP_PIN 5
#define toggle1 26
#define toggle2 28
#define toggle3 22
#define toggle4 24
#define LED1 29
#define LED2 27
#define LED3 25
#define LED4 23
int rotary1;
int rotary2;



//Physical vars, all in cm

enum RobotKindId {
  robotKindUnknown = 0,
  robotKindHangingVBot = 1,
  robotKindFlatQuadTension = 2
};

enum ContactStateId {
  contactStateUnknown = 0,
  contactStateDraw = 1,
  contactStateTravel = 2
};

enum MotionBackendId {
  motionBackendNone = 0,
  motionBackendHangingTwoAxis = 1,
  motionBackendQuadFourAxis = 2
};

const byte MAX_MOTION_AXES = 4;
const byte HANGING_VBOT_AXIS_COUNT = 2;
const byte FLAT_QUAD_AXIS_COUNT = 4;

struct LegacyRobotSetupPayloadV1 {
  float motorDistance;
  float scanOffset;
  float feedOffset;
  float height;
  float lineResolution;
  float homePosition;
  float leftCoilFeed;
  float rightCoilFeed;
  float stepsToCm;
};

struct RobotSetupPayload {
  byte robotKind;
  byte reserved0[3];
  float motorDistance;
  float scanOffset;
  float feedOffset;
  float width;
  float height;
  float lineResolution;
  float homePosition;
  float stepsToCm;
  float leftCoilFeed;
  float rightCoilFeed;
  float quadHomeScan;
  float quadHomeFeed;
  float quadCableAFeed;
  float quadCableBFeed;
  float quadCableCFeed;
  float quadCableDFeed;
  float quadDrawLiftValue;
  float quadTravelLiftValue;
};

struct LegacyRobotSetupBlockV1 {
  unsigned long magic;
  byte version;
  byte payloadSize;
  unsigned int flags;
  unsigned long crc32;
  LegacyRobotSetupPayloadV1 payload;
  byte reserved[16];
};

struct RobotSetupBlock {
  unsigned long magic;
  byte version;
  byte payloadSize;
  unsigned int flags;
  unsigned long crc32;
  RobotSetupPayload payload;
  byte reserved[8];
};

struct RobotSetupBlockHeader {
  unsigned long magic;
  byte version;
  byte payloadSize;
  unsigned int flags;
  unsigned long crc32;
};

struct PositionBlock {
  unsigned long magic;
  long scanX100;
  long feedX100;
  unsigned long crc32;
};

const unsigned long ROBOT_SETUP_MAGIC = 0x56525331UL;
const byte LEGACY_ROBOT_SETUP_VERSION = 1;
const byte ROBOT_SETUP_VERSION = 2;
const int ROBOT_SETUP_EEPROM_ADDRESS = 0;
const unsigned long POSITION_MAGIC = 0x504F5331UL;
const int POSITION_EEPROM_ADDRESS = 128;
const int LEGACY_POSITION_BLOCK_EEPROM_ADDRESS = 64;
const int LEGACY_FEED_EEPROM_ADDRESS = 0;
const int LEGACY_SCAN_EEPROM_ADDRESS = 15;

const RobotSetupPayload DEFAULT_ROBOT_SETUP = {
  robotKindHangingVBot,
  {0, 0, 0},
  62.00f,
  10.00f,
  20.00f,
  42.00f,
  50.00f,
  0.50f,
  82.00f,
  35.00f,
  1.00f,
  0.997f,
  21.00f,
  25.00f,
  1.00f,
  1.00f,
  1.00f,
  1.00f,
  0.00f,
  1.00f
};

const RobotSetupPayload DEFAULT_QUAD_ROBOT_SETUP = {
  robotKindFlatQuadTension,
  {0, 0, 0},
  0.00f,
  0.00f,
  0.00f,
  42.00f,
  50.00f,
  0.50f,
  0.00f,
  35.00f,
  1.00f,
  1.00f,
  21.00f,
  25.00f,
  1.00f,
  1.00f,
  1.00f,
  1.00f,
  0.00f,
  1.00f
};

RobotSetupPayload robotSetup = DEFAULT_ROBOT_SETUP;
boolean robotSetupEEPROMValid = false;

RobotKindId currentRobotKind = robotKindHangingVBot;
float motorDistance = DEFAULT_ROBOT_SETUP.motorDistance;
float scanOffset = DEFAULT_ROBOT_SETUP.scanOffset;
float feedOffset = DEFAULT_ROBOT_SETUP.feedOffset;
float width = DEFAULT_ROBOT_SETUP.width;
float height = DEFAULT_ROBOT_SETUP.height;
float lineResolution = DEFAULT_ROBOT_SETUP.lineResolution;
float homePosition = DEFAULT_ROBOT_SETUP.homePosition;
float leftCoilFeed = DEFAULT_ROBOT_SETUP.leftCoilFeed;
float rightCoilFeed = DEFAULT_ROBOT_SETUP.rightCoilFeed;
float stepsToCmBase = DEFAULT_ROBOT_SETUP.stepsToCm;
float stepsToCm = DEFAULT_ROBOT_SETUP.stepsToCm;
float quadHomeScan = DEFAULT_ROBOT_SETUP.quadHomeScan;
float quadHomeFeed = DEFAULT_ROBOT_SETUP.quadHomeFeed;
float quadCableAFeed = DEFAULT_ROBOT_SETUP.quadCableAFeed;
float quadCableBFeed = DEFAULT_ROBOT_SETUP.quadCableBFeed;
float quadCableCFeed = DEFAULT_ROBOT_SETUP.quadCableCFeed;
float quadCableDFeed = DEFAULT_ROBOT_SETUP.quadCableDFeed;
float quadDrawLiftValue = DEFAULT_ROBOT_SETUP.quadDrawLiftValue;
float quadTravelLiftValue = DEFAULT_ROBOT_SETUP.quadTravelLiftValue;
ContactStateId contactState = contactStateDraw;
String lastRobotSetupPayloadError = "";
unsigned long lastUnsupportedMotionReportAt = 0UL;

boolean detectCase = false;

float stepLength;
float scan;
float feed;
float desiredScan;
float desiredFeed;
float currentA;
float currentB;
float currentCableLengths[4] = {0.0f, 0.0f, 0.0f, 0.0f};

//speed vars
int minStepperDelay = 50; //200
int minStepperPulse = 50;


//Drawing vars
String type = "absolute"; // relative/absolute
long gestureCount; //counts the gestures in the drawing

//EEPROM vars
long scanINT;
long feedINT;

// SD vars
File dataFile;
int filePointer;
String dataCommand;
float dataXPos;
float dataYPos;
String dataValue;

//State Machine Vars
enum MachineState {idle, drawing, pausing, aborting, launchpad, noSD};
enum DrawOutcome {drawNone, drawFinished, drawAborted, drawError};
enum CommandType {
  cmdNone,
  cmdDrawFromFile,
  cmdSetSpeed,
  cmdMove,
  cmdStreamMove,
  cmdStepL,
  cmdStepR,
  cmdSetType,
  cmdSetMode,
  cmdSetAdjustment,
  cmdSetContact,
  cmdOutlineCanvas,
  cmdReturnToOrigin,
  cmdReturnToHome,
  cmdResetHome,
  cmdTerminate,
  cmdMonitoringOn,
  cmdMonitoringOff,
  cmdAbort,
  cmdPause,
  cmdContinue,
  cmdPosition,
  cmdRetrySD,
  cmdRobotSetupGet,
  cmdRobotSetupWrite,
  cmdRobotSetupLoad,
  cmdRobotSetupDefaults,
  cmdClearEEPROM
};

MachineState machineState = idle;
DrawOutcome drawOutcome = drawNone;
CommandType pendingCommand = cmdNone;
float pendingArgument1 = 0.0f;
float pendingArgument2 = 0.0f;
String pendingText1 = "";
String pendingText2 = "";
boolean hasPendingCommand = false;
boolean drawingFileOpen = false;
const int streamMoveQueueCapacity = 12;
float streamMoveQueueScan[streamMoveQueueCapacity];
float streamMoveQueueFeed[streamMoveQueueCapacity];
int streamMoveQueueHead = 0;
int streamMoveQueueTail = 0;
int streamMoveQueueCount = 0;
boolean streamMoveQueuePaused = false;
int loopCounter = 0;
int cycleLength = 2000;
unsigned long lastSDRetryAt = 0;
const unsigned long sdRetryIntervalMs = 2000;

///////////////////////////////////////////////////
// SETTINGS                                      //
///////////////////////////////////////////////////

// setting vars
String mode = "bresenham"; // canonical motion engine
boolean monitoring = false;
boolean waitForStep = false;
boolean upperBound = false;
boolean lowerBound = false;
boolean leftBound = true;
boolean rightBound = true;
String adjustmentType = "none";
float microstepResolution = 0.0625;

unsigned long crc32Update(unsigned long crc, const byte* data, unsigned int length) {
  crc = ~crc;
  for ( unsigned int index = 0; index < length; index++ ) {
    crc ^= data[index];
    for ( byte bit = 0; bit < 8; bit++ ) {
      if ( crc & 1UL ) {
        crc = (crc >> 1) ^ 0xEDB88320UL;
      } else {
        crc >>= 1;
      }
    }
  }
  return ~crc;
}

unsigned long calculateRobotSetupCRC(const RobotSetupPayload& payload) {
  return crc32Update(0UL, (const byte*)&payload, sizeof(RobotSetupPayload));
}

unsigned long calculateLegacyRobotSetupCRC(const LegacyRobotSetupPayloadV1& payload) {
  return crc32Update(0UL, (const byte*)&payload, sizeof(LegacyRobotSetupPayloadV1));
}

unsigned long calculatePositionCRC(long scanX100Value, long feedX100Value) {
  unsigned long crc = 0UL;
  crc = crc32Update(crc, (const byte*)&scanX100Value, sizeof(long));
  crc = crc32Update(crc, (const byte*)&feedX100Value, sizeof(long));
  return crc;
}

boolean validFloatRange(float value, float minValue, float maxValue) {
  return value >= minValue && value <= maxValue;
}

boolean failRobotSetupPayloadParse(const String& detail) {
  lastRobotSetupPayloadError = detail;
  return false;
}

String formatFloatRangeDetail(float value, float minValue, float maxValue) {
  String detail = String(value, 4);
  detail += F(" not in [");
  detail += String(minValue, 4);
  detail += F(", ");
  detail += String(maxValue, 4);
  detail += F("]");
  return detail;
}

const char* getRobotKindToken(RobotKindId robotKind) {
  switch (robotKind) {
    case robotKindFlatQuadTension: return "flat_quad_tension";
    case robotKindHangingVBot: return "hanging_vbot";
    default: return "unknown";
  }
}

RobotKindId parseRobotKindToken(const String& value) {
  if ( value == "hanging_vbot" ) return robotKindHangingVBot;
  if ( value == "flat_quad_tension" ) return robotKindFlatQuadTension;
  return robotKindUnknown;
}

const char* getContactStateToken(ContactStateId state) {
  switch (state) {
    case contactStateTravel: return "travel";
    case contactStateDraw: return "draw";
    default: return "unknown";
  }
}

ContactStateId parseContactStateToken(const String& value) {
  if ( value == "draw" ) return contactStateDraw;
  if ( value == "travel" ) return contactStateTravel;
  return contactStateUnknown;
}

const char* getMotionBackendToken(MotionBackendId backend) {
  switch (backend) {
    case motionBackendQuadFourAxis: return "quad_four_axis";
    case motionBackendHangingTwoAxis: return "hanging_two_axis";
    default: return "none";
  }
}

const char* getMotionSupportToken(boolean supported) {
  return supported ? "enabled" : "disabled";
}

const RobotSetupPayload& defaultRobotSetupForKind(RobotKindId robotKind) {
  if ( robotKind == robotKindFlatQuadTension ) return DEFAULT_QUAD_ROBOT_SETUP;
  return DEFAULT_ROBOT_SETUP;
}

void normalizeRobotSetup(RobotSetupPayload& payload) {
  if ( payload.robotKind != robotKindFlatQuadTension ) {
    payload.robotKind = robotKindHangingVBot;
    payload.width = payload.motorDistance - payload.scanOffset * 2.0f;
  }
  for ( byte index = 0; index < sizeof(payload.reserved0); index++ ) {
    payload.reserved0[index] = 0;
  }
}

String describeRobotSetupValidationError(const RobotSetupPayload& payload) {
  if ( payload.robotKind == robotKindHangingVBot ) {
    float computedWidth = payload.motorDistance - payload.scanOffset * 2.0f;
    if ( !validFloatRange(payload.motorDistance, 1.0f, 500.0f) ) return String(F("motorDistance ")) + formatFloatRangeDetail(payload.motorDistance, 1.0f, 500.0f);
    if ( !validFloatRange(payload.scanOffset, 0.0f, 200.0f) ) return String(F("scanOffset ")) + formatFloatRangeDetail(payload.scanOffset, 0.0f, 200.0f);
    if ( !validFloatRange(payload.feedOffset, 0.0f, 200.0f) ) return String(F("feedOffset ")) + formatFloatRangeDetail(payload.feedOffset, 0.0f, 200.0f);
    if ( !validFloatRange(payload.height, 1.0f, 500.0f) ) return String(F("height ")) + formatFloatRangeDetail(payload.height, 1.0f, 500.0f);
    if ( !validFloatRange(payload.lineResolution, 0.01f, 20.0f) ) return String(F("lineResolution ")) + formatFloatRangeDetail(payload.lineResolution, 0.01f, 20.0f);
    if ( !validFloatRange(payload.homePosition, 0.0f, 500.0f) ) return String(F("homePosition ")) + formatFloatRangeDetail(payload.homePosition, 0.0f, 500.0f);
    if ( !validFloatRange(payload.leftCoilFeed, 0.5f, 1.5f) ) return String(F("leftCoilFeed ")) + formatFloatRangeDetail(payload.leftCoilFeed, 0.5f, 1.5f);
    if ( !validFloatRange(payload.rightCoilFeed, 0.5f, 1.5f) ) return String(F("rightCoilFeed ")) + formatFloatRangeDetail(payload.rightCoilFeed, 0.5f, 1.5f);
    if ( !validFloatRange(payload.stepsToCm, 1.0f, 5000.0f) ) return String(F("stepsToCm ")) + formatFloatRangeDetail(payload.stepsToCm, 1.0f, 5000.0f);
    if ( computedWidth <= 0.0f ) return F("computed width must be > 0");
    return "";
  }

  if ( payload.robotKind == robotKindFlatQuadTension ) {
    if ( !validFloatRange(payload.scanOffset, 0.0f, 200.0f) ) return String(F("scanOffset ")) + formatFloatRangeDetail(payload.scanOffset, 0.0f, 200.0f);
    if ( !validFloatRange(payload.feedOffset, 0.0f, 200.0f) ) return String(F("feedOffset ")) + formatFloatRangeDetail(payload.feedOffset, 0.0f, 200.0f);
    if ( !validFloatRange(payload.width, 1.0f, 500.0f) ) return String(F("width ")) + formatFloatRangeDetail(payload.width, 1.0f, 500.0f);
    if ( !validFloatRange(payload.height, 1.0f, 500.0f) ) return String(F("height ")) + formatFloatRangeDetail(payload.height, 1.0f, 500.0f);
    if ( !validFloatRange(payload.lineResolution, 0.01f, 20.0f) ) return String(F("lineResolution ")) + formatFloatRangeDetail(payload.lineResolution, 0.01f, 20.0f);
    if ( !validFloatRange(payload.stepsToCm, 1.0f, 5000.0f) ) return String(F("stepsToCm ")) + formatFloatRangeDetail(payload.stepsToCm, 1.0f, 5000.0f);
    if ( !validFloatRange(payload.quadHomeScan, 0.0f, payload.width) ) return String(F("quadHomeScan ")) + formatFloatRangeDetail(payload.quadHomeScan, 0.0f, payload.width);
    if ( !validFloatRange(payload.quadHomeFeed, 0.0f, payload.height) ) return String(F("quadHomeFeed ")) + formatFloatRangeDetail(payload.quadHomeFeed, 0.0f, payload.height);
    if ( !validFloatRange(payload.quadCableAFeed, 0.5f, 1.5f) ) return String(F("quadCableAFeed ")) + formatFloatRangeDetail(payload.quadCableAFeed, 0.5f, 1.5f);
    if ( !validFloatRange(payload.quadCableBFeed, 0.5f, 1.5f) ) return String(F("quadCableBFeed ")) + formatFloatRangeDetail(payload.quadCableBFeed, 0.5f, 1.5f);
    if ( !validFloatRange(payload.quadCableCFeed, 0.5f, 1.5f) ) return String(F("quadCableCFeed ")) + formatFloatRangeDetail(payload.quadCableCFeed, 0.5f, 1.5f);
    if ( !validFloatRange(payload.quadCableDFeed, 0.5f, 1.5f) ) return String(F("quadCableDFeed ")) + formatFloatRangeDetail(payload.quadCableDFeed, 0.5f, 1.5f);
    if ( !validFloatRange(payload.quadDrawLiftValue, -50.0f, 50.0f) ) return String(F("quadDrawLiftValue ")) + formatFloatRangeDetail(payload.quadDrawLiftValue, -50.0f, 50.0f);
    if ( !validFloatRange(payload.quadTravelLiftValue, -50.0f, 50.0f) ) return String(F("quadTravelLiftValue ")) + formatFloatRangeDetail(payload.quadTravelLiftValue, -50.0f, 50.0f);
    return "";
  }

  return F("unknown robot kind");
}

boolean validateRobotSetup(const RobotSetupPayload& payload) {
  lastRobotSetupPayloadError = describeRobotSetupValidationError(payload);
  return !lastRobotSetupPayloadError.length();
}

void applyRobotSetup(const RobotSetupPayload& payload) {
  robotSetup = payload;
  normalizeRobotSetup(robotSetup);
  currentRobotKind = RobotKindId(robotSetup.robotKind);
  motorDistance = robotSetup.motorDistance;
  scanOffset = robotSetup.scanOffset;
  feedOffset = robotSetup.feedOffset;
  width = (
    currentRobotKind == robotKindFlatQuadTension
    ? robotSetup.width
    : motorDistance - scanOffset * 2.0f
  );
  height = robotSetup.height;
  lineResolution = robotSetup.lineResolution;
  homePosition = robotSetup.homePosition;
  leftCoilFeed = robotSetup.leftCoilFeed;
  rightCoilFeed = robotSetup.rightCoilFeed;
  stepsToCmBase = robotSetup.stepsToCm;
  stepsToCm = stepsToCmBase / microstepResolution;
  quadHomeScan = robotSetup.quadHomeScan;
  quadHomeFeed = robotSetup.quadHomeFeed;
  quadCableAFeed = robotSetup.quadCableAFeed;
  quadCableBFeed = robotSetup.quadCableBFeed;
  quadCableCFeed = robotSetup.quadCableCFeed;
  quadCableDFeed = robotSetup.quadCableDFeed;
  quadDrawLiftValue = robotSetup.quadDrawLiftValue;
  quadTravelLiftValue = robotSetup.quadTravelLiftValue;
  configureMotionAxisMapForRobotKind(currentRobotKind);
  stepLength = 1.0f / stepsToCm;
  if ( scan < 0.0f ) scan = 0.0f;
  if ( feed < 0.0f ) feed = 0.0f;
  if ( scan > width ) scan = width;
  if ( feed > height ) feed = height;
  desiredScan = scan;
  desiredFeed = feed;
}

void applyDefaultRobotSetupForKind(RobotKindId robotKind) {
  applyRobotSetup(defaultRobotSetupForKind(robotKind));
}

void applyDefaultRobotSetup() {
  applyDefaultRobotSetupForKind(robotKindHangingVBot);
}

RobotSetupPayload migrateLegacyRobotSetup(const LegacyRobotSetupPayloadV1& legacyPayload) {
  RobotSetupPayload payload = DEFAULT_ROBOT_SETUP;
  payload.robotKind = robotKindHangingVBot;
  payload.motorDistance = legacyPayload.motorDistance;
  payload.scanOffset = legacyPayload.scanOffset;
  payload.feedOffset = legacyPayload.feedOffset;
  payload.width = legacyPayload.motorDistance - legacyPayload.scanOffset * 2.0f;
  payload.height = legacyPayload.height;
  payload.lineResolution = legacyPayload.lineResolution;
  payload.homePosition = legacyPayload.homePosition;
  payload.leftCoilFeed = legacyPayload.leftCoilFeed;
  payload.rightCoilFeed = legacyPayload.rightCoilFeed;
  payload.stepsToCm = legacyPayload.stepsToCm;
  normalizeRobotSetup(payload);
  return payload;
}

boolean loadRobotSetupFromEEPROM() {
  RobotSetupBlockHeader header;
  EEPROM.get(ROBOT_SETUP_EEPROM_ADDRESS, header);
  if ( header.magic != ROBOT_SETUP_MAGIC ) return false;

  if (
    header.version == ROBOT_SETUP_VERSION
    && header.payloadSize == sizeof(RobotSetupPayload)
  ) {
    RobotSetupBlock block;
    EEPROM.get(ROBOT_SETUP_EEPROM_ADDRESS, block);
    if ( block.crc32 != calculateRobotSetupCRC(block.payload) ) return false;
    if ( !validateRobotSetup(block.payload) ) return false;
    applyRobotSetup(block.payload);
    return true;
  }

  if (
    header.version == LEGACY_ROBOT_SETUP_VERSION
    && header.payloadSize == sizeof(LegacyRobotSetupPayloadV1)
  ) {
    LegacyRobotSetupBlockV1 legacyBlock;
    EEPROM.get(ROBOT_SETUP_EEPROM_ADDRESS, legacyBlock);
    if ( legacyBlock.crc32 != calculateLegacyRobotSetupCRC(legacyBlock.payload) ) return false;

    RobotSetupPayload migrated = migrateLegacyRobotSetup(legacyBlock.payload);
    if ( !validateRobotSetup(migrated) ) return false;
    applyRobotSetup(migrated);
    return true;
  }

  return false;
}

void saveRobotSetupToEEPROM() {
  RobotSetupBlock block;
  block.magic = ROBOT_SETUP_MAGIC;
  block.version = ROBOT_SETUP_VERSION;
  block.payloadSize = sizeof(RobotSetupPayload);
  block.flags = 0;
  block.payload = robotSetup;
  normalizeRobotSetup(block.payload);
  block.crc32 = calculateRobotSetupCRC(block.payload);
  for ( byte index = 0; index < sizeof(block.reserved); index++ ) {
    block.reserved[index] = 0;
  }
  EEPROM.put(ROBOT_SETUP_EEPROM_ADDRESS, block);
}

boolean loadLegacyPositionBlockFromEEPROM() {
  PositionBlock block;
  EEPROM.get(LEGACY_POSITION_BLOCK_EEPROM_ADDRESS, block);
  if (
    block.magic == POSITION_MAGIC
    && block.crc32 == calculatePositionCRC(block.scanX100, block.feedX100)
  ) {
    scan = float(block.scanX100) / 100.0f;
    feed = float(block.feedX100) / 100.0f;
    return true;
  }
  return false;
}

boolean loadLegacyPositionFromEEPROM() {
  long legacyFeed = 0;
  long legacyScan = 0;
  EEPROM.get(LEGACY_FEED_EEPROM_ADDRESS, legacyFeed);
  EEPROM.get(LEGACY_SCAN_EEPROM_ADDRESS, legacyScan);

  if ( legacyFeed == -1L && legacyScan == -1L ) return false;
  if ( legacyScan < -50000L || legacyScan > 50000L ) return false;
  if ( legacyFeed < -50000L || legacyFeed > 50000L ) return false;

  scan = float(legacyScan) / 100.0f;
  feed = float(legacyFeed) / 100.0f;
  return true;
}

boolean loadPositionFromEEPROM() {
  PositionBlock block;
  EEPROM.get(POSITION_EEPROM_ADDRESS, block);
  if (
    block.magic == POSITION_MAGIC
    && block.crc32 == calculatePositionCRC(block.scanX100, block.feedX100)
  ) {
    scan = float(block.scanX100) / 100.0f;
    feed = float(block.feedX100) / 100.0f;
    return true;
  }

  if ( loadLegacyPositionBlockFromEEPROM() ) {
    savePositionToEEPROM();
    return true;
  }

  if ( loadLegacyPositionFromEEPROM() ) {
    savePositionToEEPROM();
    return true;
  }

  scan = 0.0f;
  feed = 0.0f;
  return false;
}

void savePositionToEEPROM() {
  PositionBlock block;
  block.magic = POSITION_MAGIC;
  block.scanX100 = long(scan * 100.0f);
  block.feedX100 = long(feed * 100.0f);
  block.crc32 = calculatePositionCRC(block.scanX100, block.feedX100);
  EEPROM.put(POSITION_EEPROM_ADDRESS, block);
}

void clearAllEEPROM() {
  for ( unsigned int address = 0; address < EEPROM.length(); address++ ) {
    EEPROM.update(address, 0xFF);
  }
}

MotionBackendId getMotionBackendForRobotKind(RobotKindId robotKind) {
  if ( robotKind == robotKindFlatQuadTension ) {
    return quadMotionPinsConfigured() ? motionBackendQuadFourAxis : motionBackendNone;
  }
  if ( robotKind == robotKindHangingVBot ) return motionBackendHangingTwoAxis;
  return motionBackendNone;
}

boolean motionImplementedForRobotKind(RobotKindId robotKind) {
  return getMotionBackendForRobotKind(robotKind) != motionBackendNone;
}

boolean manualInputPinAvailable(int pin) {
  return pin >= 0 && !activeMotionUsesPin(pin);
}

boolean manualInputIsLow(int pin) {
  return manualInputPinAvailable(pin) && digitalRead(pin) == LOW;
}

boolean manualInputIsHigh(int pin) {
  return manualInputPinAvailable(pin) && digitalRead(pin) == HIGH;
}

void reportUnsupportedMotion() {
  unsigned long now = millis();
  if ( now - lastUnsupportedMotionReportAt < 500UL ) return;
  lastUnsupportedMotionReportAt = now;
  Serial.println(F("error\tquad motion not implemented"));
}

void setContactState(ContactStateId nextState, boolean report) {
  if ( nextState == contactStateUnknown ) return;
  contactState = nextState;
  if ( report ) {
    Serial.print(F("contactState\t"));
    Serial.println(getContactStateToken(contactState));
  }
}

void updateCableTelemetryFromPosition() {
  if ( currentRobotKind == robotKindFlatQuadTension ) {
    computeQuadCableLengths(scan, feed, currentCableLengths);
    currentA = currentCableLengths[0];
    currentB = currentCableLengths[1];
    return;
  }

  currentA = getA(scan, feed);
  currentB = getB(scan, feed);
  currentCableLengths[0] = currentA;
  currentCableLengths[1] = currentB;
  currentCableLengths[2] = 0.0f;
  currentCableLengths[3] = 0.0f;
}

void printMotionCapability() {
  Serial.print(F("motionSupport\t"));
  Serial.println(getMotionSupportToken(motionImplementedForRobotKind(currentRobotKind)));
  Serial.print(F("motionBackend\t"));
  Serial.println(getMotionBackendToken(getMotionBackendForRobotKind(currentRobotKind)));
}

void printCableTelemetry() {
  Serial.print(F("cableLengths\t"));
  Serial.print(currentCableLengths[0], 4);
  Serial.print(F(","));
  Serial.print(currentCableLengths[1], 4);
  if ( currentRobotKind == robotKindFlatQuadTension ) {
    Serial.print(F(","));
    Serial.print(currentCableLengths[2], 4);
    Serial.print(F(","));
    Serial.println(currentCableLengths[3], 4);
  } else {
    Serial.println();
  }
}

void printRobotSetup() {
  Serial.print(F("robotKind\t"));
  Serial.println(getRobotKindToken(currentRobotKind));
  Serial.print(F("contactState\t"));
  Serial.println(getContactStateToken(contactState));
  printMotionCapability();
  Serial.print(F("robotSetupStatus\t"));
  Serial.println(robotSetupEEPROMValid ? F("valid") : F("invalid"));
  Serial.print(F("robotSetup\trobotKind\t"));
  Serial.println(getRobotKindToken(currentRobotKind));
  Serial.print(F("robotSetup\tmotorDistance\t"));
  Serial.println(motorDistance, 4);
  Serial.print(F("robotSetup\tscanOffset\t"));
  Serial.println(scanOffset, 4);
  Serial.print(F("robotSetup\tfeedOffset\t"));
  Serial.println(feedOffset, 4);
  Serial.print(F("robotSetup\twidth\t"));
  Serial.println(width, 4);
  Serial.print(F("robotSetup\theight\t"));
  Serial.println(height, 4);
  Serial.print(F("robotSetup\tlineResolution\t"));
  Serial.println(lineResolution, 4);
  Serial.print(F("robotSetup\thomePosition\t"));
  Serial.println(homePosition, 4);
  Serial.print(F("robotSetup\tleftCoilFeed\t"));
  Serial.println(leftCoilFeed, 4);
  Serial.print(F("robotSetup\trightCoilFeed\t"));
  Serial.println(rightCoilFeed, 4);
  Serial.print(F("robotSetup\tstepsToCm\t"));
  Serial.println(stepsToCmBase, 4);
  Serial.print(F("robotSetup\tquadHomeScan\t"));
  Serial.println(quadHomeScan, 4);
  Serial.print(F("robotSetup\tquadHomeFeed\t"));
  Serial.println(quadHomeFeed, 4);
  Serial.print(F("robotSetup\tquadCableAFeed\t"));
  Serial.println(quadCableAFeed, 4);
  Serial.print(F("robotSetup\tquadCableBFeed\t"));
  Serial.println(quadCableBFeed, 4);
  Serial.print(F("robotSetup\tquadCableCFeed\t"));
  Serial.println(quadCableCFeed, 4);
  Serial.print(F("robotSetup\tquadCableDFeed\t"));
  Serial.println(quadCableDFeed, 4);
  Serial.print(F("robotSetup\tquadDrawLiftValue\t"));
  Serial.println(quadDrawLiftValue, 4);
  Serial.print(F("robotSetup\tquadTravelLiftValue\t"));
  Serial.println(quadTravelLiftValue, 4);
}

boolean parseRobotSetupWritePayload(const String& rawValue, RobotSetupPayload& payload) {
  lastRobotSetupPayloadError = "";

  if ( rawValue.indexOf('=') < 0 ) {
    String values[9];
    for ( int index = 0; index < 9; index++ ) {
      values[index] = splitString(rawValue, ',', index);
      values[index].trim();
      if ( !values[index].length() ) {
        return failRobotSetupPayloadParse(
          String(F("missing legacy field at index ")) + String(index)
        );
      }
    }

    payload = DEFAULT_ROBOT_SETUP;
    payload.robotKind = robotKindHangingVBot;
    payload.motorDistance = values[0].toFloat();
    payload.scanOffset = values[1].toFloat();
    payload.feedOffset = values[2].toFloat();
    payload.width = payload.motorDistance - payload.scanOffset * 2.0f;
    payload.height = values[3].toFloat();
    payload.lineResolution = values[4].toFloat();
    payload.homePosition = values[5].toFloat();
    payload.leftCoilFeed = values[6].toFloat();
    payload.rightCoilFeed = values[7].toFloat();
    payload.stepsToCm = values[8].toFloat();
    normalizeRobotSetup(payload);
    return validateRobotSetup(payload);
  }

  RobotKindId requestedKind = currentRobotKind;
  for ( int index = 0; index < 24; index++ ) {
    String assignment = splitString(rawValue, ',', index);
    assignment.trim();
    if ( !assignment.length() ) break;
    String fieldName = splitString(assignment, '=', 0);
    String fieldValue = splitString(assignment, '=', 1);
    fieldName.trim();
    fieldValue.trim();
    if ( fieldName == "robotKind" ) {
      requestedKind = parseRobotKindToken(fieldValue);
      if ( requestedKind == robotKindUnknown ) {
        return failRobotSetupPayloadParse(
          String(F("unsupported robotKind '")) + fieldValue + String(F("'"))
        );
      }
      break;
    }
  }

  payload = defaultRobotSetupForKind(requestedKind);
  for ( int index = 0; index < 24; index++ ) {
    String assignment = splitString(rawValue, ',', index);
    assignment.trim();
    if ( !assignment.length() ) break;

    String fieldName = splitString(assignment, '=', 0);
    String fieldValue = splitString(assignment, '=', 1);
    fieldName.trim();
    fieldValue.trim();
    if ( !fieldName.length() ) {
      return failRobotSetupPayloadParse(
        String(F("missing field name in assignment '")) + assignment + String(F("'"))
      );
    }
    if ( !fieldValue.length() ) {
      return failRobotSetupPayloadParse(
        String(F("missing value for field '")) + fieldName + String(F("'"))
      );
    }

    if ( fieldName == "robotKind" ) {
      payload.robotKind = parseRobotKindToken(fieldValue);
      if ( payload.robotKind == robotKindUnknown ) {
        return failRobotSetupPayloadParse(
          String(F("unsupported robotKind '")) + fieldValue + String(F("'"))
        );
      }
    } else if ( fieldName == "motorDistance" ) {
      payload.motorDistance = fieldValue.toFloat();
    } else if ( fieldName == "scanOffset" ) {
      payload.scanOffset = fieldValue.toFloat();
    } else if ( fieldName == "feedOffset" ) {
      payload.feedOffset = fieldValue.toFloat();
    } else if ( fieldName == "width" ) {
      payload.width = fieldValue.toFloat();
    } else if ( fieldName == "height" ) {
      payload.height = fieldValue.toFloat();
    } else if ( fieldName == "lineResolution" ) {
      payload.lineResolution = fieldValue.toFloat();
    } else if ( fieldName == "homePosition" ) {
      payload.homePosition = fieldValue.toFloat();
    } else if ( fieldName == "stepsToCm" ) {
      payload.stepsToCm = fieldValue.toFloat();
    } else if ( fieldName == "leftCoilFeed" ) {
      payload.leftCoilFeed = fieldValue.toFloat();
    } else if ( fieldName == "rightCoilFeed" ) {
      payload.rightCoilFeed = fieldValue.toFloat();
    } else if ( fieldName == "quadHomeScan" ) {
      payload.quadHomeScan = fieldValue.toFloat();
    } else if ( fieldName == "quadHomeFeed" ) {
      payload.quadHomeFeed = fieldValue.toFloat();
    } else if ( fieldName == "quadCableAFeed" ) {
      payload.quadCableAFeed = fieldValue.toFloat();
    } else if ( fieldName == "quadCableBFeed" ) {
      payload.quadCableBFeed = fieldValue.toFloat();
    } else if ( fieldName == "quadCableCFeed" ) {
      payload.quadCableCFeed = fieldValue.toFloat();
    } else if ( fieldName == "quadCableDFeed" ) {
      payload.quadCableDFeed = fieldValue.toFloat();
    } else if ( fieldName == "quadDrawLiftValue" ) {
      payload.quadDrawLiftValue = fieldValue.toFloat();
    } else if ( fieldName == "quadTravelLiftValue" ) {
      payload.quadTravelLiftValue = fieldValue.toFloat();
    } else {
      return failRobotSetupPayloadParse(
        String(F("unsupported field '")) + fieldName + String(F("'"))
      );
    }
  }

  normalizeRobotSetup(payload);
  return validateRobotSetup(payload);
}


///////////////////////////////////////////////////
// SETUP                                         //
///////////////////////////////////////////////////

void setup() {
  //setting up controller
  pinMode(toggle1, INPUT_PULLUP);
  pinMode(toggle2, INPUT_PULLUP);
  pinMode(toggle3, INPUT_PULLUP);
  pinMode(toggle4, INPUT_PULLUP);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);

  //setting up the steppers
  pinMode(DIR_LEFT_PIN, OUTPUT);
  pinMode(STEP_LEFT_PIN, OUTPUT);
  pinMode(DIR_RIGHT_PIN, OUTPUT);
  pinMode(STEP_RIGHT_PIN, OUTPUT);
  configureMotionPulseTimer();

  //setting up microstepping
  pinMode(MICROSTEP_PIN, OUTPUT);
  digitalWrite(MICROSTEP_PIN, HIGH);

  //testing if case is connected
  //we haven't found a good way yet to test this.

  robotSetupEEPROMValid = loadRobotSetupFromEEPROM();
  if ( !robotSetupEEPROMValid ) {
    applyDefaultRobotSetup();
  }

  if ( !loadPositionFromEEPROM() ) {
    savePositionToEEPROM();
  }
  desiredScan = scan;
  desiredFeed = feed;
  updateCableTelemetryFromPosition();


  Serial.begin(115200);
  Serial.println(F("_____________________________________"));
  Serial.println(FIRMWARE_COMPAT_ID);
  Serial.println(F("_____________________________________"));

  Serial.println(F("_____________________________________"));
  Serial.println(F("SETTINGS"));
  Serial.println(F("_____________________________________"));
  Serial.print(F("detectCase:\t"));
  Serial.println(detectCase);
  Serial.print(F("canvasWidth:\t"));
  Serial.println(width);
  Serial.print(F("canvasHeight:\t"));
  Serial.println(height);
  Serial.print(F("robotKind\t"));
  Serial.println(getRobotKindToken(currentRobotKind));
  Serial.print(F("contactState\t"));
  Serial.println(getContactStateToken(contactState));
  printMotionCapability();
  Serial.print(F("monitoring:\t"));
  Serial.println(monitoring);
  Serial.print(F("upperBound:\t"));
  Serial.println(upperBound);
  Serial.print(F("lowerBound:\t"));
  Serial.println(lowerBound);
  Serial.print(F("leftBound:\t"));
  Serial.println(leftBound);
  Serial.print(F("rightBound:\t"));
  Serial.println(rightBound);
  Serial.print(F("ms at:\t"));
  Serial.println(microstepResolution, 5);
  Serial.print(F("stepperDelay:\t"));
  Serial.println(minStepperDelay);
  Serial.println("");
  printRobotSetup();
  Serial.println("");


  Serial.println(F("_____________________________________"));
  Serial.println(F("reading EEPROM"));
  Serial.print(scan);
  Serial.print(",");
  Serial.println(feed);
  Serial.println("_____________________________________");
  Serial.println("");

  currentA = getA(scan, feed);
  currentB = getB(scan, feed);

  enterState(noSD);
  if ( initialiseSD() ) enterState(idle);

  if ( scan != 0 && feed != 0 ) {
    Serial.println(F("_____________________________________"));
    Serial.println(F("PEN IS NOT AT ORIGIN!"));
    Serial.println(F("_____________________________________"));

  }

  Serial.println(F(">drawFromFile, file"));
  Serial.println(F(">abort"));
  Serial.println(F(">pause"));
  Serial.println(F(">continue"));
  Serial.println(F(">move, scan, feed"));
  Serial.println(F(">contact\\tdraw|travel"));
  Serial.println(F(">stepL, amount"));
  Serial.println(F(">stepR, amount"));
  Serial.println(F(">monitoring on/off"));
  Serial.println(F(">returnToOrigin"));
  Serial.println(F(">outlineCanvas"));
  Serial.println(F(">resetHome"));
  Serial.println(F(">returnToHome"));
  Serial.println(F(">position"));
  Serial.println(F(">robotSetupGet"));
  Serial.println(F(">robotSetupWrite\\t62,10,20,50,0.5,82,1,0.997,35"));
  Serial.println(F(">robotSetupWrite\\trobotKind=flat_quad_tension,scanOffset=0,feedOffset=0,width=42,height=50,lineResolution=0.5,stepsToCm=35,quadHomeScan=21,quadHomeFeed=25,quadCableAFeed=1,quadCableBFeed=1,quadCableCFeed=1,quadCableDFeed=1,quadDrawLiftValue=0,quadTravelLiftValue=1"));
  Serial.println(F(">robotSetupDefaults"));
  Serial.println(F(">clearEEPROM"));
}

void loop() {
  controller();

  switch (machineState) {

    case idle:
      handleIdleState();
      break;

    case drawing:
      handleDrawingState();
      break;

    case pausing:
      handlePausedState();
      break;

    case aborting:
      handleAbortingState();
      break;

    case launchpad:
      handleLaunchpadState();
      break;

    case noSD:
      handleNoSDState();
      break;

    default:
      Serial.println(F("state->default"));
      enterState(idle);
      break;

  }

}

const char* getStateName(MachineState state) {
  switch (state) {
    case idle: return "idle";
    case drawing: return "drawing";
    case pausing: return "pausing";
    case aborting: return "aborting";
    case launchpad: return "launchpad";
    case noSD: return "noSD";
    default: return "unknown";
  }
}

void enterState(MachineState nextState) {
  if ( nextState == machineState ) return;
  machineState = nextState;
  Serial.print(F("state->"));
  Serial.println(getStateName(machineState));

  if ( machineState == drawing ) {
    drawOutcome = drawNone;
    drawingFileOpen = false;
    streamMoveQueuePaused = false;
  } else if ( machineState == aborting && drawOutcome == drawNone ) {
    drawOutcome = drawAborted;
  } else if ( machineState == noSD ) {
    lastSDRetryAt = millis();
  }
}

void clearPendingCommand() {
  pendingCommand = cmdNone;
  pendingArgument1 = 0.0f;
  pendingArgument2 = 0.0f;
  pendingText1 = "";
  pendingText2 = "";
  hasPendingCommand = false;
}

boolean hasQueuedStreamMove() {
  return streamMoveQueueCount > 0;
}

boolean streamQueueActive() {
  return streamMoveQueuePaused || hasQueuedStreamMove();
}

boolean enqueueStreamMove(float scanPos, float feedPos) {
  if ( streamMoveQueueCount >= streamMoveQueueCapacity ) return false;

  streamMoveQueueScan[streamMoveQueueTail] = scanPos;
  streamMoveQueueFeed[streamMoveQueueTail] = feedPos;
  streamMoveQueueTail = (streamMoveQueueTail + 1) % streamMoveQueueCapacity;
  streamMoveQueueCount++;
  return true;
}

boolean popQueuedStreamMove(float& scanPos, float& feedPos) {
  if ( !hasQueuedStreamMove() ) return false;

  scanPos = streamMoveQueueScan[streamMoveQueueHead];
  feedPos = streamMoveQueueFeed[streamMoveQueueHead];
  streamMoveQueueHead = (streamMoveQueueHead + 1) % streamMoveQueueCapacity;
  streamMoveQueueCount--;
  return true;
}

boolean executeNextQueuedStreamMove() {
  if ( !motionImplementedForRobotKind(currentRobotKind) ) {
    reportUnsupportedMotion();
    return false;
  }

  float scanPos;
  float feedPos;
  if ( !popQueuedStreamMove(scanPos, feedPos) ) return false;

  Serial.print(F("moving to\t"));
  Serial.print(scanPos, 4);
  Serial.print(F(","));
  Serial.println(feedPos, 4);
  gesture(scanPos, feedPos);
  Serial.print(F("moveComplete\t"));
  Serial.print(scan, 4);
  Serial.print(F(","));
  Serial.println(feed, 4);
  return true;
}

void finishPendingCommand() {
  Serial.println(F("ok"));
  clearPendingCommand();
}

void failPendingCommand(const String& message) {
  Serial.print(F("error\t"));
  Serial.println(message);
  clearPendingCommand();
}

void rejectPendingCommand() {
  failPendingCommand(String(F("command unavailable in state ")) + getStateName(machineState));
}

boolean handleSharedCommand() {
  if ( !hasPendingCommand ) return false;

  switch (pendingCommand) {
    case cmdAbort:
      if ( machineState == drawing || machineState == pausing || streamQueueActive() ) {
        drawOutcome = drawAborted;
        clearPendingCommand();
        enterState(aborting);
        return true;
      }
      return false;

    case cmdSetSpeed:
      minStepperDelay = int(pendingArgument1);
      minStepperPulse = int(pendingArgument1);
      Serial.print(F("speed set at\t"));
      Serial.println(int(pendingArgument1));
      finishPendingCommand();
      return true;

    case cmdMonitoringOn:
      monitoring = true;
      Serial.println(F("monitoring on"));
      finishPendingCommand();
      return true;

    case cmdMonitoringOff:
      monitoring = false;
      Serial.println(F("monitoring off"));
      finishPendingCommand();
      return true;

    case cmdPosition:
      printPosition();
      finishPendingCommand();
      return true;

    case cmdRobotSetupGet:
      printRobotSetup();
      finishPendingCommand();
      return true;

    case cmdClearEEPROM:
      clearAllEEPROM();
      robotSetupEEPROMValid = false;
      Serial.println(F("eeprom cleared"));
      printRobotSetup();
      finishPendingCommand();
      return true;

    case cmdTerminate:
      terminate();
      finishPendingCommand();
      return true;

    default:
      return false;
  }
}

boolean handleManualMotionCommand() {
  if ( !hasPendingCommand ) return false;

  switch (pendingCommand) {
    case cmdMove:
      if ( !motionImplementedForRobotKind(currentRobotKind) ) {
        failPendingCommand(F("quad motion not implemented"));
        return true;
      }
      type = "absolute";
      Serial.print(F("moving to\t"));
      Serial.print(pendingArgument1);
      Serial.print(",");
      Serial.println(pendingArgument2);
      gesture(pendingArgument1, pendingArgument2);
      printPosition();
      finishPendingCommand();
      return true;

    case cmdStreamMove:
      if ( !motionImplementedForRobotKind(currentRobotKind) ) {
        failPendingCommand(F("quad motion not implemented"));
        return true;
      }
      if ( enqueueStreamMove(pendingArgument1, pendingArgument2) ) {
        finishPendingCommand();
        return true;
      }

      if ( executeNextQueuedStreamMove() && enqueueStreamMove(pendingArgument1, pendingArgument2) ) {
        finishPendingCommand();
        return true;
      }

      failPendingCommand(F("stream move queue stalled"));
      return true;

    case cmdStepL:
      if ( !motionImplementedForRobotKind(currentRobotKind) ) {
        failPendingCommand(F("quad motion not implemented"));
        return true;
      }
      Serial.print(F("stepping left motor: "));
      Serial.println(int(pendingArgument1));
      stepL(int(pendingArgument1));
      finishPendingCommand();
      return true;

    case cmdStepR:
      if ( !motionImplementedForRobotKind(currentRobotKind) ) {
        failPendingCommand(F("quad motion not implemented"));
        return true;
      }
      Serial.print(F("stepping right motor: "));
      Serial.println(int(pendingArgument1));
      stepR(int(pendingArgument1));
      finishPendingCommand();
      return true;

    case cmdSetType:
      if ( hasQueuedStreamMove() ) {
        failPendingCommand(F("stream move queue not empty"));
        return true;
      }
      type = pendingText1;
      finishPendingCommand();
      return true;

    case cmdSetMode:
      if ( hasQueuedStreamMove() ) {
        failPendingCommand(F("stream move queue not empty"));
        return true;
      }
      if ( !applyMotionMode(pendingText1) ) {
        failPendingCommand(F("unsupported mode"));
        return true;
      }
      finishPendingCommand();
      return true;

    case cmdSetAdjustment:
      if ( hasQueuedStreamMove() ) {
        failPendingCommand(F("stream move queue not empty"));
        return true;
      }
      adjustmentType = pendingText1;
      finishPendingCommand();
      return true;

    case cmdSetContact: {
      if ( hasQueuedStreamMove() ) {
        failPendingCommand(F("stream move queue not empty"));
        return true;
      }
      ContactStateId nextState = parseContactStateToken(pendingText1);
      if ( nextState == contactStateUnknown ) {
        failPendingCommand(F("unsupported contact state"));
        return true;
      }
      setContactState(nextState, true);
      finishPendingCommand();
      return true;
    }

    case cmdOutlineCanvas:
      if ( !motionImplementedForRobotKind(currentRobotKind) ) {
        failPendingCommand(F("quad motion not implemented"));
        return true;
      }
      type = "absolute";
      printPosition();
      for ( int x = 0; x <= width; x++ ) {
        gesture(x, 0);
      }
      printPosition();
      for ( int x = 0; x <= height; x++ ) {
        gesture(width, x);
      }
      printPosition();
      for ( int x = 0; x <= width; x++ ) {
        gesture(width - x, height);
      }
      printPosition();
      for ( int x = 0; x <= height; x++ ) {
        gesture(0, height - x);
      }
      printPosition();
      Serial.println(F("done"));
      finishPendingCommand();
      return true;

    case cmdReturnToOrigin:
      if ( !motionImplementedForRobotKind(currentRobotKind) ) {
        failPendingCommand(F("quad motion not implemented"));
        return true;
      }
      returnToOrigin();
      if ( machineState == launchpad ) {
        enterState(idle);
      }
      finishPendingCommand();
      return true;

    case cmdReturnToHome:
      if ( !motionImplementedForRobotKind(currentRobotKind) ) {
        failPendingCommand(F("quad motion not implemented"));
        return true;
      }
      returnToHome();
      finishPendingCommand();
      return true;

    case cmdResetHome:
      resetHome();
      enterState(launchpad);
      finishPendingCommand();
      return true;

    case cmdPause:
      streamMoveQueuePaused = true;
      Serial.println(F("stream queue paused"));
      finishPendingCommand();
      return true;

    case cmdContinue:
      streamMoveQueuePaused = false;
      Serial.println(F("stream queue resumed"));
      finishPendingCommand();
      return true;

    case cmdRobotSetupWrite: {
      RobotSetupPayload payload;
      if ( !parseRobotSetupWritePayload(pendingText1, payload) ) {
        String message = F("invalid robot setup payload");
        if ( lastRobotSetupPayloadError.length() ) {
          message += F(": ");
          message += lastRobotSetupPayloadError;
        }
        failPendingCommand(message);
        return true;
      }
      applyRobotSetup(payload);
      saveRobotSetupToEEPROM();
      robotSetupEEPROMValid = true;
      updateCableTelemetryFromPosition();
      printRobotSetup();
      printPosition();
      finishPendingCommand();
      return true;
    }

    case cmdRobotSetupLoad:
      robotSetupEEPROMValid = loadRobotSetupFromEEPROM();
      if ( !robotSetupEEPROMValid ) applyDefaultRobotSetup();
      updateCableTelemetryFromPosition();
      printRobotSetup();
      printPosition();
      finishPendingCommand();
      return true;

    case cmdRobotSetupDefaults:
      applyDefaultRobotSetupForKind(currentRobotKind);
      saveRobotSetupToEEPROM();
      robotSetupEEPROMValid = true;
      updateCableTelemetryFromPosition();
      printRobotSetup();
      printPosition();
      finishPendingCommand();
      return true;

    default:
      return false;
  }
}

boolean handleDrawingInstruction() {
  if ( !readInstruction() ) return false;

  if ( dataCommand == "move" ) {
    if ( !gesture(dataXPos, dataYPos) ) {
      return false;
    }
  } else if ( dataCommand == "type" ) {
    type = dataValue;
  } else if ( dataCommand == "mode" ) {
    if ( !applyMotionMode(dataValue) ) {
      Serial.print(F("unknown mode:\t"));
      Serial.println(dataValue);
    }
  } else if ( dataCommand == "adjustment" ) {
    adjustmentType = dataValue;
  } else if ( dataCommand == "contact" ) {
    ContactStateId nextState = parseContactStateToken(dataValue);
    if ( nextState == contactStateUnknown ) {
      Serial.print(F("unknown contact:\t"));
      Serial.println(dataValue);
    } else {
      setContactState(nextState, true);
    }
  } else if ( dataCommand.length() ) {
    Serial.print(F("unknown instruction:\t"));
    Serial.println(dataCommand);
  }

  return true;
}

void handleIdleState() {
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, LOW);

  filePointer = getRotaryPosition(A14);
  setCarriageSpeed();

  if ( handleSharedCommand() ) return;
  if ( handleManualMotionCommand() ) return;

  if ( hasPendingCommand ) {
    if ( pendingCommand == cmdDrawFromFile ) {
      if ( !motionImplementedForRobotKind(currentRobotKind) ) {
        failPendingCommand(F("quad motion not implemented"));
        return;
      }
      if ( streamQueueActive() ) {
        failPendingCommand(F("stream move queue not empty"));
        return;
      }
      if ( !initialiseSDQuietly() ) {
        Serial.println(F("SD unavailable"));
        enterState(noSD);
      } else {
        filePointer = int(pendingArgument1);
        clearPendingCommand();
        enterState(drawing);
      }
      return;
    }

    rejectPendingCommand();
    return;
  }

  if ( !streamMoveQueuePaused && executeNextQueuedStreamMove() ) return;

  if ( manualInputIsLow(toggle1) ) {
    if ( !motionImplementedForRobotKind(currentRobotKind) ) {
      reportUnsupportedMotion();
      return;
    }
    if ( streamQueueActive() ) return;
    if ( initialiseSDQuietly() ) {
      enterState(drawing);
    } else {
      Serial.println(F("SD unavailable"));
      enterState(noSD);
    }
    return;
  }

  if ( manualInputIsLow(toggle3) ) {
    resetHome();
    enterState(launchpad);
    return;
  }

  if ( manualInputIsLow(toggle4) ) {
    returnToOrigin();
  }
}

void handleDrawingState() {
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, HIGH);
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, LOW);

  setCarriageSpeed();

  if ( !drawingFileOpen ) {
    if ( !openDataFile(filePointer) ) {
      drawOutcome = drawError;
      enterState(aborting);
      return;
    }
    drawingFileOpen = true;
  }

  if ( handleSharedCommand() ) return;

  if ( hasPendingCommand ) {
    if ( pendingCommand == cmdAbort ) {
      drawOutcome = drawAborted;
      clearPendingCommand();
      enterState(aborting);
      return;
    }

    if ( pendingCommand == cmdPause ) {
      clearPendingCommand();
      enterState(pausing);
      return;
    }

    rejectPendingCommand();
    return;
  }

  if ( manualInputIsHigh(toggle1) ) {
    drawOutcome = drawAborted;
    enterState(aborting);
    return;
  }

  if ( manualInputIsLow(toggle2) ) {
    enterState(pausing);
    return;
  }

  if ( !dataFile || !dataFile.available() ) {
    drawOutcome = drawFinished;
    enterState(aborting);
    return;
  }

  if ( !handleDrawingInstruction() ) {
    drawOutcome = drawError;
    enterState(aborting);
  }
}

void handlePausedState() {
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, HIGH);
  digitalWrite(LED3, HIGH);
  digitalWrite(LED4, LOW);

  if ( handleSharedCommand() ) return;

  if ( hasPendingCommand ) {
    if ( pendingCommand == cmdAbort ) {
      drawOutcome = drawAborted;
      clearPendingCommand();
      enterState(aborting);
      return;
    }

    if ( pendingCommand == cmdContinue ) {
      clearPendingCommand();
      enterState(drawing);
      return;
    }

    rejectPendingCommand();
    return;
  }

  if ( manualInputIsHigh(toggle1) ) {
    drawOutcome = drawAborted;
    enterState(aborting);
    return;
  }

  if ( manualInputIsHigh(toggle2) ) {
    enterState(drawing);
  }
}

void handleAbortingState() {
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);

  if ( dataFile ) {
    dataFile.close();
    Serial.println(F("closing dataFile"));
  }
  drawingFileOpen = false;
  streamMoveQueueHead = 0;
  streamMoveQueueTail = 0;
  streamMoveQueueCount = 0;
  streamMoveQueuePaused = false;

  if ( drawOutcome == drawFinished ) {
    Serial.println(F("drawing complete"));
    if ( motionImplementedForRobotKind(currentRobotKind) ) {
      returnToOrigin();
    }
  } else if ( drawOutcome == drawAborted ) {
    Serial.println(F("drawing aborted"));
  } else if ( drawOutcome == drawError ) {
    Serial.println(F("drawing error"));
  }

  gestureCount = 0;
  drawOutcome = drawNone;
  enterState(idle);
}

void handleLaunchpadState() {
  if ( loopCounter > 0 && loopCounter < cycleLength / 2 ) {
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, HIGH);
    digitalWrite(LED3, HIGH);
    digitalWrite(LED4, HIGH);
  } else {
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    digitalWrite(LED3, LOW);
    digitalWrite(LED4, LOW);
  }
  loopCounter++;
  if ( loopCounter == cycleLength ) loopCounter = 0;

  if ( handleSharedCommand() ) return;
  if ( handleManualMotionCommand() ) return;
  if ( hasPendingCommand ) {
    rejectPendingCommand();
    return;
  }

  if ( !streamMoveQueuePaused && executeNextQueuedStreamMove() ) return;

  if ( manualInputIsLow(toggle4) ) {
    returnToOrigin();
    enterState(idle);
  }
}

void handleNoSDState() {
  if ( loopCounter > 0 && loopCounter < cycleLength / 2 ) {
    digitalWrite(LED1, HIGH);
  } else {
    digitalWrite(LED1, LOW);
  }
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, LOW);

  loopCounter++;
  if ( loopCounter == cycleLength ) loopCounter = 0;

  filePointer = getRotaryPosition(A14);
  setCarriageSpeed();

  if ( handleSharedCommand() ) return;
  if ( handleManualMotionCommand() ) return;

  if ( hasPendingCommand ) {
    if ( pendingCommand == cmdRetrySD ) {
      if ( initialiseSD() ) enterState(idle);
      clearPendingCommand();
      return;
    }

    if ( pendingCommand == cmdDrawFromFile ) {
      if ( !motionImplementedForRobotKind(currentRobotKind) ) {
        failPendingCommand(F("quad motion not implemented"));
        return;
      }
      if ( streamQueueActive() ) {
        failPendingCommand(F("stream move queue not empty"));
        return;
      }
      if ( initialiseSD() ) {
        filePointer = int(pendingArgument1);
        clearPendingCommand();
        enterState(drawing);
      } else {
        clearPendingCommand();
      }
      return;
    }

    rejectPendingCommand();
    return;
  }

  if ( !streamMoveQueuePaused && executeNextQueuedStreamMove() ) return;

  if ( manualInputIsLow(toggle3) ) {
    resetHome();
    enterState(launchpad);
    return;
  }

  if ( manualInputIsLow(toggle4) ) {
    returnToOrigin();
    return;
  }

  if ( millis() - lastSDRetryAt >= sdRetryIntervalMs ) {
    lastSDRetryAt = millis();
    if ( initialiseSDQuietly() ) enterState(idle);
  }
}

///////////////////////////////////////////////////
//                                               //
// FUNCTION FAMILIES                             //
//                                               //
///////////////////////////////////////////////////

///////////////////////////////////////////////////
// GESTURE FUNCTION FAMILY                       //
///////////////////////////////////////////////////

boolean gesture(float xPos, float yPos) {
  if ( !motionImplementedForRobotKind(currentRobotKind) ) {
    reportUnsupportedMotion();
    return false;
  }
  movePenBresenham(xPos, yPos);
  return true;
}

float segmentAdjustment(float scanLine, float feedLine, String getter) {
  if ( adjustmentType == "none" && getter == "scan" ) return 0;
  if ( adjustmentType == "none" && getter == "feed" ) return 0;
  if ( adjustmentType == "largeSin" && getter == "scan" ) return 3 * cos(feedLine / PI);
  if ( adjustmentType == "largeSin" && getter == "feed" ) return 3 * sin(scanLine / PI);
  if ( adjustmentType == "complexSin" && getter == "scan" )
    return 3 * cos(feedLine / PI) + 0.3 * cos(5 * feedLine / PI);
  if ( adjustmentType == "complexSin" && getter == "feed" )
    return 3 * sin(scanLine / PI) + 0.3 * sin(5 * scanLine / PI);
  if ( adjustmentType == "noise" && getter == "scan" )
    return 20 * float(inoise8_raw(scanLine / 5, feedLine / 5)) / 70;
  if ( adjustmentType == "noise" && getter == "feed" )
    return 20 * float(inoise8_raw(scanLine / 5, feedLine / 5)) / 70;
}

///////////////////////////////////////////////////
// SD FUNCTION FAMILY                            //
///////////////////////////////////////////////////

boolean attemptSDInitialisation(boolean verbose) {
  if ( verbose ) Serial.println(F("Initialising SD card..."));
  if ( !SD.begin(10, 11, 12, 13)) {
    if ( verbose ) Serial.println(F("Initialisation failed"));
    return false;
  }

  if ( verbose ) {
    Serial.println(F("Initialisation done"));
    Serial.println("");
  }
  return true;
}

boolean initialiseSD() {
  return attemptSDInitialisation(true);
}

boolean initialiseSDQuietly() {
  return attemptSDInitialisation(false);
}

boolean openDataFile(int file) {
  if ( dataFile ) dataFile.close();

  if ( file == 0 ) dataFile = SD.open("0.txt", FILE_READ);
  if ( file == 1 ) dataFile = SD.open("1.txt", FILE_READ);
  if ( file == 2 ) dataFile = SD.open("2.txt", FILE_READ);
  if ( file == 3 ) dataFile = SD.open("3.txt", FILE_READ);
  if ( file == 4 ) dataFile = SD.open("4.txt", FILE_READ);
  if ( file == 5 ) dataFile = SD.open("5.txt", FILE_READ);
  if ( file == 6 ) dataFile = SD.open("6.txt", FILE_READ);
  if ( file == 7 ) dataFile = SD.open("7.txt", FILE_READ);
  if ( file == 8 ) dataFile = SD.open("8.txt", FILE_READ);
  if ( file == 9 ) dataFile = SD.open("9.txt", FILE_READ);
  if ( file == 10 ) dataFile = SD.open("10.txt", FILE_READ);
  if ( file == 11 ) dataFile = SD.open("11.txt", FILE_READ);
  if ( !dataFile ) {
    Serial.println(F("->error opening data.txt"));
    return false;
  }

  Serial.print(F("opened data file\t"));
  Serial.println(file);
  return true;
}

boolean initialiseWriteData(int file) {
  if ( dataFile ) dataFile.close();

  if ( file == 0 ) dataFile = SD.open("0.txt", FILE_WRITE | O_TRUNC);
  if ( file == 1 ) dataFile = SD.open("1.txt", FILE_WRITE | O_TRUNC);
  if ( file == 2 ) dataFile = SD.open("2.txt", FILE_WRITE | O_TRUNC);
  if ( file == 3 ) dataFile = SD.open("3.txt", FILE_WRITE | O_TRUNC);
  if ( file == 4 ) dataFile = SD.open("4.txt", FILE_WRITE | O_TRUNC);
  if ( file == 5 ) dataFile = SD.open("5.txt", FILE_WRITE | O_TRUNC);
  if ( file == 6 ) dataFile = SD.open("6.txt", FILE_WRITE | O_TRUNC);
  if ( file == 7 ) dataFile = SD.open("7.txt", FILE_WRITE | O_TRUNC);
  if ( file == 8 ) dataFile = SD.open("8.txt", FILE_WRITE | O_TRUNC);
  if ( file == 9 ) dataFile = SD.open("9.txt", FILE_WRITE | O_TRUNC);
  if ( file == 10 ) dataFile = SD.open("10.txt", FILE_WRITE | O_TRUNC);
  if ( file == 11 ) dataFile = SD.open("11.txt", FILE_WRITE | O_TRUNC);
  if ( !dataFile ) {
    Serial.println(F("->error opening data.txt"));
    return false;
  }

  Serial.println(F("dataFile open"));
  Serial.println(F("started writing data"));
  return true;
}

void writeGesture(float xPos, float yPos) {
  if ( dataFile ) {
    dataFile.print("move\t");
    dataFile.print(xPos);
    dataFile.print(",");
    dataFile.println(yPos);
  }
}

void writeType(String value) {
  if ( dataFile ) {
    dataFile.print("type\t");
    dataFile.println(value);
  }
}

void writeMode(String value) {
  if ( dataFile ) {
    String normalizedMode = normalizeMotionMode(value);
    dataFile.print("mode\t");
    if ( normalizedMode.length() ) {
      dataFile.println(normalizedMode);
    } else {
      dataFile.println(value);
    }
  }
}

void writeAdjustment(String value) {
  if ( dataFile ) {
    dataFile.print("adjustment\t");
    dataFile.println(value);
  }
}

void writeContact(String value) {
  if ( dataFile ) {
    dataFile.print("contact\t");
    dataFile.println(value);
  }
}

boolean readInstruction() {
  String command;
  String value;
  String line;
  dataCommand = "";
  dataValue = "";
  dataXPos = 0.0f;
  dataYPos = 0.0f;

  if ( !dataFile.available()) return false;

  line = dataFile.readStringUntil('\n');
  command = splitString(line, '\t', 0);
  value = splitString(line, '\t', 1);
  command.trim();
  value.trim();

  if ( command == "move" ) {
    String stXPos = splitString(value, ',', 0);
    String stYPos = splitString(value, ',', 1);
    dataCommand = command;
    dataXPos = stXPos.toFloat();
    dataYPos = stYPos.toFloat();
  } else {
    dataValue = value;
    dataCommand = command;
  }

  return true;
}

void closeData() {
  dataFile.close();
  Serial.println("dataFile written and closed");
}

String splitString(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

///////////////////////////////////////////////////
// INTERFACING FUNCTIONS                         //
///////////////////////////////////////////////////

int setCarriageSpeed() {
  int stepperDelay = map(getRotaryPosition(A15), 1, 11, 400, 10);
  minStepperDelay = stepperDelay;
  minStepperPulse = stepperDelay;
}

float getRotaryPosition(int readPin) {
  int valuePad = 25;
  rotary1 = analogRead(readPin);
  if ( rotary1 > 1023 - valuePad && rotary1 < 1023 + valuePad ) {
    return 1;
  }
  if ( rotary1 > 922 - valuePad && rotary1 < 922 + valuePad ) {
    return 2;
  }
  if ( rotary1 > 819 - valuePad && rotary1 < 819 + valuePad ) {
    return 3;
  }
  if ( rotary1 > 717 - valuePad && rotary1 < 717 + valuePad ) {
    return 4;
  }
  if ( rotary1 > 614 - valuePad && rotary1 < 614 + valuePad ) {
    return 5;
  }
  if ( rotary1 > 511 - valuePad && rotary1 < 511 + valuePad ) {
    return 6;
  }
  if ( rotary1 > 408 - valuePad && rotary1 < 408 + valuePad ) {
    return 7;
  }
  if ( rotary1 > 306 - valuePad && rotary1 < 306 + valuePad ) {
    return 8;
  }
  if ( rotary1 > 203 - valuePad && rotary1 < 203 + valuePad ) {
    return 9;
  }
  if ( rotary1 > 100 - valuePad && rotary1 < 100 + valuePad ) {
    return 10;
  }
  if ( rotary1 > 0 - valuePad && rotary1 < 0 + valuePad ) {
    return 11;
  }
}

///////////////////////////////////////////////////
// LOW LEVEL FUNCTIONS                           //
///////////////////////////////////////////////////
// getDeltaA/B takes the desired feed and scan from the new
// position and returns the change in wire length in cm

float getDeltaA(float desS, float desF, float cA) {
  float desA = getA(desS, desF);
  return desA - cA;
}

float getDeltaB(float desS, float desF, float cB) {
  float desB = getB(desS, desF);
  return desB - cB;
}

float getA(float s, float f) {
  float a = sqrt(sq(f + feedOffset) + sq(s + scanOffset));
  return a;
}

float getB(float s, float f) {
  float b = sqrt(sq(f + feedOffset) + sq(width + scanOffset - s));
  return b;
}

float getScanAndFeed(float a, float b, String getter) {
  float x = (a + b + width + 2 * scanOffset) / 2; //calculate semiperimeter
  float f = 2 * sqrt(x * (x - a) * (x - b) * (x - (width + 2 * scanOffset)));
  f = f / (width + 2 * scanOffset) - feedOffset;
  float s = sqrt(sq(a) - sq(f + feedOffset)) - scanOffset;
  if ( getter == "feed" ) return f;
  if ( getter == "scan" ) return s;
}

float cartesianInput(float heading, float distance, String getter) {
  if ( getter == "feed" ) return distance * -sin(heading * (PI / 180));
  if ( getter == "scan" ) return distance * cos(heading * (PI / 180));
}

boolean getIntersection
(float p0_x, float p0_y, float p1_x, float p1_y,
 float p2_x, float p2_y, float p3_x, float p3_y, float ix, float iy ) {
  //returns true if the linesegments intersect, false otherwise
  //intersection point gets stored in ix, iy
  float s1_x, s1_y, s2_x, s2_y;
  s1_x = p1_x - p0_x;
  s1_y = p1_y - p0_y;
  s2_x = p3_x - p2_x;
  s2_y = p3_y - p2_y;

  float s, t;
  s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) /
      (-s2_x * s1_y + s1_x * s2_y);
  t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) /
      (-s2_x * s1_y + s1_x * s2_y);

  if (s >= 0 && s <= 1 && t >= 0 && t <= 1) {
    ix = p0_x + (t * s1_x);
    iy = p0_y + (t * s1_y);
    return true;
  }
  return false; // No collision
}

float getPointByDistance
(float x1, float y1, float x2, float y2, float dist, String getter) {
  float l = sqrt(sq(x1 - x2) + sq(y1 - y2));
  float dx = (x2 - x1) / l;
  float dy = (y2 - y1) / l;
  if ( getter == "scan" ) return x1 + dx * dist;
  if ( getter == "feed" ) return y1 + dy * dist;
}

float dist(float x1, float y1, float x2, float y2) {
  return sqrt(sq(x1 - x2) + sq(y1 - y2));
}

float noise(float x, float y) {
  float baseReduction = 35;
  return float(inoise8_raw(x, y)) / baseReduction;
}

float complexNoise(float x, float y, int octaves ) {
  float value = noise(x, y);
  for ( int i = 1; i < octaves; i++ ) {
    value += (1 / pow(2, i)) * noise(x * pow(2, i), y * pow(2, i));
  }
  return value;
}

void terminate() {
  //Serial.println("");
  //Serial.println("stopping program");
  //Serial.println("saving scan and feed to EEPROM");
  feedINT = feed * 100;
  scanINT = scan * 100;
  updateCableTelemetryFromPosition();
  savePositionToEEPROM();
  //Serial.print(feedINT);
  //Serial.print(",");
  //Serial.println(scanINT);
}

void resetHome() {
  if ( currentRobotKind == robotKindFlatQuadTension ) {
    scan = quadHomeScan;
    feed = quadHomeFeed;
  } else {
    feed = homePosition - feedOffset;
    scan = width / 2;
  }
  desiredScan = scan;
  desiredFeed = feed;
  updateCableTelemetryFromPosition();
  Serial.println("_____________________________________");
  Serial.println("carriage reset at: ");
  Serial.print(scan);
  Serial.print(",");
  Serial.println(feed);
  Serial.print("robotKind\t");
  Serial.println(getRobotKindToken(currentRobotKind));
  Serial.print("contactState\t");
  Serial.println(getContactStateToken(contactState));
  printMotionCapability();
  printCableTelemetry();
  Serial.println("_____________________________________");
  Serial.println("");
  terminate();
}

void returnToOrigin() {
  if ( !motionImplementedForRobotKind(currentRobotKind) ) {
    reportUnsupportedMotion();
    return;
  }
  Serial.println("returning to origin");
  digitalWrite(LED4, HIGH);
  type = "absolute";
  gesture(0, 0);
  terminate();
  Serial.println("carriage at origin");
  digitalWrite(LED4, LOW);
}

void returnToHome() {
  if ( !motionImplementedForRobotKind(currentRobotKind) ) {
    reportUnsupportedMotion();
    return;
  }
  Serial.println("returning to home");
  type = "absolute";
  if ( currentRobotKind == robotKindFlatQuadTension ) {
    gesture(quadHomeScan, quadHomeFeed);
  } else {
    gesture(width / 2, homePosition - feedOffset);
  }
  terminate();
  Serial.println("carriage at home");
}

void printPosition() {
  updateCableTelemetryFromPosition();
  Serial.println("_____________________________________");
  Serial.print("robotKind\t");
  Serial.println(getRobotKindToken(currentRobotKind));
  Serial.print("contactState\t");
  Serial.println(getContactStateToken(contactState));
  printMotionCapability();
  Serial.println("carriage at:");
  Serial.print(scan);
  Serial.print(",");
  Serial.println(feed);
  Serial.println("lineLength A/B:");
  Serial.print(currentA);
  Serial.print(",");
  Serial.println(currentB);
  printCableTelemetry();
  Serial.println("_____________________________________");
}

///////////////////////////////////////////////////
// MOTOR LEVEL FUNCTIONS                         //
///////////////////////////////////////////////////

void stepL(int amount) {
  runBresenhamSteps(amount, 0);
}

void stepR(int amount) {
  runBresenhamSteps(0, amount);
}
