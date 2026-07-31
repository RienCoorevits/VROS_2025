#include <FastLED.h>
#include <EEPROM.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>

// heightmaps
//https://tangrams.github.io/heightmapper

const char FIRMWARE_PRODUCT_NAME[] = "VROS_caseController";
const char FIRMWARE_SEMVER[] = "2.5.17";
const char FIRMWARE_COMPAT_ID[] = "VROS_2.5.17_caseController";
const byte HOST_PROTOCOL_VERSION = 1;


// pin definitions
#define DIR_LEFT_PIN 6
#define STEP_LEFT_PIN 7
#define DIR_RIGHT_PIN 8
#define STEP_RIGHT_PIN 9
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

enum MotionBackendId {
  motionBackendNone = 0,
  motionBackendHangingTwoAxis = 1,
  motionBackendQuadFourAxis = 2
};

const byte MAX_MOTION_AXES = 4;
const byte HANGING_VBOT_AXIS_COUNT = 2;
const byte FLAT_QUAD_AXIS_COUNT = 4;
extern byte activeMotionAxisCount;

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

struct LegacyRobotSetupPayloadV2 {
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

struct LegacyRobotSetupPayloadV3 {
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
  float microstepResolution;
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
  float microstepResolution;
  float leftCoilFeed;
  float rightCoilFeed;
  float quadHomeScan;
  float quadHomeFeed;
  float quadCableAFeed;
  float quadCableBFeed;
  float quadCableCFeed;
  float quadCableDFeed;
  float quadMotorHeight;
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

struct LegacyRobotSetupBlockV2 {
  unsigned long magic;
  byte version;
  byte payloadSize;
  unsigned int flags;
  unsigned long crc32;
  LegacyRobotSetupPayloadV2 payload;
  byte reserved[8];
};

struct LegacyRobotSetupBlockV3 {
  unsigned long magic;
  byte version;
  byte payloadSize;
  unsigned int flags;
  unsigned long crc32;
  LegacyRobotSetupPayloadV3 payload;
  byte reserved[4];
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

struct LegacyPositionBlock {
  unsigned long magic;
  long scanX100;
  long feedX100;
  unsigned long crc32;
};

struct PositionBlock {
  unsigned long magic;
  long scanX100;
  long feedX100;
  long zX100;
  unsigned long crc32;
};

struct SpeedSettingsPayload {
  unsigned int stepperDelay;
  unsigned int stepperPulse;
};

struct SpeedSettingsBlock {
  unsigned long magic;
  byte version;
  byte payloadSize;
  unsigned int flags;
  unsigned long crc32;
  SpeedSettingsPayload payload;
};

const unsigned long ROBOT_SETUP_MAGIC = 0x56525331UL;
const byte LEGACY_ROBOT_SETUP_VERSION_V1 = 1;
const byte LEGACY_ROBOT_SETUP_VERSION_V2 = 2;
const byte LEGACY_ROBOT_SETUP_VERSION_V3 = 3;
const byte ROBOT_SETUP_VERSION = 4;
const int ROBOT_SETUP_EEPROM_ADDRESS = 0;
const unsigned long SPEED_SETTINGS_MAGIC = 0x53504431UL;
const byte SPEED_SETTINGS_VERSION = 1;
const int SPEED_SETTINGS_EEPROM_ADDRESS = 96;
const unsigned long LEGACY_POSITION_MAGIC = 0x504F5331UL;
const unsigned long POSITION_MAGIC = 0x504F5332UL;
const int POSITION_EEPROM_ADDRESS = 128;
const int LEGACY_POSITION_BLOCK_EEPROM_ADDRESS = 64;
const int LEGACY_FEED_EEPROM_ADDRESS = 0;
const int LEGACY_SCAN_EEPROM_ADDRESS = 15;
static_assert(
  SPEED_SETTINGS_EEPROM_ADDRESS >= ROBOT_SETUP_EEPROM_ADDRESS + sizeof(RobotSetupBlock),
  "Speed settings EEPROM address overlaps robot setup block"
);
static_assert(
  SPEED_SETTINGS_EEPROM_ADDRESS + sizeof(SpeedSettingsBlock) <= POSITION_EEPROM_ADDRESS,
  "Speed settings EEPROM block overlaps position block"
);
const unsigned int DEFAULT_STEPPER_DELAY = 50;
const unsigned int DEFAULT_STEPPER_PULSE = 50;
const float DEFAULT_MICROSTEP_RESOLUTION = 0.0625f;
const float MICROSTEP_RESOLUTION_OPTIONS[] = {
  1.0f,
  0.5f,
  0.25f,
  0.125f,
  0.0625f,
  0.03125f
};
const byte MICROSTEP_RESOLUTION_OPTION_COUNT = sizeof(MICROSTEP_RESOLUTION_OPTIONS) / sizeof(MICROSTEP_RESOLUTION_OPTIONS[0]);

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
  DEFAULT_MICROSTEP_RESOLUTION,
  1.00f,
  0.997f,
  21.00f,
  25.00f,
  1.00f,
  1.00f,
  1.00f,
  1.00f,
  10.00f
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
  DEFAULT_MICROSTEP_RESOLUTION,
  1.00f,
  1.00f,
  21.00f,
  25.00f,
  1.00f,
  1.00f,
  1.00f,
  1.00f,
  10.00f
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
float quadMotorHeight = DEFAULT_ROBOT_SETUP.quadMotorHeight;
String lastRobotSetupPayloadError = "";
unsigned long lastUnsupportedMotionReportAt = 0UL;

boolean detectCase = false;

float stepLength;
float scan;
float feed;
float carriageZ;
float desiredScan;
float desiredFeed;
float desiredZ;
float currentA;
float currentB;
float currentCableLengths[4] = {0.0f, 0.0f, 0.0f, 0.0f};

//speed vars
int minStepperDelay = DEFAULT_STEPPER_DELAY; //200
int minStepperPulse = DEFAULT_STEPPER_PULSE;


//Drawing vars
String type = "absolute"; // relative/absolute
long gestureCount; //counts the gestures in the drawing

//EEPROM vars
long scanINT;
long feedINT;
long zINT;

// SD vars
File dataFile;
int filePointer;
String dataCommand;
float dataXPos;
float dataYPos;
float dataZPos;
boolean dataHasZ = false;
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
  cmdStepA,
  cmdStepB,
  cmdStepC,
  cmdStepD,
  cmdStepAll,
  cmdMotorsOn,
  cmdMotorsOff,
  cmdMoveX,
  cmdMoveY,
  cmdMoveZ,
  cmdMoveLeft,
  cmdMoveRight,
  cmdMoveUp,
  cmdMoveDown,
  cmdSetType,
  cmdSetMode,
  cmdSetAdjustment,
  cmdSetContact,
  cmdOutlineCanvas,
  cmdReturnToOrigin,
  cmdReturnToHome,
  cmdFeedToHome,
  cmdResetHome,
  cmdTerminate,
  cmdMonitoringOn,
  cmdMonitoringOff,
  cmdAbort,
  cmdPause,
  cmdContinue,
  cmdPosition,
  cmdGetSpeed,
  cmdSaveSpeed,
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
float pendingArgument3 = 0.0f;
boolean pendingArgument3Provided = false;
String pendingText1 = "";
String pendingText2 = "";
boolean hasPendingCommand = false;
boolean drawingFileOpen = false;
const int streamMoveQueueCapacity = 12;
float streamMoveQueueScan[streamMoveQueueCapacity];
float streamMoveQueueFeed[streamMoveQueueCapacity];
float streamMoveQueueZ[streamMoveQueueCapacity];
boolean streamMoveQueueHasZ[streamMoveQueueCapacity];
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
float microstepResolution = DEFAULT_ROBOT_SETUP.microstepResolution;

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

unsigned long calculateLegacyRobotSetupCRC(const LegacyRobotSetupPayloadV2& payload) {
  return crc32Update(0UL, (const byte*)&payload, sizeof(LegacyRobotSetupPayloadV2));
}

unsigned long calculateLegacyRobotSetupCRC(const LegacyRobotSetupPayloadV3& payload) {
  return crc32Update(0UL, (const byte*)&payload, sizeof(LegacyRobotSetupPayloadV3));
}

unsigned long calculateLegacyPositionCRC(long scanX100Value, long feedX100Value) {
  unsigned long crc = 0UL;
  crc = crc32Update(crc, (const byte*)&scanX100Value, sizeof(long));
  crc = crc32Update(crc, (const byte*)&feedX100Value, sizeof(long));
  return crc;
}

unsigned long calculatePositionCRC(long scanX100Value, long feedX100Value, long zX100Value) {
  unsigned long crc = 0UL;
  crc = crc32Update(crc, (const byte*)&scanX100Value, sizeof(long));
  crc = crc32Update(crc, (const byte*)&feedX100Value, sizeof(long));
  crc = crc32Update(crc, (const byte*)&zX100Value, sizeof(long));
  return crc;
}

unsigned long calculateSpeedSettingsCRC(const SpeedSettingsPayload& payload) {
  unsigned long crc = 0UL;
  crc = crc32Update(crc, (const byte*)&payload.stepperDelay, sizeof(payload.stepperDelay));
  crc = crc32Update(crc, (const byte*)&payload.stepperPulse, sizeof(payload.stepperPulse));
  return crc;
}

boolean validFloatRange(float value, float minValue, float maxValue) {
  return value >= minValue && value <= maxValue;
}

boolean validMicrostepResolution(float value) {
  for ( byte index = 0; index < MICROSTEP_RESOLUTION_OPTION_COUNT; index++ ) {
    if ( fabsf(value - MICROSTEP_RESOLUTION_OPTIONS[index]) < 0.00001f ) return true;
  }
  return false;
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

boolean isLegacyContactStateToken(const String& value) {
  return value == "draw" || value == "travel";
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

void emitProtocolPrefix(const __FlashStringHelper* section, const __FlashStringHelper* fieldName) {
  Serial.print(F("vros\t"));
  Serial.print(section);
  Serial.print(F("\t"));
  Serial.print(fieldName);
  Serial.print(F("\t"));
}

void emitProtocolText(
  const __FlashStringHelper* section,
  const __FlashStringHelper* fieldName,
  const __FlashStringHelper* value
) {
  emitProtocolPrefix(section, fieldName);
  Serial.println(value);
}

void emitProtocolText(
  const __FlashStringHelper* section,
  const __FlashStringHelper* fieldName,
  const char* value
) {
  emitProtocolPrefix(section, fieldName);
  Serial.println(value);
}

void emitProtocolText(
  const __FlashStringHelper* section,
  const __FlashStringHelper* fieldName,
  const String& value
) {
  emitProtocolPrefix(section, fieldName);
  Serial.println(value);
}

void emitProtocolBool(
  const __FlashStringHelper* section,
  const __FlashStringHelper* fieldName,
  boolean value
) {
  emitProtocolPrefix(section, fieldName);
  Serial.println(value ? F("1") : F("0"));
}

void emitProtocolInt(
  const __FlashStringHelper* section,
  const __FlashStringHelper* fieldName,
  long value
) {
  emitProtocolPrefix(section, fieldName);
  Serial.println(value);
}

void emitProtocolFloat(
  const __FlashStringHelper* section,
  const __FlashStringHelper* fieldName,
  float value,
  byte precision = 4
) {
  emitProtocolPrefix(section, fieldName);
  Serial.println(value, precision);
}

void emitProtocolPair(
  const __FlashStringHelper* section,
  const __FlashStringHelper* fieldName,
  float first,
  float second,
  byte precision = 4
) {
  emitProtocolPrefix(section, fieldName);
  Serial.print(first, precision);
  Serial.print(F(","));
  Serial.println(second, precision);
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
  if ( !validMicrostepResolution(payload.microstepResolution) ) {
    return F("microstepResolution must be one of 1, 0.5, 0.25, 0.125, 0.0625, 0.03125");
  }

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
    if ( !validFloatRange(payload.quadMotorHeight, 0.01f, 500.0f) ) return String(F("quadMotorHeight ")) + formatFloatRangeDetail(payload.quadMotorHeight, 0.01f, 500.0f);
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
  microstepResolution = robotSetup.microstepResolution;
  stepsToCm = stepsToCmBase / microstepResolution;
  quadHomeScan = robotSetup.quadHomeScan;
  quadHomeFeed = robotSetup.quadHomeFeed;
  quadCableAFeed = robotSetup.quadCableAFeed;
  quadCableBFeed = robotSetup.quadCableBFeed;
  quadCableCFeed = robotSetup.quadCableCFeed;
  quadCableDFeed = robotSetup.quadCableDFeed;
  quadMotorHeight = robotSetup.quadMotorHeight;
  configureMotionAxisMapForRobotKind(currentRobotKind);
  stepLength = 1.0f / stepsToCm;
  if ( scan < 0.0f ) scan = 0.0f;
  if ( feed < 0.0f ) feed = 0.0f;
  if ( scan > width ) scan = width;
  if ( feed > height ) feed = height;
  if ( currentRobotKind == robotKindFlatQuadTension ) {
    if ( carriageZ < 0.0f ) carriageZ = 0.0f;
    if ( carriageZ > quadMotorHeight ) carriageZ = quadMotorHeight;
  } else {
    carriageZ = 0.0f;
  }
  desiredScan = scan;
  desiredFeed = feed;
  desiredZ = carriageZ;
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

RobotSetupPayload migrateLegacyRobotSetupV2(const LegacyRobotSetupPayloadV2& legacyPayload) {
  RobotKindId robotKind = (
    legacyPayload.robotKind == robotKindFlatQuadTension
    ? robotKindFlatQuadTension
    : robotKindHangingVBot
  );
  RobotSetupPayload payload = defaultRobotSetupForKind(robotKind);
  payload.robotKind = byte(robotKind);
  payload.motorDistance = legacyPayload.motorDistance;
  payload.scanOffset = legacyPayload.scanOffset;
  payload.feedOffset = legacyPayload.feedOffset;
  payload.width = legacyPayload.width;
  payload.height = legacyPayload.height;
  payload.lineResolution = legacyPayload.lineResolution;
  payload.homePosition = legacyPayload.homePosition;
  payload.stepsToCm = legacyPayload.stepsToCm;
  payload.microstepResolution = DEFAULT_MICROSTEP_RESOLUTION;
  payload.leftCoilFeed = legacyPayload.leftCoilFeed;
  payload.rightCoilFeed = legacyPayload.rightCoilFeed;
  payload.quadHomeScan = legacyPayload.quadHomeScan;
  payload.quadHomeFeed = legacyPayload.quadHomeFeed;
  payload.quadCableAFeed = legacyPayload.quadCableAFeed;
  payload.quadCableBFeed = legacyPayload.quadCableBFeed;
  payload.quadCableCFeed = legacyPayload.quadCableCFeed;
  payload.quadCableDFeed = legacyPayload.quadCableDFeed;
  payload.quadMotorHeight = DEFAULT_QUAD_ROBOT_SETUP.quadMotorHeight;
  normalizeRobotSetup(payload);
  return payload;
}

RobotSetupPayload migrateLegacyRobotSetupV3(const LegacyRobotSetupPayloadV3& legacyPayload) {
  RobotKindId robotKind = (
    legacyPayload.robotKind == robotKindFlatQuadTension
    ? robotKindFlatQuadTension
    : robotKindHangingVBot
  );
  RobotSetupPayload payload = defaultRobotSetupForKind(robotKind);
  payload.robotKind = byte(robotKind);
  payload.motorDistance = legacyPayload.motorDistance;
  payload.scanOffset = legacyPayload.scanOffset;
  payload.feedOffset = legacyPayload.feedOffset;
  payload.width = legacyPayload.width;
  payload.height = legacyPayload.height;
  payload.lineResolution = legacyPayload.lineResolution;
  payload.homePosition = legacyPayload.homePosition;
  payload.stepsToCm = legacyPayload.stepsToCm;
  payload.microstepResolution = legacyPayload.microstepResolution;
  payload.leftCoilFeed = legacyPayload.leftCoilFeed;
  payload.rightCoilFeed = legacyPayload.rightCoilFeed;
  payload.quadHomeScan = legacyPayload.quadHomeScan;
  payload.quadHomeFeed = legacyPayload.quadHomeFeed;
  payload.quadCableAFeed = legacyPayload.quadCableAFeed;
  payload.quadCableBFeed = legacyPayload.quadCableBFeed;
  payload.quadCableCFeed = legacyPayload.quadCableCFeed;
  payload.quadCableDFeed = legacyPayload.quadCableDFeed;
  payload.quadMotorHeight = DEFAULT_QUAD_ROBOT_SETUP.quadMotorHeight;
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
    header.version == LEGACY_ROBOT_SETUP_VERSION_V3
    && header.payloadSize == sizeof(LegacyRobotSetupPayloadV3)
  ) {
    LegacyRobotSetupBlockV3 legacyBlock;
    EEPROM.get(ROBOT_SETUP_EEPROM_ADDRESS, legacyBlock);
    if ( legacyBlock.crc32 != calculateLegacyRobotSetupCRC(legacyBlock.payload) ) return false;

    RobotSetupPayload migrated = migrateLegacyRobotSetupV3(legacyBlock.payload);
    if ( !validateRobotSetup(migrated) ) return false;
    applyRobotSetup(migrated);
    return true;
  }

  if (
    header.version == LEGACY_ROBOT_SETUP_VERSION_V2
    && header.payloadSize == sizeof(LegacyRobotSetupPayloadV2)
  ) {
    LegacyRobotSetupBlockV2 legacyBlock;
    EEPROM.get(ROBOT_SETUP_EEPROM_ADDRESS, legacyBlock);
    if ( legacyBlock.crc32 != calculateLegacyRobotSetupCRC(legacyBlock.payload) ) return false;

    RobotSetupPayload migrated = migrateLegacyRobotSetupV2(legacyBlock.payload);
    if ( !validateRobotSetup(migrated) ) return false;
    applyRobotSetup(migrated);
    return true;
  }

  if (
    header.version == LEGACY_ROBOT_SETUP_VERSION_V1
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
  LegacyPositionBlock block;
  EEPROM.get(LEGACY_POSITION_BLOCK_EEPROM_ADDRESS, block);
  if (
    block.magic == LEGACY_POSITION_MAGIC
    && block.crc32 == calculateLegacyPositionCRC(block.scanX100, block.feedX100)
  ) {
    scan = float(block.scanX100) / 100.0f;
    feed = float(block.feedX100) / 100.0f;
    carriageZ = 0.0f;
    return true;
  }
  return false;
}

boolean loadLegacyPositionBlockFromAddress(int address) {
  LegacyPositionBlock block;
  EEPROM.get(address, block);
  if (
    block.magic == LEGACY_POSITION_MAGIC
    && block.crc32 == calculateLegacyPositionCRC(block.scanX100, block.feedX100)
  ) {
    scan = float(block.scanX100) / 100.0f;
    feed = float(block.feedX100) / 100.0f;
    carriageZ = 0.0f;
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
  carriageZ = 0.0f;
  return true;
}

boolean loadPositionFromEEPROM() {
  PositionBlock block;
  EEPROM.get(POSITION_EEPROM_ADDRESS, block);
  if (
    block.magic == POSITION_MAGIC
    && block.crc32 == calculatePositionCRC(block.scanX100, block.feedX100, block.zX100)
  ) {
    scan = float(block.scanX100) / 100.0f;
    feed = float(block.feedX100) / 100.0f;
    carriageZ = float(block.zX100) / 100.0f;
    return true;
  }

  if ( loadLegacyPositionBlockFromAddress(POSITION_EEPROM_ADDRESS) ) {
    savePositionToEEPROM();
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
  carriageZ = 0.0f;
  return false;
}

void savePositionToEEPROM() {
  PositionBlock block;
  block.magic = POSITION_MAGIC;
  block.scanX100 = long(scan * 100.0f);
  block.feedX100 = long(feed * 100.0f);
  block.zX100 = long(carriageZ * 100.0f);
  block.crc32 = calculatePositionCRC(block.scanX100, block.feedX100, block.zX100);
  EEPROM.put(POSITION_EEPROM_ADDRESS, block);
}

void applySpeedSettings(const SpeedSettingsPayload& payload) {
  minStepperDelay = int(payload.stepperDelay);
  minStepperPulse = int(payload.stepperPulse);
  if ( minStepperDelay < 1 ) minStepperDelay = 1;
  if ( minStepperPulse < 1 ) minStepperPulse = 1;
}

void applyDefaultSpeedSettings() {
  SpeedSettingsPayload payload = {
    DEFAULT_STEPPER_DELAY,
    DEFAULT_STEPPER_PULSE
  };
  applySpeedSettings(payload);
}

boolean validateSpeedSettings(const SpeedSettingsPayload& payload) {
  if ( payload.stepperDelay < 1 || payload.stepperDelay > 60000U ) return false;
  if ( payload.stepperPulse < 1 || payload.stepperPulse > 60000U ) return false;
  return true;
}

boolean loadSpeedSettingsFromEEPROM() {
  SpeedSettingsBlock block;
  EEPROM.get(SPEED_SETTINGS_EEPROM_ADDRESS, block);
  if ( block.magic != SPEED_SETTINGS_MAGIC ) return false;
  if ( block.version != SPEED_SETTINGS_VERSION ) return false;
  if ( block.payloadSize != sizeof(SpeedSettingsPayload) ) return false;
  if ( block.crc32 != calculateSpeedSettingsCRC(block.payload) ) return false;
  if ( !validateSpeedSettings(block.payload) ) return false;
  applySpeedSettings(block.payload);
  return true;
}

void saveSpeedSettingsToEEPROM() {
  SpeedSettingsBlock block;
  block.magic = SPEED_SETTINGS_MAGIC;
  block.version = SPEED_SETTINGS_VERSION;
  block.payloadSize = sizeof(SpeedSettingsPayload);
  block.flags = 0;
  block.payload.stepperDelay = (unsigned int)minStepperDelay;
  block.payload.stepperPulse = (unsigned int)minStepperPulse;
  block.crc32 = calculateSpeedSettingsCRC(block.payload);
  EEPROM.put(SPEED_SETTINGS_EEPROM_ADDRESS, block);
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
  emitProtocolText(F("event"), F("error"), F("motion not implemented for current robot"));
}

void updateCableTelemetryFromPosition() {
  if ( currentRobotKind == robotKindFlatQuadTension ) {
    computeQuadCableLengths(scan, feed, carriageZ, currentCableLengths);
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
  emitProtocolText(
    F("status"),
    F("motionSupport"),
    getMotionSupportToken(motionImplementedForRobotKind(currentRobotKind))
  );
  emitProtocolText(
    F("status"),
    F("motionBackend"),
    getMotionBackendToken(getMotionBackendForRobotKind(currentRobotKind))
  );
}

void printCableTelemetry() {
  emitProtocolPrefix(F("status"), F("cableLengths"));
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

void emitSpeedStatus() {
  emitProtocolInt(F("status"), F("speedDelayMs"), long(minStepperDelay));
}

void printSpeed() {
  Serial.print(F("speedDelayMs\t"));
  Serial.println(minStepperDelay);
  emitSpeedStatus();
}

void printRobotSetup() {
  emitProtocolText(F("status"), F("robotKind"), getRobotKindToken(currentRobotKind));
  printMotionCapability();
  emitProtocolText(
    F("config"),
    F("robotSetupStatus"),
    robotSetupEEPROMValid ? F("valid") : F("invalid")
  );
  emitProtocolText(F("config"), F("robotSetup.robotKind"), getRobotKindToken(currentRobotKind));
  emitProtocolFloat(F("config"), F("robotSetup.motorDistance"), motorDistance);
  emitProtocolFloat(F("config"), F("robotSetup.scanOffset"), scanOffset);
  emitProtocolFloat(F("config"), F("robotSetup.feedOffset"), feedOffset);
  emitProtocolFloat(F("config"), F("robotSetup.width"), width);
  emitProtocolFloat(F("config"), F("robotSetup.height"), height);
  emitProtocolFloat(F("config"), F("robotSetup.lineResolution"), lineResolution);
  emitProtocolFloat(F("config"), F("robotSetup.homePosition"), homePosition);
  emitProtocolFloat(F("config"), F("robotSetup.leftCoilFeed"), leftCoilFeed);
  emitProtocolFloat(F("config"), F("robotSetup.rightCoilFeed"), rightCoilFeed);
  emitProtocolFloat(F("config"), F("robotSetup.stepsToCm"), stepsToCmBase);
  emitProtocolFloat(F("config"), F("robotSetup.microstepResolution"), microstepResolution, 5);
  emitProtocolFloat(F("config"), F("robotSetup.quadHomeScan"), quadHomeScan);
  emitProtocolFloat(F("config"), F("robotSetup.quadHomeFeed"), quadHomeFeed);
  emitProtocolFloat(F("config"), F("robotSetup.quadCableAFeed"), quadCableAFeed);
  emitProtocolFloat(F("config"), F("robotSetup.quadCableBFeed"), quadCableBFeed);
  emitProtocolFloat(F("config"), F("robotSetup.quadCableCFeed"), quadCableCFeed);
  emitProtocolFloat(F("config"), F("robotSetup.quadCableDFeed"), quadCableDFeed);
  emitProtocolFloat(F("config"), F("robotSetup.quadMotorHeight"), quadMotorHeight);
  emitSpeedStatus();
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
    String legacyMicrostepValue = splitString(rawValue, ',', 9);
    legacyMicrostepValue.trim();
    if ( legacyMicrostepValue.length() ) {
      payload.microstepResolution = legacyMicrostepValue.toFloat();
    }
    normalizeRobotSetup(payload);
    return validateRobotSetup(payload);
  }

  RobotKindId requestedKind = currentRobotKind;
  for ( int index = 0; index < 32; index++ ) {
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
  for ( int index = 0; index < 32; index++ ) {
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
    } else if ( fieldName == "microstepResolution" ) {
      payload.microstepResolution = fieldValue.toFloat();
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
    } else if ( fieldName == "quadMotorHeight" ) {
      payload.quadMotorHeight = fieldValue.toFloat();
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

  //testing if case is connected
  //we haven't found a good way yet to test this.

  robotSetupEEPROMValid = loadRobotSetupFromEEPROM();
  if ( !robotSetupEEPROMValid ) {
    applyDefaultRobotSetup();
  }

  if ( !loadSpeedSettingsFromEEPROM() ) {
    applyDefaultSpeedSettings();
  }

  if ( !loadPositionFromEEPROM() ) {
    savePositionToEEPROM();
  }
  desiredScan = scan;
  desiredFeed = feed;
  desiredZ = carriageZ;
  updateCableTelemetryFromPosition();


  Serial.begin(115200);
  Serial.println(F("_____________________________________"));
  Serial.println(FIRMWARE_COMPAT_ID);
  Serial.println(F("_____________________________________"));
  emitProtocolInt(F("compat"), F("protocolVersion"), HOST_PROTOCOL_VERSION);
  emitProtocolText(F("compat"), F("firmwareVersion"), FIRMWARE_COMPAT_ID);
  emitProtocolText(F("compat"), F("product"), FIRMWARE_PRODUCT_NAME);

  Serial.println(F("_____________________________________"));
  Serial.println(F("SETTINGS"));
  Serial.println(F("_____________________________________"));
  Serial.print(F("detectCase:\t"));
  Serial.println(detectCase);
  Serial.print(F("canvasWidth:\t"));
  Serial.println(width);
  emitProtocolFloat(F("status"), F("canvasWidth"), width);
  Serial.print(F("canvasHeight:\t"));
  Serial.println(height);
  emitProtocolFloat(F("status"), F("canvasHeight"), height);
  printMotionCapability();
  Serial.print(F("monitoring:\t"));
  Serial.println(monitoring);
  emitProtocolBool(F("status"), F("monitoring"), monitoring);
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
  emitSpeedStatus();
  Serial.println("");
  printRobotSetup();
  Serial.println("");


  Serial.println(F("_____________________________________"));
  Serial.println(F("reading EEPROM"));
  Serial.print(scan);
  Serial.print(",");
  Serial.print(feed);
  Serial.print(",");
  Serial.println(carriageZ);
  Serial.println("_____________________________________");
  Serial.println("");

  currentA = getA(scan, feed);
  currentB = getB(scan, feed);

  enterState(noSD);
  if ( initialiseSD() ) enterState(idle);

  if ( scan != 0 || feed != 0 || carriageZ != 0 ) {
    Serial.println(F("_____________________________________"));
    Serial.println(F("PEN IS NOT AT ORIGIN!"));
    Serial.println(F("_____________________________________"));
    emitProtocolText(F("status"), F("positionConfidence"), F("stale-eeprom"));
  }

  Serial.println(F(">drawFromFile, file"));
  Serial.println(F(">abort"));
  Serial.println(F(">pause"));
  Serial.println(F(">continue"));
  Serial.println(F(">move, scan, feed[,z]"));
  Serial.println(F(">moveX, deltaScan"));
  Serial.println(F(">moveY, deltaFeed"));
  Serial.println(F(">moveZ, deltaZ"));
  Serial.println(F(">moveLeft, distance"));
  Serial.println(F(">moveRight, distance"));
  Serial.println(F(">moveUp, distance"));
  Serial.println(F(">moveDown, distance"));
  Serial.println(F(">contact\\tdraw|travel (legacy no-op)"));
  Serial.println(F(">stepA, amount"));
  Serial.println(F(">stepB, amount"));
  Serial.println(F(">stepC, amount"));
  Serial.println(F(">stepD, amount"));
  Serial.println(F(">stepAll, amount"));
  Serial.println(F(">stepL, amount"));
  Serial.println(F(">stepR, amount"));
  Serial.println(F(">motors on/off"));
  Serial.println(F(">monitoring on/off"));
  Serial.println(F(">returnToOrigin"));
  Serial.println(F(">outlineCanvas"));
  Serial.println(F(">resetHome"));
  Serial.println(F(">returnToHome"));
  Serial.println(F(">feedToHome"));
  Serial.println(F(">position"));
  Serial.println(F(">getSpeed"));
  Serial.println(F(">saveSpeed"));
  Serial.println(F(">robotSetupGet"));
  Serial.println(F(">robotSetupWrite\\t62,10,20,50,0.5,82,1,0.997,35[,0.0625]"));
  Serial.println(F(">robotSetupWrite\\trobotKind=hanging_vbot,motorDistance=62,scanOffset=10,feedOffset=20,height=50,lineResolution=0.5,homePosition=82,leftCoilFeed=1,rightCoilFeed=0.997,stepsToCm=35,microstepResolution=0.0625"));
  Serial.println(F(">robotSetupWrite\\trobotKind=flat_quad_tension,scanOffset=0,feedOffset=0,width=42,height=50,lineResolution=0.5,stepsToCm=35,microstepResolution=0.0625,quadHomeScan=21,quadHomeFeed=25,quadCableAFeed=1,quadCableBFeed=1,quadCableCFeed=1,quadCableDFeed=1,quadMotorHeight=10"));
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
      emitProtocolText(F("event"), F("error"), F("unknown state fallback"));
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
  emitProtocolText(F("status"), F("state"), getStateName(machineState));
  emitProtocolBool(F("status"), F("sdAvailable"), machineState != noSD);

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
  pendingArgument3 = 0.0f;
  pendingArgument3Provided = false;
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

boolean enqueueStreamMove(float scanPos, float feedPos, boolean hasZ, float zPos) {
  if ( streamMoveQueueCount >= streamMoveQueueCapacity ) return false;

  streamMoveQueueScan[streamMoveQueueTail] = scanPos;
  streamMoveQueueFeed[streamMoveQueueTail] = feedPos;
  streamMoveQueueZ[streamMoveQueueTail] = zPos;
  streamMoveQueueHasZ[streamMoveQueueTail] = hasZ;
  streamMoveQueueTail = (streamMoveQueueTail + 1) % streamMoveQueueCapacity;
  streamMoveQueueCount++;
  return true;
}

boolean popQueuedStreamMove(float& scanPos, float& feedPos, boolean& hasZ, float& zPos) {
  if ( !hasQueuedStreamMove() ) return false;

  scanPos = streamMoveQueueScan[streamMoveQueueHead];
  feedPos = streamMoveQueueFeed[streamMoveQueueHead];
  zPos = streamMoveQueueZ[streamMoveQueueHead];
  hasZ = streamMoveQueueHasZ[streamMoveQueueHead];
  streamMoveQueueHead = (streamMoveQueueHead + 1) % streamMoveQueueCapacity;
  streamMoveQueueCount--;
  return true;
}

void emitTargetTelemetry(float scanPos, float feedPos, float zPos) {
  emitProtocolPair(F("status"), F("target"), scanPos, feedPos);
  emitProtocolFloat(F("status"), F("targetZ"), zPos);
}

void emitMoveCompleteTelemetry() {
  emitProtocolPair(F("event"), F("moveComplete"), scan, feed);
  emitProtocolFloat(F("event"), F("moveCompleteZ"), carriageZ);
}

boolean executeNextQueuedStreamMove() {
  if ( !motionImplementedForRobotKind(currentRobotKind) ) {
    reportUnsupportedMotion();
    return false;
  }

  float scanPos;
  float feedPos;
  float zPos;
  boolean hasZ;
  if ( !popQueuedStreamMove(scanPos, feedPos, hasZ, zPos) ) return false;

  Serial.print(F("moving to\t"));
  Serial.print(scanPos, 4);
  Serial.print(F(","));
  Serial.print(feedPos, 4);
  if ( hasZ ) {
    Serial.print(F(","));
    Serial.println(zPos, 4);
  } else {
    Serial.println();
  }
  emitTargetTelemetry(scanPos, feedPos, hasZ ? zPos : carriageZ);
  emitProtocolText(F("status"), F("currentAction"), F("moving"));
  emitProtocolText(F("status"), F("positionConfidence"), F("updating"));
  if ( hasZ ) {
    gesture(scanPos, feedPos, zPos);
  } else {
    gesture(scanPos, feedPos);
  }
  emitMoveCompleteTelemetry();
  return true;
}

void finishPendingCommand() {
  Serial.println(F("ok"));
  clearPendingCommand();
}

void failPendingCommand(const String& message) {
  emitProtocolText(F("event"), F("error"), message);
  clearPendingCommand();
}

void rejectPendingCommand() {
  failPendingCommand(String(F("command unavailable in state ")) + getStateName(machineState));
}

void moveToTargetAndReport(float targetScan, float targetFeed, float targetZ) {
  type = "absolute";
  Serial.print(F("moving to\t"));
  Serial.print(targetScan);
  Serial.print(F(","));
  Serial.print(targetFeed);
  Serial.print(F(","));
  Serial.println(targetZ);
  emitTargetTelemetry(
    targetScan,
    targetFeed,
    currentRobotKind == robotKindFlatQuadTension ? targetZ : 0.0f
  );
  emitProtocolText(F("status"), F("currentAction"), F("moving"));
  emitProtocolText(F("status"), F("positionConfidence"), F("updating"));
  if ( currentRobotKind == robotKindFlatQuadTension ) {
    gesture(targetScan, targetFeed, targetZ);
  } else {
    gesture(targetScan, targetFeed);
  }
  emitMoveCompleteTelemetry();
  printPosition();
}

void moveToTargetAndReport(float targetScan, float targetFeed) {
  moveToTargetAndReport(targetScan, targetFeed, carriageZ);
}

boolean stepMotorAndReport(byte axisIndex, long stepAmount) {
  if ( !stepActiveMotionAxis(axisIndex, stepAmount) ) {
    failPendingCommand(F("motor unavailable for current robot"));
    return false;
  }

  char motorLabel = char('A' + axisIndex);
  Serial.print(F("stepping motor "));
  Serial.print(motorLabel);
  Serial.print(F(": "));
  Serial.println(stepAmount);

  String action = F("stepping motor ");
  action += motorLabel;
  emitProtocolText(F("status"), F("currentAction"), action);
  return true;
}

boolean stepAllMotorsAndReport(long stepAmount) {
  if ( activeMotionAxisCount == 0 ) {
    failPendingCommand(F("motor unavailable for current robot"));
    return false;
  }

  Serial.print(F("stepping all motors: "));
  Serial.println(stepAmount);
  emitProtocolText(F("status"), F("currentAction"), F("stepping all motors"));

  long stepPlan[MAX_MOTION_AXES] = {0, 0, 0, 0};
  for ( byte axisIndex = 0; axisIndex < activeMotionAxisCount; axisIndex++ ) {
    stepPlan[axisIndex] = stepAmount;
  }
  runMotionSteps(stepPlan, activeMotionAxisCount);
  return true;
}

boolean setMotorEnableStateAndReport(boolean enabled) {
  if ( !motionEnableControlAvailable() ) {
    failPendingCommand(F("motor enable control unavailable for current robot"));
    return false;
  }
  if ( !setMotionEnabled(enabled) ) {
    failPendingCommand(F("motor enable control failed"));
    return false;
  }

  if ( enabled ) {
    Serial.println(F("motors enabled"));
    emitProtocolText(F("status"), F("currentAction"), F("motors enabled"));
  } else {
    Serial.println(F("motors disabled"));
    emitProtocolText(F("status"), F("currentAction"), F("motors disabled"));
  }
  return true;
}

boolean feedQuadHomeCableAndReport() {
  if ( currentRobotKind != robotKindFlatQuadTension ) {
    failPendingCommand(F("feedToHome is only available for flat quad"));
    return false;
  }
  if ( !motionImplementedForRobotKind(currentRobotKind) ) {
    failPendingCommand(F("quad motion not implemented"));
    return false;
  }
  if ( streamQueueActive() ) {
    failPendingCommand(F("stream move queue not empty"));
    return false;
  }

  float homeCableLengths[FLAT_QUAD_AXIS_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
  computeQuadCableLengths(quadHomeScan, quadHomeFeed, 0.0f, homeCableLengths);

  long axisSteps[FLAT_QUAD_AXIS_COUNT] = {0, 0, 0, 0};
  for ( byte axisIndex = 0; axisIndex < FLAT_QUAD_AXIS_COUNT; axisIndex++ ) {
    axisSteps[axisIndex] = cableDeltaToSteps(
      homeCableLengths[axisIndex],
      getQuadCableCompensation(axisIndex)
    );
  }

  Serial.println(F("feeding wire to quad home"));
  emitProtocolText(F("status"), F("currentAction"), F("feeding wire to home"));
  Serial.println(F("logical position unchanged; run resetHome after attaching carriage"));
  runMotionSteps(axisSteps, FLAT_QUAD_AXIS_COUNT);

  Serial.println(F("quad home cable feed complete"));
  emitProtocolText(F("status"), F("currentAction"), F("home cable feed complete"));
  return true;
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
      {
        unsigned int requestedSpeed = (unsigned int)max(1L, long(pendingArgument1));
        SpeedSettingsPayload payload = {
          requestedSpeed,
          requestedSpeed
        };
        applySpeedSettings(payload);
      }
      Serial.print(F("speed set at\t"));
      Serial.println(minStepperDelay);
      emitSpeedStatus();
      emitProtocolText(F("status"), F("currentAction"), F("speed updated"));
      finishPendingCommand();
      return true;

    case cmdGetSpeed:
      printSpeed();
      finishPendingCommand();
      return true;

    case cmdSaveSpeed:
      saveSpeedSettingsToEEPROM();
      Serial.println(F("speed saved"));
      emitSpeedStatus();
      emitProtocolText(F("status"), F("currentAction"), F("speed saved"));
      finishPendingCommand();
      return true;

    case cmdMonitoringOn:
      monitoring = true;
      Serial.println(F("monitoring on"));
      emitProtocolBool(F("status"), F("monitoring"), true);
      finishPendingCommand();
      return true;

    case cmdMonitoringOff:
      monitoring = false;
      Serial.println(F("monitoring off"));
      emitProtocolBool(F("status"), F("monitoring"), false);
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
      if ( pendingArgument3Provided ) {
        if ( currentRobotKind != robotKindFlatQuadTension ) {
          failPendingCommand(F("z moves are only available for flat quad"));
          return true;
        }
        moveToTargetAndReport(pendingArgument1, pendingArgument2, pendingArgument3);
      } else {
        moveToTargetAndReport(pendingArgument1, pendingArgument2);
      }
      finishPendingCommand();
      return true;

    case cmdStreamMove:
      if ( !motionImplementedForRobotKind(currentRobotKind) ) {
        failPendingCommand(F("quad motion not implemented"));
        return true;
      }
      if ( pendingArgument3Provided && currentRobotKind != robotKindFlatQuadTension ) {
        failPendingCommand(F("z moves are only available for flat quad"));
        return true;
      }
      if ( enqueueStreamMove(pendingArgument1, pendingArgument2, pendingArgument3Provided, pendingArgument3) ) {
        finishPendingCommand();
        return true;
      }

      if ( executeNextQueuedStreamMove() && enqueueStreamMove(pendingArgument1, pendingArgument2, pendingArgument3Provided, pendingArgument3) ) {
        finishPendingCommand();
        return true;
      }

      failPendingCommand(F("stream move queue stalled"));
      return true;

    case cmdStepL:
    case cmdStepA:
      if ( !motionImplementedForRobotKind(currentRobotKind) ) {
        failPendingCommand(F("quad motion not implemented"));
        return true;
      }
      if ( !stepMotorAndReport(0, long(pendingArgument1)) ) return true;
      finishPendingCommand();
      return true;

    case cmdStepR:
    case cmdStepB:
      if ( !motionImplementedForRobotKind(currentRobotKind) ) {
        failPendingCommand(F("quad motion not implemented"));
        return true;
      }
      if ( !stepMotorAndReport(1, long(pendingArgument1)) ) return true;
      finishPendingCommand();
      return true;

    case cmdStepC:
      if ( !motionImplementedForRobotKind(currentRobotKind) ) {
        failPendingCommand(F("quad motion not implemented"));
        return true;
      }
      if ( !stepMotorAndReport(2, long(pendingArgument1)) ) return true;
      finishPendingCommand();
      return true;

    case cmdStepD:
      if ( !motionImplementedForRobotKind(currentRobotKind) ) {
        failPendingCommand(F("quad motion not implemented"));
        return true;
      }
      if ( !stepMotorAndReport(3, long(pendingArgument1)) ) return true;
      finishPendingCommand();
      return true;

    case cmdStepAll:
      if ( !motionImplementedForRobotKind(currentRobotKind) ) {
        failPendingCommand(F("quad motion not implemented"));
        return true;
      }
      if ( !stepAllMotorsAndReport(long(pendingArgument1)) ) return true;
      finishPendingCommand();
      return true;

    case cmdMotorsOn:
      if ( hasQueuedStreamMove() ) {
        failPendingCommand(F("stream move queue not empty"));
        return true;
      }
      if ( !setMotorEnableStateAndReport(true) ) return true;
      finishPendingCommand();
      return true;

    case cmdMotorsOff:
      if ( hasQueuedStreamMove() ) {
        failPendingCommand(F("stream move queue not empty"));
        return true;
      }
      if ( !setMotorEnableStateAndReport(false) ) return true;
      finishPendingCommand();
      return true;

    case cmdMoveX:
      if ( !motionImplementedForRobotKind(currentRobotKind) ) {
        failPendingCommand(F("quad motion not implemented"));
        return true;
      }
      moveToTargetAndReport(scan + pendingArgument1, feed);
      finishPendingCommand();
      return true;

    case cmdMoveY:
      if ( !motionImplementedForRobotKind(currentRobotKind) ) {
        failPendingCommand(F("quad motion not implemented"));
        return true;
      }
      moveToTargetAndReport(scan, feed + pendingArgument1);
      finishPendingCommand();
      return true;

    case cmdMoveZ:
      if ( currentRobotKind != robotKindFlatQuadTension ) {
        failPendingCommand(F("moveZ is only available for flat quad"));
        return true;
      }
      moveToTargetAndReport(scan, feed, carriageZ + pendingArgument1);
      finishPendingCommand();
      return true;

    case cmdMoveLeft: {
      if ( !motionImplementedForRobotKind(currentRobotKind) ) {
        failPendingCommand(F("quad motion not implemented"));
        return true;
      }
      float distance = pendingArgument1 >= 0.0f ? pendingArgument1 : -pendingArgument1;
      moveToTargetAndReport(scan - distance, feed);
      finishPendingCommand();
      return true;
    }

    case cmdMoveRight: {
      if ( !motionImplementedForRobotKind(currentRobotKind) ) {
        failPendingCommand(F("quad motion not implemented"));
        return true;
      }
      float distance = pendingArgument1 >= 0.0f ? pendingArgument1 : -pendingArgument1;
      moveToTargetAndReport(scan + distance, feed);
      finishPendingCommand();
      return true;
    }

    case cmdMoveUp: {
      if ( !motionImplementedForRobotKind(currentRobotKind) ) {
        failPendingCommand(F("quad motion not implemented"));
        return true;
      }
      float distance = pendingArgument1 >= 0.0f ? pendingArgument1 : -pendingArgument1;
      moveToTargetAndReport(scan, feed - distance);
      finishPendingCommand();
      return true;
    }

    case cmdMoveDown: {
      if ( !motionImplementedForRobotKind(currentRobotKind) ) {
        failPendingCommand(F("quad motion not implemented"));
        return true;
      }
      float distance = pendingArgument1 >= 0.0f ? pendingArgument1 : -pendingArgument1;
      moveToTargetAndReport(scan, feed + distance);
      finishPendingCommand();
      return true;
    }

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
      if ( !isLegacyContactStateToken(pendingText1) ) {
        failPendingCommand(F("unsupported contact state"));
        return true;
      }
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

    case cmdFeedToHome:
      if ( !feedQuadHomeCableAndReport() ) return true;
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
      emitProtocolText(F("status"), F("currentAction"), F("stream queue paused"));
      finishPendingCommand();
      return true;

    case cmdContinue:
      streamMoveQueuePaused = false;
      Serial.println(F("stream queue resumed"));
      emitProtocolText(F("status"), F("currentAction"), F("stream queue resumed"));
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
    boolean moveAccepted = (
      dataHasZ
      ? gesture(dataXPos, dataYPos, dataZPos)
      : gesture(dataXPos, dataYPos)
    );
    if ( !moveAccepted ) {
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
    if ( !isLegacyContactStateToken(dataValue) ) {
      Serial.print(F("unknown contact:\t"));
      Serial.println(dataValue);
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
        emitProtocolText(F("status"), F("currentAction"), F("sd unavailable"));
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
      emitProtocolText(F("status"), F("currentAction"), F("sd unavailable"));
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
    emitProtocolText(F("status"), F("currentAction"), F("closing file"));
  }
  drawingFileOpen = false;
  streamMoveQueueHead = 0;
  streamMoveQueueTail = 0;
  streamMoveQueueCount = 0;
  streamMoveQueuePaused = false;

  if ( drawOutcome == drawFinished ) {
    Serial.println(F("drawing complete"));
    emitProtocolText(F("status"), F("lastDrawResult"), F("complete"));
    emitProtocolText(F("status"), F("currentAction"), F("drawing complete"));
    if ( motionImplementedForRobotKind(currentRobotKind) ) {
      returnToOrigin();
    }
  } else if ( drawOutcome == drawAborted ) {
    Serial.println(F("drawing aborted"));
    emitProtocolText(F("status"), F("lastDrawResult"), F("aborted"));
    emitProtocolText(F("status"), F("currentAction"), F("drawing aborted"));
  } else if ( drawOutcome == drawError ) {
    Serial.println(F("drawing error"));
    emitProtocolText(F("status"), F("lastDrawResult"), F("error"));
    emitProtocolText(F("status"), F("currentAction"), F("drawing error"));
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

boolean gesture(float xPos, float yPos, float zPos) {
  if ( !motionImplementedForRobotKind(currentRobotKind) ) {
    reportUnsupportedMotion();
    return false;
  }
  if ( currentRobotKind != robotKindFlatQuadTension ) {
    emitProtocolText(F("event"), F("error"), F("z moves are only available for flat quad"));
    return false;
  }
  movePenBresenham(xPos, yPos, zPos);
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
  dataZPos = 0.0f;
  dataHasZ = false;

  if ( !dataFile.available()) return false;

  line = dataFile.readStringUntil('\n');
  command = splitString(line, '\t', 0);
  value = splitString(line, '\t', 1);
  command.trim();
  value.trim();

  if ( command == "move" ) {
    String stXPos = splitString(value, ',', 0);
    String stYPos = splitString(value, ',', 1);
    String stZPos = splitString(value, ',', 2);
    stXPos.trim();
    stYPos.trim();
    stZPos.trim();
    dataCommand = command;
    dataXPos = stXPos.toFloat();
    dataYPos = stYPos.toFloat();
    if ( stZPos.length() ) {
      dataZPos = stZPos.toFloat();
      dataHasZ = true;
    }
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
  if ( currentRobotKind != robotKindHangingVBot ) return minStepperDelay;
  int stepperDelay = map(getRotaryPosition(A15), 1, 11, 400, 10);
  minStepperDelay = stepperDelay;
  minStepperPulse = stepperDelay;
  return stepperDelay;
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
  zINT = carriageZ * 100;
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
    carriageZ = 0.0f;
  } else {
    feed = homePosition - feedOffset;
    scan = width / 2;
    carriageZ = 0.0f;
  }
  desiredScan = scan;
  desiredFeed = feed;
  desiredZ = carriageZ;
  updateCableTelemetryFromPosition();
  Serial.println("_____________________________________");
  Serial.println("carriage reset at: ");
  Serial.print(scan);
  Serial.print(",");
  Serial.print(feed);
  Serial.print(",");
  Serial.println(carriageZ);
  Serial.print("robotKind\t");
  Serial.println(getRobotKindToken(currentRobotKind));
  printMotionCapability();
  printCableTelemetry();
  Serial.println("_____________________________________");
  Serial.println("");
  emitProtocolText(F("status"), F("currentAction"), F("home reset"));
  printPosition();
  terminate();
}

void returnToOrigin() {
  if ( !motionImplementedForRobotKind(currentRobotKind) ) {
    reportUnsupportedMotion();
    return;
  }
  Serial.println("returning to origin");
  emitProtocolText(F("status"), F("currentAction"), F("returning to origin"));
  emitProtocolText(F("status"), F("positionConfidence"), F("updating"));
  digitalWrite(LED4, HIGH);
  type = "absolute";
  if ( currentRobotKind == robotKindFlatQuadTension ) {
    gesture(0, 0, 0);
  } else {
    gesture(0, 0);
  }
  terminate();
  Serial.println("carriage at origin");
  printPosition();
  emitProtocolText(F("status"), F("currentAction"), F("at origin"));
  digitalWrite(LED4, LOW);
}

void returnToHome() {
  if ( !motionImplementedForRobotKind(currentRobotKind) ) {
    reportUnsupportedMotion();
    return;
  }
  Serial.println("returning to home");
  emitProtocolText(F("status"), F("currentAction"), F("returning to home"));
  emitProtocolText(F("status"), F("positionConfidence"), F("updating"));
  type = "absolute";
  if ( currentRobotKind == robotKindFlatQuadTension ) {
    gesture(quadHomeScan, quadHomeFeed, 0.0f);
  } else {
    gesture(width / 2, homePosition - feedOffset);
  }
  terminate();
  Serial.println("carriage at home");
  printPosition();
  emitProtocolText(F("status"), F("currentAction"), F("at home"));
}

void printPosition() {
  updateCableTelemetryFromPosition();
  Serial.println("_____________________________________");
  Serial.print("robotKind\t");
  Serial.println(getRobotKindToken(currentRobotKind));
  printMotionCapability();
  Serial.println("carriage at:");
  Serial.print(scan);
  Serial.print(",");
  Serial.print(feed);
  Serial.print(",");
  Serial.println(carriageZ);
  emitProtocolPair(F("status"), F("position"), scan, feed);
  emitProtocolFloat(F("status"), F("positionZ"), carriageZ);
  emitProtocolText(F("status"), F("positionConfidence"), F("reported"));
  Serial.println("lineLength A/B:");
  Serial.print(currentA);
  Serial.print(",");
  Serial.println(currentB);
  emitProtocolPair(F("status"), F("lineLengths"), currentA, currentB);
  printCableTelemetry();
  emitSpeedStatus();
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
