///////////////////////////////////////////////////
// movePenBresenham                               //
///////////////////////////////////////////////////

const int HANGING_VBOT_DIR_PINS[HANGING_VBOT_AXIS_COUNT] = {
  DIR_LEFT_PIN,
  DIR_RIGHT_PIN
};

const int HANGING_VBOT_STEP_PINS[HANGING_VBOT_AXIS_COUNT] = {
  STEP_LEFT_PIN,
  STEP_RIGHT_PIN
};

const int HANGING_VBOT_ENABLE_PINS[HANGING_VBOT_AXIS_COUNT] = {
  -1,
  -1
};

const boolean HANGING_VBOT_DIR_INVERTED[HANGING_VBOT_AXIS_COUNT] = {
  false,
  false
};

// Standard RAMPS 1.4 assignments as defined by Marlin:
// X  step/dir/enable: 54/55/38
// Y  step/dir/enable: 60/61/56
// Z  step/dir/enable: 46/48/62
// E0 step/dir/enable: 26/28/24
// E1 step/dir/enable: 36/34/30
const int RAMPS_14_X_STEP_PIN = 54;
const int RAMPS_14_X_DIR_PIN = 55;
const int RAMPS_14_X_ENABLE_PIN = 38;
const int RAMPS_14_Y_STEP_PIN = 60;
const int RAMPS_14_Y_DIR_PIN = 61;
const int RAMPS_14_Y_ENABLE_PIN = 56;
const int RAMPS_14_Z_STEP_PIN = 46;
const int RAMPS_14_Z_DIR_PIN = 48;
const int RAMPS_14_Z_ENABLE_PIN = 62;
const int RAMPS_14_E0_STEP_PIN = 26;
const int RAMPS_14_E0_DIR_PIN = 28;
const int RAMPS_14_E0_ENABLE_PIN = 24;
const int RAMPS_14_E1_STEP_PIN = 36;
const int RAMPS_14_E1_DIR_PIN = 34;
const int RAMPS_14_E1_ENABLE_PIN = 30;

// Cable A/B/C/D map to the RAMPS X/Y/E0/E1 sockets.
const int FLAT_QUAD_DIR_PINS[FLAT_QUAD_AXIS_COUNT] = {
  RAMPS_14_X_DIR_PIN,
  RAMPS_14_Y_DIR_PIN,
  RAMPS_14_E0_DIR_PIN,
  RAMPS_14_E1_DIR_PIN
};

const int FLAT_QUAD_STEP_PINS[FLAT_QUAD_AXIS_COUNT] = {
  RAMPS_14_X_STEP_PIN,
  RAMPS_14_Y_STEP_PIN,
  RAMPS_14_E0_STEP_PIN,
  RAMPS_14_E1_STEP_PIN
};

const int FLAT_QUAD_ENABLE_PINS[FLAT_QUAD_AXIS_COUNT] = {
  RAMPS_14_X_ENABLE_PIN,
  RAMPS_14_Y_ENABLE_PIN,
  RAMPS_14_E0_ENABLE_PIN,
  RAMPS_14_E1_ENABLE_PIN
};

// Keep these false until the physical spool orientations are verified on hardware.
const boolean FLAT_QUAD_DIR_INVERTED[FLAT_QUAD_AXIS_COUNT] = {
  false,
  false,
  false,
  false
};

enum PulseTimerPhase {
  pulseTimerPhaseIdle,
  pulseTimerPhaseDriveLow,
  pulseTimerPhaseDriveHigh
};

volatile PulseTimerPhase pulseTimerPhase = pulseTimerPhaseIdle;
volatile boolean pulseTimerPlanActive = false;
volatile byte pulseTimerAxisCount = 0;
volatile byte pulseTimerActiveStepMask = 0;
volatile unsigned int pulseTimerPulseCompare = 0;
volatile unsigned int pulseTimerDelayCompare = 0;
volatile long pulseTimerAxisCounts[MAX_MOTION_AXES] = {0, 0, 0, 0};
volatile long pulseTimerDominantCount = 0;
volatile long pulseTimerAxisErrors[MAX_MOTION_AXES] = {0, 0, 0, 0};
volatile long pulseTimerTickIndex = 0;

byte activeMotionAxisCount = 0;
volatile uint8_t* activeMotionDirPorts[MAX_MOTION_AXES] = {0, 0, 0, 0};
volatile uint8_t* activeMotionStepPorts[MAX_MOTION_AXES] = {0, 0, 0, 0};
uint8_t activeMotionDirMasks[MAX_MOTION_AXES] = {0, 0, 0, 0};
uint8_t activeMotionStepMasks[MAX_MOTION_AXES] = {0, 0, 0, 0};
boolean activeMotionDirInverted[MAX_MOTION_AXES] = {false, false, false, false};
int activeMotionDirPins[MAX_MOTION_AXES] = {-1, -1, -1, -1};
int activeMotionStepPins[MAX_MOTION_AXES] = {-1, -1, -1, -1};
int activeMotionEnablePins[MAX_MOTION_AXES] = {-1, -1, -1, -1};

unsigned int microsecondsToTimerCompare(unsigned int microseconds) {
  unsigned long timerCounts = (unsigned long)microseconds * 2UL;
  if ( timerCounts < 1UL ) timerCounts = 1UL;
  if ( timerCounts > 65535UL ) timerCounts = 65535UL;
  return (unsigned int)(timerCounts - 1UL);
}

boolean isMotionPinConfigured(int pin) {
  return pin >= 0 && digitalPinToPort((uint8_t)pin) != NOT_A_PIN;
}

void clearMotionAxisMap() {
  activeMotionAxisCount = 0;
  for ( byte axisIndex = 0; axisIndex < MAX_MOTION_AXES; axisIndex++ ) {
    activeMotionDirPorts[axisIndex] = 0;
    activeMotionStepPorts[axisIndex] = 0;
    activeMotionDirMasks[axisIndex] = 0;
    activeMotionStepMasks[axisIndex] = 0;
    activeMotionDirInverted[axisIndex] = false;
    activeMotionDirPins[axisIndex] = -1;
    activeMotionStepPins[axisIndex] = -1;
    activeMotionEnablePins[axisIndex] = -1;
  }
}

boolean configureMotionEnablePin(int enablePin) {
  if ( enablePin < 0 ) return true;
  if ( !isMotionPinConfigured(enablePin) ) return false;

  pinMode(enablePin, OUTPUT);
  // RAMPS stepper enables are active-low.
  digitalWrite(enablePin, LOW);
  return true;
}

boolean configureMotionAxis(
  byte axisIndex,
  int dirPin,
  int stepPin,
  int enablePin,
  boolean invertDirection
) {
  if ( axisIndex >= MAX_MOTION_AXES ) return false;
  if ( !isMotionPinConfigured(dirPin) || !isMotionPinConfigured(stepPin) ) return false;
  if ( !configureMotionEnablePin(enablePin) ) return false;

  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  digitalWrite(stepPin, LOW);

  activeMotionDirPorts[axisIndex] = portOutputRegister(digitalPinToPort((uint8_t)dirPin));
  activeMotionStepPorts[axisIndex] = portOutputRegister(digitalPinToPort((uint8_t)stepPin));
  activeMotionDirMasks[axisIndex] = digitalPinToBitMask((uint8_t)dirPin);
  activeMotionStepMasks[axisIndex] = digitalPinToBitMask((uint8_t)stepPin);
  activeMotionDirInverted[axisIndex] = invertDirection;
  activeMotionDirPins[axisIndex] = dirPin;
  activeMotionStepPins[axisIndex] = stepPin;
  activeMotionEnablePins[axisIndex] = enablePin;
  return true;
}

boolean configureMotionAxisMap(
  const int* dirPins,
  const int* stepPins,
  const int* enablePins,
  const boolean* invertDirections,
  byte axisCount
) {
  clearMotionAxisMap();
  if ( axisCount == 0 || axisCount > MAX_MOTION_AXES ) return false;

  for ( byte axisIndex = 0; axisIndex < axisCount; axisIndex++ ) {
    int enablePin = enablePins ? enablePins[axisIndex] : -1;
    boolean invertDirection = invertDirections ? invertDirections[axisIndex] : false;
    if ( !configureMotionAxis(axisIndex, dirPins[axisIndex], stepPins[axisIndex], enablePin, invertDirection) ) {
      clearMotionAxisMap();
      return false;
    }
  }

  activeMotionAxisCount = axisCount;
  return true;
}

boolean quadMotionPinsConfigured() {
  for ( byte axisIndex = 0; axisIndex < FLAT_QUAD_AXIS_COUNT; axisIndex++ ) {
    if (
      !isMotionPinConfigured(FLAT_QUAD_DIR_PINS[axisIndex])
      || !isMotionPinConfigured(FLAT_QUAD_STEP_PINS[axisIndex])
      || !isMotionPinConfigured(FLAT_QUAD_ENABLE_PINS[axisIndex])
    ) {
      return false;
    }
  }
  return true;
}

void configureMotionAxisMapForRobotKind(RobotKindId robotKind) {
  if ( robotKind == robotKindFlatQuadTension && quadMotionPinsConfigured() ) {
    configureMotionAxisMap(
      FLAT_QUAD_DIR_PINS,
      FLAT_QUAD_STEP_PINS,
      FLAT_QUAD_ENABLE_PINS,
      FLAT_QUAD_DIR_INVERTED,
      FLAT_QUAD_AXIS_COUNT
    );
    return;
  }

  if ( robotKind == robotKindHangingVBot ) {
    configureMotionAxisMap(
      HANGING_VBOT_DIR_PINS,
      HANGING_VBOT_STEP_PINS,
      HANGING_VBOT_ENABLE_PINS,
      HANGING_VBOT_DIR_INVERTED,
      HANGING_VBOT_AXIS_COUNT
    );
    return;
  }

  clearMotionAxisMap();
}

boolean activeMotionUsesPin(int pin) {
  if ( pin < 0 ) return false;
  for ( byte axisIndex = 0; axisIndex < activeMotionAxisCount; axisIndex++ ) {
    if ( activeMotionDirPins[axisIndex] == pin ) return true;
    if ( activeMotionStepPins[axisIndex] == pin ) return true;
    if ( activeMotionEnablePins[axisIndex] == pin ) return true;
  }
  return false;
}

inline void setAxisDirection(byte axisIndex, long axisSteps) {
  if ( axisIndex >= activeMotionAxisCount ) return;
  if ( activeMotionDirPorts[axisIndex] == 0 ) return;
  long effectiveAxisSteps = activeMotionDirInverted[axisIndex] ? -axisSteps : axisSteps;
  if ( effectiveAxisSteps > 0 ) {
    *activeMotionDirPorts[axisIndex] |= activeMotionDirMasks[axisIndex];
  } else if ( effectiveAxisSteps < 0 ) {
    *activeMotionDirPorts[axisIndex] &= byte(~activeMotionDirMasks[axisIndex]);
  }
}

inline void setAxisStepHigh(byte axisIndex) {
  if ( axisIndex >= activeMotionAxisCount ) return;
  if ( activeMotionStepPorts[axisIndex] == 0 ) return;
  *activeMotionStepPorts[axisIndex] |= activeMotionStepMasks[axisIndex];
}

inline void setAxisStepLow(byte axisIndex) {
  if ( axisIndex >= activeMotionAxisCount ) return;
  if ( activeMotionStepPorts[axisIndex] == 0 ) return;
  *activeMotionStepPorts[axisIndex] &= byte(~activeMotionStepMasks[axisIndex]);
}

void disablePulseTimer() {
  TIMSK1 &= ~_BV(OCIE1A);
  TCCR1B = _BV(WGM12);
}

void finishPulsePlan() {
  for ( byte axisIndex = 0; axisIndex < pulseTimerAxisCount; axisIndex++ ) {
    setAxisStepLow(axisIndex);
  }
  pulseTimerActiveStepMask = 0;
  pulseTimerPhase = pulseTimerPhaseIdle;
  pulseTimerPlanActive = false;
  disablePulseTimer();
}

void configureMotionPulseTimer() {
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1C = 0;
  TCNT1 = 0;
  OCR1A = microsecondsToTimerCompare(10);
  TIFR1 = _BV(OCF1A);
  clearMotionAxisMap();
  disablePulseTimer();
}

void setPulsePlanDirections(const long* axisSteps, byte axisCount) {
  for ( byte axisIndex = 0; axisIndex < axisCount; axisIndex++ ) {
    setAxisDirection(axisIndex, axisSteps[axisIndex]);
  }
}

boolean startPulsePlan(const long* axisSteps, byte axisCount) {
  if ( axisCount == 0 || axisCount > activeMotionAxisCount ) return false;

  long dominantCount = 0;
  long axisCounts[MAX_MOTION_AXES] = {0, 0, 0, 0};
  for ( byte axisIndex = 0; axisIndex < axisCount; axisIndex++ ) {
    axisCounts[axisIndex] = labs(axisSteps[axisIndex]);
    if ( axisCounts[axisIndex] > dominantCount ) dominantCount = axisCounts[axisIndex];
  }
  if ( dominantCount == 0 ) return false;

  unsigned int pulseWidthMicroseconds = (unsigned int)minStepperPulse;
  unsigned int pulseDelayMicroseconds = (unsigned int)minStepperDelay;
  if ( pulseWidthMicroseconds < 1U ) pulseWidthMicroseconds = 1U;
  if ( pulseDelayMicroseconds < 1U ) pulseDelayMicroseconds = 1U;

  noInterrupts();
  setPulsePlanDirections(axisSteps, axisCount);
  pulseTimerAxisCount = axisCount;
  for ( byte axisIndex = 0; axisIndex < MAX_MOTION_AXES; axisIndex++ ) {
    pulseTimerAxisCounts[axisIndex] = axisIndex < axisCount ? axisCounts[axisIndex] : 0;
    pulseTimerAxisErrors[axisIndex] = 0;
    if ( axisIndex < axisCount ) setAxisStepLow(axisIndex);
  }
  pulseTimerDominantCount = dominantCount;
  pulseTimerTickIndex = 0;
  pulseTimerActiveStepMask = 0;
  pulseTimerPulseCompare = microsecondsToTimerCompare(pulseWidthMicroseconds);
  pulseTimerDelayCompare = microsecondsToTimerCompare(pulseDelayMicroseconds);
  pulseTimerPhase = pulseTimerPhaseDriveLow;
  pulseTimerPlanActive = true;
  TCNT1 = 0;
  OCR1A = 1;
  TIFR1 = _BV(OCF1A);
  TIMSK1 |= _BV(OCIE1A);
  TCCR1B = _BV(WGM12) | _BV(CS11);
  interrupts();

  return true;
}

void waitForPulsePlan() {
  while ( pulseTimerPlanActive ) {
  }
}

ISR(TIMER1_COMPA_vect) {
  if ( !pulseTimerPlanActive ) {
    disablePulseTimer();
    return;
  }

  if ( pulseTimerPhase == pulseTimerPhaseDriveLow ) {
    if ( pulseTimerTickIndex >= pulseTimerDominantCount ) {
      finishPulsePlan();
      return;
    }

    byte stepMask = 0;
    for ( byte axisIndex = 0; axisIndex < pulseTimerAxisCount; axisIndex++ ) {
      pulseTimerAxisErrors[axisIndex] += pulseTimerAxisCounts[axisIndex];
      if ( pulseTimerAxisErrors[axisIndex] >= pulseTimerDominantCount ) {
        pulseTimerAxisErrors[axisIndex] -= pulseTimerDominantCount;
        setAxisStepHigh(axisIndex);
        stepMask |= byte(1U << axisIndex);
      }
    }

    pulseTimerActiveStepMask = stepMask;
    pulseTimerPhase = pulseTimerPhaseDriveHigh;
    OCR1A = pulseTimerPulseCompare;
    return;
  }

  for ( byte axisIndex = 0; axisIndex < pulseTimerAxisCount; axisIndex++ ) {
    if ( pulseTimerActiveStepMask & byte(1U << axisIndex) ) {
      setAxisStepLow(axisIndex);
    }
  }
  pulseTimerActiveStepMask = 0;
  pulseTimerTickIndex++;

  if ( pulseTimerTickIndex >= pulseTimerDominantCount ) {
    finishPulsePlan();
    return;
  }

  pulseTimerPhase = pulseTimerPhaseDriveLow;
  OCR1A = pulseTimerDelayCompare;
}

String normalizeMotionMode(String requestedMode) {
  requestedMode.trim();
  if ( requestedMode == "bresenham" ) return "bresenham";
  if ( requestedMode == "segmented" ) return "bresenham";
  if ( requestedMode == "movePen" ) return "bresenham";
  return "";
}

boolean applyMotionMode(String requestedMode) {
  String normalizedMode = normalizeMotionMode(requestedMode);
  if ( !normalizedMode.length() ) return false;
  mode = normalizedMode;
  return true;
}

long cableDeltaToSteps(float deltaCm, float compensation) {
  return long(roundf(deltaCm * stepsToCm * compensation));
}

void buildAdjustedPoint(
  float rawScan,
  float rawFeed,
  float& adjustedScan,
  float& adjustedFeed
) {
  float scanAdjustment = segmentAdjustment(rawScan, rawFeed, "scan");
  float feedAdjustment = segmentAdjustment(rawScan, rawFeed, "feed");

  adjustedScan = rawScan + scanAdjustment;
  adjustedFeed = rawFeed + feedAdjustment;

  if ( leftBound && adjustedScan < 0.0f ) adjustedScan = 0.0f;
  if ( rightBound && adjustedScan > width ) adjustedScan = width;
  if ( upperBound && adjustedFeed < 0.0f ) adjustedFeed = 0.0f;
  if ( lowerBound && adjustedFeed > height ) adjustedFeed = height;
}

void runMotionSteps(const long* axisSteps, byte axisCount) {
  if ( !startPulsePlan(axisSteps, axisCount) ) return;
  waitForPulsePlan();
}

void runBresenhamSteps(long leftSteps, long rightSteps) {
  long axisSteps[HANGING_VBOT_AXIS_COUNT] = {
    leftSteps,
    rightSteps
  };
  runMotionSteps(axisSteps, HANGING_VBOT_AXIS_COUNT);
}

boolean stepActiveMotionAxis(byte axisIndex, long axisSteps) {
  if ( axisIndex >= activeMotionAxisCount ) return false;
  if ( axisSteps == 0 ) return true;

  long stepPlan[MAX_MOTION_AXES] = {0, 0, 0, 0};
  stepPlan[axisIndex] = axisSteps;
  runMotionSteps(stepPlan, activeMotionAxisCount);
  return true;
}

float getQuadCableCompensation(byte axisIndex) {
  if ( axisIndex == 0 ) return quadCableAFeed;
  if ( axisIndex == 1 ) return quadCableBFeed;
  if ( axisIndex == 2 ) return quadCableCFeed;
  return quadCableDFeed;
}

void computeQuadCableLengths(
  float scanPos,
  float feedPos,
  float cableLengths[FLAT_QUAD_AXIS_COUNT]
) {
  float leftAnchor = -scanOffset;
  float rightAnchor = width + scanOffset;
  float topAnchor = -feedOffset;
  float bottomAnchor = height + feedOffset;
  cableLengths[0] = dist(scanPos, feedPos, leftAnchor, topAnchor);
  cableLengths[1] = dist(scanPos, feedPos, rightAnchor, topAnchor);
  cableLengths[2] = dist(scanPos, feedPos, rightAnchor, bottomAnchor);
  cableLengths[3] = dist(scanPos, feedPos, leftAnchor, bottomAnchor);
}

void prepareGestureTarget(float xPos, float yPos) {
  if ( type == "relative" ) {
    desiredScan += xPos;
    desiredFeed += yPos;
  } else {
    desiredScan = xPos;
    desiredFeed = yPos;
  }
}

void printGestureHeader(
  float startScan,
  float startFeed,
  float xPos,
  float yPos,
  float targetScan,
  float targetFeed
) {
  if ( !monitoring ) return;

  Serial.println("_____________________________________");
  Serial.print("GESTURE\t");
  Serial.print(gestureCount);
  Serial.print(" ");
  Serial.println(type);
  Serial.println("_____________________________________");
  Serial.print("s/f \t");
  Serial.print(startScan);
  Serial.print("\t");
  Serial.println(startFeed);
  Serial.print("travel \t");
  Serial.print(xPos);
  Serial.print("\t");
  Serial.println(yPos);
  Serial.print("dS/dF:\t");
  Serial.print(targetScan);
  Serial.print("\t");
  Serial.println(targetFeed);
}

long computeSegmentCount(float travel) {
  if ( travel <= 0.01f ) return 0;

  float resolution = lineResolution;
  if ( resolution < 0.01f ) resolution = 0.01f;

  long segmentCount = long(ceilf(travel / resolution));
  if ( segmentCount < 1 ) segmentCount = 1;

  if ( monitoring ) {
    Serial.print("mode:\t");
    Serial.println(mode);
    Serial.print("lineResolution:\t");
    Serial.println(resolution, 4);
    Serial.print("segments:\t");
    Serial.println(segmentCount);
  }

  return segmentCount;
}

void printGestureFooter() {
  if ( !monitoring ) return;

  Serial.println("desScan, desFeed");
  Serial.print(desiredScan);
  Serial.print("\t");
  Serial.println(desiredFeed);
  Serial.println("actScan, actFeed");
  Serial.print(scan);
  Serial.print("\t");
  Serial.println(feed);
}

void movePenBresenhamHanging(float xPos, float yPos) {
  prepareGestureTarget(xPos, yPos);

  float startScan = scan;
  float startFeed = feed;
  float targetScan = desiredScan;
  float targetFeed = desiredFeed;
  float travel = dist(startScan, startFeed, targetScan, targetFeed);

  printGestureHeader(startScan, startFeed, xPos, yPos, targetScan, targetFeed);

  long segmentCount = computeSegmentCount(travel);
  if ( segmentCount == 0 ) {
    if ( monitoring ) Serial.println("aborted due to short travel");
    terminate();
    return;
  }

  currentA = getA(scan, feed);
  currentB = getB(scan, feed);

  for ( long segmentIndex = 1; segmentIndex <= segmentCount; segmentIndex++ ) {
    float interpolation = float(segmentIndex) / float(segmentCount);
    float rawScan = startScan + (targetScan - startScan) * interpolation;
    float rawFeed = startFeed + (targetFeed - startFeed) * interpolation;

    float adjustedScan = rawScan;
    float adjustedFeed = rawFeed;
    buildAdjustedPoint(rawScan, rawFeed, adjustedScan, adjustedFeed);
    float nextA = getA(adjustedScan, adjustedFeed);
    float nextB = getB(adjustedScan, adjustedFeed);
    long leftSteps = cableDeltaToSteps(nextA - currentA, leftCoilFeed);
    long rightSteps = cableDeltaToSteps(nextB - currentB, rightCoilFeed);

    if ( monitoring ) {
      Serial.print("segment\t");
      Serial.print(segmentIndex);
      Serial.print("\t");
      Serial.print(adjustedScan, 4);
      Serial.print("\t");
      Serial.print(adjustedFeed, 4);
      Serial.print("\t");
      Serial.print(leftSteps);
      Serial.print("\t");
      Serial.println(rightSteps);
    }

    runBresenhamSteps(leftSteps, rightSteps);

    currentA += float(leftSteps) / (stepsToCm * leftCoilFeed);
    currentB += float(rightSteps) / (stepsToCm * rightCoilFeed);
    scan = getScanAndFeed(currentA, currentB, "scan");
    feed = getScanAndFeed(currentA, currentB, "feed");
  }

  printGestureFooter();
  terminate();
}

void movePenBresenhamQuad(float xPos, float yPos) {
  prepareGestureTarget(xPos, yPos);

  float startScan = scan;
  float startFeed = feed;
  float targetScan = desiredScan;
  float targetFeed = desiredFeed;
  float travel = dist(startScan, startFeed, targetScan, targetFeed);

  printGestureHeader(startScan, startFeed, xPos, yPos, targetScan, targetFeed);

  long segmentCount = computeSegmentCount(travel);
  if ( segmentCount == 0 ) {
    if ( monitoring ) Serial.println("aborted due to short travel");
    terminate();
    return;
  }

  float currentLengths[FLAT_QUAD_AXIS_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
  computeQuadCableLengths(scan, feed, currentLengths);

  for ( long segmentIndex = 1; segmentIndex <= segmentCount; segmentIndex++ ) {
    float interpolation = float(segmentIndex) / float(segmentCount);
    float rawScan = startScan + (targetScan - startScan) * interpolation;
    float rawFeed = startFeed + (targetFeed - startFeed) * interpolation;

    float adjustedScan = rawScan;
    float adjustedFeed = rawFeed;
    buildAdjustedPoint(rawScan, rawFeed, adjustedScan, adjustedFeed);

    float nextLengths[FLAT_QUAD_AXIS_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
    computeQuadCableLengths(adjustedScan, adjustedFeed, nextLengths);

    long axisSteps[FLAT_QUAD_AXIS_COUNT] = {0, 0, 0, 0};
    for ( byte axisIndex = 0; axisIndex < FLAT_QUAD_AXIS_COUNT; axisIndex++ ) {
      axisSteps[axisIndex] = cableDeltaToSteps(
        nextLengths[axisIndex] - currentLengths[axisIndex],
        getQuadCableCompensation(axisIndex)
      );
    }

    if ( monitoring ) {
      Serial.print("segment\t");
      Serial.print(segmentIndex);
      Serial.print("\t");
      Serial.print(adjustedScan, 4);
      Serial.print("\t");
      Serial.print(adjustedFeed, 4);
      for ( byte axisIndex = 0; axisIndex < FLAT_QUAD_AXIS_COUNT; axisIndex++ ) {
        Serial.print("\t");
        Serial.print(axisSteps[axisIndex]);
      }
      Serial.println();
    }

    runMotionSteps(axisSteps, FLAT_QUAD_AXIS_COUNT);

    for ( byte axisIndex = 0; axisIndex < FLAT_QUAD_AXIS_COUNT; axisIndex++ ) {
      currentLengths[axisIndex] = nextLengths[axisIndex];
    }
    scan = adjustedScan;
    feed = adjustedFeed;
  }

  updateCableTelemetryFromPosition();
  printGestureFooter();
  terminate();
}

void movePenBresenham(float xPos, float yPos) {
  gestureCount++;
  if ( currentRobotKind == robotKindFlatQuadTension ) {
    movePenBresenhamQuad(xPos, yPos);
    return;
  }
  movePenBresenhamHanging(xPos, yPos);
}
