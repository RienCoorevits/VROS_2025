///////////////////////////////////////////////////
// movePenBresenham                               //
///////////////////////////////////////////////////

enum PulseTimerPhase {
  pulseTimerPhaseIdle,
  pulseTimerPhaseDriveLow,
  pulseTimerPhaseDriveHigh
};

const byte DIR_LEFT_MASK = _BV(PH3);
const byte STEP_LEFT_MASK = _BV(PH4);
const byte DIR_RIGHT_MASK = _BV(PH5);
const byte STEP_RIGHT_MASK = _BV(PH6);
const byte STEP_MASK = STEP_LEFT_MASK | STEP_RIGHT_MASK;

volatile PulseTimerPhase pulseTimerPhase = pulseTimerPhaseIdle;
volatile boolean pulseTimerPlanActive = false;
volatile byte pulseTimerActiveStepMask = 0;
volatile unsigned int pulseTimerPulseCompare = 0;
volatile unsigned int pulseTimerDelayCompare = 0;
volatile long pulseTimerLeftCount = 0;
volatile long pulseTimerRightCount = 0;
volatile long pulseTimerDominantCount = 0;
volatile long pulseTimerErrorLeft = 0;
volatile long pulseTimerErrorRight = 0;
volatile long pulseTimerTickIndex = 0;

unsigned int microsecondsToTimerCompare(unsigned int microseconds) {
  unsigned long timerCounts = (unsigned long)microseconds * 2UL;
  if ( timerCounts < 1UL ) timerCounts = 1UL;
  if ( timerCounts > 65535UL ) timerCounts = 65535UL;
  return (unsigned int)(timerCounts - 1UL);
}

void disablePulseTimer() {
  TIMSK1 &= ~_BV(OCIE1A);
  TCCR1B = _BV(WGM12);
}

void finishPulsePlan() {
  PORTH &= byte(~STEP_MASK);
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
  PORTH &= byte(~STEP_MASK);
  disablePulseTimer();
}

void setPulsePlanDirections(long leftSteps, long rightSteps) {
  if ( leftSteps > 0 ) PORTH |= DIR_LEFT_MASK;
  if ( leftSteps < 0 ) PORTH &= byte(~DIR_LEFT_MASK);
  if ( rightSteps > 0 ) PORTH |= DIR_RIGHT_MASK;
  if ( rightSteps < 0 ) PORTH &= byte(~DIR_RIGHT_MASK);
}

boolean startPulsePlan(long leftSteps, long rightSteps) {
  long leftCount = labs(leftSteps);
  long rightCount = labs(rightSteps);
  long dominantCount = leftCount;
  if ( rightCount > dominantCount ) dominantCount = rightCount;
  if ( dominantCount == 0 ) return false;

  unsigned int pulseWidthMicroseconds = (unsigned int)minStepperPulse;
  unsigned int pulseDelayMicroseconds = (unsigned int)minStepperDelay;
  if ( pulseWidthMicroseconds < 1U ) pulseWidthMicroseconds = 1U;
  if ( pulseDelayMicroseconds < 1U ) pulseDelayMicroseconds = 1U;

  noInterrupts();
  setPulsePlanDirections(leftSteps, rightSteps);
  pulseTimerLeftCount = leftCount;
  pulseTimerRightCount = rightCount;
  pulseTimerDominantCount = dominantCount;
  pulseTimerErrorLeft = 0;
  pulseTimerErrorRight = 0;
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

    pulseTimerErrorLeft += pulseTimerLeftCount;
    if ( pulseTimerErrorLeft >= pulseTimerDominantCount ) {
      pulseTimerErrorLeft -= pulseTimerDominantCount;
      stepMask |= STEP_LEFT_MASK;
    }

    pulseTimerErrorRight += pulseTimerRightCount;
    if ( pulseTimerErrorRight >= pulseTimerDominantCount ) {
      pulseTimerErrorRight -= pulseTimerDominantCount;
      stepMask |= STEP_RIGHT_MASK;
    }

    PORTH |= stepMask;
    pulseTimerActiveStepMask = stepMask;
    pulseTimerPhase = pulseTimerPhaseDriveHigh;
    OCR1A = pulseTimerPulseCompare;
    return;
  }

  PORTH &= byte(~pulseTimerActiveStepMask);
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

void runBresenhamSteps(long leftSteps, long rightSteps) {
  if ( !startPulsePlan(leftSteps, rightSteps) ) return;
  waitForPulsePlan();
}

void movePenBresenham(float xPos, float yPos) {
  gestureCount++;

  if ( type == "relative" ) {
    desiredScan += xPos;
    desiredFeed += yPos;
  } else {
    desiredScan = xPos;
    desiredFeed = yPos;
  }

  float startScan = scan;
  float startFeed = feed;
  float targetScan = desiredScan;
  float targetFeed = desiredFeed;
  float travel = dist(startScan, startFeed, targetScan, targetFeed);

  if ( monitoring ) {
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

  if ( travel <= 0.01f ) {
    if ( monitoring ) Serial.println("aborted due to short travel");
    terminate();
    return;
  }

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

  if ( monitoring ) {
    Serial.println("desScan, desFeed");
    Serial.print(desiredScan);
    Serial.print("\t");
    Serial.println(desiredFeed);
    Serial.println("actScan, actFeed");
    Serial.print(scan);
    Serial.print("\t");
    Serial.println(feed);
  }

  terminate();
}
