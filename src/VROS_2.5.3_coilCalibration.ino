#include <FastLED.h>
#include <EEPROM.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>

// heightmaps
//https://tangrams.github.io/heightmapper


// pin definitions
#define DPL 6
#define SPL 7
#define DPR 8
#define SPR 9
#define FAULTLEFT 0
#define FAULTRIGHT 1
#define M0 5
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

/*
  //Prototype Robot
  const float motorDistance = 57.00f; // previously 57.00
  const float scanOffset = 5.00f;//11.00f; //10.00 + 1.00 spool offset
  const float feedOffset = 15.50f; //15.50f; //15.00 + 1.00 spool offset
  const float width = motorDistance - scanOffset * 2;
  const float height = 50.00f;
  const float lineResolution = 0.50f;
  const float homePosition = 91.7;
  const float leftCoilFeed = 1.00;
  const float rightCoilFeed = 1.00;
*/

//Black robot
const float motorDistance = 62.00f; //63
const float scanOffset = 10.00f;
const float feedOffset = 20.00f;
const float width = motorDistance - scanOffset * 2;
const float height = 50.00f;
const float lineResolution = 0.50f;
const float homePosition = 82.00f;//81.1;
const float leftCoilFeed = 1.00;
const float rightCoilFeed = 0.997;
float stepsToCm = 35.00f;//33.58f;

boolean detectCase = false;

float stepLength;
float scan;
float feed;
float desiredScan;
float desiredFeed;
float currentA;
float currentB;

//speed vars
int minStepperDelay = 50; //200
int minStepperPulse = 50;


//Drawing vars
String type = "absolute"; // relative/absolute
long gestureCount; //counts the gestures in the drawing

//movePenSegmented vars
int segmentLength = 200;
float scanSegment[200];
float feedSegment[200];
boolean directionA[200];
boolean directionB[200];
float motorRatioL[200];
float motorRatioR[200];

//error vars
//only movePen() uses this and its functionality can be
//removed completely
int totalOvershotLeft;
int totalOvershotRight;

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
  cmdWriteToFile,
  cmdSetSpeed,
  cmdMove,
  cmdStreamMove,
  cmdStepL,
  cmdStepR,
  cmdSetType,
  cmdSetMode,
  cmdSetAdjustment,
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
  cmdRetrySD
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
int loopCounter = 0;
int cycleLength = 2000;
unsigned long lastSDRetryAt = 0;
const unsigned long sdRetryIntervalMs = 2000;

///////////////////////////////////////////////////
// SETTINGS                                      //
///////////////////////////////////////////////////

// setting vars
String mode = "segmented"; //selects drawing algorithm
boolean monitoring = false;
boolean waitForStep = false;
boolean upperBound = false;
boolean lowerBound = false;
boolean leftBound = true;
boolean rightBound = true;
String adjustmentType = "none";
float microstepResolution = 0.0625;


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
  pinMode(DPL, OUTPUT);
  pinMode(SPL, OUTPUT);
  pinMode(DPR, OUTPUT);
  pinMode(SPR, OUTPUT);

  //setting up microstepping
  pinMode(M0, OUTPUT);
  digitalWrite(M0, HIGH);

  //testing if case is connected
  //we haven't found a good way yet to test this.

  stepsToCm = stepsToCm / microstepResolution;
  stepLength = 1 / stepsToCm;


  Serial.begin(115200);
  Serial.println(F("_____________________________________"));
  Serial.println(F("VROS_2.5.1_caseController"));
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


  Serial.println(F("_____________________________________"));
  Serial.println(F("reading EEPROM"));
  EEPROM.get(0, feedINT);
  EEPROM.get(15, scanINT);
  feed = float(feedINT) / 100;
  scan = float(scanINT) / 100;
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
  Serial.println(F(">writeAndPlot, drawing, file"));
  Serial.println(F(">abort"));
  Serial.println(F(">pause"));
  Serial.println(F(">continue"));
  Serial.println(F(">move, scan, feed"));
  Serial.println(F(">stepL, amount"));
  Serial.println(F(">stepR, amount"));
  Serial.println(F(">monitoring on/off"));
  Serial.println(F(">returnToOrigin"));
  Serial.println(F(">outlineCanvas"));
  Serial.println(F(">resetHome"));
  Serial.println(F(">returnToHome"));
  Serial.println(F(">position"));
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
  float scanPos;
  float feedPos;
  if ( !popQueuedStreamMove(scanPos, feedPos) ) return false;

  gesture(scanPos, feedPos);
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
      Serial.print(F("stepping left motor: "));
      Serial.println(int(pendingArgument1));
      stepL(int(pendingArgument1));
      finishPendingCommand();
      return true;

    case cmdStepR:
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
      mode = pendingText1;
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

    case cmdOutlineCanvas:
      type = "absolute";
      printPosition();
      for ( int x = 0; x <= width; x++ ) {
        movePenSegmented(x, 0);
      }
      printPosition();
      for ( int x = 0; x <= height; x++ ) {
        movePenSegmented(width, x);
      }
      printPosition();
      for ( int x = 0; x <= width; x++ ) {
        movePenSegmented(width - x, height);
      }
      printPosition();
      for ( int x = 0; x <= height; x++ ) {
        movePenSegmented(0, height - x);
      }
      printPosition();
      Serial.println(F("done"));
      finishPendingCommand();
      return true;

    case cmdReturnToOrigin:
      returnToOrigin();
      finishPendingCommand();
      return true;

    case cmdReturnToHome:
      returnToHome();
      finishPendingCommand();
      return true;

    case cmdResetHome:
      resetHome();
      enterState(launchpad);
      finishPendingCommand();
      return true;

    default:
      return false;
  }
}

boolean handleDrawingInstruction() {
  if ( !readInstruction() ) return false;

  if ( dataCommand == "move" ) {
    gesture(dataXPos, dataYPos);
  } else if ( dataCommand == "type" ) {
    type = dataValue;
  } else if ( dataCommand == "mode" ) {
    mode = dataValue;
  } else if ( dataCommand == "adjustment" ) {
    adjustmentType = dataValue;
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

    if ( pendingCommand == cmdWriteToFile ) {
      if ( !initialiseSDQuietly() ) {
        Serial.println(F("SD unavailable"));
        enterState(noSD);
      } else if ( initialiseWriteData(int(pendingArgument2)) ) {
        Serial.println(F("started writing"));
        drawingLibrary(int(pendingArgument1));
        closeData();
      }
      clearPendingCommand();
      return;
    }

    rejectPendingCommand();
    return;
  }

  if ( executeNextQueuedStreamMove() ) return;

  if ( digitalRead(toggle1) == LOW ) {
    if ( initialiseSDQuietly() ) {
      enterState(drawing);
    } else {
      Serial.println(F("SD unavailable"));
      enterState(noSD);
    }
    return;
  }

  if ( digitalRead(toggle3) == LOW ) {
    resetHome();
    enterState(launchpad);
    return;
  }

  if ( digitalRead(toggle4) == LOW ) {
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

  if ( digitalRead(toggle1) == HIGH ) {
    drawOutcome = drawAborted;
    enterState(aborting);
    return;
  }

  if ( digitalRead(toggle2) == LOW ) {
    enterState(pausing);
    return;
  }

  if ( !dataFile || !dataFile.available() ) {
    drawOutcome = drawFinished;
    enterState(aborting);
    return;
  }

  if ( !handleDrawingInstruction() ) {
    drawOutcome = drawFinished;
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

  if ( digitalRead(toggle1) == HIGH ) {
    drawOutcome = drawAborted;
    enterState(aborting);
    return;
  }

  if ( digitalRead(toggle2) == HIGH ) {
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

  if ( drawOutcome == drawFinished ) {
    Serial.println(F("drawing complete"));
    returnToOrigin();
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

  if ( executeNextQueuedStreamMove() ) return;

  if ( digitalRead(toggle4) == LOW ) {
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
      if ( initialiseSD() ) {
        filePointer = int(pendingArgument1);
        clearPendingCommand();
        enterState(drawing);
      } else {
        clearPendingCommand();
      }
      return;
    }

    if ( pendingCommand == cmdWriteToFile ) {
      if ( initialiseSD() && initialiseWriteData(int(pendingArgument2)) ) {
        Serial.println(F("started writing"));
        drawingLibrary(int(pendingArgument1));
        closeData();
        enterState(idle);
      }
      clearPendingCommand();
      return;
    }

    rejectPendingCommand();
    return;
  }

  if ( executeNextQueuedStreamMove() ) return;

  if ( digitalRead(toggle3) == LOW ) {
    resetHome();
    enterState(launchpad);
    return;
  }

  if ( digitalRead(toggle4) == LOW ) {
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

void gesture(float xPos, float yPos) {
  if ( mode == "segmented" ) movePenSegmented(xPos, yPos);
  if ( mode == "movePen" ) movePen(xPos, yPos);
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
    dataFile.print("mode\t");
    dataFile.println(value);
  }
}

void writeAdjustment(String value) {
  if ( dataFile ) {
    dataFile.print("adjustment\t");
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
  EEPROM.put(0, feedINT);
  EEPROM.put(15, scanINT);
  //Serial.print(feedINT);
  //Serial.print(",");
  //Serial.println(scanINT);
}

void resetHome() {
  feed = homePosition - feedOffset;
  scan = width / 2;
  currentA = getA(scan, feed);
  currentB = getB(scan, feed);
  Serial.println("_____________________________________");
  Serial.println("carriage reset at: ");
  Serial.print(scan);
  Serial.print(",");
  Serial.println(feed);
  Serial.println("lineLength A/B: ");
  Serial.print(currentA);
  Serial.print(",");
  Serial.println(currentB);
  Serial.println("_____________________________________");
  Serial.println("");
  terminate();
}

void returnToOrigin() {
  Serial.println("returning to origin");
  digitalWrite(LED4, HIGH);
  type = "absolute";
  gesture(0, 0);
  terminate();
  Serial.println("carriage at origin");
  digitalWrite(LED4, LOW);
}

void returnToHome() {
  Serial.println("returning to home");
  type = "absolute";
  gesture(width / 2, homePosition - feedOffset);
  terminate();
  Serial.println("carriage at home");
}

void printPosition() {
  Serial.println("_____________________________________");
  Serial.println("carriage at:");
  Serial.print(scan);
  Serial.print(",");
  Serial.println(feed);
  Serial.println("lineLength A/B:");
  Serial.print(currentA);
  Serial.print(",");
  Serial.println(currentB);
  Serial.println("_____________________________________");
}

///////////////////////////////////////////////////
// MOTOR LEVEL FUNCTIONS                         //
///////////////////////////////////////////////////

void stepL(int amount) {
  if ( amount > 0) {
    digitalWrite(DPL, HIGH);
  } else if ( amount < 0) {
    digitalWrite(DPL, LOW);
  }
  for ( int x = 0; x < abs(amount); x++ ) {
    digitalWrite(SPL, HIGH);
    delayMicroseconds(minStepperDelay);
    digitalWrite(SPL, LOW);
    delayMicroseconds(minStepperDelay);
  }
}

void stepR(int amount) {
  if ( amount > 0) {
    digitalWrite(DPR, HIGH);
  } else if ( amount < 0) {
    digitalWrite(DPR, LOW);
  }
  for ( int x = 0; x < abs(amount); x++ ) {
    digitalWrite(SPR, HIGH);
    delayMicroseconds(minStepperDelay);
    digitalWrite(SPR, LOW);
    delayMicroseconds(minStepperDelay);
  }
}
