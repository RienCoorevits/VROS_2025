///////////////////////////////////////////////////
// CONTROLLER                                    //
///////////////////////////////////////////////////

const unsigned int MAX_SERIAL_COMMAND_LENGTH = 512;
String serialCommandBuffer = "";
boolean serialCommandOverflow = false;

boolean readNextSerialCommand(String& command) {
  while ( Serial.available() ) {
    char nextChar = char(Serial.read());
    if ( nextChar == '\r' ) continue;

    if ( nextChar == '\n' ) {
      if ( serialCommandOverflow ) {
        serialCommandBuffer = "";
        serialCommandOverflow = false;
        emitProtocolText(F("event"), F("error"), F("serial command too long"));
        return false;
      }

      command = serialCommandBuffer;
      serialCommandBuffer = "";
      command.trim();
      if ( !command.length() ) continue;
      return true;
    }

    if ( serialCommandOverflow ) continue;

    if ( serialCommandBuffer.length() >= MAX_SERIAL_COMMAND_LENGTH ) {
      serialCommandBuffer = "";
      serialCommandOverflow = true;
      continue;
    }

    serialCommandBuffer += nextChar;
  }

  return false;
}

void controller() {
  String command;
  String advCommand;
  String rawValue;
  if ( !readNextSerialCommand(command) ) return;

  if ( hasPendingCommand ) {
    emitProtocolText(F("event"), F("error"), F("busy: command dropped"));
    return;
  }

  pendingArgument1 = 0.0f;
  pendingArgument2 = 0.0f;
  pendingArgument3 = 0.0f;
  pendingArgument3Provided = false;
  pendingText1 = "";
  pendingText2 = "";

  if ( command.indexOf('\t') >= 0 ) {
    advCommand = splitString(command, '\t', 0);
    rawValue = splitString(command, '\t', 1);
    advCommand.trim();
    rawValue.trim();
    pendingText1 = rawValue;

    if ( advCommand.equals("move") ) {
      pendingText1 = splitString(rawValue, ',', 0);
      pendingText2 = splitString(rawValue, ',', 1);
      String pendingText3 = splitString(rawValue, ',', 2);
      pendingText1.trim();
      pendingText2.trim();
      pendingText3.trim();
      pendingArgument1 = pendingText1.toFloat();
      pendingArgument2 = pendingText2.toFloat();
      if ( pendingText3.length() ) {
        pendingArgument3 = pendingText3.toFloat();
        pendingArgument3Provided = true;
      }
      pendingCommand = cmdStreamMove;
    } else if ( advCommand.equals("robotSetupWrite") ) {
      pendingCommand = cmdRobotSetupWrite;
    } else if ( advCommand.equals("type") ) {
      pendingCommand = cmdSetType;
    } else if ( advCommand.equals("mode") ) {
      pendingCommand = cmdSetMode;
    } else if ( advCommand.equals("adjustment") ) {
      pendingCommand = cmdSetAdjustment;
    } else if ( advCommand.equals("contact") ) {
      pendingCommand = cmdSetContact;
    } else {
      emitProtocolText(F("event"), F("error"), F("Invalid command"));
      clearPendingCommand();
      return;
    }

    hasPendingCommand = true;
    return;
  }

  advCommand = splitString(command, ',', 0);
  pendingText1 = splitString(command, ',', 1);
  pendingText2 = splitString(command, ',', 2);
  String pendingText3 = splitString(command, ',', 3);
  advCommand.trim();
  pendingText1.trim();
  pendingText2.trim();
  pendingText3.trim();
  pendingArgument1 = pendingText1.toFloat();
  pendingArgument2 = pendingText2.toFloat();
  if ( pendingText3.length() ) {
    pendingArgument3 = pendingText3.toFloat();
    pendingArgument3Provided = true;
  }

  if ( advCommand.equals("drawFromFile") ) {
    pendingCommand = cmdDrawFromFile;
  } else if ( advCommand.equals("setSpeed") ) {
    pendingCommand = cmdSetSpeed;
  } else if ( advCommand.equals("move") ) {
    pendingCommand = cmdMove;
  } else if ( advCommand.equals("moveX") ) {
    pendingCommand = cmdMoveX;
  } else if ( advCommand.equals("moveY") ) {
    pendingCommand = cmdMoveY;
  } else if ( advCommand.equals("moveZ") ) {
    pendingCommand = cmdMoveZ;
  } else if ( advCommand.equals("moveLeft") ) {
    pendingCommand = cmdMoveLeft;
  } else if ( advCommand.equals("moveRight") ) {
    pendingCommand = cmdMoveRight;
  } else if ( advCommand.equals("moveUp") ) {
    pendingCommand = cmdMoveUp;
  } else if ( advCommand.equals("moveDown") ) {
    pendingCommand = cmdMoveDown;
  } else if ( advCommand.equals("type") ) {
    pendingCommand = cmdSetType;
  } else if ( advCommand.equals("mode") ) {
    pendingCommand = cmdSetMode;
  } else if ( advCommand.equals("adjustment") ) {
    pendingCommand = cmdSetAdjustment;
  } else if ( advCommand.equals("contact") ) {
    pendingCommand = cmdSetContact;
  } else if ( advCommand.equals("stepA") ) {
    pendingCommand = cmdStepA;
  } else if ( advCommand.equals("stepB") ) {
    pendingCommand = cmdStepB;
  } else if ( advCommand.equals("stepC") ) {
    pendingCommand = cmdStepC;
  } else if ( advCommand.equals("stepD") ) {
    pendingCommand = cmdStepD;
  } else if ( advCommand.equals("stepAll") ) {
    pendingCommand = cmdStepAll;
  } else if ( advCommand.equals("stepL") ) {
    pendingCommand = cmdStepL;
  } else if ( advCommand.equals("stepR") ) {
    pendingCommand = cmdStepR;
  } else if ( command.equals("motors on") ) {
    pendingCommand = cmdMotorsOn;
  } else if ( command.equals("motors off") ) {
    pendingCommand = cmdMotorsOff;
  } else if ( command.equals("outlineCanvas") ) {
    pendingCommand = cmdOutlineCanvas;
  } else if ( command.equals("returnToOrigin") ) {
    pendingCommand = cmdReturnToOrigin;
  } else if ( command.equals("returnToHome") ) {
    pendingCommand = cmdReturnToHome;
  } else if ( command.equals("feedToHome") ) {
    pendingCommand = cmdFeedToHome;
  } else if ( command.equals("resetHome") ) {
    pendingCommand = cmdResetHome;
  } else if ( command.equals("terminate") ) {
    pendingCommand = cmdTerminate;
  } else if ( command.equals("monitoring on") ) {
    pendingCommand = cmdMonitoringOn;
  } else if ( command.equals("monitoring off") ) {
    pendingCommand = cmdMonitoringOff;
  } else if ( command.equals("abort") ) {
    pendingCommand = cmdAbort;
  } else if ( command.equals("pause") ) {
    pendingCommand = cmdPause;
  } else if ( command.equals("continue") ) {
    pendingCommand = cmdContinue;
  } else if ( command.equals("position") ) {
    pendingCommand = cmdPosition;
  } else if ( command.equals("getSpeed") ) {
    pendingCommand = cmdGetSpeed;
  } else if ( command.equals("saveSpeed") ) {
    pendingCommand = cmdSaveSpeed;
  } else if ( command.equals("retrySD") ) {
    pendingCommand = cmdRetrySD;
  } else if ( command.equals("robotSetupGet") ) {
    pendingCommand = cmdRobotSetupGet;
  } else if ( command.equals("robotSetupLoad") ) {
    pendingCommand = cmdRobotSetupLoad;
  } else if ( command.equals("robotSetupDefaults") ) {
    pendingCommand = cmdRobotSetupDefaults;
  } else if ( command.equals("clearEEPROM") ) {
    pendingCommand = cmdClearEEPROM;
  } else {
    emitProtocolText(F("event"), F("error"), F("Invalid command"));
    clearPendingCommand();
    return;
  }

  hasPendingCommand = true;
}
