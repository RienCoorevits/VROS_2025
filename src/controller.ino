///////////////////////////////////////////////////
// CONTROLLER                                    //
///////////////////////////////////////////////////

void controller() {
  String command;
  String advCommand;
  String rawValue;
  if ( !Serial.available() ) return;

  command = Serial.readStringUntil('\n');
  command.trim();
  if ( !command.length() ) return;

  if ( hasPendingCommand ) {
    Serial.println(F("error\tbusy: command dropped"));
    return;
  }

  pendingArgument1 = 0.0f;
  pendingArgument2 = 0.0f;
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
      pendingText1.trim();
      pendingText2.trim();
      pendingArgument1 = pendingText1.toFloat();
      pendingArgument2 = pendingText2.toFloat();
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
      Serial.println(F("error\tInvalid command"));
      clearPendingCommand();
      return;
    }

    hasPendingCommand = true;
    return;
  }

  advCommand = splitString(command, ',', 0);
  pendingText1 = splitString(command, ',', 1);
  pendingText2 = splitString(command, ',', 2);
  advCommand.trim();
  pendingText1.trim();
  pendingText2.trim();
  pendingArgument1 = pendingText1.toFloat();
  pendingArgument2 = pendingText2.toFloat();

  if ( advCommand.equals("drawFromFile") ) {
    pendingCommand = cmdDrawFromFile;
  } else if ( advCommand.equals("setSpeed") ) {
    pendingCommand = cmdSetSpeed;
  } else if ( advCommand.equals("move") ) {
    pendingCommand = cmdMove;
  } else if ( advCommand.equals("type") ) {
    pendingCommand = cmdSetType;
  } else if ( advCommand.equals("mode") ) {
    pendingCommand = cmdSetMode;
  } else if ( advCommand.equals("adjustment") ) {
    pendingCommand = cmdSetAdjustment;
  } else if ( advCommand.equals("contact") ) {
    pendingCommand = cmdSetContact;
  } else if ( advCommand.equals("stepL") ) {
    pendingCommand = cmdStepL;
  } else if ( advCommand.equals("stepR") ) {
    pendingCommand = cmdStepR;
  } else if ( command.equals("outlineCanvas") ) {
    pendingCommand = cmdOutlineCanvas;
  } else if ( command.equals("returnToOrigin") ) {
    pendingCommand = cmdReturnToOrigin;
  } else if ( command.equals("returnToHome") ) {
    pendingCommand = cmdReturnToHome;
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
    Serial.println(F("error\tInvalid command"));
    clearPendingCommand();
    return;
  }

  hasPendingCommand = true;
}
