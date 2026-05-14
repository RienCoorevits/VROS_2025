///////////////////////////////////////////////////
// CONTROLLER                                    //
///////////////////////////////////////////////////

void controller() {
  String command;
  String advCommand;
  if ( !Serial.available() ) return;

  command = Serial.readStringUntil('\n');
  command.trim();
  if ( !command.length() ) return;

  if ( hasPendingCommand ) {
    Serial.println(F("busy: command dropped"));
    return;
  }

  advCommand = splitString(command, ',', 0);
  pendingArgument1 = splitString(command, ',', 1).toFloat();
  pendingArgument2 = splitString(command, ',', 2).toFloat();

  if ( advCommand.equals("writeToFile") ) {
    pendingCommand = cmdWriteToFile;
  } else if ( advCommand.equals("drawFromFile") ) {
    pendingCommand = cmdDrawFromFile;
  } else if ( advCommand.equals("setSpeed") ) {
    pendingCommand = cmdSetSpeed;
  } else if ( advCommand.equals("move") ) {
    pendingCommand = cmdMove;
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
  } else {
    Serial.println(F("Invalid command"));
    clearPendingCommand();
    return;
  }

  hasPendingCommand = true;
}
