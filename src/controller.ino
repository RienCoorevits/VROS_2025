///////////////////////////////////////////////////
// CONTROLLER                                    //
///////////////////////////////////////////////////

void controller() {
  String command;
  String advCommand;
  int argument1;
  int argument2;
  
  if (Serial.available()) {
    command = Serial.readStringUntil('\n');
    advCommand = splitString(command, ',', 0);
    argument1 = splitString(command, ',', 1).toInt();
    argument2 = splitString(command, ',', 2).toInt();

    if ( advCommand.equals("writeToFile")){
      Serial.println("started writing");
      initialiseWriteData(argument2);
      drawingLibrary(argument1);
      closeData();
    }
    if (advCommand.equals("drawFromFile")) {
      Serial.println("started drawing");
      filePointer = argument1;
      machineState = drawing;
    }

    if (advCommand.equals("setSpeed")) {
      minStepperDelay = argument1;
      minStepperPulse = argument1;
      Serial.print("speed set at\t");
      Serial.println(argument1);
    }

    if (advCommand.equals("move")) {
      type = "absolute";
      Serial.print("moving to\t");
      Serial.print(argument1);
      Serial.print(",");
      Serial.println(argument2);
      movePenSegmented(argument1,argument2);
      printPosition();
    }

    if (advCommand.equals("stepL")) {
      Serial.print("stepping left motor: ");
      Serial.println(argument1);
      stepL(argument1);
    }

    if (advCommand.equals("stepR")) {
      Serial.print("stepping right motor: ");
      Serial.println(argument1);
      stepR(argument1);
    }
    
    
    if (command.equals("outlineCanvas")) {
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
      Serial.println("done");

    }

    if (command.equals("returnToOrigin")) {
      machineState = idle;
      returnToOrigin();
    }

    if (command.equals("returnToHome")) {
      returnToHome();
    }

    if (command.equals("resetHome")) {
      resetHome();
      machineState = launchpad;
    }
    if (command.equals("terminate")) {
      terminate();
    }
    if (command.equals("monitoring on")) {
      Serial.println("monitoring on");
      monitoring = true;
    }
    if (command.equals("monitoring off")) {
      Serial.println("monitoring off");
      monitoring = false;
    }
    if (command.equals("abort")) {
      Serial.println("started aborting");
      machineState = aborting;
    }
    if (command.equals("position")) {
      printPosition();
    }

    else {
      //Serial.println("Invalid command");
    }
  }
}
