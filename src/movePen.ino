///////////////////////////////////////////////////
// MOVE PEN                                      //
///////////////////////////////////////////////////

void movePen(float xPos, float yPos) {
  gestureCount++;

  if ( type == "relative") {
    //desiredScan = scan + xPos;
    //desiredFeed = feed + yPos;
    //tried a fix for relative correction behavior
    desiredScan += xPos;
    desiredFeed += yPos;
  } else if ( type == "absolute") {
    desiredScan = xPos;
    desiredFeed = yPos;
  }

  if ( monitoring) {
    Serial.println("_____________________________________");
    if ( type == "relative") {
      Serial.print("MOVEPENRELATIVE(");
    } else if ( type == "absolute") {
      Serial.print("MOVEPENABSOLUTE(");
    }
    Serial.print(xPos);
    Serial.print(",");
    Serial.print(yPos);
    Serial.println(")");
    Serial.print("GESTURE ");
    Serial.println(gestureCount);
    Serial.println("_____________________________________");

    Serial.print("desiredScan:\t\t");
    Serial.println(desiredScan);
    Serial.print("desiredFeed:\t\t");
    Serial.println(desiredFeed);
  }

  currentA = getA(scan, feed);
  currentB = getB(scan, feed);
  float a = getDeltaA(desiredScan, desiredFeed, currentA); //outputs in cm
  float b = getDeltaB(desiredScan, desiredFeed, currentB); //outputs in cm

  if ( monitoring) {
    Serial.print("currentA:\t");
    Serial.println(currentA);
    Serial.print("currentB:\t");
    Serial.println(currentB);

    Serial.print("deltaA:\t\t");
    Serial.println(a);
    Serial.print("deltaB:\t\t");
    Serial.println(b);
  }

  //determine motor directions

  if ( a > 0) {
    digitalWrite(DIR_LEFT_PIN, HIGH);
  } else if ( a < 0) {
    digitalWrite(DIR_LEFT_PIN, LOW);
  }
  if ( b > 0) {
    digitalWrite(DIR_RIGHT_PIN, HIGH);
  } else if ( b < 0) {
    digitalWrite(DIR_RIGHT_PIN, LOW);
  }

  //determine amount of steps
  long stepsLeft = abs(a * stepsToCm);
  long stepsRight = abs(b * stepsToCm);
  if ( monitoring) {
    Serial.print("forecast steps left:\t");
    Serial.println(stepsLeft);
    Serial.print("forecast steps right:\t");
    Serial.println(stepsRight);
  }

  long longestTravel = stepsRight;
  if ( stepsLeft > stepsRight ) {
    longestTravel = stepsLeft;
  }

  long testSumLeft = 0;
  long testSumRight = 0;

  ///////////////////////////////////////////////
  // THIS BLOCK RUNS EACH STEPPER              //
  // IN SERIES                                 //
  ///////////////////////////////////////////////
  /*
    for (int x = 0; x < stepsLeft; x++ ) {
    testSumLeft++;
    digitalWrite(SPL, HIGH);
    delayMicroseconds(minStepperPulse);
    digitalWrite(SPL, LOW);
    delayMicroseconds(minStepperDelay);
    }
    for (int y = 0; y < stepsRight; y++ ) {
    testSumRight++;
    digitalWrite(SPR, HIGH);
    delayMicroseconds(minStepperPulse);
    digitalWrite(SPR, LOW);
    delayMicroseconds(minStepperDelay);
    }
  */

  ///////////////////////////////////////////////
  // THIS BLOCK RUNS EACH STEPPER              //
  // PARALLEL WITH THE MODULO FUNCTION         //
  ///////////////////////////////////////////////
  /*
    for ( int x = 0; x < longestTravel; x++ ) {
    if (x % int(longestTravel / stepsLeft) == 0) {
      testSumLeft++;
      digitalWrite(STEP_LEFT_PIN, HIGH);
      delayMicroseconds(minStepperPulse);
      digitalWrite(STEP_LEFT_PIN, LOW);
    }
    if (x % int(longestTravel / stepsRight) == 0) {
      testSumRight++;
      digitalWrite(STEP_RIGHT_PIN, HIGH);
      delayMicroseconds(minStepperPulse);
      digitalWrite(STEP_RIGHT_PIN, LOW);
    }
    delayMicroseconds(minStepperDelay);
    }
  */

  ///////////////////////////////////////////////
  // THIS BLOCK RUNS EACH STEPPER              //
  // PARALLEL WITH A MUCH MORE LOGICAL DEVIS   //
  ///////////////////////////////////////////////
  float ratioL = float(longestTravel) / float(stepsLeft);
  float ratioR = float(longestTravel) / float(stepsRight);
  for ( int x = 0; x < longestTravel + 1; x++ ) {
    if ( testSumLeft < int(x / ratioL)) {
      testSumLeft++;
      digitalWrite(STEP_LEFT_PIN, HIGH);
      delayMicroseconds(minStepperPulse);
      digitalWrite(STEP_LEFT_PIN, LOW);
    }

    if ( testSumRight < int(x / ratioR)) {
      testSumRight++;
      digitalWrite(STEP_RIGHT_PIN, HIGH);
      delayMicroseconds(minStepperPulse);
      digitalWrite(STEP_RIGHT_PIN, LOW);
    }
    delayMicroseconds(minStepperDelay);
  }

  if ( monitoring) {
    Serial.println("");
    Serial.println("completed crudely by casting floats as ints");
    Serial.print("actual steps left:\t");
    Serial.println(testSumLeft);
    Serial.print("actual steps Right:\t");
    Serial.println(testSumRight);

    if ( testSumLeft - stepsLeft > 0 ) {
      Serial.print("ERROR: left overshot:\t");
      Serial.println(abs(testSumLeft - stepsLeft));
    } else if ( testSumLeft - stepsLeft < 0 ) {
      Serial.print("ERROR: left undershot:\t");
      Serial.println(abs(testSumLeft - stepsLeft));
    }

    if ( testSumRight - stepsRight > 0 ) {
      Serial.print("ERROR: right overshot:\t");
      Serial.println(abs(testSumRight - stepsRight));
    } else if ( testSumRight - stepsRight < 0 ) {
      Serial.print("ERROR: right undershot:\t");
      Serial.println(abs(testSumRight - stepsRight));
    }
  }

  scan = desiredScan;
  feed = desiredFeed;
  float projectedA = getA(scan, feed);
  float projectedB = getB(scan, feed);

  if ( monitoring) {
    Serial.println("");
    Serial.println("projected A and B");
    Serial.print("projectedA:\t");
    Serial.println(projectedA);
    Serial.print("projectedB:\t");
    Serial.println(projectedB);
  }

  if ( a > 0) {
    currentA = currentA + testSumLeft / stepsToCm;
  } else if ( a < 0) {
    currentA = currentA - testSumLeft / stepsToCm;
  }
  if ( b > 0) {
    currentB = currentB + testSumRight / stepsToCm;
  } else if ( b < 0) {
    currentB = currentB - testSumRight / stepsToCm;
  }

  if ( monitoring) {
    Serial.println("");
    Serial.println("updated A and B based on actual movement");
    Serial.print("currentA:\t");
    Serial.println(currentA);
    Serial.print("currentB:\t");
    Serial.println(currentB);
  }

  scan = getScanAndFeed(currentA, currentB, "scan");
  feed = getScanAndFeed(currentA, currentB, "feed");

  if ( monitoring) {
    Serial.println("");
    Serial.println("updated scan and feed based on actual movement");
    Serial.print("scan:\t");
    Serial.println(scan);
    Serial.print("feed:\t");
    Serial.println(feed);
  }

  totalOvershotLeft += (testSumLeft - stepsLeft);
  totalOvershotRight += (testSumRight - stepsRight);

  if ( monitoring) {
    Serial.println("");
    Serial.println("total overshot steps so far:");
    Serial.print("left:\t");
    Serial.println(totalOvershotLeft);
    Serial.print("right:\t");
    Serial.println(totalOvershotRight);
  }

  terminate();

}
