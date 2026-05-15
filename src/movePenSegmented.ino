///////////////////////////////////////////////////
// movePenSegmented                              //
///////////////////////////////////////////////////

void movePenSegmented(float xPos, float yPos) {
  gestureCount++;
  int segmentAmount = 0;

  //calculate desired scan/feed
  if ( type == "relative" ) {
    desiredScan += xPos;
    desiredFeed += yPos;
  } else if ( type == "absolute" ) {
    desiredScan = xPos;
    desiredFeed = yPos;
  }

  if ( monitoring ) {
    Serial.println("_____________________________________");
    Serial.print("GESTURE\t");
    Serial.print(gestureCount);
    Serial.print(" ");
    Serial.println(type);
    Serial.println("_____________________________________");
    Serial.print("s/f \t");
    Serial.print(scan);
    Serial.print("\t");
    Serial.println(feed);
    Serial.print("travel \t");
    Serial.print(xPos);
    Serial.print("\t");
    Serial.println(yPos);
    Serial.print("dS/dF:\t");
    Serial.print(desiredScan);
    Serial.print("\t");
    Serial.println(desiredFeed);
  }

  if ( monitoring ) {
    for ( int i = 0 ; i < segmentLength; i++ ) {
      Serial.print(i);
      Serial.print("\t");
    }
    Serial.println("");
  }


  //devide a gesture into segments by lineResolution
  //initialise segmentArrays
  for ( int i = 0; i < segmentLength; i++ ) {
    scanSegment[i] = 0.0f;
    feedSegment[i] = 0.0f;
    directionA[i] = false;
    directionB[i] = false;
    motorRatioL[i] = 0.0f;
    motorRatioR[i] = 0.0f;
  }

  //fill the segmentArrays
  //[0] is always the current scan/feed
  if ( dist(scan, feed, xPos, yPos) < lineResolution
       && dist(scan, feed, xPos, yPos) > 0.01 ) {
    scanSegment[0] = scan;
    feedSegment[0] = feed;
    scanSegment[1] = xPos;
    feedSegment[1] = yPos;
    segmentAmount = 0;
  } else if ( dist(scan, feed, xPos, yPos) > lineResolution ) {
    int index = 1;
    scanSegment[0] = scan;
    feedSegment[0] = feed;
    while ( dist(scanSegment[index - 1], feedSegment[index - 1],
                 desiredScan, desiredFeed) > lineResolution ) {

      scanSegment[index] =
        getPointByDistance(scanSegment[index - 1],
                           feedSegment[index - 1],
                           desiredScan, desiredFeed, lineResolution, "scan");
      feedSegment[index] =
        getPointByDistance(scanSegment[index - 1],
                           feedSegment[index - 1],
                           desiredScan, desiredFeed, lineResolution, "feed");
      index++;
      segmentAmount++;
    }
    scanSegment[index] = desiredScan;
    feedSegment[index] = desiredFeed;
  } else {
    if ( monitoring ) Serial.println("aborted due to short travel");
    segmentAmount = -1;
  }

  if ( monitoring ) {
    Serial.print("segmentAmount:\t");
    Serial.println(segmentAmount);
    Serial.println("absolute scanSegment / feedSegment");
    for ( int i = 0; i < segmentLength; i++ ) {
      //Serial.print("[");
      //Serial.print(i);
      //Serial.print("]");
      Serial.print(scanSegment[i]);
      Serial.print("\t");
    }
    Serial.println("");

    for ( int i = 0; i < segmentLength; i++ ) {
      //Serial.print("[");
      //Serial.print(i);
      //Serial.print("]");
      Serial.print(feedSegment[i]);
      Serial.print("\t");
    }
    Serial.println("");
  }

  //with the scan/feed arrays we can calculate adjustments
  //i don't know how to go about this architecturally...
  //we'll just get them from another function in the shape
  //family


  for ( int i = 1; i < segmentAmount + 2; i++ ) {
    scanSegment[i] += segmentAdjustment(scanSegment[i], feedSegment[i], "scan");
    feedSegment[i] += segmentAdjustment(scanSegment[i], feedSegment[i], "feed");
  }



  if ( monitoring ) {
    Serial.println("scanAdjustment / feedAdjustment");
    for ( int i = 0; i < segmentLength; i++ ) {
      //Serial.print("[");
      //Serial.print(i);
      //Serial.print("]");
      Serial.print(sin(scanSegment[i] / width * PI));
      Serial.print("\t");
    }
    Serial.println("");

    for ( int i = 0; i < segmentLength; i++ ) {
      //Serial.print("[");
      //Serial.print(i);
      //Serial.print("]");
      Serial.print(cos(feedSegment[i] / height * PI));
      Serial.print("\t");
    }
    Serial.println("");

    Serial.println("adjusted scan / adjusted feed");
    for ( int i = 0; i < segmentLength; i++ ) {
      //Serial.print("[");
      //Serial.print(i);
      //Serial.print("]");
      Serial.print(scanSegment[i]);
      Serial.print("\t");
    }
    Serial.println("");

    for ( int i = 0; i < segmentLength; i++ ) {
      //Serial.print("[");
      //Serial.print(i);
      //Serial.print("]");
      Serial.print(feedSegment[i]);
      Serial.print("\t");
    }
    Serial.println("");
  }



  //now limit the values
  for ( int i = 1; i < segmentAmount + 2; i++ ) {
    if ( leftBound && scanSegment[i] < 0 ) scanSegment[i] = 0;
    if ( rightBound && scanSegment[i] > width ) scanSegment[i] = width;
    if ( upperBound && feedSegment[i] < 0 ) feedSegment[i] = 0;
    if ( lowerBound && feedSegment[i] > height ) feedSegment[i] = height;
  }

  //updating currentA / current B here is possibly redundant
  currentA = getA(scan, feed);
  currentB = getB(scan, feed);

  //calculate deltaA and deltaB, put it in an array
  float aAtSegment;
  float bAtSegment;
  for ( int i = 0; i < segmentAmount + 1; i++ ) {
    aAtSegment = getA(scanSegment[i], feedSegment[i]);
    bAtSegment = getB(scanSegment[i], feedSegment[i]);
    scanSegment[i] = getDeltaA(scanSegment[i+1], feedSegment[i+1], aAtSegment);
    feedSegment[i] = getDeltaB(scanSegment[i+1], feedSegment[i+1], bAtSegment);
  }

  //deltaA / deltaB amounts got put 1 index out of place
  //move all values one place -1
  for ( int i = 0; i < segmentAmount + 1; i++ ) {
    scanSegment[segmentAmount + 1 - i] = scanSegment[segmentAmount - i];
    feedSegment[segmentAmount + 1 - i] = feedSegment[segmentAmount - i];
  }
  //set [0] to zero
  scanSegment[0] = 0.00f;
  feedSegment[0] = 0.00f;

  if ( monitoring ) {
    Serial.println("deltaA / deltaB");
    for ( int i = 0; i < segmentLength; i++ ) {
      //Serial.print("[");
      //Serial.print(i);
      //Serial.print("]");
      Serial.print(scanSegment[i]);
      Serial.print("\t");
    }
    Serial.println("");

    for ( int i = 0; i < segmentLength; i++ ) {
      //Serial.print("[");
      //Serial.print(i);
      //Serial.print("]");
      Serial.print(feedSegment[i]);
      Serial.print("\t");
    }
    Serial.println("");
  }

  //calculate a step direction array
  for ( int i = 1; i < segmentAmount + 2; i++ ) {
    if ( scanSegment[i] > 0 ) {
      directionA[i] = true; //HIGH
    } else if ( scanSegment[i] < 0 ) {
      directionA[i] = false; // LOW
    }
    if ( feedSegment[i] > 0 ) {
      directionB[i] = true; //HIGH
    } else if ( feedSegment[i] < 0 ) {
      directionB[i] = false; //LOW
    }
  }

  if ( monitoring ) {
    Serial.println("directionA / directionB");
    for ( int i = 0; i < segmentLength; i++ ) {
      //Serial.print("[");
      //Serial.print(i);
      //Serial.print("]");
      Serial.print(directionA[i]);
      Serial.print("\t");
    }
    Serial.println("");

    for ( int i = 0; i < segmentLength; i++ ) {
      //Serial.print("[");
      //Serial.print(i);
      //Serial.print("]");
      Serial.print(directionB[i]);
      Serial.print("\t");
    }
    Serial.println("");
  }


  //convert deltaA/deltaB to a stepAmount array
  for ( int i = 0; i < segmentAmount + 2; i++ ) {
    scanSegment[i] = abs(scanSegment[i] * stepsToCm) * leftCoilFeed;
    feedSegment[i] = abs(feedSegment[i] * stepsToCm) * rightCoilFeed;
  }

  if ( monitoring ) {
    Serial.println("deltaA / deltaB stepamount");
    for ( int i = 0; i < segmentLength; i++ ) {
      //Serial.print("[");
      //Serial.print(i);
      //Serial.print("]");
      Serial.print(scanSegment[i]);
      Serial.print("\t");
    }
    Serial.println("");

    for ( int i = 0; i < segmentLength; i++ ) {
      //Serial.print("[");
      //Serial.print(i);
      //Serial.print("]");
      Serial.print(feedSegment[i]);
      Serial.print("\t");
    }
    Serial.println("");
  }

  //calculate motor ratios
  long longestTravel;
  for ( int i = 1; i < segmentAmount + 2; i++ ) {
    if ( feedSegment[i] ) {
      longestTravel = feedSegment[i];
      if ( scanSegment[i] > feedSegment[i] ) {
        longestTravel = scanSegment[i];
      }
      motorRatioL[i] = float(longestTravel) / float(scanSegment[i]);
      motorRatioR[i] = float(longestTravel) / float(feedSegment[i]);
    }
  }

  if ( monitoring ) {
    Serial.println("motorRatioL / motorRatioR");
    for ( int i = 0; i < segmentLength; i++ ) {
      //Serial.print("[");
      //Serial.print(i);
      //Serial.print("]");
      Serial.print(motorRatioL[i]);
      Serial.print("\t");
    }
    Serial.println("");

    for ( int i = 0; i < segmentLength; i++ ) {
      //Serial.print("[");
      //Serial.print(i);
      //Serial.print("]");
      Serial.print(motorRatioR[i]);
      Serial.print("\t");
    }
    Serial.println("");

    Serial.println("");
    Serial.println("starting movement:");
    Serial.println("part:\tleft:\tright:\tsumL\tsumR");
  }


  // we did all the precalculations to make this segment
  // run as smoothly as possible
  int i = 1;
  long totalSumLeft = 0;
  long totalSumRight = 0;
  while ( scanSegment[i] || feedSegment[i] ) {
    if ( directionA[i] ) {
      digitalWrite(DIR_LEFT_PIN, HIGH);
    } else {
      digitalWrite(DIR_LEFT_PIN, LOW);
    }
    if ( directionB[i] ) {
      digitalWrite(DIR_RIGHT_PIN, HIGH);
    } else {
      digitalWrite(DIR_RIGHT_PIN, LOW);
    }
    int testSumLeft = 0;
    int testSumRight = 0;
    if ( scanSegment[i] > feedSegment[i] ) {
      for ( int x = 0; x < scanSegment[i] + 1; x++ ) {
        if ( testSumLeft < int(x / motorRatioL[i])) {
          testSumLeft++;
          digitalWrite(STEP_LEFT_PIN, HIGH);
          delayMicroseconds(minStepperPulse);
          digitalWrite(STEP_LEFT_PIN, LOW);
        }
        if ( testSumRight < int(x / motorRatioR[i])) {
          testSumRight++;
          digitalWrite(STEP_RIGHT_PIN, HIGH);
          delayMicroseconds(minStepperPulse);
          digitalWrite(STEP_RIGHT_PIN, LOW);
        }
        delayMicroseconds(minStepperDelay);
      }
    } else if ( scanSegment[i] < feedSegment[i] ) {
      for ( int x = 0; x < feedSegment[i] + 1; x++ ) {
        if ( testSumLeft < int(x / motorRatioL[i])) {
          testSumLeft++;
          digitalWrite(STEP_LEFT_PIN, HIGH);
          delayMicroseconds(minStepperPulse);
          digitalWrite(STEP_LEFT_PIN, LOW);
        }
        if ( testSumRight < int(x / motorRatioR[i])) {
          testSumRight++;
          digitalWrite(STEP_RIGHT_PIN, HIGH);
          delayMicroseconds(minStepperPulse);
          digitalWrite(STEP_RIGHT_PIN, LOW);
        }
        delayMicroseconds(minStepperDelay);
      }
    }
    if ( directionA[i] ) {
      totalSumLeft += testSumLeft/leftCoilFeed;
    } else {
      totalSumLeft -= testSumLeft/leftCoilFeed;
    }
    if ( directionB[i] ) {
      totalSumRight += testSumRight/rightCoilFeed;
    } else {
      totalSumRight -= testSumRight/rightCoilFeed;
    }

    if ( monitoring ) {
      Serial.print(i);
      Serial.print("\t");
      Serial.print(testSumLeft);
      Serial.print("\t");
      Serial.print(testSumRight);
      Serial.print("\t");
      Serial.print(totalSumLeft);
      Serial.print("\t");
      Serial.println(totalSumRight);
    }
    i++;
  }

  //finalise by updating the global variables
  //before moving on to the next gesture

  currentA = currentA + totalSumLeft / stepsToCm;
  currentB = currentB + totalSumRight / stepsToCm;
  scan = getScanAndFeed(currentA, currentB, "scan");
  feed = getScanAndFeed(currentA, currentB, "feed");

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
