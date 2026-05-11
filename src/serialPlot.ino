///////////////////////////////////////////////////
// serialPlot                                    //
///////////////////////////////////////////////////

void serialPlot(float xPos, float yPos) {
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
    scanSegment[i] = NULL;
    feedSegment[i] = NULL;
    directionA[i] = NULL;
    directionB[i] = NULL;
    motorRatioL[i] = NULL;
    motorRatioR[i] = NULL;
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
  // we'll just get them from another function in the shape
  // family


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


  for ( int i = 1; i < segmentAmount + 2 ; i++ ) {
    Serial.print(gestureCount);
    Serial.print("\t");
    Serial.print(i);
    Serial.print("\t");
    Serial.print(scanSegment[i]);
    Serial.print("\t");
    Serial.println(feedSegment[i]);
  }

  scan = desiredScan;
  feed = desiredFeed;
  
}
