void drawingLibrary(int pointer) {

  if (pointer == 1) {
    writeType("relative");
    int tD = 1;
    writeGesture(width / 2, height / 2);
    for ( int x = 0; x < 3; x++) {
      writeGesture(-tD, -tD);
      writeGesture(-tD, tD);
      writeGesture(tD, tD);
      tD++;
      writeGesture(tD, -tD);
    }
  }

  if (pointer == 2) {
    writeType("absolute");
    writeAdjustment("none");
    writeGesture(width / 2, height / 2);
    for ( int j = 4; j < 120 ; j++ ) {
      writeType("relative");
      writeGesture(0, float(j) / 4);
      drawCircle(40, float(j) / 2);
      writeType("absolute");
      writeGesture(width / 2, height / 2);
    }
  }

  if (pointer == 3) {
    writeType("absolute");
    writeAdjustment("noise");
    writeGesture(width / 2, height / 2);
    for ( int j = 1; j < 60 ; j++ ) {
      writeType("relative");
      writeGesture(0, j / 2);
      drawCircle(40, j);
      writeType("absolute");
      writeGesture(width / 2, height / 2);
    }
  }

  if (pointer == 4) {
    writeAdjustment("none");
    fillRect(0, 0, width, height, 50, "horizontal");
    closeData();
  }

  if (pointer == 5) {
    writeAdjustment("largeSin");
    writeType("absolute");
    for ( int x = 0; x < width / 4; x++ ) {
      for ( int y = 0; y < height / 4; y++ ) {
        drawRect(4, 4);
        writeType("absolute");
        writeGesture(x * 4, y * 4);
      }
    }
  }

  if (pointer == 6) {
    writeAdjustment("largeSin");
    writeType("absolute");
    for ( int y = 0; y < height * 5; y++ ) {
      for ( int x = 0; x < width; x++ ) {
        writeType("absolute");
        writeGesture(x, float(y) / 5);
        int r = random(0, 100);
        if ( r == 0 ) {
          drawRect(1, 2);
        }
      }
    }
  }

  if (pointer == 7) {
    writeAdjustment("none");
    writeType("absolute");
    for ( int y = 0; y < height; y++ ) {
      for ( int x = 0; x < width; x++ ) {
        writeGesture(x, y);
        int r = random(0, 100);
        if ( r == 0 ) {
          fillRect(x, y, 1, 2, 5, "horizontal");
        }
      }
    }
  }

  if (pointer == 8) {
    writeAdjustment("largeSin");
    writeType("absolute");
    for ( int y = 0; y < height * 3; y++ ) {
      for ( int x = 0; x < width; x++ ) {
        writeGesture(x, float(y) / 3);
        int r = random(0, 100);
        if ( r == 0 ) {
          fillRect(x, y, 1, 2, 5, "horizontal");
        }
      }
    }
  }
  if (pointer == 9) {
    writeType("absolute");
    int xDevider = 5;
    int yDevider = 5;
    for ( int y = 0; y < height * yDevider; y++ ) {
      for ( int x = 0; x <= width * xDevider; x++ ) {
        float noiseY = 1 * complexNoise(float(x) / xDevider * 35, float(y) / yDevider / 5 * 35, 5);
        //if ( x == 0 || x == width) {
        //  writeGesture(x/xDevider,float(y)/yDevider);
        //} else {
        writeGesture(float(x) / xDevider, (float(y) / yDevider) + noiseY);
        //}
      }
    }
  }
  if (pointer == 10) {
    writeType("absolute");
    int xDevider = 5;
    int yDevider = 5;
    float ySteps = height * yDevider;
    for ( int y = 0; y < height * yDevider; y++ ) {
      for ( int x = 0; x <= width * xDevider; x++ ) {
        float amplitude = float(map(y / yDevider, 0, ySteps, 0, 300)) / 100;
        float noiseY = amplitude * complexNoise(float(x) / xDevider * 35, float(y) / yDevider / 5 * 35, 5);
        if ( x == 0 || x == width * xDevider) {
          writeGesture(float(x) / xDevider, float(y) / yDevider);
        } else {
          writeGesture(float(x) / xDevider, (float(y) / yDevider) + noiseY);
        }
      }
    }
  }
  if (pointer == 11) {
    writeType("absolute");
    int xDevider = 5;
    int yDevider = 5;
    float ySteps = height * yDevider;
    for ( int y = 0; y < height * yDevider; y++ ) {
      for ( int x = 0; x <= width * xDevider; x++ ) {
        float amplitude = float(map(y / yDevider, 0, ySteps, 0, 500)) / 100;
        float noiseY = amplitude * complexNoise(float(x) / xDevider * 35, float(y) / yDevider / 5 * 35, 5);
        if ( x == 0 || x == width * xDevider) {
          writeGesture(float(x) / xDevider, float(y) / yDevider);
        } else {
          writeGesture(float(x) / xDevider, (float(y) / yDevider) + noiseY);
        }
      }
    }
  }
  if (pointer == 12) {
    writeType("absolute");
    int xDevider = 5;
    int yDevider = 5;
    float ySteps = height * yDevider;
    for ( int y = 0; y < (height * yDevider) / 2; y++ ) {
      float amplitude = float(map(y / yDevider, 0, ySteps, 0, 500)) / 100;
      float noiseY;
      for ( int x = 0; x <= width * xDevider; x++ ) {
        float xPos = float(x) / xDevider;
        float yPos = (float(y) / yDevider) * 2;
        noiseY = amplitude * complexNoise(xPos * 35, yPos / 5 * 35, 5);
        writeGesture(xPos, yPos + noiseY);
      }
      for ( int x = 0; x <= width * xDevider; x++ ) {
        float xPos = width - float(x) / xDevider;
        float yPos = (float(y) + 1) / yDevider * 2;
        noiseY = amplitude * complexNoise(xPos * 35, yPos / 5 * 35, 5);
        writeGesture(xPos, yPos + noiseY);
      }
    }
  }
  if (pointer == 13) {
    writeType("absolute");
    int xDevider = 5;
    int yDevider = 5;
    float ySteps = height * yDevider;
    for ( int y = 0; y < (height * yDevider) / 2; y++ ) {
      float amplitude = float(map(y / yDevider, 0, ySteps, 0, 500)) / 100;
      float noiseY;
      for ( int x = 0; x <= width * xDevider; x++ ) {
        float xPos = float(x) / xDevider;
        float yPos = (float(y) / yDevider) * 2;
        noiseY = amplitude * complexNoise(xPos * 35, yPos / 5 * 35, 5);
        writeGesture(xPos, yPos + noiseY);
      }
      for ( int x = 0; x <= width * xDevider; x++ ) {
        float xPos = width - float(x) / xDevider;
        float yPos = (float(y) + 1) / yDevider * 2;
        noiseY = amplitude * complexNoise(xPos * 35, yPos / 5 * 35, 5);
        writeGesture(xPos, yPos + noiseY);
      }
    }
  }
  if (pointer == 14) {
    writeType("absolute");
    int xDevider = 5;
    int yDevider = 5;
    float ySteps = height * yDevider;
    for ( int y = 0; y < height * yDevider; y++ ) {
      for ( int x = 0; x <= width * xDevider; x++ ) {
        float amplitude = float(map(y / yDevider, 0, ySteps, 0, 800)) / 100;
        float noiseY = amplitude * complexNoise(float(x) / xDevider * 35, float(y) / yDevider / 5 * 35, 5);
        if ( x == 0 || x == width * xDevider) {
          writeGesture(float(x) / xDevider, float(y) / yDevider);
        } else {
          writeGesture(float(x) / xDevider, (float(y) / yDevider) + noiseY);
        }
      }
    }
  }
  if (pointer == 15) {
    writeType("absolute");
    int xDevider = 9;
    int yDevider = 9;
    float ySteps = height * yDevider;
    for ( int y = 0; y < height * yDevider; y++ ) {
      for ( int x = 0; x <= width * xDevider; x++ ) {
        float amplitude = float(map(y / yDevider, 0, ySteps, 0, 200)) / 100;
        float noiseY = amplitude * complexNoise(float(x) / xDevider * 35, float(y) / yDevider / 5 * 35, 5);
        if ( x == 0 || x == width * xDevider) {
          writeGesture(float(x) / xDevider, float(y) / yDevider);
        } else {
          writeGesture(float(x) / xDevider, (float(y) / yDevider) + noiseY);
        }
      }
    }
  }
  if ( pointer == 16) {
    writeType("absolute");
    int xDevider = 1;
    int yDevider = 1;
    float xCellSize = 1 / xDevider;
    float yCellSize = 1 / yDevider;
    for ( int y = 0; y < height * yDevider; y++ ) {
      for ( int x = 0; x <= width * xDevider; x++ ) {
        float xPos = float(x) / xDevider;
        float yPos = float(y) / yDevider;
        writeType("absolute");
        writeGesture(xPos, yPos);
        if ( complexNoise(xPos * 100, yPos * 100, 5) > 0 ) {
          fillRect(xPos, yPos, xCellSize, yCellSize, 6, "vertical");
        } else {
          drawRect(xCellSize, yCellSize);
        }
      }
    }
  }
  if ( pointer == 17 ) {
    float devider = 2;
    float nS = 10; //noiseScale
    float d = 0.5;
    writeType("absolute");
    for ( int y = 0; y <= height / devider; y++ ) {
      for ( int x = 0; x <= width / devider; x ++ ) {
        writeGesture(x * devider, 0);
        writeGesture(x * devider, y * devider);
        float xPos = x * devider;
        float yPos = y * devider;
        while ( true ) {
          float a = complexNoise(xPos * nS, yPos * nS, 5) * 360;
          float sTravel = cartesianInput(a, d, "scan");
          float fTravel = cartesianInput(a, d, "feed");
          xPos += sTravel;
          yPos += fTravel;
          writeGesture(xPos, yPos);
          //Serial.print(xPos);
          //Serial.print(',');
          //Serial.println(yPos);
          if ( xPos < 0 || xPos > width || yPos < 0 || yPos > height ) {
            if ( xPos > width / 2 ) {
              writeGesture(width, yPos);
              writeGesture(width, 0);
            } else {
              writeGesture(0, yPos);
              writeGesture(0, 0);
            }
            break;
          }
        }
      }
    }
  }
  if ( pointer == 18 ) {
    writeType("absolute");
    writeGesture(width, 0);
    writeGesture(width, height);
    writeGesture(0, height);
    writeGesture(0,0);
  }
}
