void drawRect(float x, float y) {
  writeType("relative");
  writeGesture(x, 0);
  writeGesture(0, y);
  writeGesture(-x, 0);
  writeGesture(0, -y);
}

void drawV(float x, float y) {
  writeType("relative");
  writeGesture(x/2, y);
  writeGesture(x/2, -y);
  writeGesture(-x/2, y);
  writeGesture(-x/2, -y);
}

void drawCircle(int segments, float radius) {
  float circumference = PI * radius;
  float segmentLength = circumference / segments;
  writeType("relative");
  for ( int i = 0; i < segments; i++ ) {
    float angle = (i + 0.5) * (360 / segments);
    writeGesture(cartesianInput(angle, segmentLength, "scan"),
                 cartesianInput(angle, segmentLength, "feed"));
  }
}

void fillTriangle(float aS, float aF, float x, float y, int d ) {
  writeType("absolute");
  for ( int i = 0; i < d; i++ ) {
    writeGesture(aS + x / d * i, aF + 0);
    writeGesture(aS + 0, aF + y / d * i);
  }
  writeGesture(0,0);
}

void fillRect(float aS, float aF, float x, float y, int d, String mode ) {
  //m0 -> diagonal
  //m1 -> vertical
  //m2 -> horizontal
  writeType("absolute");
  if ( mode == "diagonal" ) {
    for ( int i = 0; i < d; i++ ) {
      writeGesture(aS + x / d * i, aF + 0);
      writeGesture(aS + 0, aF + y / d * i);
    }
    for ( int i = 0; i < d; i++ ) {
      writeGesture(aS + x, aF + y / d * i);
      writeGesture(aS + x / d * i, aF + y);
    }
  } else if ( mode == "vertical" ) {
    for ( int i = 0; i < d; i++ ) {
      writeGesture(aS + x / d * i, aF + 0);
      writeGesture(aS + x / d * i, aF + y);
    }
  } else if ( mode == "horizontal" ) {
    for ( int i = 0; i < d; i++ ) {
      writeGesture(aS + 0, aF + y / d * i);
      writeGesture(aS + x, aF + y / d * i);
    }
  }
  writeGesture(aS,aF);
}
