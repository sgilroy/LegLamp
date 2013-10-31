void renderEffectNewtonsCradle(byte idx) {
  if(fxVars[idx][0] == 0) { // Initialize effect?
    gammaRespondsToForce = false;
    fxVars[idx][1] = random(maxHue + 1); // Random hue
    // Number of repetitions (complete loops around color wheel);
    // any more than 4 per meter just looks too chaotic.
    // Store as distance around complete belt in half-degree units:
//    fxVars[idx][2] = (1 + random(4 * ((numPixels + 31) / 32))) * 720;
    fxVars[idx][2] = 1 * 720;
    // Frame-to-frame increment (speed) -- may be positive or negative,
    // but magnitude shouldn't be so small as to be boring.  It's generally
    // still less than a full pixel per frame, making motion very smooth.
//    fxVars[idx][3] = 1 + random(720) / numPixels;
//    fxVars[idx][3] = 1;
    fxVars[idx][3] = 4;
    // Reverse direction half the time.
    if(random(2) == 0) fxVars[idx][3] = -fxVars[idx][3];
    fxVars[idx][4] = 0; // Current position
    fxVars[idx][0] = 1; // Effect initialized
//    fxVars[idx][5] = 15 + random(360); // wave period
//    fxVars[idx][5] = 30 + random(150); // wave period (width)
    fxVars[idx][5] = 720 * 4 / numPixels; // wave period (width)
//    fxVars[idx][5] = random(720 * 2 / numPixels, 180); // wave period (width)

  }

  byte *ptr = &imgData[idx][0];
  int alpha;
  int halfPeriod = fxVars[idx][5] / 2;
  int distance;
  long color;
  long hue = pickHue(fxVars[idx][1]);
  for(long i=0; i<numPixels; i++) {

//    int test = int((i + frontOffset) % numPixels);
//    Serial.println(test);
//    if ( test >numPixels )
//    test = numPixels;
//    
//    if ( test <= 0)
//    test = 0;
//    alpha = getPointChaseAlpha(idx, test, halfPeriod); // ?
  
    alpha = getPointChaseAlpha(idx, (i + frontOffset + 1) % numPixels, halfPeriod) + getPointChaseAlpha(idx, (numPixels - 1 - i + (numPixels - frontOffset)) % numPixels, halfPeriod);
//    alpha = getPointChaseAlpha(idx, i, halfPeriod); // no crash
//    alpha = getPointChaseAlpha(idx, (i + frontOffset) % numPixels, halfPeriod); // crash
//    alpha = getPointChaseAlpha(idx, (numPixels - 1 - i + (numPixels - frontOffset)) % numPixels, halfPeriod); // crash

    if (alpha > 255) alpha = 255;
    if (alpha < 0) alpha = 0;
//    alpha = 255;
    
    color = hsv2rgb(hue, 255, alpha);
    *ptr++ = color >> 16; *ptr++ = color >> 8; *ptr++ = color;
  }
  fxVars[idx][4] += fxVars[idx][3];
  fxVars[idx][4] %= 720;
}

