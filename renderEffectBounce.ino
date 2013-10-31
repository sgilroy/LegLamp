void renderEffectBounce(byte idx) {
  if(fxVars[idx][0] == 0) { // Initialize effect?
    gammaRespondsToForce = false;
    fxVars[idx][1] = random(maxHue + 1); // Random hue

    resetBouncePosition(idx);
    fxVars[idx][0] = 1; // Effect initialized
  }

  byte *ptr = &imgData[idx][0];
  int alpha;
  int halfPeriod = fxVars[idx][5] / 2;
  int distance;
  long color;
  long hue = pickHue(fxVars[idx][1]);
  for(long i=0; i<numPixels; i++) {
    alpha = getPointChaseAlpha(idx, (i + frontOffset) % numPixels, halfPeriod);
    if (alpha > 255) alpha = 255;
    
    color = hsv2rgb(hue, 255, alpha);
    *ptr++ = color >> 16; *ptr++ = color >> 8; *ptr++ = color;
  }
  
  const int gravityCentimetersPerSecondSquared = -980;
  const int pixelsPerMeter = 60;
  const int halfDegreesPerLoop = 720;
  const int acceleration = (((long) gravityCentimetersPerSecondSquared * pixelsPerMeter / 100) * halfDegreesPerLoop / numPixels) / fps;
//  const int acceleration = -180 * 60 / fps;
  fxVars[idx][3] += acceleration;
  fxVars[idx][4] += fxVars[idx][3] / fps;

  if (debugRenderEffects && idx == backImgIdx) {
    Serial.print("before ");
    Serial.println(fxVars[idx][3]);
  }

  if (fxVars[idx][4] < 0) {
    // bounce back with slightly less velocity
    int newVelocity = (((long) fxVars[idx][3]) * -95) / 100;
//    int newVelocity = -fxVars[idx][3] * 9 / 10;
    if (newVelocity < -acceleration)
    {
      if (debugRenderEffects && idx == backImgIdx) {
        Serial.print(" ball dead, reset ----------------- ");
        Serial.print(acceleration);
        Serial.print(" newVelocity: ");
        Serial.println(newVelocity);
      }
      resetBouncePosition(idx);
    }
    else
    {
      if (debugRenderEffects && idx == backImgIdx) Serial.println("bounce ----------------- ");

      fxVars[idx][3] = newVelocity;
      fxVars[idx][4] = 0;
    }
  }
  
  if (debugRenderEffects && idx == backImgIdx) {
    Serial.print("after  ");
    Serial.println(fxVars[idx][3]);
  }
}

void resetBouncePosition(int idx)
{
  fxVars[idx][3] = 0; // velocity
  fxVars[idx][5] = 720 * 8 / numPixels; // wave period (width)
  fxVars[idx][4] = 720 - fxVars[idx][5] / 2; // position (start near the top)
}
