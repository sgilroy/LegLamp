#include <Adafruit_NeoPixel.h> // https://github.com/adafruit/Adafruit_NeoPixel
//#include <avr/pgmspace.h>
//#include "SPI.h"
//#include "LPD8806.h"
#include <TimerOne.h> // http://playground.arduino.cc/code/timer1
#include <Streaming.h> // http://arduiniana.org/libraries/streaming/

#define debugSettings true

int transitionEndCounter = 0;
int transitionStartCounter = 0;
int droppedFrames = 0;

const int fps = 60;

const int dataPin = 4;

const bool DEBUG_PRINTS = false;

int brightnessLimiter = 0;

// Declare the number of pixels in strand; 32 = 32 pixels in a row.  The
// LED strips have 32 LEDs per meter, but you can extend or cut the strip.
//const int numPixels = 30; // backpack
//const int numPixels = 42; // shoes
const int numPixels = 50; // belt
// 'const' makes subsequent array declarations possible, otherwise there
// would be a pile of malloc() calls later.

// Index (0 based) of the pixel at the front of the shoe. Used by some of the render effects.
int frontOffset = 4;

const int subPixels = 16;

// Instantiate LED strips; arguments are the total number of pixels in strip,
// the data pin number and clock pin number:
//LPD8806 stripLeft = LPD8806(numPixels, dataLeftPin, clockLeftPin);
//LPD8806 stripRight = LPD8806(numPixels, dataRightPin, clockRightPin);
//LPD8806 stripLeft2 = LPD8806(numPixels, dataLeft2Pin, clockLeft2Pin);
//LPD8806 stripRight2 = LPD8806(numPixels, dataRight2Pin, clockRight2Pin);

Adafruit_NeoPixel strip = Adafruit_NeoPixel(numPixels, dataPin, NEO_GRB + NEO_KHZ800);

// Principle of operation: at any given time, the LEDs depict an image or
// animation effect (referred to as the "back" image throughout this code).
// Periodically, a transition to a new image or animation effect (referred
// to as the "front" image) occurs.  During this transition, a third buffer
// (the "alpha channel") determines how the front and back images are
// combined; it represents the opacity of the front image.  When the
// transition completes, the "front" then becomes the "back," a new front
// is chosen, and the process repeats.
byte imgData[2][numPixels * 3], // Data for 2 strips worth of imagery
     alphaMask[numPixels],      // Alpha channel for compositing images
     backImgIdx = 0,            // Index of 'back' image (always 0 or 1)
     fxIdx[3];                  // Effect # for back & front images + alpha
int  fxVars[3][50],             // Effect instance variables (explained later)
     tCounter   = -1,           // Countdown to next transition
     transitionTime;            // Duration (in frames) of current transition
     
byte statusOverlay[numPixels * 3],
    statusOverlayAlpha[numPixels];
const int statusOverlayOffset = 36; // number of pixels to offset for visualizing the front clicks 

boolean visualizeClicks = false;

const static byte SYNCHRONIZED_EFFECT_BUFFERS = 3;
const static byte SYNCHRONIZED_EFFECT_VARIABLES = 5;

const boolean startWithTransition = true;
const boolean debugRenderEffects = false; // if true, use Serial.print to debug

boolean autoTransition = true; // if true, transition to a different render effect after some time elapses

// function prototypes, leave these be :)
void renderEffectSolidFill(byte idx);
void renderEffectLampSolidFill(byte idx);
void renderEffectLampFlicker(byte idx);
void renderEffectBluetoothLamp(byte idx);
void renderEffectRainbow(byte idx);
void renderEffectSineWaveChase(byte idx);
void renderEffectPointChase(byte idx);
void renderEffectNewtonsCradle(byte idx);
void renderEffectMonochromeChase(byte idx);
void renderEffectWavyFlag(byte idx);
void renderEffectThrob(byte idx);
void renderEffectDebug1(byte idx);
void renderEffectBlast(byte idx);
void renderEffectBounce(byte idx);
void renderEffectClickVisualization(byte idx);
void renderEffectPressureVis(byte idx);
void renderEffectSlide(byte idx);
void renderEffectSpectrum(byte idx);

void renderAlphaFade(void);
void renderAlphaWipe(void);
void renderAlphaDither(void);
void callback();
byte gamma(byte x, boolean useForce);
byte gamma(byte x);
long hsv2rgb(long h, byte s, byte v);
char fixSin(int angle);
char fixCos(int angle);
int getPointChaseAlpha(byte idx, long i, int halfPeriod);
long pickHue(long currentHue);

bool crazyMode = true;
long modeFrame = 0;
long crazyModeTime = fps * 60;
long lampModeTime = fps * 60 * 5;

// List of image effect and alpha channel rendering functions; the code for
// each of these appears later in this file.  Just a few to start with...
// simply append new ones to the appropriate list here:
void (*renderEffect[])(byte) = {
  renderEffectLampSolidFill,
  renderEffectLampSolidFill,
  renderEffectLampThrob,
//  renderEffectSpectrum,

  renderEffectThrob,
  renderEffectSineWaveChase,
  renderEffectSlide,
  renderEffectMonochromeChase,
  renderEffectRainbow,
  renderEffectSineWaveChase,
//  renderEffectPointChase,
//  renderEffectNewtonsCradle,
//  renderEffectWavyFlag,


//  renderEffectBlast,
//  renderEffectClickVisualization,
//  renderEffectMonochromeChase,
//  renderEffectRainbow,
//  renderEffectSineWaveChase,
//  renderEffectPointChase,
//  renderEffectNewtonsCradle,
//  renderEffectWavyFlag,
//  renderEffectBounce,
//  renderEffectPressureVis,

//  renderEffectDebug1
},
(*renderAlpha[])(void)  = {
  renderAlphaFade,
  renderAlphaWipe,
  renderAlphaDither
  };

/* ---------------------------------------------------------------------------------------------------
   FSR Variables
 
Connect one end of FSR to power, the other end to an analog input pin.
Then connect one end of a 10K resistor from the analog input pin to ground.
 
For more information see www.ladyada.net/learn/sensors/fsr.html */
 
const bool forceResistorInUse = false;
#define debugFsrReading false

int frontFsrStepFraction = 0;
int backFsrStepFraction = 0;
const int fsrStepFractionMax = 60;
bool gammaRespondsToForce = false;

// optional filtering to smooth out values if readings are too unstable
#define numFsrReadings 1
#if numFsrReadings > 1
int fsrReadingIndex = 0;
int fsrReadings[numFsrReadings];
#endif

const int maxHue = 1535;

boolean slaveMode = false;

boolean inCallback = false;

// ---------------------------------------------------------------------------
//
//                  SETUP
//
// ---------------------------------------------------------------------------

void setup() {
  
#if numFsrReadings > 1
  for (int i = 0; i < numFsrReadings; i++)
  {
    fsrReadings[i] = 0;
  }
#endif

  // Start up the LED strip.  Note that strip.show() is NOT called here --
  // the callback function will be invoked immediately when attached, and
  // the first thing the calback does is update the strip.
  strip.begin();

  // Initialize random number generator from a floating analog input.
  randomSeed(analogRead(0));
  memset(imgData, 0, sizeof(imgData)); // Clear image data
  memset(statusOverlay, 0, sizeof(statusOverlay));
  memset(statusOverlayAlpha, 0, sizeof(statusOverlayAlpha));
  fxVars[backImgIdx][0] = 1;           // Mark back image as initialized

  if (startWithTransition) {
    tCounter = -1;
  } else {
    tCounter = 0;
  }

  // Timer1 is used so the strip will update at a known fixed frame rate.
  // Each effect rendering function varies in processing complexity, so
  // the timer allows smooth transitions between effects (otherwise the
  // effects and transitions would jump around in speed...not attractive).
  Timer1.initialize();
  
  Timer1.attachInterrupt(callback, 1000000 / fps);
//  Timer1.attachInterrupt(callback, 1000000 * 2); // 1 frame / 2 seconds
}

void loop() {
  // All rendering happens in the callback() function below.
}

long pickHue(long currentHue)
{
  return currentHue;
}

// Timer1 interrupt handler.  Called at equal intervals; 60 Hz by default.
void callback() {
  // don't do anything if we have not yet finished with the previous callback
  if (inCallback) {
    droppedFrames++;
    return;
  }
  
  inCallback = true;
  
  // Very first thing here is to issue the strip data generated from the
  // *previous* callback.  It's done this way on purpose because show() is
  // roughly constant-time, so the refresh will always occur on a uniform
  // beat with respect to the Timer1 interrupt.  The various effects
  // rendering and compositing code is not constant-time, and that
  // unevenness would be apparent if show() were called at the end.
  strip.show(); // Initialize all pixels to 'off'
  
  byte frontImgIdx = 1 - backImgIdx,
       *backPtr    = &imgData[backImgIdx][0],
       r, g, b;
  int  i;

  // Always render back image based on current effect index:
  (*renderEffect[fxIdx[backImgIdx]])(backImgIdx);

  // Front render and composite only happen during transitions...
  if(tCounter > 0) {
    // Transition in progress
    byte *frontPtr = &imgData[frontImgIdx][0];
    int  alpha, inv;

    // Render front image and alpha mask based on current effect indices...
    (*renderEffect[fxIdx[frontImgIdx]])(frontImgIdx);
    (*renderAlpha[fxIdx[2]])();

    // ...then composite front over back:
    for(i=0; i<numPixels; i++) {
      alpha = alphaMask[i] + 1; // 1-256 (allows shift rather than divide)
      inv   = 257 - alpha;      // 1-256 (ditto)
      // r, g, b are placed in variables (rather than directly in the
      // setPixelColor parameter list) because of the postincrement pointer
      // operations -- C/C++ leaves parameter evaluation order up to the
      // implementation; left-to-right order isn't guaranteed.
      r = gamma((*frontPtr++ * alpha + *backPtr++ * inv) >> 8);
      g = gamma((*frontPtr++ * alpha + *backPtr++ * inv) >> 8);
      b = gamma((*frontPtr++ * alpha + *backPtr++ * inv) >> 8);
      strip.setPixelColor(i, r, g, b);
    }
  } else {
    // No transition in progress; just show back image
    for(i=0; i<numPixels; i++) {
      // See note above re: r, g, b vars.
      r = gamma(*backPtr++);
      g = gamma(*backPtr++);
      b = gamma(*backPtr++);
      strip.setPixelColor(i, r, g, b);
    }
  }
  
  for(i=0; i<numPixels; i++) {
    if (statusOverlayAlpha[i] > 0) {
      statusOverlayAlpha[i]--;
      
      int alpha = statusOverlayAlpha[i] + 1; // 1-256 (allows shift rather than divide)
      byte *overlayPtr = &statusOverlay[i * 3];

      // See note above re: r, g, b vars.
      r = gamma(*overlayPtr++ * alpha >> 8, false);
      g = gamma(*overlayPtr++ * alpha >> 8, false);
      b = gamma(*overlayPtr++ * alpha >> 8, false);
      strip.setPixelColor(i, r, g, b);
    }
  }
  
  if (!slaveMode)
  {
    modeFrame++;
    if (modeFrame == 0) { // Crazy mode start
      tCounter == 0;
      crazyMode = true;
    } else if (modeFrame >= crazyModeTime) {
      tCounter == 0;
      modeFrame = -lampModeTime;
      crazyMode = false;
    }

    // Count up to next transition (or end of current one):
    if (autoTransition || tCounter >= 0)
    {
      tCounter++;
      if(tCounter == 0) { // Transition start
        startImageTransition(frontImgIdx);
      } else if(tCounter >= transitionTime) { // End transition
        endImageTransition(frontImgIdx);
      }
    }
  }
  

  if (DEBUG_PRINTS)
  {
    Serial.print("callback complete ");
    Serial.print("dataPin ");
    Serial.print(dataPin);
    Serial.println();
  }

  inCallback = false;
}

void startImageTransition(byte frontImgIdx, byte effectFunctionIndex, int inTransitionTime)
{
  fxIdx[frontImgIdx] = effectFunctionIndex;
  fxIdx[2]           = random((sizeof(renderAlpha)  / sizeof(renderAlpha[0])));
  transitionTime     = inTransitionTime;
  fxVars[frontImgIdx][0] = 0; // Effect not yet initialized
  fxVars[2][0]           = 0; // Transition not yet initialized

  transitionStartCounter++;
  Serial << "transitionStartCounter " << transitionStartCounter << endl;
}

void startImageTransition(byte frontImgIdx)
{
  int effectFunctionIndex = -1;
  int inTransitionTime;
  
  // Randomly pick next image effect and alpha effect indices:
  // 0.5 to 3 second transitions
  if (effectFunctionIndex == -1) {
    if (crazyMode)
      effectFunctionIndex = random((sizeof(renderEffect) / sizeof(renderEffect[0])) - 3) + 3;
    else
      effectFunctionIndex = random(3);

    inTransitionTime = random(fps / 2, fps * 3);
  }
  
  startImageTransition(frontImgIdx, effectFunctionIndex, inTransitionTime);
}

void endImageTransition(byte frontImgIdx)
{
  fxIdx[backImgIdx] = fxIdx[frontImgIdx]; // Move front effect index to back
  backImgIdx        = 1 - backImgIdx;     // Invert back index
  tCounter          = -120 - random(240); // Hold image 2 to 6 seconds
//    tCounter          = -600; // Hold image 10 seconds
  transitionEndCounter++;
  Serial << "transitionEndCounter " << transitionEndCounter << endl;
}  


// ---------------------------------------------------------------------------
// Image effect rendering functions.  Each effect is generated parametrically
// (that is, from a set of numbers, usually randomly seeded).  Because both
// back and front images may be rendering the same effect at the same time
// (but with different parameters), a distinct block of parameter memory is
// required for each image.  The 'fxVars' array is a two-dimensional array
// of integers, where the major axis is either 0 or 1 to represent the two
// images, while the minor axis holds 50 elements -- this is working scratch
// space for the effect code to preserve its "state."  The meaning of each
// element is generally unique to each rendering effect, but the first element
// is most often used as a flag indicating whether the effect parameters have
// been initialized yet.  When the back/front image indexes swap at the end of
// each transition, the corresponding set of fxVars, being keyed to the same
// indexes, are automatically carried with them.

// Simplest rendering effect: fill entire image with solid color
void renderEffectSolidFill(byte idx) {
  // Only needs to be rendered once, when effect is initialized:
  if(fxVars[idx][0] == 0) {
    gammaRespondsToForce = true;
    fxVars[idx][1] = random(256);
    fxVars[idx][2] = random(256);
    fxVars[idx][3] = random(256);
    fxVars[idx][0] = 1; // Effect initialized
  }
  
  byte *ptr = &imgData[idx][0],
    r = fxVars[idx][1], g = fxVars[idx][2], b = fxVars[idx][3];
  for(int i=0; i<numPixels; i++) {
    *ptr++ = r; *ptr++ = g; *ptr++ = b;
  }
  
}

void renderEffectDebug1(byte idx) {
  // Only needs to be rendered once, when effect is initialized:
  if(fxVars[idx][0] == 0) {
    gammaRespondsToForce = true;
    byte *ptr = &imgData[idx][0],
      r = 0, g = 0, b = 0;
    for(int i=0; i<numPixels; i++) {
      if (i % (numPixels / 4) == 0)
      {
        r = g = b = 255;
      }
      else
      {
        r = g = b = 0;
      }
        
      *ptr++ = r; *ptr++ = g; *ptr++ = b;
    }
    fxVars[idx][0] = 1; // Effect initialized
  }
}

// Rainbow effect (1 or more full loops of color wheel at 100% saturation).
// Not a big fan of this pattern (it's way overused with LED stuff), but it's
// practically part of the Geneva Convention by now.
void renderEffectRainbow(byte idx) {
  if(fxVars[idx][0] == 0) { // Initialize effect?
    gammaRespondsToForce = true;
    // Number of repetitions (complete loops around color wheel); any
    // more than 4 per meter just looks too chaotic and un-rainbow-like.
    // Store as hue 'distance' around complete belt:
    fxVars[idx][1] = (1 + random(4 * ((numPixels + 31) / 32))) * (maxHue + 1);
    // Frame-to-frame hue increment (speed) -- may be positive or negative,
    // but magnitude shouldn't be so small as to be boring.  It's generally
    // still less than a full pixel per frame, making motion very smooth.
    fxVars[idx][2] = 4 + random(fxVars[idx][1]) / numPixels;
    // Reverse speed and hue shift direction half the time.
    if(random(2) == 0) fxVars[idx][1] = -fxVars[idx][1];
    if(random(2) == 0) fxVars[idx][2] = -fxVars[idx][2];
    fxVars[idx][3] = 0; // Current position
    fxVars[idx][0] = 1; // Effect initialized
  }

  byte *ptr = &imgData[idx][0];
  long color, i;
  for(i=0; i<numPixels; i++) {
    color = hsv2rgb(fxVars[idx][3] + fxVars[idx][1] * i / numPixels,
      255, 255);
    *ptr++ = color >> 16; *ptr++ = color >> 8; *ptr++ = color;
  }
  fxVars[idx][3] += fxVars[idx][2];
}

// Sine wave chase effect
void renderEffectSineWaveChase(byte idx) {
  if(fxVars[idx][0] == 0) { // Initialize effect?
    gammaRespondsToForce = true;
    fxVars[idx][1] = random(maxHue + 1); // Random hue
    // Number of repetitions (complete loops around color wheel);
    // any more than 4 per meter just looks too chaotic.
    // Store as distance around complete belt in half-degree units:
    fxVars[idx][2] = (1 + random(4 * ((numPixels + 31) / 32))) * 720;
    // Frame-to-frame increment (speed) -- may be positive or negative,
    // but magnitude shouldn't be so small as to be boring.  It's generally
    // still less than a full pixel per frame, making motion very smooth.
    fxVars[idx][3] = 8 + random(fxVars[idx][1]) / numPixels;
    // Reverse direction half the time.
    if(random(2) == 0) fxVars[idx][3] = -fxVars[idx][3];
    fxVars[idx][4] = 0; // Current position
    fxVars[idx][0] = 1; // Effect initialized
  }

  byte *ptr = &imgData[idx][0];
  int  foo;
  long color, i;
  long hue = pickHue(fxVars[idx][1]);
  for(long i=0; i<numPixels; i++) {
    foo = fixSin(fxVars[idx][4] + fxVars[idx][2] * i / numPixels);
    // Peaks of sine wave are white, troughs are black, mid-range
    // values are pure hue (100% saturated).
    color = (foo >= 0) ?
       hsv2rgb(hue, 254 - (foo * 2), 255) :
       hsv2rgb(hue, 255, 254 + foo * 2);
    *ptr++ = color >> 16; *ptr++ = color >> 8; *ptr++ = color;
  }
  fxVars[idx][4] += fxVars[idx][3];
}

void renderEffectPointChase(byte idx) {
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
    fxVars[idx][3] = 1 + random(720) / numPixels;
//    fxVars[idx][3] = 1;
    // Reverse direction half the time.
    if(random(2) == 0) fxVars[idx][3] = -fxVars[idx][3];
    fxVars[idx][4] = 0; // Current position
    fxVars[idx][0] = 1; // Effect initialized
//    fxVars[idx][5] = 15 + random(360); // wave period
    fxVars[idx][5] = random(720 * 2 / numPixels, 180); // wave period (width)
  }

  byte *ptr = &imgData[idx][0];
  int  foo;
  int theta;
  int offset;
  long color, i;
  int halfPeriod = fxVars[idx][5] / 2;
  int distance;
  long hue = pickHue(fxVars[idx][1]);
  for(long i=0; i<numPixels; i++) {
    // position of current pixel in 1/2 degrees
    offset = fxVars[idx][2] * i / numPixels;
    theta = offset - fxVars[idx][4];
    distance = (offset + fxVars[idx][4]) % fxVars[idx][2];
    foo = distance > fxVars[idx][5] || distance < 0 ? -127 : fixSin((distance * 360 / halfPeriod) - 180);
    // Peaks of sine wave are white, troughs are black, mid-range
    // values are pure hue (100% saturated).
    color = hsv2rgb(hue, 255, 127 + foo);
    *ptr++ = color >> 16; *ptr++ = color >> 8; *ptr++ = color;
  }
  fxVars[idx][4] += fxVars[idx][3];
  fxVars[idx][4] %= 720;
}

void renderEffectSpectrum(byte idx) {
  if(fxVars[idx][0] == 0) { // Initialize effect?
    gammaRespondsToForce = true;
    // min hue
    fxVars[idx][1] = 0;
    // max hue
    fxVars[idx][2] = maxHue;
    fxVars[idx][3] = 0; // Current position
    fxVars[idx][0] = 1; // Effect initialized
  }

  byte *ptr = &imgData[idx][0];
  long color, i;
  for(i=0; i<numPixels; i++) {
//    color = hsv2rgb(map(i, 0, numPixels - 1, 100, 600), 255, 255); // range of colors, red to green
//    color = hsv2rgb(300, map(i, 0, numPixels - 1, 0, 255), 255); // range of saturations, 0 to 255
    color = hsv2rgb(300, 80, map(i, 0, numPixels - 1, 0, 255)); // range of brightnesses
    *ptr++ = color >> 16; *ptr++ = color >> 8; *ptr++ = color;
  }
}

void renderEffectThrob(byte idx) {
  if(fxVars[idx][0] == 0) { // Initialize effect?
    gammaRespondsToForce = false;
    fxVars[idx][1] = random(maxHue + 1); // Random hue
    // Number of repetitions (complete loops around color wheel);
    // any more than 4 per meter just looks too chaotic.
    // Store as distance around complete belt in half-degree units:
    fxVars[idx][2] = (1 + random(4 * ((numPixels + 31) / 32))) * 720;
//    fxVars[idx][2] = 4;
    // Frame-to-frame increment (speed) -- may be positive or negative,
    // but magnitude shouldn't be so small as to be boring.  It's generally
    // still less than a full pixel per frame, making motion very smooth.
    fxVars[idx][3] = fps / 8 + random(fps * 3);
    // Reverse direction half the time.
    if(random(2) == 0) fxVars[idx][3] = -fxVars[idx][3];
    fxVars[idx][4] = 0; // Current position
    fxVars[idx][0] = 1; // Effect initialized
  }

  byte *ptr = &imgData[idx][0];
  int  foo;
  long color, i;
  long hue = pickHue(fxVars[idx][1]);
  foo = fixSin(fxVars[idx][4]); // -127 (?) to 128 for 0 to 100% brightness
//  foo = fixSin(fxVars[idx][4]) / 2 + 64; // 50 to 100 % brightness
  for(long i=0; i<numPixels; i++) {
    // Peaks of sine wave are white, troughs are black, mid-range
    // values are pure hue (100% saturated).
    color = hsv2rgb(hue, 255, 127 + foo);
//    color = hsv2rgb(300, 80, 127 + foo);
    *ptr++ = color >> 16; *ptr++ = color >> 8; *ptr++ = color;
  }
  fxVars[idx][4] += fxVars[idx][3];
}

void renderEffectLampThrob(byte idx) {
  if(fxVars[idx][0] == 0) { // Initialize effect?
    gammaRespondsToForce = false;
    fxVars[idx][1] = random(maxHue + 1); // Random hue
    // Number of repetitions (complete loops around color wheel);
    // any more than 4 per meter just looks too chaotic.
    // Store as distance around complete belt in half-degree units:
    fxVars[idx][2] = (1 + random(4 * ((numPixels + 31) / 32))) * 720;
//    fxVars[idx][2] = 4;
    // Frame-to-frame increment (speed) -- may be positive or negative,
    // but magnitude shouldn't be so small as to be boring.  It's generally
    // still less than a full pixel per frame, making motion very smooth.
    fxVars[idx][3] = 2 + random(fxVars[idx][1] / 5) / numPixels;
    // Reverse direction half the time.
    if(random(2) == 0) fxVars[idx][3] = -fxVars[idx][3];
    fxVars[idx][4] = 0; // Current position
    fxVars[idx][0] = 1; // Effect initialized
  }

  byte *ptr = &imgData[idx][0];
  int  foo;
  long color, i;
  long hue = pickHue(fxVars[idx][1]);
//  foo = fixSin(fxVars[idx][4]); // -127 (?) to 128 for 0 to 100% brightness
  foo = fixSin(fxVars[idx][4]) / 2 + 64; // 50 to 100 % brightness
  for(long i=0; i<numPixels; i++) {
    // Peaks of sine wave are white, troughs are black, mid-range
    // values are pure hue (100% saturated).
//    color = hsv2rgb(hue, 255, 127 + foo);
    color = hsv2rgb(300, 80, 127 + foo);
    *ptr++ = color >> 16; *ptr++ = color >> 8; *ptr++ = color;
  }
  fxVars[idx][4] += fxVars[idx][3];
}

// TO DO: Add more effects here...Larson scanner, etc.

// Simplest rendering effect: fill entire image with solid color
void renderEffectLampSolidFill(byte idx) {
  // Only needs to be rendered once, when effect is initialized:
  if(fxVars[idx][0] == 0) {
    byte *ptr = &imgData[idx][0];
    long color, i;
    for(long i=0; i<numPixels; i++) {
      color = hsv2rgb(300, 80, 255);
      *ptr++ = color >> 16; *ptr++ = color >> 8; *ptr++ = color;
    }

    fxVars[idx][0] = 1; // Effect initialized
  }
}

void renderEffectFlicker(byte idx) {
  // Only needs to be rendered once, when effect is initialized:
  if(fxVars[idx][0] == 0) {
    fxVars[idx][1] = fps * 1 + random(fps * 2); // flicker episode delay
    fxVars[idx][2] = random(fps / 2); // time between flickers in an episode
    fxVars[idx][3] = random(5); // number of flickers in an episode
    fxVars[idx][4] = 0; // current frame
    fxVars[idx][5] = 0; // current flicker stage (0: pre-flicker, >0: flicker episode)

    fxVars[idx][0] = 1; // Effect initialized
  }

  long b = 255;
//  if (

  byte *ptr = &imgData[idx][0];
  long color, i;
  for(long i=0; i<numPixels; i++) {
    color = hsv2rgb(300, 80, b);
    *ptr++ = color >> 16; *ptr++ = color >> 8; *ptr++ = color;
  }
}

void renderEffectSlide(byte idx) {
  if(fxVars[idx][0] == 0) { // Initialize effect?
    gammaRespondsToForce = false;
    fxVars[idx][1] = random(maxHue + 1); // Random hue
    // Frame-to-frame increment (speed) -- may be positive or negative,
    // but magnitude shouldn't be so small as to be boring.
//    fxVars[idx][3] = 1;
    // Reverse direction half the time.
//    if(random(2) == 0) fxVars[idx][3] = -fxVars[idx][3];
    fxVars[idx][5] = 0; // Current position
    fxVars[idx][0] = 1; // Effect initialized
    fxVars[idx][4] = 0;
    fxVars[idx][3] = random(10) + 1; // acceleration
    // Reverse direction half the time.
    if(random(2) == 0) fxVars[idx][3] = -fxVars[idx][3];
  }

  int hue = pickHue(fxVars[idx][1]);

  clearImage(idx);
  
  int x = fxVars[idx][5];
  int width = subPixels * 8; // width;
  drawLine(idx, x, width, hue);
  
//  fxVars[idx][3] += fxVars[idx][6];
//  if (abs(fxVars[idx][6]) > 

  // velocity
//  int v = map(frontFsrStepFraction, 0, fsrStepFractionMax, 0, 100);
  
//  if (!slaveMode)
//    fxVars[idx][3] = fixSin(720 * millis() / 10000 / 3) / 2;
//    
  fxVars[idx][4] += fxVars[idx][3];
  fxVars[idx][5] += fxVars[idx][4] / 30;
  if (fxVars[idx][5] < 0)
  {
    fxVars[idx][5] += numPixels * subPixels;
  }
  fxVars[idx][5] %= numPixels * subPixels;
}

void drawLine(byte idx, int x, int width, int hue)
{
  long color, i;
  byte alpha = 0;
  
  byte *ptr;
  int xp = 0;
  // draw a line at position x that is width subpixels wide
  for(long i=x; i<x + width; i++) {
    alpha++;
    if ((i + 1) % subPixels == 0)
    {
      color = hsv2rgb(hue, 255, alpha * 255 / subPixels);
      xp = i / subPixels;
//      ptr = &imgData[idx][xp % numPixels * 3];
//      *ptr++ = color >> 16; *ptr++ = color >> 8; *ptr++ = color;
      setPixel(idx, xp % numPixels, color);
      alpha = 0;
    }
  }

  color = hsv2rgb(hue, 255, alpha * 255 / subPixels);
  setPixel(idx, (xp + 1) % numPixels, color);
}

void setPixel(byte idx, int xp, long color)
{
  if (idx >= 0 && idx < 3 && xp >= 0 && xp < numPixels * 3)
  {
    byte *ptr = &imgData[idx][xp * 3];
    *ptr++ = color >> 16; *ptr++ = color >> 8; *ptr++ = color;
  }
}

void clearImage(byte idx)
{
  byte *ptr = &imgData[idx][0];
  for(long i = 0; i < numPixels; i++) {
    *ptr++ = 0; *ptr++ = 0; *ptr++ = 0;
  }
}
