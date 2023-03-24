//---------------------------------------------------------------------
// Polar Waves - a simple demo for testing performance on different platforms & setups
//
// VO.1 unofficial preview version
// (c) by Stefan Petrick 2023
// I publish this code under creative commons license CC BY-NC 3.0

#include <FastLED.h>

#define ENABLE_LEDMAP
#define USE_LOCAL_LEDMAP

#define PANEL_WIDTH 16
#define PANEL_HEIGHT 16
#define NUM_PANELS_PER_ROW 7
#define NUM_PANELS_PER_COLUMN 2
#define SCREEN_WIDTH PANEL_WIDTH * NUM_PANELS_PER_ROW
#define SCREEN_HEIGHT PANEL_HEIGHT * NUM_PANELS_PER_COLUMN
#define NUM_LEDS ((SCREEN_WIDTH) * (SCREEN_HEIGHT))

#include "I2SClocklessLedDriver.h"

struct ColorSum
{
  int red = 0;
  int green = 0;
  int blue = 0;
};

struct Parameters
{
  float dist, newangle;                // parameters for reconstruction
  uint32_t z;                                // 3rd dimension for the 3d noise function
  float offset_x, offset_y;               // wanna shift the cartesians during runtime?
  float scale_x, scale_y;                 // cartesian scaling in 2 dimensions
  float distSqrt, angle;                      // the actual polar coordinates
  float center_x = (SCREEN_WIDTH / 2) - 0.5;     // the reference point for polar coordinates
  float center_y = (SCREEN_HEIGHT / 2) - 0.5;     // (can also be outside of the actual xy matrix)
};

struct
{
  CRGB *currentBuffer;
  double notTime = 0;                // global anaimation speed
  double linear_c = 0;               // some linear rising offsets
  double linear_d = 0;
  double linear_e = 0;

  float angle_c = fmod(linear_c, 2 * PI);     // some angle offsets
  float angle_d = fmod(linear_d, 2 * PI);
  float angle_e = fmod(linear_e, 2 * PI);

} volatile shared;

int debugPeriod = 100;

I2SClocklessLedDriver driver;
OffsetDisplay offd;

// Background for setting the following 2 numbers: the FastLED inoise16() function returns
// raw values ranging from 0-65535. In order to improve contrast we filter this output and
// stretch the remains. In histogram (photography) terms this means setting a blackpoint and
// a whitepoint. low_limit MUST be smaller than high_limit.

uint16_t low_limit  = 30000;            // everything lower drawns in black
                                        // higher numer = more black & more contrast present
uint16_t high_limit = 50000;            // everything higher gets maximum brightness & bleeds out
                                        // lower number = the result will be more bright & shiny

CRGB leds[SCREEN_WIDTH * SCREEN_HEIGHT * 2];              // framebuffer
// CRGB leds[SCREEN_WIDTH * SCREEN_HEIGHT];              // framebuffer
// CRGB *currentBuffer = leds;

float theta[SCREEN_WIDTH] [SCREEN_HEIGHT];
float distanceSqrt[SCREEN_WIDTH] [SCREEN_HEIGHT];

// struct OpenSimplex2F_context *ctx;

TaskHandle_t task;
volatile xSemaphoreHandle loop1go = xSemaphoreCreateBinary();
volatile xSemaphoreHandle loop2go = xSemaphoreCreateBinary();

void doTheThing(int xStart, int xEnd, int yStart, int yEnd)
{
  struct Parameters parameters;
  
  parameters.scale_x = 15000;
  parameters.scale_y = 15000;
  // newdist = sqrtf(dist);
  // newdist = sqrtApprox(dist);
  // parameters.z = shared.linear_c * 100000;
  // z = 500000000 - (( dist * 0.166666667 ) - linear_c) * 100000;
  parameters.offset_x = 0;
  parameters.offset_y = 0;
  uint32_t z = fmod(shared.linear_c * 100000.0, 4294967296);
  for (int x = xStart; x < xEnd; x++)
  {
    for (int y = yStart; y < yEnd; y++)
    {
      parameters.distSqrt  = distanceSqrt [x][y];
      parameters.dist = parameters.distSqrt * parameters.distSqrt; // * parameters.distSqrt * parameters.distSqrt;
      parameters.z = z - (uint32_t)(parameters.dist * 16666.66667);
      parameters.angle = theta    [x][y];
      parameters.newangle = parameters.angle + shared.angle_c - (parameters.dist * 0.1);
      shared.currentBuffer[x + y * SCREEN_WIDTH].r = render_pixel(parameters);
    }
  }

  parameters.scale_x = 14000;
  parameters.scale_y = 14000;
  // parameters.z = shared.linear_d * 110000;
  // z = 500000000 - (( dist * 0.25 ) - linear_d) * 110000;
  parameters.offset_x = 42;
  parameters.offset_y = 69;
  z = fmod(shared.linear_d * 110000.0, 4294967296);
  for (int x = xStart; x < xEnd; x++)
  {
    for (int y = yStart; y < yEnd; y++)
    {
      parameters.distSqrt  = distanceSqrt [x][y];
      parameters.dist = parameters.distSqrt * parameters.distSqrt; // * parameters.distSqrt * parameters.distSqrt;
      parameters.z = z - (uint32_t)(parameters.dist * 22000.0);
      parameters.angle = theta    [x][y];
      parameters.newangle = parameters.angle + shared.angle_c - (parameters.dist * 0.115);
      shared.currentBuffer[x + y * SCREEN_WIDTH].g = render_pixel(parameters);
    }
  }

  parameters.scale_x = 13000;
  parameters.scale_y = 13000;
  // parameters.z = shared.linear_e * 120000;
  // z = 500000000 - (( dist * 0.25 ) - linear_e) * 120000;
  parameters.offset_x = 420;
  parameters.offset_y = 690;
  z = fmod(shared.linear_e * 120000.0, 4294967296);
  for (int x = xStart; x < xEnd; x++)
  {
    for (int y = yStart; y < yEnd; y++)
    {
      parameters.distSqrt  = distanceSqrt [x][y];
      parameters.dist = parameters.distSqrt * parameters.distSqrt; // * parameters.distSqrt * parameters.distSqrt;
      parameters.z = z - (uint32_t)(parameters.dist * 30000.0);
      parameters.angle = theta    [x][y];
      parameters.newangle = parameters.angle + shared.angle_e - (parameters.dist * 0.095);
      shared.currentBuffer[x + y * SCREEN_WIDTH].b = render_pixel(parameters);
    }
  }
}

void loop2(void* parameter)
{
  while (true)
  {
    xSemaphoreTake(loop2go, portMAX_DELAY);
    doTheThing(0, SCREEN_WIDTH, SCREEN_HEIGHT / 2, SCREEN_HEIGHT);
    struct ColorSum colorSum = adjust_gamma(&shared.currentBuffer[NUM_LEDS / 2], NUM_LEDS / 2);
    ESP_LOGV(LOG, "giving it back");
    xSemaphoreGive(loop1go);
  }
}

void setup() {

  Serial.begin(115200);                 // check serial monitor for current fps count
  
  // Teensy 3.x & 4.x users: make sure to use the hardware SPI pins 11 & 13
  // for best performance
  
  // FastLED.addLeds<APA102, 11, 13, BGR, DATA_RATE_MHZ(12)>(leds, NUM_LEDS);    
  Serial.println("Booted up");
 
  generate_lookup_table();                // precalculate as much data as possible to improve the framerate
  
  int pins[] = {32, 33, 25, 26, 27, 14, 12, 23, 22, 21, 19, 18, 5, 17};              // esp32 dev kit v1
  #ifndef WAIT_UNTIL_DRAWING_DONE
  driver.__displayMode = NO_WAIT;
  #endif
  driver._offsetDisplay.panel_width = SCREEN_WIDTH;
  driver._offsetDisplay.panel_height = SCREEN_HEIGHT;
  driver._defaultOffsetDisplay.panel_height = driver._offsetDisplay.panel_height;
  
  driver.initled((uint8_t*)shared.currentBuffer, pins, NUM_PANELS_PER_ROW * NUM_PANELS_PER_COLUMN, PANEL_WIDTH * PANEL_HEIGHT, ORDER_GRB);
  
  // This is not cool. These settings are nuked by initled()
  driver._offsetDisplay.panel_width = SCREEN_WIDTH;
  driver._offsetDisplay.panel_height = SCREEN_HEIGHT;
  driver._defaultOffsetDisplay.panel_height = driver._offsetDisplay.panel_height;
    
  offd = driver.getDefaultOffset();
  offd.panel_width=SCREEN_WIDTH;
  offd.panel_height=SCREEN_HEIGHT;
  
  // OpenSimplex2F(77374, &ctx);
  
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  
  Serial.print("ESP.getFreeHeap() = ");
  Serial.println(ESP.getFreeHeap());
  Serial.print("heap_caps_get_largest_free_block(MALLOC_CAP_8BIT) = ");
  Serial.println(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  Serial.print("heap_caps_get_largest_free_block(MALLOC_CAP_32BIT) = ");
  Serial.println(heap_caps_get_largest_free_block(MALLOC_CAP_32BIT));

  Serial.print("ESP.getFreeHeap() = ");
  Serial.println(ESP.getFreeHeap());
  Serial.print("heap_caps_get_largest_free_block(MALLOC_CAP_8BIT) = ");
  Serial.println(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  Serial.print("heap_caps_get_largest_free_block(MALLOC_CAP_32BIT) = ");
  Serial.println(heap_caps_get_largest_free_block(MALLOC_CAP_32BIT));
  
  xTaskCreatePinnedToCore(
      loop2, // Function to implement the task
      "random babble", // Name of the task
      10000,  // Stack size in words
      NULL,  // Task input parameter
      0,  // Priority of the task
      &task,  // Task handle.
      0); // not core 1
  
  delay(100);
  
}

void loop() {

  // float c, d, e, f;
  // float notTime;                 // some timedependant counters
  // int x, y;                               // the cartesian coordiantes

  // set timers
  static struct ColorSum cumulativeColorSum;
  static uint32_t frameNumber = 0;
  static uint32_t newTime, oldTime = 0;
  ESP_LOGV(LOG, "first");
  shared.currentBuffer = &leds[(frameNumber % 2) * NUM_LEDS];
  ESP_LOGV(LOG, "second");
  uint32_t loopStartMillis = millis();                  // save elapsed ms since start up

  shared.notTime = loopStartMillis * 0.6;                // global anaimation speed

  shared.linear_c = shared.notTime * 0.0025;               // some linear rising offsets
  shared.linear_d = shared.notTime * 0.0030;
  shared.linear_e = shared.notTime * 0.0023;

  shared.angle_c = fmod(shared.linear_c, 2 * PI);     // some angle offsets
  shared.angle_d = fmod(shared.linear_d, 2 * PI);
  shared.angle_e = fmod(shared.linear_e, 2 * PI);

  /*
  float dir_c = sinf(c);                        // some multiplicator offests (for direction control)
  float dir_d = sinf(d);
  float dir_e = sinf(e);
  */

  // calculate whats constant in this current frame

  // center_x = (SCREEN_WIDTH / 2) - 0.5;     // the polar origin
  // center_y = (SCREEN_HEIGHT / 2) - 0.5;
  xSemaphoreGive(loop2go);
  // the HOT LOOP where the magic happens
  ESP_LOGV(LOG, "third");
  doTheThing(0, SCREEN_WIDTH, 0, SCREEN_HEIGHT / 2);
  
  // make the frame appear nice to humans 
  // (correct for non-linear brightness sensitivity of the human eye)
  ESP_LOGV(LOG, "fourth");
  struct ColorSum colorSum = adjust_gamma(shared.currentBuffer, NUM_LEDS / 2);
  cumulativeColorSum.red += colorSum.red;
  cumulativeColorSum.green += colorSum.green;
  cumulativeColorSum.blue += colorSum.blue;
  
  // BRING IT ON! SHOW WHAT YOU GOT!
  // FastLED.show();
  xSemaphoreTake(loop1go, portMAX_DELAY);
  ESP_LOGV(LOG, "fifth");
  driver.showPixels((uint8_t*)shared.currentBuffer);
  // delay(9);

  // check serial monitor for current performance numbers
  // EVERY_N_MILLIS(500) report_performance();
  if (frameNumber % debugPeriod == 0)
  {
    newTime = micros();
    ESP_LOGI(LOG, "frameNumber / %d = %d, fps = %f", debugPeriod, frameNumber / debugPeriod, debugPeriod * 1000000.0 / (newTime - oldTime));
    oldTime = newTime;
    ESP_LOGI(LOG, "reds = %d", cumulativeColorSum.red / debugPeriod);
    ESP_LOGI(LOG, "greens = %d", cumulativeColorSum.green / debugPeriod);
    ESP_LOGI(LOG, "blues = %d", cumulativeColorSum.blue / debugPeriod);
    ESP_LOGI(LOG, "all = %d\n", (cumulativeColorSum.red + cumulativeColorSum.green + cumulativeColorSum.blue) / debugPeriod);
    cumulativeColorSum.red = 0;
    cumulativeColorSum.green = 0;
    cumulativeColorSum.blue = 0;
  }
  
  ESP_LOGV(LOG, "test %d", frameNumber);
  frameNumber++;
  // delay(199);
}


// calculate distance and angle of the point relative to
// the polar origin defined by center_x & center_y
/*
void get_polar_values() {

  // calculate current cartesian distances (deltas) from polar origin point

  float dx = x - center_x;
  float dy = y - center_y;

  // calculate distance between current point & polar origin
  // (length of the origin vector, pythgorean theroem)
  // dist = sqrt((dx*dx)+(dy*dy));

  dist = hypotf(dx, dy);

  // calculate the angle
  // (where around the polar origin is the current point?)

  angle = atan2f(dy, dx);

  // done, that's all we need
}
*/

// convert polar coordinates back to cartesian
// & render noise value there

uint8_t render_pixel(struct Parameters parameters) {

  // convert polar coordinates back to cartesian ones
  //fmodf(newangle, 2 * PI);  //NO, not here!

  float newx = (parameters.offset_x + parameters.center_x - (cosf(parameters.newangle) * parameters.distSqrt)) * parameters.scale_x;
  float newy = (parameters.offset_y + parameters.center_y - (sinf(parameters.newangle) * parameters.distSqrt)) * parameters.scale_y;

  // this was actually slower
  // float newx = (offset_x + center_x - (cos16((uint16_t)(newangle * 10430.21920) * newdist))) * scale_x * 0.00003051850948;
  // float newy = (offset_y + center_y - (sin16((uint16_t)(newangle * 10430.21920) * newdist))) * scale_y * 0.00003051850948;

  // render noisevalue at this new cartesian point

  uint16_t raw_noise_field_value = inoise16(newx, newy, parameters.z);
  // float raw_noise_field_value_float = snoise4(newx / 65535, newy / 65535, z / 65535, 0.0);
  // double raw_noise_field_value_float = (OpenSimplex2F_noise3_Classic(ctx, newx, newy, z));
  // uint16_t raw_noise_field_value = OpenSimplex2F_noise3_Classic(ctx, newx, newy, z) * 32767 + 32767;
  // uint16_t raw_noise_field_value = 35000;

  // a lot is happening here, namely
  // A) enhance histogram (improve contrast) by setting the black and white point
  // B) scale the result to a 0-255 range
  // it's the contrast boosting & the "colormapping" (technically brightness mapping)
  
  if (raw_noise_field_value < low_limit)  raw_noise_field_value =  low_limit;
  if (raw_noise_field_value > high_limit) raw_noise_field_value = high_limit;

  uint8_t scaled_noise_value = map(raw_noise_field_value, low_limit, high_limit, 0, 255);
  
  /*
  if (raw_noise_field_value_float < 0)  raw_noise_field_value_float =  0;
  if (raw_noise_field_value_float > 0.8) raw_noise_field_value_float = 0.8;

  uint8_t scaled_noise_value = (raw_noise_field_value_float * 1.25) * 255;
  */

  return scaled_noise_value;

  // done, we've just rendered one color value for one single pixel
}

float sqrtApprox(float number)
{
  union { float f; uint32_t u; } y = {number};
  y.u = 0x5F1FFFF9ul - (y.u >> 1);
  return number * 0.703952253f * y.f * (2.38924456f - number * y.f * y.f);
}


// find the right led index
int mapToSnake(int x, int y) {
  if (y & 1)
    return (y + 1) * SCREEN_WIDTH - 1 - x;
  else
    return y * SCREEN_WIDTH + x;
}

// make it look nicer - expand low brightness values and compress high brightness values,
// basically we perform gamma curve bending for all 3 color chanels,
// making more detail visible which otherwise tends to get lost in brightness
struct ColorSum adjust_gamma(CRGB *leds, int numLeds)
{
  ColorSum colorSum;
  for (uint16_t i = 0; i < numLeds; i++)
  {
    leds[i].r = dim8_video(leds[i].r) / 1;
    colorSum.red += leds[i].r;
    leds[i].g = dim8_video(leds[i].g) / 1;
    colorSum.green += leds[i].g;
    leds[i].b = dim8_video(leds[i].b) / 1;
    colorSum.blue += leds[i].b;
  }
  return colorSum;
}

// given a static polar origin we can precalculate 
// all the (expensive) polar coordinates

void generate_lookup_table() {

  float center_x = (SCREEN_WIDTH / 2) - 0.5;     
  float center_y = (SCREEN_HEIGHT / 2) - 0.5;

  for (int xx = 0; xx < SCREEN_WIDTH; xx++) {
    for (int yy = 0; yy < SCREEN_HEIGHT; yy++) {

        float dx = xx - center_x;
        float dy = yy - center_y;

      distanceSqrt[xx] [yy] = sqrt(hypotf(dx, dy));
      theta[xx] [yy]    = atan2f(dy, dx);
    }
  }
}

// show current framerate and rendered pixels per second
/*
void report_performance() {
 
  int fps = FastLED.getFPS();
  int kpps = (fps * HEIGHT * WIDTH) / 1000;

  Serial.print(kpps); Serial.print(" Kilopixel/s @ ");
  Serial.print(fps); Serial.println(" fps");
  
}
*/
