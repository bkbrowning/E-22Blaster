// 
// E22Blaster
//
// Electronics controller code for an Arduino Nano Every based sound/light effects setup for a Star Wars
// E-22 Blaster.  This blaster was utilized by the Shoretroopers in Rogue One/Andor
//
// Electronic features include the following:
//   - Sound output via Speaker driven by a DFPlayerMini board
//   - Buttons for (1) trigger; (2) Hengstler counter; (3) selection lever
//   - LEDs for (1) Barrel; (2) Rear upper ports; (3) Underneath m-plate; (4) Hengstler tri-port
//   - Small OLED displays for (1) Hengstler counter for ammo remaining; (2) Scope display
//
// Versions
// 1.0 - initial working version - Brent Browning - 25-AUG-2024
// 1.1 - revised counter graphics function - Brent Browning - 03-SEP-2024

//
// INCLUDES
//
#include <Adafruit_NeoPixel.h>  // controlling variable LED strips
#include <Arduino.h> // main Arduino board library
#include <DFPlayerMini_Fast.h>  // DFPlayer Mini library that seems to work with all of the DFPlayer Minis (NOTE: be sure files on SD card are located and named to spec!)
#include <U8g2lib.h> // monochrome OLED library
#include <SoftwareSerial.h>  // library to utilize alternate pins on the Nano board as Serial RX and TX
#include <Wire.h>  // I2C library for OLED signals

//
// PIN DEFINITIONS - on the Arduino Nano Every board
//
// TX - default pins so no #define needed: TX pin
// RX - default pins so no #define needed: RX pin
#define COUNTER_BUTTON 3  // D3 - Hengstler Counter Button
//5 D5 - Body LEDs behind M Plate - Not used for now
//6 D6 - Hengstler Counter LEDs - Not used for now
//7 D7 - Selector button - Not used for now
#define TRIGGER_BUTTON 8  // D8 - Trigger Button
#define COUNTER_OLED_SCL 9   // D9 - SCL for Hengstler Counter OLED
#define COUNTER_OLED_SDA 10  // D10 - SDA for Hengstler Counter OLED
#define PORT_WINDOW_LEDS 11  // D11 - Port Window LEDs signal wire
#define MUZZLE_LED 12  // D12 - Muzzle LED signal wire
// A4/D18 - default pins so no #define needed: A4/D18 - SDA for Scope Screen
// A5/D19 - default pins so no #define needed: A5/D19 - SCL for Scope Screen
#define NUM_PORT_WINDOW_LEDS 6  // Number of LEDs that will be lit in the two port windows == 2 * 6 == 6
#define NUM_MUZZLE_LEDS 1

//
// CONSTRUCTORS FOR HARDWARE
//
// Configure constructor for 128x32 Counter display
U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C u8g2(U8G2_R1, COUNTER_OLED_SCL, COUNTER_OLED_SDA, /* reset=*/ U8X8_PIN_NONE);   // First argument sets screen rotation, my installation orientation needs 90 degrees
// Configure constructor for 64x32 Scope display
U8G2_SSD1306_64X32_1F_1_HW_I2C u8g3(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);  //  This is on the default pins A4 & A5 for native Arduino Nano Every SDA/SCL
// Configure constructors for LED strips
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_PORT_WINDOW_LEDS, PORT_WINDOW_LEDS, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2 = Adafruit_NeoPixel(NUM_MUZZLE_LEDS, MUZZLE_LED, NEO_GRB + NEO_KHZ800);
// Configure constructors for DFPlayerMini
DFPlayerMini_Fast myMP3;
SoftwareSerial mySerial(2, 1); // RX, TX Serial ports assinged on Nano for DFPlayer signals

//
// GLOBAL VARIABLES
//
int count1 = 1;
int counterMode = 1;  // denotes the current mode for the Counter Button where 1 == kill mode and 2 == stun mode
int lastButtonState1 = 1;
int lastButtonState2 = 1;
long unsigned int lastPress1;
long unsigned int lastPress2;
volatile int buttonFlag1;
volatile int buttonFlag2;
int debounceTime = 20;
int maximumAmmo = 30;  // the size of the "ammo clip" for max shots fired before a reload is required
int remainingAmmo = maximumAmmo; // the current remaining ammo count
bool pinLEDHIGH = false;

//
// SETUP
//
void setup() {
  // Initialize pin modes and pin actions
  pinMode(COUNTER_BUTTON, INPUT_PULLUP);
  pinMode(TRIGGER_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TRIGGER_BUTTON), ISR_button1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(COUNTER_BUTTON), ISR_button2, CHANGE);

  // Initialize Serial output for monitor
  Serial.begin(9600); // dont forget to set your serial monitor speed to whatever is set here
  Serial.println("Initializing E-22 Blaster...");

  // Initialize LED Strips
  strip.begin();
  strip.show();
  strip2.begin();
  strip2.show();
  
  // Initialize OLED displays
  u8g2.begin();
  u8g3.begin();

  // Initialize DFPlayerMini
  mySerial.begin(9600);
  myMP3.begin(mySerial, true);
  Serial.println("...setting initial volume on DFPlayerMini...");
  myMP3.volume(20); // Set volume 0 to 30 (max is 30)

  // Initial drawing pass on displays
  // Ammo Count Boxes
  drawCounterGraphic(maximumAmmo, remainingAmmo);

  //Play initial startup sound
  Serial.println("...playing startup sound...");
  myMP3.playFolder(01, 3);
  Serial.println("...initialization complete!");
}

//
// LOOP
//
void loop() {
  //Draw Scope Images
  u8g3.firstPage(); 
  do {
    // draw the reticle
    u8g3.drawCircle(30, 15, 15);//Outer Ring
    u8g3.drawCircle(30, 15, 2);//Inner ring
    u8g3.drawLine(18, 15, 13, 15);//left hash
    u8g3.drawLine(42, 15, 47, 15);//right hash
    u8g3.drawLine(30, 30, 30, 25);//bottom hash
    u8g3.drawLine(30, 1, 30, 5);//top hash

    // set the default font for the text in the Scope Display
    u8g3.setFont(u8g2_font_tom_thumb_4x6_tf);

    // draw the upper right text
    u8g3.setCursor (45, 6);
    u8g3.print("BB-23");
    // draw lower right text
    u8g3.setCursor(52, 26);
    u8g3.print("RNG");
    // draw the range indicator for the 'meters'
    u8g3.setCursor(52, 32);
    u8g3.print(random(70,90));
    u8g3.print("m");

    // draw the randomly sized top left bars
    u8g3.drawLine(1, 1, random(1,12), 1); //bar 1
    u8g3.drawLine(1, 3, random(1,12), 3); //bar 2
    u8g3.drawLine(1, 5, random(1,12), 5); //bar 3
    u8g3.drawLine(1, 7, random(1,12), 7); //bar 4
    delay(50);
  } while ( u8g3.nextPage() );

  // Trigger button check loop
  if((millis() - lastPress1) > debounceTime && buttonFlag1) {
    lastPress1 = millis();   //update lastPress                                                     
    if(digitalRead(TRIGGER_BUTTON) == 0 && lastButtonState1 == 1) {   //if button is pressed and was released last change
      count1 = count1 + 1;
      //Blast - subroutine to play blast sound and light
      if (counterMode == 1 && remainingAmmo != 0) {
        colorWipe(0xff,0x00,0x00, 2);
        colorWipe(0x00,0x00,0x00, 2); 
        myMP3.playFolder(01, 1);
      }
      //Stun - Subroutine if in stun mode
      if (counterMode == 2 & remainingAmmo != 0) {
        colorWipe(0x00,0x00,0xff, 2);
        colorWipe(0x00,0x00,0x00, 2); 
        myMP3.playFolder(01, 2);
      }
      //Empty - Subroutine if empty; Plays empty sound until mode button is pressed for reload
      if (remainingAmmo == 0) {
        myMP3.playFolder(01, 5);
      }
      // Ammo Counter updates
      if (remainingAmmo > 0) {
        remainingAmmo = remainingAmmo - 1;
        drawCounterGraphic(maximumAmmo, remainingAmmo);
      }

      //lastButtonState1 = 0;    //record the lastButtonState
    }
    
    else if(digitalRead(TRIGGER_BUTTON) == 1 && lastButtonState1 == 0) {   // if button is not pressed, and was pressed last change
      lastButtonState1 = 1;    //record the lastButtonState
    }
    buttonFlag1 = 0;
  }

  // Counter Button check loop - checks if Counter Button has been pressed and either changes mode OR resets counter
  if((millis() - lastPress2) > debounceTime && buttonFlag2) {
    lastPress2 = millis();   //update lastPress                                                     
    if(digitalRead(COUNTER_BUTTON) == 0 && lastButtonState2 == 1) {    // if button is pressed and was released last change
      // counterMode is used to determine mode 1=kill, 2=stun
      counterMode = counterMode + 1;
      if (counterMode == 3) {
        counterMode = 1;
      }
      //Play sound for mode change
      if (remainingAmmo != 0) {
        myMP3.playFolder(01, 4);
      }
      //Reload Sequence
      if (remainingAmmo == 0) {
        myMP3.playFolder(01, 3);
        remainingAmmo = maximumAmmo;
        drawCounterGraphic(maximumAmmo, remainingAmmo);
        //Decreases counterMode by 1 to prevent mode change on reload
        counterMode = counterMode - 1;
        delay(1000);
      }
      
      //lastButtonState2 = 0;    // record the lastButtonState
    }
    
    else if(digitalRead(COUNTER_BUTTON) == 1 && lastButtonState2 == 0) {    //if button is not pressed, and was pressed last change
      lastButtonState2 = 1;    // record the lastButtonState
    }
    buttonFlag2 = 0;
  }
}

//
// FUNCTIONS
//
void ISR_button1() {
  buttonFlag1 = 1;
}

void ISR_button2() {
  buttonFlag2 = 1;
}


int SpeedDelay = 0.5;

void colorWipe(byte red, byte green, byte blue, int SpeedDelay) {
  for(uint16_t i=0; i<NUM_PORT_WINDOW_LEDS; i++) {
    setPixel(i, red, green, blue);
    setPixel2(0, red, green, blue);
    showStrip();
    showFrontLED();
    delay(SpeedDelay);
  }
}

void showStrip() {
 #ifdef ADAFRUIT_NEOPIXEL_H
   // NeoPixel
   strip.show();
 #endif
 #ifndef ADAFRUIT_NEOPIXEL_H
   // FastLED
   FastLED.show();
 #endif
}

void showFrontLED() {
 #ifdef ADAFRUIT_NEOPIXEL_H
   // NeoPixel
   strip2.show();
 #endif
 #ifndef ADAFRUIT_NEOPIXEL_H
   // FastLED
   FastLED.show();
 #endif
}

void setPixel(int Pixel, byte red, byte green, byte blue) {
 #ifdef ADAFRUIT_NEOPIXEL_H
   // NeoPixel
   strip.setPixelColor(Pixel, strip.Color(red, green, blue));
 #endif
 #ifndef ADAFRUIT_NEOPIXEL_H
   // FastLED
   leds[Pixel].r = red;
   leds[Pixel].g = green;
   leds[Pixel].b = blue;
 #endif
}

void setPixel2(int Pixel, byte red, byte green, byte blue) {
 #ifdef ADAFRUIT_NEOPIXEL_H
   // NeoPixel
   strip2.setPixelColor(Pixel, strip.Color(red, green, blue));
 #endif
 #ifndef ADAFRUIT_NEOPIXEL_H
   // FastLED
   leds[Pixel].r = red;
   leds[Pixel].g = green;
   leds[Pixel].b = blue;
 #endif
}

void drawCounterGraphic(int maxAmmo, int currentAmmo) {
  int max_row_count = 10;
  int max_box_width = 28;
  int max_box_height = 93;
  int max_column_count = 5;
  int box_width_padding = 2;
  int box_height_padding = 2;
  int box_height = max_box_height;
  int box_width = max_box_width;
  int num_of_rows = 1;
  int init_x_pos = 2;
  int init_y_pos = 34;
  // counters
  int current_column = 0;
  int i = 0;

  if (maxAmmo > max_row_count) {
    box_width = (maxAmmo - ((maxAmmo/max_row_count)*box_width_padding)) / (maxAmmo / max_row_count);
    box_height = (max_box_height - ((max_row_count - 1) * box_height_padding)) / max_row_count;
    num_of_rows = max_row_count;
  } else {
    box_width = max_box_width;
    box_height = (max_box_height - ((maxAmmo - 1)* box_height_padding)) / maxAmmo;
    num_of_rows = maxAmmo;
  }

  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_inb19_mn);

  while(i <= maxAmmo) {
    for (int j = 0; j < num_of_rows; j++){
      if (i >= currentAmmo) {
        u8g2.drawFrame(init_x_pos + (current_column * (box_width+box_width_padding)), init_y_pos + (j * (box_height+box_height_padding)), box_width, box_height);      
      } else {
        u8g2.drawBox(init_x_pos + (current_column * (box_width+box_width_padding)), init_y_pos + (j * (box_height+box_height_padding)), box_width, box_height);
      }
      i++;
    }
    current_column++;
  }

  u8g2.setCursor(2,2);
  u8g2.print(currentAmmo);
  u8g2.sendBuffer();

}