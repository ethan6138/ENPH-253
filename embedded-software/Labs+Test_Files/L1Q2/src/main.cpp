#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define I2C_SDA 15
#define I2C_SCL 16
#define inputSwitch 1
#define pin48 48

volatile int state{ 1 }; 

bool success = Wire.begin(I2C_SDA, I2C_SCL); // sets the I2C pins to the specified values

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void OledSetup(){
  display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Displays Adafruit logo by default. call clearDisplay immediately if you don't want this.
  // display_handler.display();
  // delay(2000);
 
  display_handler.clearDisplay();

  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);
  display_handler.setCursor(0,0); // set the cursor start location
  display_handler.println("Hello world");// Displays "Hello world!" on the screen
  display_handler.display();
}

void IRAM_ATTR convcallback(){ 
  state = digitalRead(inputSwitch);
}

void setup() {
  pinMode(inputSwitch, INPUT_PULLUP);
  pinMode(pin48, OUTPUT); 

  attachInterrupt(inputSwitch, convcallback, CHANGE); 
  digitalWrite(pin48, HIGH); 

  OledSetup(); 
}

void loop() {
  if(!state){ 
    display_handler.clearDisplay();
    display_handler.setTextSize(1);
    display_handler.setTextColor(SSD1306_WHITE);
    display_handler.setCursor(0,0); // set the cursor start location
    display_handler.println("PIN LOW");
    display_handler.display();
  }
  else{ 
  display_handler.clearDisplay();
  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);
  display_handler.setCursor(0,0); // set the cursor start location
  display_handler.println("PIN HIGH");
  display_handler.display(); 
  }
}