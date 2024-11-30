/*Using LVGL with Arduino requires some extra steps:
 *Be sure to read the docs here: https://docs.lvgl.io/master/integration/framework/arduino.html  */


/*
IMPORTANT INFORMATION FOR THIS EXAMPLE
This example uses an 480x320 ILI9488 display with touch for an lvgl based UI (lvgl widgets demo). 
The display is controller by abd ESPE 32 Devkit4 board (set in Arduino IDE as a generic ESP32 Dev Module)

This example is based no the LVGL_Arduino example which comes with the lvgl installation
Parts of the original sample have been removed or updated

lvgl version: 9.2.2

TFT_eSPI by Bodmer version 2.5.43
- this library is used to drive the diplay and the touch screen and is used by lvgl.

ESP module:
ESP32 Devkit v.4

Resolution: 480x320
Display controller: ILI9488
Touch controller: XPT2046 (no specific configuration other that the touch CS pin had to be done)

=== lvgl configuration ===

== lv_conf.h ==
after a clean installation of the library in Arduino IDE 
lv_conf_template.h must be copied as lv_conf.h in the root of the Arduino libraries folder

Enable its content right at the beginning
#if 1 //Set it to "1" to enable content

Enable the use of the integrated TFT_eSPI support
#define LV_USE_TFT_ESPI         1

Enable the use of the demo_widgets. This includes all the files needed to build the widget demo application.
#define LV_USE_DEMO_WIDGETS 1


=== TFT_eSPI configuration ===

In the Arduino libraries folder modify the following files:
== User_Setup.h ==
- uncomment #define ILI9488_DRIVER
- uncomment and modify if needed the lines for ESP board. For this example ESP32 Devkit v.4 was used and the lines for ESP32 Dev board where uncommented

#define TFT_MISO 19
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS   15  // Chip select control pin
#define TFT_DC    2  // Data Command control pin
#define TFT_RST   4  // Reset pin (could connect to RST pin)

also uncomment and update if needed the line 
#define TOUCH_CS 21     // Chip select pin (T_CS) of touch screen

== User_Setup_Select.h ==
uncomment:
#include <User_Setups/Setup21_ILI9488.h>

== Setup21_ILI9488.h ==
in this corresponding config update the pin configuration 
in this example the following are used
#define TFT_MISO 19 // (leave TFT SDO disconnected if other SPI devices share MISO)
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS    15  // Chip select control pin
#define TFT_DC    2  // Data Command control pin
#define TFT_RST   4  // Reset pin (could connect to RST pin)

*/

#include "FS.h"

#include <lvgl.h>

#if LV_USE_TFT_ESPI
#include <TFT_eSPI.h>
#endif

#define CALIBRATION_FILE "/calibrationData"

#include <demos/lv_demos.h>


///!!!!!!!!!!!!!
//There is something happening in the lvgl / lv_tft_espi implementation which 
//makes this rotation needed. As if the screen size is defined in portrait mode and the rotated.
//this has implications also on the touch screen input interpretation. The coordinates as not 
//automatically rotated, despite the fact the the touch is read though the integrated TFT driver.
//The rotation is done in the touch screen reading callback function.

#define TFT_HOR_RES   320
#define TFT_VER_RES   480
#define TFT_ROTATION  LV_DISPLAY_ROTATION_90


/*LVGL draw into this buffer, 1/10 screen size usually works well. The size is in bytes*/
#define DRAW_BUF_SIZE (TFT_HOR_RES * TFT_VER_RES / 10 * (LV_COLOR_DEPTH / 8))
uint32_t draw_buf[DRAW_BUF_SIZE / 4];

//This gets the same instace that is used by the display. It is needed in order to access the touch 
//part of the driver
TFT_eSPI tft = TFT_eSPI(TFT_VER_RES, TFT_HOR_RES); /* TFT instance */



#if LV_USE_LOG != 0
void my_print( lv_log_level_t level, const char * buf )
{
    LV_UNUSED(level);
    Serial.println(buf);
    Serial.flush();
}
#endif

void setup()
{
    String LVGL_Arduino = "Hello Arduino! ";
    LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

    Serial.begin( 115200 );
    Serial.println( LVGL_Arduino );

    lv_init();

    /*Set a tick source so that LVGL will know how much time elapsed. */
    lv_tick_set_cb(my_tick);

    /* register print function for debugging */
#if LV_USE_LOG != 0
    lv_log_register_print_cb( my_print );
#endif


lv_display_t * disp;  

//We use in this example LV_USE_TFT_ESPI so it should be set
#if LV_USE_TFT_ESPI
    /*TFT_eSPI must be enabled lv_conf.h to initialize the display in a simple way*/
    disp = lv_tft_espi_create(TFT_HOR_RES, TFT_VER_RES, draw_buf, sizeof(draw_buf));
    lv_display_set_rotation(disp, TFT_ROTATION);

#else
    /*Else create a display yourself*/
    //NOT THE CASE
    //for this example we set LV_USE_TFT_ESPI lv_conf.h so we should not get here
    disp = lv_display_create(TFT_HOR_RES, TFT_VER_RES);
    lv_display_set_flush_cb(disp, my_disp_flush);
    lv_display_set_buffers(disp, draw_buf, NULL, sizeof(draw_buf), LV_DISPLAY_RENDER_MODE_PARTIAL);
#endif

    /*Initialize the (dummy) input device driver*/
    lv_indev_t * indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER); /*Touchpad should have POINTER type*/
    lv_indev_set_read_cb(indev, my_touchpad_read);

//
//
// TOUCH SCREEN CALIBRATION
//
//
Serial.println("Performing calibration...");

tft.setRotation(TFT_ROTATION);
uint16_t calibrationData[5];
uint8_t calDataOK = 0;

/*
  //Perform calibration on every reset for test purposes
  
  tft.calibrateTouch(calibrationData, TFT_WHITE, TFT_RED, 15);
  tft.setTouch(calibrationData);
*/

/*
  Or make the calibration once and store it in the file system.
  Set erase all flash before sketch upload to delete the file an retrigger an calibration
*/
// check file system
  if (!SPIFFS.begin()) {
    Serial.println("formatting file system");
    SPIFFS.format();
    SPIFFS.begin();
  }
  
  // check if calibration file exists
  Serial.println("Checking if calibration file exists...");
  if (SPIFFS.exists(CALIBRATION_FILE)) {
    File f = SPIFFS.open(CALIBRATION_FILE, "r");
    if (f) {
      
      if (f.readBytes((char *)calibrationData, 14) == 14){
        calDataOK = 1;
        Serial.println("Calibration file read. Perform a full flash erase if you need to delete the file and retrigger a calibration.");
      } 
      f.close();
    }
  }

  if (calDataOK) {
    Serial.println("Set touch using the existing calibration.");
    // calibration data valid
    tft.setTouch(calibrationData);
  } else {
    // data not valid. recalibrate
    Serial.println("No calibration data available. Performing touch calibration.");
    tft.calibrateTouch(calibrationData, TFT_WHITE, TFT_RED, 15);
    // store data
    File f = SPIFFS.open(CALIBRATION_FILE, "w");
    if (f) {
      f.write((const unsigned char *)calibrationData, 14);
      f.close();
    }
  }
     
    /*
    //Basic print using lvgl
    lv_obj_t *label = lv_label_create( lv_screen_active() );
    lv_label_set_text( label, "Hello Arduino, I'm LVGL!" );
    lv_obj_align( label, LV_ALIGN_CENTER, 0, 0 );
    */

    //create all the controls for the widgets demo
    lv_demo_widgets();

    Serial.println( "Setup done" );
}



/* NO USE FOR THIS FUNCTION
if LV_USE_TFT_ESPI is not set, LVGL calls it when a rendered image needs to copied to the display*/
void my_disp_flush( lv_display_t *disp, const lv_area_t *area, uint8_t * px_map)
{
  //LV_USE_TFT_ESPI in lv_conf.h should be set so that lvgl handles the flush with its own driver for this display.
  //In this case we should not get here
  //see lvgl_Arduino original example for details
  Serial.println("my_disp_flush");

}

/*Read the touchpad*/
void my_touchpad_read( lv_indev_t * indev, lv_indev_data_t * data )
{
  //Serial.println("Touchpad read!");

  uint16_t touchX, touchY, mappedX, mappedY;

  bool touched = tft.getTouch( &touchX, &touchY, 600 );
  
  if( !touched )
  {
      data->state = LV_INDEV_STATE_REL;
  }
  else
  {
      data->state = LV_INDEV_STATE_PR;
      //
      //data->point.x = touchX;
      //data->point.y = touchY;
      //
      //The touch is not properly rotated by lvgl therefore we have to make this manualy
      
      mappedX = map(touchX, 0, 480, 480, 0);
      mappedY = touchY;

      //tft.setCursor(touchX, touchY, 1);
      //tft.printf("tx:%i mx:%i ty:%i my:%i", touchX, mappedX, touchY, mappedY);
      
      //Transmit lvgl the scaled and rotated coordinates
      data->point.x = mappedY;
      data->point.y = mappedX;

      //Serial.println( "touchX:" + String(touchX)  + "   mappedX: " + String(mappedX));
      //Serial.println( "touchY:" + String(touchY)  + "   mappedY: " + String(mappedY));
  }
  
}

/*use Arduinos millis() as tick source*/
static uint32_t my_tick(void)
{
    return millis();
}

void loop()
{
    lv_timer_handler(); /* let the GUI do its work */
    delay(5); /* let this time pass */
/*
  uint16_t x, y;
  static uint16_t color;

  if (tft.getTouch(&x, &y)) {

    tft.setCursor(5, 5, 2);
    tft.printf("x: %i     ", x);
    tft.setCursor(5, 20, 2);
    tft.printf("y: %i    ", y);

    tft.drawPixel(x, y, color);
    color += 155;
  }*/
}
