/*Team Id: eYRC#HB#1476
 * Author List: Mohit.K,Puran.Y
 * Filename: Bot_1esp
 * Theme: Hologlyph Bot
 * Functions: error_loop,servo_init,servo_control,pen_down,flag,subscription_callback,pen_callback,flag_callback,color_callback,setup,meteorRain,setPixel,setAll,fadeToBlack,loop
 * Global Variables: None*/

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/color_rgba.h>
#include <ESP32Servo.h>
#define FASTLED_FORCE_SOFTWARE_SPI
#include <FastLED.h>

rcl_subscription_t cmd_vel_sub;
geometry_msgs__msg__Twist msg;

rcl_subscription_t pen_sub;
std_msgs__msg__Bool bool_msg;

rcl_subscription_t color_sub;
std_msgs__msg__ColorRGBA color_msg;

rcl_subscription_t flag_sub;
std_msgs__msg__Bool flag_bool_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
//Servo_object initialisation
Servo s1; 
Servo s2; 
Servo s3;
Servo s4;
//servo pins
#define S1 33
#define S2 25
#define S3 26
#define S4 27
//led intislisation
#define LED_PIN     2
#define LED_DATA    2 
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
#define NUM_LEDS    36
CRGB leds[NUM_LEDS];
#define BRIGHTNESS  96

/*
 * Function Name:error_loop
 * Input: none
 * Output: none
 * Logic: if ros agent is disconnected led will turn to red
 * Example Call: error_loop()*/

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    meteorRain(0xff,0x00,0x00,10, 32, true, 30);
  }
}

/*
 *Function Name: servo_init
 *Input: None
 *Output: None
 *Logic: Attaches the servo objects to their respective pins and sets PWM range
 *Example Call: servo_init() */

void servo_init(){
    s1.attach(S1,1000,2000);
    s2.attach(S2,970,2045);
    s3.attach(S3,1000,2000);
    s4.attach(S4);
}
/*
 *Function Name: servo_control
 *Input: w1, w2, w3 - angular velocity values for the 3 servos
 *Output: None
 *Logic: Maps the input velocity values to appropriate servo angles based on constraints. Writes the angles to the servos.
 *Example Call: servo_control(0.5, 1.0, -0.8)
*/

void servo_control(float w1 , float w2, float w3){
// Mapping and constraining of velocity values to servo angles
    float StopLimMax = 2.0;
    int s1_speed = (abs(w1)>=0 && abs(w1) <=StopLimMax) ? 89:(w1 < 0) ? map(abs(w1),0,255,100,180) : map(w1,0,255,80,0);
    int s2_speed = (abs(w2)>=0 && abs(w2) <=StopLimMax) ? 90 :(w2 < 0) ?map(abs(w2),0,255,100,180) : map(w2,0,255,80,0);
    int s3_speed = (abs(w1)>=0 && abs(w3) <=StopLimMax) ? 90 :(w3 < 0) ?map(abs(w3),0,255,102,180) : map(w3,0,255,80,0);
 // Writing angles to servos
    s1.write(s1_speed);
    s2.write(s2_speed);
    s3.write(s3_speed);
    Serial.print(s1_speed);Serial.print("|");Serial.print(s2_speed);Serial.print("|");Serial.print(s3_speed);Serial.print(w1);Serial.print(w2);Serial.println(w3);
    
}
/*
 *Function Name: pen_down
 *Input: is_pen_down - boolean indicating whether pen should be lowered
 *Output: None
 *Logic: Checks is_pen_down value, writes servo angle accordingly to lower or raise pen
  *Example Call: pen_down(true) */
void pen_down( bool is_pen_down){
// Lower pen
    if (is_pen_down == true ){
      s4.write(0);
      Serial.println("Pendown");
    }
    // Raise pen    
    else {
      s4.write(50);
      Serial.println("PenUp");
    }
}
/*
 *Function Name: flag
 *Input: stop_flag - boolean flag
 *Output: None
 *Logic: If stop flag is true, sets servos to stop position
 *Example Call: flag(true) */

void flag( bool stop_flag){

    if (stop_flag == true ){
      s1.write(90);
      s2.write(90);
      s3.write(90);
      Serial.println("Stopping the bot");
    }
}

/*
 *Function Name: subscription_callback
 *Input: msgin - message object received
 *Output: None
 *Logic: Callback function that extracts data from received message and passes to servo_control()
 *Example Call: Called automatically when message received */

void subscription_callback(const void * msgin)
{  
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  float linear_x = msg -> linear.x;
  float linear_y = msg -> linear.y; 
  float angular_z = msg -> angular.z;

  servo_control(linear_x,linear_y,angular_z);
  
}

/*
 *Function Name: pen_callback
 *Input: bool_rec - received boolean message
 *Output: None
 *Logic: Callback function that extracts boolean from message and passes to pen_down()
 *Example Call: Called automatically when message received */

void pen_callback (const void * bool_rec)
{
  const std_msgs__msg__Bool * pen_msg = (const std_msgs__msg__Bool *)bool_rec;
  bool is_pen_down = pen_msg -> data; 
  pen_down(is_pen_down);
  Serial.println("ReceivingPenpose");
}

/*
 *Function Name: flag_callback
 *Input: bool_rec - received boolean message
 *Output: None
 *Logic: Callback to extract boolean and pass to flag() function
 *Example Call: Called automatically on message received */

  void flag_callback (const void * bool_rec)
  {
    const std_msgs__msg__Bool * flag_sub = (const std_msgs__msg__Bool *)bool_rec;
    bool stop_flag = flag_sub -> data; 
    flag(stop_flag);
    Serial.println("FlagReceived");
  }

  /*
 *Function Name: color_callback
 *Input: color_rec - received color message
 *Output: None
 *Logic: Callback function that extracts color values and prints them
 *Example Call: Called automatically when message received */

void color_callback(const void *color_rec)
{
  const std_msgs__msg__ColorRGBA *color_msg = (const std_msgs__msg__ColorRGBA *)color_rec;
  int color_r = color_msg->r;
  int color_g = color_msg->g;
  int color_b = color_msg->b;

  // Calculate buffer size needed for formatted string
  int buffer_size = snprintf(NULL, 0, "R %.3f|G %.3f|B %.3f", color_r, color_g, color_b) + 1;

  // Allocate buffer with calculated size
  char formattedString[buffer_size];

  // Format the string
  sprintf(formattedString, "R %.3f|G %.3f|B %.3f", color_r, color_g, color_b);

  // Print the formatted string
  Serial.println(formattedString);
}

/*
 *Function Name: setup
 *Input: None
 *Output: None
 *Logic: Initialize LEDs, servos, ROS node and subscribers
 *Example Call: Called once at start */

void setup() {

  FastLED.addLeds<LED_TYPE,LED_DATA,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);

  servo_init();
  set_microros_wifi_transports("Puju 2.5","puju0402","192.168.0.181",7777);
  Serial.begin(115200);

  meteorRain(0x00,0x00,0xff,10, 32, true, 30);

  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "Sub_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &pen_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),"/pen1_down"));

  RCCHECK(rclc_subscription_init_default(
    &color_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, ColorRGBA),"/color1"));

  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),"/cmd_vel/bot1"));

  RCCHECK(rclc_subscription_init_default(
    &flag_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),"/flag1")); //Topic to be subscribed

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &pen_sub, &bool_msg, &pen_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &color_sub, &color_msg, &color_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &flag_sub, &flag_bool_msg, &flag_callback, ON_NEW_DATA));
}

/*
 *Function Name: meteorRain
 *Input: color values, size, decay etc
 *Output: None
 *Logic: Animates meteor rain effect on LED strip
 *Example Call: meteorRain(0x00, 0x00, 0xFF, 10, 32, true, 30) */

void meteorRain(byte red, byte green, byte blue, byte meteorSize, byte meteorTrailDecay, boolean meteorRandomDecay, int SpeedDelay) {  
  setAll(0,0,0);
 
  for(int i = 0; i < NUM_LEDS+NUM_LEDS; i++) {
   
   
    // fade brightness all LEDs one step
    for(int j=0; j<NUM_LEDS; j++) {
      if( (!meteorRandomDecay) || (random(10)>5) ) {
        fadeToBlack(j, meteorTrailDecay );        
      }
    }
   
    // draw meteor
    for(int j = 0; j < meteorSize; j++) {
      if( ( i-j <NUM_LEDS) && (i-j>=0) ) {
        setPixel(i-j, red, green, blue);
      }
    }
   
    FastLED.show();
    delay(SpeedDelay);
  }
}

/*
 *Function Name: setPixel
 *Input: Pixel - LED number, red, green, blue - color values
 *Output: None
 *Logic: Sets the color for given LED pixel
 *Example Call: setPixel(10, 0x00, 0x00, 0xFF) - Sets LED 10 to blue */
  
void setPixel(int Pixel, byte red, byte green, byte blue) {
   leds[Pixel].r = red;
   leds[Pixel].g = green;
   leds[Pixel].b = blue;
}

/*
 *Function Name: setAll
 *Input: red, green, blue - color values
 *Output: None
 *Logic: Sets the color for all LEDs in the strip
 *Example Call: setAll(0xFF, 0x00, 0x00) - Sets all LEDs to red */

void setAll(byte red, byte green, byte blue) {
  for(int i = 0; i < NUM_LEDS; i++ ) {
    setPixel(i, red, green, blue);
  }
  FastLED.show();
}

/*
 *Function Name: fadeToBlack
 *Input: ledNo - LED number, fadeValue - fade amount
 *Output: None
 *Logic: Decreases the brightness of the given LED by the fade amount
 *to create a fade to black effect.
 *Example Call: fadeToBlack(10, 32) - Fades LED 10 by 32 steps. */

void fadeToBlack(int ledNo, byte fadeValue) {
  leds[ledNo].fadeToBlackBy(fadeValue);

}

/*
 *Function Name: loop
 *Input: None
 *Output: None
 *Logic: Runs executor to process subscriptions
 *Example Call: Called repeatedly after setup */

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

}
