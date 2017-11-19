// based on part of  http://www.esp8266.com/viewtopic.php?f=8&t=4865#
// 
// ver YY-MM-DD    stuff
   String my_ver = "16c" ;
//  16 17-11-14  last version had some sucess at balancing - trying out logging arrays. cleaning commented code
//               -might want to examine Brokkings code to make speeds linear - do a spreadsheet calculation
//               -wonder if our balance point is close to vertical (until battery shifts)
//               a) added debug logging to arrays, and display to console at 30 degree siesta
//               b) include control parameters at top of dump to help analysis
//               - need to change I-fade formula
//               -need to check the final flounder: wheels still turning after bot is laid down
//               c) red led tracks debug data storage, goes off when bot passes 30 degrees
//               -check balance point by hanging bot upside down by the wheels.
//  15 17-11-12  adapt to new A4988 motor drivers in setup and timer ISR
//               a) remove obsolete i2c_flag method of avoiding motor shield I2B bus interruptions
//               b) add spd macros, rework debug output
//               c) remove self_balance_checkpoint adjustment that forced pid_error_temp to grow
//               d) modify PID I component so error history fades out over time
//               e) change directions of commands to wheels
//               f) parameterized wheel speed controls in bot_slow and bot_fast variables
//               g) change wheel mode to DOUBLE to avoid some slipping. Nope - A4988 can't do that. So,
//                  I'll raise motor current, based on having 2 coils in series
//               h) new verticl constant - 1125 (battery moves around, so balance varies)
//  14 17-11-06  add code to display Wifi MAC address on startup
//  13 17-11-01  Make version number a string variable (see just above) that can be printed in setup so serial code is documented
//               a)Fix IMU read offset (0x43>45) and order of receiving variables in setup calibration loop

// found the following at https://gist.github.com/ah01/762576
// helper macro
//#define LINE(name,val) Serial.print(name); Serial.print("\t"); Serial.println(val);

//  12 17-10-25  adjust rotation correction to make calibration make sense
//               change 8200 value in accell calibration calculations to 8192 for accuracy
//   spl(WiFi.macAddress());  //andrew 60:01:94:1A:CA:31
//   find GPIO pins descriptions
//   design for 2nd cpu comms
//   how to get I2C bus speed at 400 KHz
//   try scope in I2C clock pulse
//   find a numchuk
//   clean up I2C signal
//   
//  11 17-10-15  remove the debugging stuff for interrupt length - micros() known to have issues in ISR's.
//               make consistent gyro config have FS_SEL=00, thus +/- 250 scale, multiplier = 131 
//               accumulation constant = 4 msec / 131 multiplier = .004 / 131 = .000030534
//               obsrvation: bot leaning forward generates negative angle_gyro, backwards gives positive angle
//               original motor wiring:  blue     green        modified wiring: blue     black <<<<
//                                       red      black                         red      green <<<<
//                                       green    red                      >>>> black    red
//                                       black    blue                     >>>> green    blue
//               Limit speed to wheel count >= 25 to reduce interrupt load that causes main loop to exceed 4 msec
//  10 17-10-14  use i2c_flag to prevent i2c calls at interrupt level (onestep) while we're doing
//                 i2c calls in the background (IMU reading). this didn't fix long interrupts & console delays
//               disable interrupts briefly to ensure parameters are updated simultaneously for both wheels
//               fix pin number for LED in calibration loop - should be zero
//               when speed=100, long onesteps are very rare, but do occur.
//                  Console millis correctly increment by about 1000. wheels go BACKWARDS
//               need to build stripped down code to see if basic onestep calls occasionally take a long time.
//  9  17-10-12  fix sign bit extension problems reading 16 bits over I2C into a 32 bit integer
//               add test speed handling, controlled by variable speed
//               correct main calculation for angle_gyro, using corrected constant .000061069
//               add wheel disable, controlled by wheel_disable, for testing without motor activity
//               add sp, spl and spc defines to simplify serial output
//               desync motors, so they don't call onestep from same interrupt. 
//                 -after above change, don't see 7 msec interrupts any more, just 3.5 msec ones
//  8  17-10-10  test IMU calibration, PID math...
//  7  17-10-07  start checking out IMU code
//  6  17-10-07  try to drive wheels via motor controller, using a sequential demo of "speeds"
//  5  17-10-07  make chunk vriables volatile to dodge compiler optimization, shorten chunk loop to 6 repeats
//               add code using micros() to time length of interrupt routine. POW takes 159 usec with F1 volatile,
//               but about 1 usec if optimised.
//  4  17-10-04  disable WIFI and compare performance measurements
//               -with WIFI disabled, current drsw is 34.9 milliamps while running
//               -with WIFI enabled,  current draw is 88.2 millampss when running, so disable does something
//               USB wiring detaails: http://www.hobbytronics.co.uk/usb-connector-pinout
//               However, graphing background chunk processing shows huge unexplained variations.
//               -removing DigitalWrite library call from ISR & Chunk code takes chunks/sec from 31,000 to 87,000.
//  3  17-10-04  add some bakground tasks, and measure impact of interrupt load on it
//  2  17-09-28  parameterized interval between timer interrupts with t0_count
//  1  17-09-27  stole code from web and played with timers

// ---------- parameters ----- put frequently changed parameters near top of file so they can be found easily
const int acc_calibration_value = -1125;                            //15 Enter the accelerometer calibration value. was -1200
                                                                    // andrew's constant = +535, doug's constant = -1356, no, -1125
const volatile int speed = -1;                                      // for initial testing of interrupt driven steppers
                                                                    //  speed = -1 enables IMU based balancing. 
                                                                    //  speed = n enables fixed forward speed interval of n, 0 is brakes on
const int bot_slow = 2300;                                          //15 # of interrupts between steps at slowest workable bot speed
const int bot_fast = 250;                                           //15 # of interrupts between steps at fastest workable bot speed
const float PID_I_fade = .80;                                         //15 how much of pid_i_mem history to retain

//Various PID settings
float pid_p_gain = 30;                                              //Gain setting for the P-controller (15)
float pid_i_gain = 1.2;                                             //Gain setting for the I-controller (1.5)
float pid_d_gain = 30;                                              //Gain setting for the D-controller (30)
float turning_speed = 30;                                           //Turning speed (20)
float max_target_speed = 150;                                       //Max target speed (100)

//  spreadsheet formula for speed
//  =IF(D8=0,0,ROUND($F$1 * (1000000/(D8*20))*3.1415926*4/200+25,1))
//  $F$1 is a scaling constant to make the graph comparable in size to other graphs
//  D8 is the step interval, in units of 20 usec
//  The 25 near the end is to offset the graphs so they don't overlap, particularly for 0

//  speed = [ steps per second ] * [ distance per step ]
//   = [10e6 usec / (step interval * step duration)] * [circumference / steps per rotation]
//   = [1,000,000 / ( D8 * 20 usec) ] * [ (PI * Diameter ) / 200 ]
//   = [ 50,000 / D8 ] * [ ( 3.15.1926 * 4 inches ) / 200 ]
//   = ( 50,000 * 3.1415926 * 4 ) / ( D8 * 200 )
//   = ( 1000 * 3.1415926 ) / D8
//   = 3141.5926 / D8  inches per second

// observed top speed = 300 for step interval > 10.47 inches per second
// observed slowest speed = 2300 step interval > 1.37 inches per second


const long usec_per_t0_int = 20;                                          // number of microseconds between t0 timer interrupts
const int pid_max = 400;                                            //11 upper bound for pid values
const int pid_min = -400;                                           //11 lower bound for pid values
const int debug_out1 = 1;                                           //11 non-zero to enable once a second serial debug info
const int debug_out2 = 0;                                           //11 non-zero to enable just once a second display of previous loop time in msec
   
#include <ESP8266WiFi.h>                                            // from https://forum.mysensors.org/topic/5120/esp8266-with-wifi-off/2
#include <Wire.h>                                                   // from Adafruit-stepper-dc-motor-featherwing.pdf, page 38

// -----------------preprocessor defines
//  define shortforms to make serial output easier
//   spl();  will do just a new line
#define sp Serial.print
#define spl Serial.println
#define spc Serial.print(", ")
#define spdl(label,val) sp(label); spl(val)
    // displays one labelled variable, ending with new-line
    // example: spdl(" param2= ",param2);     
#define spd(label,val) sp(label); sp(val)
    // suitable for displaying multiple variables per line
    // example: spd("left motor= ",left_motor); spdl("  right motor= ",right_motor);

//15 define the GPIO pins that perform functions on A4988 controller
#define pin_left_step 14                                            //15 GPIO 14 initiates a step on motor on robot's left side (top A4988)
#define pin_left_dir 12                                             //   GPIO 12 controls the direction of left motor
#define pin_right_step 13                                           //15 GPIO 13 initiates a step on motor on robot's right side (bottom A4988)
#define pin_right_dir 15                                            //   GPIO 15 controls the direction of right motor

//16 ------------------- define debug arrays -----------------------
// -idea is to be able to store telemetry data in real time without significantly impacting the performance we're trying to measure
// -a snapshot consists of data stored at a particular index in a group of arrays. One array will likely hold a timestamp of some sort
// -may wish to start and stop logging to these arrays under program control, to focus in on the right activity
// -may want to have generic array names with usage varying on what we're troubleshooting

#define log_size 2000                                               //16 make it easy to change length of debug arrays
unsigned int log_millis[log_size];                                  //16 timestamp, output of millis()
float log_angle[log_size];                                        // gyro angle, in degrees, -ve means leaning forward
float log_pid[log_size];                                            // value of pid_output_left, the final pid that's used
int log_motor[log_size];                                            // value of left_motor - actual step interval, as an interrupt count
float log_1[log_size];                                              //16 value of pid_i_mem for tracking PID_I_fade performance
//float log_2[log_size];
int lognum;                                                         // general index into debug logging arrays
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables and setting some values (YABR cut and paste)

byte start, received_byte=0, low_bat;
// received_byte came from serial stuff I've omitted - suspect it is numchuk controller

volatile int left_motor, throttle_left_motor, throttle_counter_left_motor, throttle_left_motor_memory;
volatile int right_motor, throttle_right_motor, throttle_counter_right_motor, throttle_right_motor_memory;
int battery_voltage;
int receive_counter;
int gyro_pitch_data_raw, gyro_yaw_data_raw, accelerometer_data_raw;

long gyro_yaw_calibration_value, gyro_pitch_calibration_value;

unsigned long loop_timer;

float angle_gyro, angle_acc, angle, self_balance_pid_setpoint;
float pid_error_temp, pid_i_mem, pid_setpoint, gyro_input, pid_output, pid_last_d_error;
float pid_output_left, pid_output_right;
float hold, hold1, hold2, hold3, hold4, hold5;

int gyro_address = 0x68;                                            //MPU-6050 I2C address (0x68 or 0x69)


volatile long t0_per_sec;

int vertical_calibration;                                           // this is calculated and displayed iff speed == 0, which puts the brakes on
int temp;                                                           // to investigate sign bit extension in calibration loop
volatile float ftemp;                                               //10 temp replacement calculation to replace onestep
long mics, loop_mics, last_millis;                                  //11 measure length of loop, and print occasionally
int t0_count;
int dump_count;                                                     //16 need to differentiate multiple dumps form same compile
long t,t_old,d;

  int millis_now, i=1;

//16  int n = 1;
  long mil, mic;
  volatile long int_time;
  volatile int h_flag;                                             //10 flag to capture and hold one vector of debug interrupt times

//================================= ISR
void inline handler (void){

//Left motor pulse calculations - - - - - - - - - - - - - - - - - - - - - - - - - -
  throttle_counter_left_motor ++;                                   //Increase the throttle_counter_left_motor variable by 1 every time this routine is executed
  if(throttle_counter_left_motor > throttle_left_motor_memory)      //If the number of loops is larger then the throttle_left_motor_memory variable
  {
    throttle_counter_left_motor = 0;                                //Reset the throttle_counter_left_motor variable
    throttle_left_motor_memory = throttle_left_motor;               //Load the next throttle_left_motor variable
    if(throttle_left_motor_memory < 0)                              //If the throttle_left_motor_memory is negative
      {
       digitalWrite(pin_left_dir, LOW);                             //15 change left wheel rotation to the reverse direction
       throttle_left_motor_memory *= -1;                            //negate the throttle_left_motor_memory variable
      }
    else digitalWrite(pin_left_dir, HIGH);                         //15 otherwise set left wheel rotation to the forward direction
  }
  else if(throttle_counter_left_motor == 1)digitalWrite(pin_left_step, HIGH);  //15 Set left motor step pin high to start a pulse for the stepper controller
  else if(throttle_counter_left_motor == 2)digitalWrite(pin_left_step, LOW);    //15 one interrupt later, lower the pin because the pulse only has to last for 20us 
                                                                               //15 wiring of M1, M2, and M3 pins on A4988 controls step type - we default to SINGLE
//right motor pulse calculations - - - - - - - - - - - - - - - - - - - - - - - - - - 
  throttle_counter_right_motor ++;                                  //Increase the throttle_counter_right_motor variable by 1 every time the routine is executed
  if(throttle_counter_right_motor > throttle_right_motor_memory)    //If the number of loops is larger then the throttle_right_motor_memory variable
  {
    throttle_counter_right_motor = 0;                               //Reset the throttle_counter_right_motor variable
    throttle_right_motor_memory = throttle_right_motor;             //Load the next throttle_right_motor variable
    if(throttle_right_motor_memory < 0)                             //If the throttle_right_motor_memory is negative
      {
       digitalWrite(pin_right_dir, LOW);                            //15 change right wheel rotation to the reverse direction
       throttle_right_motor_memory *= -1;                           //negate the throttle_right_motor_memory variable
      }
    else digitalWrite(pin_right_dir, HIGH);                         //15 otherwise set right wheel rotation to the forward direction
  }
  else if(throttle_counter_right_motor == 1)digitalWrite(pin_right_step, HIGH); //15 Set right motor step pin high to start a pulse for the stepper controller
  else if(throttle_counter_right_motor == 2)digitalWrite(pin_right_step, LOW);  //15 one interrupt later, lower the pin because the pulse only has to last for 20us 
                                                                                //15 wiring of M1, M2, and M3 pins on A4988 controls step type - we default to SINGLE
  timer0_write(ESP.getCycleCount() + t0_count -1 );                 // prime next interrupt to go off after proper interval
  t0_per_sec++ ;                                                    //15 count one more t0 int seen in this second
}
//================================ setup
void setup() {

  Serial.begin(115200);delay(100);
  spl(); spl("[setup] starting up setup");
  sp("[setup] MAC Address= "); spl(WiFi.macAddress());
  
//  WiFi.disconnect(true); delay(1);                                  // disable WIFI altogether
//  WiFi.mode(WIFI_OFF); delay(1);
//  WiFi.forceSleepBegin(); delay(1);
  
  sp("[setup] --- starting sketch ");               //13 include filename and version in startup message
  display_Running_Sketch();
 
 
  pinMode(BUILTIN_LED, OUTPUT);

//---------------------------------- setup for timer0 interrupt
  t0_count = usec_per_t0_int * 80;                                  // # of 80 MHz CPU clocks between T0 interrupts
  mil = millis();                                                   // start of a new second for t0 int counting
  t0_per_sec = 0;                                                   // init to no t0 ints yet. (inc'd by ISR)
  h_flag = 0;                                                       // we're not holding a set of interrupt debug timer values

  noInterrupts();
     timer0_isr_init();
     timer0_attachInterrupt(handler);
     timer0_write(ESP.getCycleCount() + t0_count);
  interrupts();

//--------------------------------- setup for IMU & I2C
  Wire.begin();                                                     //Start the I2C bus as master
  //TWBR = 12;                                                      //Set the I2C clock speed to 400kHz
// compiler didn't like above line. Think the default is 400 KHz, so I'm ignoring it.
//   if needed, I'll look at  void twi_setClock(unsigned int freq)

  //By default the MPU-6050 sleeps. So we have to wake it up.
  
  Wire.beginTransmission(gyro_address);                             //Start communication with the address found during search.
  Wire.write(0x6B);                                                 //We want to write to the PWR_MGMT_1 register (6B hex)
  Wire.write(0x00);                                                 //Set the register bits as 00000000 to activate the gyro
  Wire.endTransmission();                                           //End the transmission with the gyro.
  //Set the full scale of the gyro to +/- 250 degrees per second
  Wire.beginTransmission(gyro_address);                             //Start communication with the address found during search.
  Wire.write(0x1B);                                                 //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x00);                                                 //Set bits as 00000000 (250dps full scale, 131 multiplier)
  Wire.endTransmission();                                           //End the transmission with the gyro
  //Set the full scale of the accelerometer to +/- 4g.
  Wire.beginTransmission(gyro_address);                             //Start communication with the address found during search.
  Wire.write(0x1C);                                                 //We want to write to the ACCEL_CONFIG register (1A hex)
  Wire.write(0x08);                                                 //Set bits as 00001000 (+/- 4g full scale range, 8192 multiplier) page 29
  Wire.endTransmission();                                           //End the transmission with the gyro
  //Set some filtering to improve the raw data.
  Wire.beginTransmission(gyro_address);                             //Start communication with the address found during search
  Wire.write(0x1A);                                                 //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                                                 //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();                                           //End the transmission with the gyro 

  for(receive_counter = 0; receive_counter < 500; receive_counter++){ //Create 500 loops
    if(receive_counter % 25 == 0)digitalWrite(0, !digitalRead(0));  //Change the state of the LED every 15 loops to make the LED blink fast
    Wire.beginTransmission(gyro_address);                           //Start communication with the gyro
    Wire.write(0x45);                                               //13 Start reading the Y and Z registers
    Wire.endTransmission();                                         //End the transmission
    Wire.requestFrom(gyro_address, 4);                              //Request 2 bytes from the gyro

    temp = Wire.read()<<8|Wire.read();                              //Combine the two bytes to make one integer, that could be negative
    if(temp > 32767) temp = temp - 65536;                           // if it's really a negative number, fix it
    gyro_pitch_calibration_value += temp;                           //13  16 bit Y value from gyro, accumulating in 32 bit variable, sign extended

    temp = Wire.read()<<8|Wire.read();                              //Combine the two bytes to make one integer, that could be negative
    if(temp > 32767) temp = temp - 65536;                           // if it's really a negative number, fix it
    gyro_yaw_calibration_value += temp;                             //13  16 bit Z value from gyro, accumulating in 32 bit variable, sign extended

     delayMicroseconds(3700);                                       //Wait for 3700 microseconds to simulate the main program loop time
  }

  sp("[setup] aggregate pitch calibration value= "); spl(gyro_pitch_calibration_value);
  sp("[setup] aggregate yaw   calibration value= "); spl(gyro_yaw_calibration_value);
  
  if(gyro_pitch_calibration_value == -500 && gyro_yaw_calibration_value == -500)  //11 if calibration numbers are characteristically weird
    {
     while( 1==1)                                                    // request an IMU reset, because it's not working
        {
          spl("[setup] ******* IMU is stuck and needs a reset **********");
          delay(3000);
        }
    }

 gyro_pitch_calibration_value /= 500;                               //Divide the total value by 500 to get the avarage gyro pitch offset
 gyro_yaw_calibration_value /= 500;                                 //Divide the total value by 500 to get the avarage gyro yaw offset

  sp("[setup] gyro_pitch_calibration_value= "); spl(gyro_pitch_calibration_value);
  sp("[setup] gyro_yaw_calibration_value= "); spl(gyro_yaw_calibration_value);
 
  sp("[setup] speed override value= "); spl(speed);
    
        sp("[setupIMU): Balance value (if upright): ");             //9 keep serial output narrow to allow 2 dev windows side by side
        Wire.beginTransmission(gyro_address);                       //Start communication with the IMU
        Wire.write(0x3F);                                           // Get the MPU6050_ACCEL_ZOUT_H value
        Wire.endTransmission();                                     //End the transmission with the gyro
        Wire.requestFrom(gyro_address,2);
        temp = Wire.read()<<8|Wire.read();                          // read the 16 bit number from IMU
        if(temp > 32767) temp = temp - 65536;                       // if it's really a negative number, fix it
        spl(temp *-1);                                              //9 note that value is negated before display
        delay(20);

 loop_timer = micros() + 4000;                                      //Set the loop_timer variable at the next end loop time

//15 ------------------------------- setup for A4988 motor controllers
  pinMode(pin_left_dir,OUTPUT);                                     //15 note that our motor control pins are Outputs
  pinMode(pin_left_step,OUTPUT);
  pinMode(pin_right_dir,OUTPUT);
  pinMode(pin_right_step,OUTPUT);
  
  digitalWrite(pin_left_dir, HIGH);                                  //15 init direction and step to known starting points
  digitalWrite(pin_left_step, LOW);                                  //15   i.e. Forward (High) and not stepping (Low)
  digitalWrite(pin_right_dir, HIGH);
  digitalWrite(pin_right_step, LOW);
  
  sp("[setup] u-seconds between t0 interrupts = "); spl(usec_per_t0_int);   //9 

/*
 * //16 pre-fill debug arrays, to see if this triggers dynamic memory allocation and resulting problems
  for (lognum=1; lognum<=log_size; lognum++)
  {
    log_millis[lognum] = millis();
    log_angle[lognum] = -12.567*sqrt(lognum);
    log_pid[lognum] = lognum % 400;
    log_motor[lognum] = (lognum*3)%400;    
//    log_1[lognum] = sqrt(lognum);
//    log_2[lognum] = lognum * lognum;
  }
  */
    lognum = log_size+1;                                                  //15 index into debug arrays. This value means not yet logging
    dump_count = 1;                                                       //16 add to filename to separate multiple dumps from same compile
 
  Serial.println("--Setup complete");
}

// ------------- routine to display Sketch filename & version & compile time at startup ------------- //13
//
//13 following code based on: https://stackoverflow.com/questions/14143517/find-the-name-of-an-arduino-sketch-programmatically                                   
void display_Running_Sketch (void){                                 //13 called once, at start of setup above
  String the_path = __FILE__;                                       //13 grab info in built in Arduino variable
  int slash_loc = the_path.lastIndexOf('/');                        //13 select a portion of the full path
  String the_cpp_name = the_path.substring(slash_loc+1);
  int dot_loc = the_cpp_name.lastIndexOf('.');
//  String the_sketchname = the_cpp_name.substring(0, dot_loc);
  String the_sketchname = the_cpp_name.substring(21, dot_loc);      //13 skip over 22 chars: "C:\Users\DougElliott\"
                                                                    //13 above might not work so well on Mac
//  Serial.print("\nArduino is running Sketch: ");
  Serial.println(the_sketchname);
  sp("            Version "); sp(my_ver); spc;
  Serial.print("Compiled on: ");
  Serial.print(__DATE__);
  Serial.print(" at ");
  Serial.print(__TIME__);
  Serial.print("\n");
}

//===================================== main loop ========================================================
//========================================================================================================

void loop() {
  mics = micros();                                                  //11 spot check the length of the main loop
  millis_now = millis();

//-------------------------angle calculations   

  Wire.beginTransmission(gyro_address);                             //Start communication with the gyro
  Wire.write(0x3F);                                                 //Start reading at register 3F
  Wire.endTransmission();                                           //End the transmission
  Wire.requestFrom(gyro_address, 2);                                //Request 2 bytes from the gyro
  temp = Wire.read()<<8|Wire.read();                                //Combine the two bytes to make one integer
    if(temp > 32767) temp = temp - 65536;                           // if it's really a negative number, fix it
  accelerometer_data_raw = temp;
  accelerometer_data_raw += acc_calibration_value;                  //Add the accelerometer calibration value
  if(accelerometer_data_raw > 8192)accelerometer_data_raw = 8192;   //Prevent division by zero by limiting the acc data to +/-8200;
  if(accelerometer_data_raw < -8192)accelerometer_data_raw = -8192; //Prevent division by zero by limiting the acc data to +/-8200;

  angle_acc = asin((float)accelerometer_data_raw/8192.0)* 57.296;   //Calculate the current angle according to the accelerometer

  if(start == 0 && angle_acc > -0.5&& angle_acc < 0.5){             //If the accelerometer angle is almost 0
    angle_gyro = angle_acc;                                         //Load the accelerometer angle in the angle_gyro variable
    start = 1;                                                      //Set the start variable to start the PID controller
    digitalWrite(BUILTIN_LED,LOW);                                  //16 turn on red LED to show we're storing debug data, no longer dormant.
    if(lognum == log_size + 1) lognum = 0;                          //16 and start debug logging into memory arrays
  }
  
  Wire.beginTransmission(gyro_address);                             //Start communication with the gyro
  Wire.write(0x43);                                                 //Start reading at register 43
  Wire.endTransmission();                                           //End the transmission
  Wire.requestFrom(gyro_address, 4);                                //Request 4 bytes from the gyro

  temp = Wire.read()<<8|Wire.read();                                //Combine the two bytes read to make one 16 bit signed integer
  if(temp > 32767) temp = temp - 65536;                             // if it's really a negative number, fix it
  gyro_yaw_data_raw = temp;                                         // and use result as raw data, which is yaw degrees/sec * 65.5

  temp = Wire.read()<<8|Wire.read();                                //Combine the two bytes read to make one 16 bit signed integer
  if(temp > 32767) temp = temp - 65536;                             // if it's really a negative number, fix it
  gyro_pitch_data_raw = temp;                                       // and use result as raw data, which is pitch degrees/sec * 65.5
  
  gyro_pitch_data_raw -= gyro_pitch_calibration_value;              //Add the gyro calibration value
  angle_gyro += gyro_pitch_data_raw * 0.000030534;                  //11 Calculate the traveled during this loop angle and add this to the angle_gyro variable

//-----------------------------------MPU-6050 offset compensation

  //Not every gyro is mounted 100% level with the axis of the robot. This can be cause by misalignments during manufacturing of the breakout board. 
  //As a result the robot will not rotate at the exact same spot and start to make larger and larger circles.
  //To compensate for this behavior a VERY SMALL angle compensation is needed when the robot is rotating.
  //Try 0.0000003 or -0.0000003 first to see if there is any improvement.

  gyro_yaw_data_raw -= gyro_yaw_calibration_value;                  //Add the gyro calibration value
  //Uncomment the following line to make the compensation active
  //12 re-comment the line below to see if angle calibration gets more accurate
  angle_gyro -= gyro_yaw_data_raw * 0.0000003;                      //11 Compensate the gyro offset when the robot is rotating

//11  angle_gyro = angle_gyro * 0.9996 + angle_acc * 0.0004;            //Correct the drift of the gyro angle with the accelerometer angle
  angle_gyro = angle_gyro * 0.996 + angle_acc * 0.004;              //11 Correct the drift of the gyro angle with the accelerometer angle

//----------------------------------PID controller calculations

  //The balancing robot is angle driven. First the difference between the desired angel (setpoint) and actual angle (process value)
  //is calculated. The self_balance_pid_setpoint variable is automatically changed to make sure that the robot stays balanced all the time.
  //The (pid_setpoint - pid_output * 0.015) part functions as a brake function.
  pid_error_temp = angle_gyro - self_balance_pid_setpoint - pid_setpoint;
  if(pid_output > 10 || pid_output < -10)pid_error_temp += pid_output * 0.015 ;

//15 try to reduce longevity of pid_i_mem, which gets big, and stays big
//  pid_i_mem += pid_i_gain * pid_error_temp;                         //Calculate the I-controller value and add it to the pid_i_mem variable
  temp = pid_i_gain * pid_error_temp;                               //15 current I controller value
  hold2 = pid_i_mem;                                                //15 grab it for debugging before it gets changed
  pid_i_mem =temp + PID_I_fade * pid_i_mem;                         //15 allow impact of past pid_i_mem history to fade out over time
  if(pid_i_mem > pid_max)pid_i_mem = pid_max;                       //Limit the I-controller to the parameterized maximum controller output
  else if(pid_i_mem < pid_min)pid_i_mem = pid_min;
  
  //Calculate the PID output value
  pid_output = pid_p_gain * pid_error_temp + pid_i_mem + pid_d_gain * (pid_error_temp - pid_last_d_error);
  hold3 = pid_output;
  if(pid_output > pid_max)pid_output = pid_max;                     //Limit the PI-controller to the maximum controller output
  else if(pid_output < pid_min)pid_output = pid_min;

  pid_last_d_error = pid_error_temp;                                //Store the error for the next loop

  if(pid_output < 5 && pid_output > -5)pid_output = 0;              //Create a dead-band to stop the motors when the robot is balanced

  if(angle_gyro > 30 || angle_gyro < -30 || start == 0 || low_bat == 1){    //If the robot tips over or the start variable is zero or the battery is empty
    pid_output = 0;                                                 //Set the PID controller output to 0 so the motors stop moving
    pid_i_mem = 0;                                                  //Reset the I-controller memory
    start = 0;                                                      //Set the start variable to 0
    self_balance_pid_setpoint = 0;                                  //Reset the self_balance_pid_setpoint variable
    throttle_left_motor = 0;                                        //16 stop the wheels from moving
    throttle_right_motor = 0;
    digitalWrite(BUILTIN_LED,HIGH);                                 //16 turn off red LED to show we're dumping debug data, and going dormant.
    if(lognum < log_size + 1) log_dump();                           //16 if we logged data, dump it to console for processing, & reprime logging
  }
//-----------------------Control Calculations (does nothing without numchuk controller to provide "received_byte", which stays at zero

  pid_output_left = pid_output;                                     //Copy the controller output to the pid_output_left variable for the left motor
  pid_output_right = pid_output;                                    //Copy the controller output to the pid_output_right variable for the right motor

  if(received_byte & B00000001){                                    //If the first bit of the receive byte is set change the left and right variable to turn the robot to the left
    pid_output_left += turning_speed;                               //Increase the left motor speed
    pid_output_right -= turning_speed;                              //Decrease the right motor speed
  }
  if(received_byte & B00000010){                                    //If the second bit of the receive byte is set change the left and right variable to turn the robot to the right
    pid_output_left -= turning_speed;                               //Decrease the left motor speed
    pid_output_right += turning_speed;                              //Increase the right motor speed
  }

  if(received_byte & B00000100){                                    //If the third bit of the receive byte is set change the left and right variable to turn the robot to the right
    if(pid_setpoint > -2.5)pid_setpoint -= 0.05;                    //Slowly change the setpoint angle so the robot starts leaning forewards
    if(pid_output > max_target_speed * -1)pid_setpoint -= 0.005;    //Slowly change the setpoint angle so the robot starts leaning forewards
  }
  if(received_byte & B00001000){                                    //If the forth bit of the receive byte is set change the left and right variable to turn the robot to the right
    if(pid_setpoint < 2.5)pid_setpoint += 0.05;                     //Slowly change the setpoint angle so the robot starts leaning backwards
    if(pid_output < max_target_speed)pid_setpoint += 0.005;         //Slowly change the setpoint angle so the robot starts leaning backwards
  }   

  if(!(received_byte & B00001100)){                                 //Slowly reduce the setpoint to zero if no foreward or backward command is given
    if(pid_setpoint > 0.5)pid_setpoint -=0.05;                      //If the PID setpoint is larger then 0.5 reduce the setpoint with 0.05 every loop
    else if(pid_setpoint < -0.5)pid_setpoint +=0.05;                //If the PID setpoint is smaller then -0.5 increase the setpoint with 0.05 every loop
    else pid_setpoint = 0;                                          //If the PID setpoint is smaller then 0.5 or larger then -0.5 set the setpoint to 0
  }
 
  //The self balancing point is adjusted when there is not forward or backwards movement from the transmitter. This way the robot will always find it's balancing point
  if(pid_setpoint == 0){                                            //If the setpoint is zero degrees
//15    if(pid_output < 0)self_balance_pid_setpoint += 0.0015;          //Increase the self_balance_pid_setpoint if the robot is still moving forewards
//15    if(pid_output > 0)self_balance_pid_setpoint -= 0.0015;          //Decrease the self_balance_pid_setpoint if the robot is still moving backwards
  }
//-----------------------------Motor Pulse calculations

/*
//To compensate for the non-linear behaviour of the stepper motors the folowing calculations are needed to get a linear speed behaviour.
  if(pid_output_left > 0)pid_output_left = 405 - (1/(pid_output_left + 9)) * 5500;
  else if(pid_output_left < 0)pid_output_left = -405 - (1/(pid_output_left - 9)) * 5500;

  if(pid_output_right > 0)pid_output_right = 405 - (1/(pid_output_right + 9)) * 5500;
  else if(pid_output_right < 0)pid_output_right = -405 - (1/(pid_output_right - 9)) * 5500;

  //Calculate the needed pulse time for the left and right stepper motor controllers
  if(pid_output_left > 0)left_motor = 400 - pid_output_left;
  else if(pid_output_left < 0)left_motor = -400 - pid_output_left;
  else left_motor = 0;

  if(pid_output_right > 0)right_motor = 400 - pid_output_right;
  else if(pid_output_right < 0)right_motor = -400 - pid_output_right;
  else right_motor = 0;
*/
// save a value for once a second debugging before we overwrite it
hold = pid_output_left;

// modifications for A4988 based motor controllers:
//  - ignore linearity considerations. Don't think the difference is substantial
//  - map the pid output directly to speed range of motors.
//  - reliable speeds are from 300 steps/sec (fast) to 2300 steps/sec (slow)
//  - pid range is 1(slow) to 400 (fast)
//  - all of this applies to both directions, + and - values
//  - so, linearly map (400 > 0) to (300 > 2300)
//
//Calculate the needed pulse time for the left and right stepper motor controllers
  if(pid_output_left > 0)left_motor = bot_slow - (pid_output_left/400)*(bot_slow - bot_fast);
  else if(pid_output_left < 0)left_motor = -1*bot_slow - (pid_output_left/400)*(bot_slow - bot_fast);
  else left_motor = 0;

  if(pid_output_right > 0)right_motor = bot_slow - (pid_output_right/400)*(bot_slow - bot_fast);
  else if(pid_output_right < 0)right_motor = -bot_slow - (pid_output_right/400)*(bot_slow - bot_fast);
  else right_motor = 0;


  //Copy the pulse time to the throttle variables so the interrupt subroutine can use them

  if (speed >= 0)                                                   // if we're overriding IMU to force a constant test speed
     {
      noInterrupts();                                               //11 ensure interrupt can't happen when only one wheel is updated
      throttle_left_motor = speed;                                  // overwrite the calculated wheel intervals
      throttle_right_motor= speed;                                  //  ...with fixed value, maybe zero to brake for vertical calibration
      interrupts();                                                 //11 ..by briefly disabling interrupts
     }
  else                                                              //11 restructure conditional so we don't have a brief wrong setting
     {
/*
     if(0 < left_motor  < 25) left_motor =  25;                     //11 avoid high speeds that have a high interrupt load
     if(0 > left_motor  >-25) left_motor  = -25;                    //11 avoid high speeds that have a high interrupt load
     if(0 < right_motor < 25) right_motor =  25;
     if(0 > right_motor >-25) right_motor = -25;
*/
     noInterrupts();                                                //10 ensure interrupt can't happen when only one wheel is updated
     throttle_left_motor = -1* left_motor;                          //15 oops - corections were in wrong direction
     throttle_right_motor = -1 *right_motor;                        //15   ..so need to negate the throttle values
     interrupts();                                                  //10 ..by briefly disabling interrupts
     }
//16 do some data logging / telemetry for debugging purpose
     if(lognum < log_size)                                          //16 lognum is NOT set to log_size+1 in setup, meaning no logging, capture snapshot
     {
       lognum++ ;                                                   //16 increment the index into debug logging arrays
       log_millis[lognum] = millis();                               //16 note timestamp
       log_angle[lognum] = angle_gyro;                              //16 gyro angle
       log_pid[lognum] = pid_output_left;                           //16 left motor pid
       log_motor[lognum] = left_motor;                              //16 interval for interrupt level steps
       log_1[lognum] = pid_i_mem;                                   //16 track pid_i_mem to see how PID_I_fade is working
//       log_2[lognum] = 0;         
     }

// do some debug output to serial monitor to see whats going on

//count 4 millesecond loops to get to a second, and dump debug info once a second
  if (i++ > 250)                                                      // if a full second has passed, display debug info
    {
     i = 0;                                                           // prepare to count up to next second

     spd("--pid_error_temp= ",pid_error_temp); spd("  angle_gyro= ",angle_gyro); spd("  self_balance_pid_setpoint= ",self_balance_pid_setpoint); 
        spdl("  pid_setpoint= ",pid_setpoint);
     spd("  pid_output= ",pid_output);spdl("  pid_output_left= ",hold);
     spd("  pid_i_mem= ",hold2);spdl("  initial pid_output= ",hold3);
//     spd("  throttle_left_motor= ",throttle_left_motor); spd("  left_motor= ",left_motor);         

       loop_mics = micros() - mics;                                   //11  calculate main loop time, in microseconds
//       sp("main loop time(mics,millis)= ");sp(loop_mics);           //11  and print the one that contains once a second prints 
//       spc; sp(millis()-millis_now); spc; 
//       if(debug_out2 != 0) spl(last_millis);                        //11 conditionalize display of previous (non-debug) loop length   
//       spc; spl(throttle_left_motor);                               //11  and print the one that contains once a second prints 
    }
           last_millis = millis()-millis_now;                         //11 track previous loop's length as well, to see one without serial output

//------------------------------ Loop time timer

  //The angle calculations are tuned for a loop time of 4 milliseconds. To make sure every loop is exactly 4 milliseconds a wait loop
  //is created by setting the loop_timer variable to +4000 microseconds every loop.
  if(loop_timer > micros())                                           //16 if the target loop end time is still in the future..
  {
    while(loop_timer > micros()) {};                                  // spin in this while until micros() catches up with loop_timer
  }                                                                   //16 no spin time needed if we're already past old target loop end time
    loop_timer = micros() + 4000;                                       //16 next target loop end time is 4 msec from now.

}

//16 ------------------- dump log arrays to console for capture/processing -----------------
void log_dump() {
// dump debug arrays to console in a format suitable for excel graphing (script processing, and ploticus graphing would be better?)

  int log_n;                                                          // local loop index
  String fname,comp_date,comp_time;                                         // prebuild a file name for debug data, and display it with data
  comp_date = String(__DATE__);
  comp_time = String(__TIME__);
  fname = comp_date.substring(0,3)+"-"+comp_date.substring(4,6)+"-"+comp_time.substring(0,2)+comp_time.substring(3,5)+ "-" + String(dump_count);

  spl(); spl();                                                       // make sure you start on a clean line
  spl("====================================== START ============ ");  //16 make it easy to recognize start of data
  sp(fname); spl(".xlsx");                                                        //15 show suitable filename to be captured with data & used to store it
  // now put the control parameters right into the dump, so we know how numbers were made
  spdl("bot_slow, ",bot_slow);
  spdl("bot_fast, ",bot_fast);
  spdl("PID_I_fade, ",PID_I_fade);
  spdl("p_gain, ",pid_p_gain);
  spdl("i_gain, ",pid_i_gain);
  spdl("d_gain, ",pid_d_gain);

// output the dump arrays in Excel friendly format  
  for(log_n=1; log_n<lognum; log_n++)
  {
  sp(log_millis[log_n]); spc; sp(log_angle[log_n]); spc; sp(log_pid[log_n]); spc; sp(log_motor[log_n]); spc; spl(log_1[log_n]);
  yield();                                                            // avoid a watchdog timeout
  }
  sp("====================================== END ============ ");     // make it easy to recognize start of data
  spl(fname);                                                         // append suitable filename with compile date and time for ID purposes
  dump_count++ ;                                                      // on to next dump number, for filename identifier purposes                    
  lognum = log_size + 1;                                              // prep for logging to start again when bot gets vertical
}  // void log_dump

