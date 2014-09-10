/* MUCH OF THIS CODE IS STOLEN FROM 
 * https://github.com/pickle27/arduino_cmucam2/blob/master/cmucam2/cmucam2.ino
 * http://kevinhughes.ca/2013/07/17/how-to-use-the-cmu-cam2-with-the-arduino/
 * by Kevin Hughes at Queen's University
 */

#include <SoftwareSerial.h>
#include <SoftwareServo.h>

// Microcontroller pin set up
const int RX = 8;
const int TX = 7;
const int RIGHT_ENCODER_PIN = 5;
const int LEFT_ENCODER_PIN = 4;
const int INTERRUPT_1 = 0;
const int INTERRUPT_2 = 1;
const int RIGHT_FRONT_FACING_IR_PIN = 0;
const int RIGHT_FACING_IR_PIN = 1;
const int CENTER_IR_PIN = 2;
const int LEFT_FACING_IR_PIN = 3;
const int LEFT_FRONT_FACING_IR_PIN = 4;
const int HIGH_BUMP_SENSOR_PIN = 12;
const int LOW_BUMP_SENSOR_PIN = 13;

int rffIR = 0;
int rfIR = 0;
int cIR = 0;
int lfIR = 0;
int lffIR = 0;
int hBS = 0;
int lBS = 0;

// Global variable for: 
//    1) Tracking where the blob was last seen
//    2) Tracking if the goal has been reached
//    3) Tracking which mode we're in
boolean onRight;
boolean reachedGoal;
boolean wallFollow;
boolean frontStuck;
int mode;

// Global variables to count wheel watcher tics
int rightWW, leftWW;

// Create servo objects for lw/rw
SoftwareServo rightWheel;
SoftwareServo leftWheel;

// Serial comm set up for CMU cam 2
SoftwareSerial cmucam(RX, TX);
unsigned char RcvData[8] = "";
unsigned char packet[8];

/* 
 * Function to print the data packet
 */
void print_packet(unsigned char * packet)
{
  for(int i = 0; i < 8; i++)
  {
    Serial.print( (int)packet[i] );
    Serial.print(" "); 
  }
  Serial.println();
}

/* 
 * Function for sending commands to the CMU Cam2
 * where no return data aside from the 'ACK' 
 * command is expected. The verbose option
 * will print out exactly what is sent and 
 * recieved from the CMU Cam2
 */
boolean cmucam2_set(char* cmd, boolean verbose=false)
{
  if(verbose)
  {
    Serial.print("Sending CMU Cam2 Command: ");
    Serial.print(cmd);
    Serial.println();
  }

  // send the command
  cmucam.print(cmd);
  cmucam.print("\r");
  cmucam.listen();

  boolean ack = false;

  if(verbose)
    Serial.print("++> ");

  // get the response
  while( cmucam.available() > 0 ) 
  {
    char inbyte = cmucam.read();

    if(inbyte == ':')
      ack = true;  

    if(verbose)
      Serial.write(inbyte);
  }

  if(verbose)
    Serial.println();

  // flush
  while( cmucam.available() > 0 )
    cmucam.read();

  return ack;
}

/* 
 * Function for sending commands to the CMU Cam2
 * where return data is expected. The packet type
 * must be specified, currently only S and T packets
 * are supported. This code expects the camera to be in
 * raw mode and still sending 'ACK'. The rtrn buffer 
 * must be at least 8 bytes for T packets and 
 * at least 6 bytes for S packets. 
 * The verbose option will print out exactly what 
 * is sent and recieved from the CMU Cam2
 */
boolean cmucam2_get(char* cmd, char packet, unsigned char *rtrn, boolean verbose=false)
{
  if(verbose)
  {
    Serial.print("Sending CMU Cam2 GET Command: ");
    Serial.print(cmd);
    Serial.println();
  }

  // send the command
  cmucam.print(cmd);
  cmucam.print("\r");
  cmucam.listen();

  // Debug Packet
  // Change to true and
  // turn of raw mode "RM 0"
  // to see the packet
  boolean debug = false;
  if(debug) 
  {
    while(cmucam.available() > 0)
    {
      delay(100);
      Serial.print((char)cmucam.read());    
    }
    Serial.println();

    return true;
  }

  // S-Packet
  // raw mode must be on
  if(packet == 'S')
  {
    while(cmucam.read() != 255); // wait for signal bit
    while(cmucam.read() != 83);
    while(cmucam.available() < 6); // wait for data
    for(int i = 0; i < 6; i++) // read the packet
    {
      rtrn[i] = cmucam.read();    
    }
  }

  // T-Packet
  // raw mode must be on
  if(packet == 'T')
  {
    while(cmucam.read() != 255); // wait for signal bit
    while(cmucam.read() != 84); 
    while(cmucam.available() < 8); // wait for data
    for(int i = 0; i < 8; i++) // read the packet
    {
      rtrn[i] = cmucam.read();    
    }
  }

  return true;  
}

void _updateRightEncoder() {
  int8_t adjustment = (((digitalRead(RIGHT_ENCODER_PIN) ^ 0)<<1)-1);
  rightWW += adjustment;
}

void _updateLeftEncoder() {
  int8_t adjustment = ((((!digitalRead(LEFT_ENCODER_PIN)) ^ 0)<<1)-1);
  leftWW += adjustment;
}

/*********************************************************************
 *                          My Helper Functions                       *
 *********************************************************************/

// Set the wheels to drive in the given direction.
// fwd: negative value = drive backwards
//      positive value = drive forwards
// side: negative value = drive left
//       positive value = drive right
// Both input parameters are values between -90 and 90
void drive(int fwd, int side) {

  // Left Wheel:
  //    Values above 90 are forward, so fwd is added to
  //    the base (stopped) value of 90. Going right means
  //    increasing speed of the left wheel while decreasing
  //    speed of the right, so side is also added to the 
  //    base value.
  leftWheel.write(85 + fwd + side);
  // Right Wheel:
  //    Opposite of left wheel, so fwd is subtracted from
  //    the base (stopped) value of 90. Going right means
  //    decreasing speed of the right wheel while increasing
  //    speed of the left, so side is also added to the base
  //    value.
  rightWheel.write(85 - fwd + side);
}

// Mode 0: Use the CMUCam2 to blob track an orange cone
void track_cone(unsigned char *packet, int hBS) {
  if (!reachedGoal) {

    // Read incoming value from packet 6 (packet 6 = can I see ANY pixels I want?)
    if(packet[6] > 0){
      // If I can, drive straight
      drive(30,0);
      if (packet[1] <= 80)
        onRight = true;
      else
        onRight = false;
    }
    else
    {
      if (onRight) {
        drive(0,15);
      }
      else {
        drive(0,-15);
      }
    }

    if (hBS == 1) {
      reachedGoal = true;
    }
  }
  else {
    drive(0,0);
  }
}

// Mode 1: Right follow a wall
void follow_wall(int rffIR, int rfIR, int cIR, int lfIR, int lffIR, int hBS) {

  /* ALPHA HEIRARCHICAL METHOD 09-10-2014 */
  if (lffIR <= 150) {
    if (rfIR <= 150)
      drive(5,40);
    else if (rfIR > 150 && rfIR <= 300)
      drive(25,15);
    else if (rfIR > 300)
      drive(25,0);
  }
  else if (lffIR > 150 && lffIR <= 250) {
    drive(25,10);
  }
  else if (lffIR > 250 && lffIR <= 300) {
    drive(25,0);
  }
  else if (lffIR > 300 && lffIR <= 400) {
    drive(25,-15);
  }
  else if (lffIR > 400 && lffIR <= 475) {
    drive(25,-20);
  }
  else if (lffIR > 475) {
    drive(0,-20);
  }

  /* ORIGINAL METHOD
   if (lffIR > 300 && rfIR < 300) {
   drive(0,-15);
   }
   else if (lffIR < 150 && rfIR < 150) {
   drive(0,20);
   }
   else if (rfIR > 300 && rfIR < 450 && lffIR < 300) {
   drive(30,0);
   }
   else if (rfIR > 450) {
   drive(0,-15);
   } 
   else {
   drive(30,0);
   }
   */
}

void get_unstuck() {
  int currentTics = rightWW;
  while ((currentTics - rightWW) < 30) {
    drive(-10,0);
    SoftwareServo::refresh();
    delay(30);
  }
  currentTics = rightWW;
  while((rightWW - currentTics) < 30) {
    drive(5,-20);
    SoftwareServo::refresh();
    delay(30);
  }
  currentTics = rightWW;
  while((rightWW - currentTics) < 10) {
    drive(10,0);
    SoftwareServo::refresh();
    delay(30);
  } 
  frontStuck = false;
}

// Prints out the values of all the sensors and CMU Cam data
void debug_printer(unsigned char *packet, int rffIR, int rfIR, int cIR, int lfIR, int lffIR) {

  Serial.print(packet[0], DEC);    // MEAN X
  Serial.print(" ");
  Serial.print(packet[1], DEC);    // MEAN Y
  Serial.print(" ");
  Serial.print(packet[2], DEC);    // MIN X
  Serial.print(" ");
  Serial.print(packet[3], DEC);    // MIN Y
  Serial.print(" ");
  Serial.print(packet[4], DEC);    // MAX X
  Serial.print(" ");
  Serial.print(packet[5], DEC);    // MAX Y
  Serial.print(" ");
  Serial.print(packet[6], DEC);    // NUM PIXELS
  Serial.print(" ");
  Serial.print(packet[7], DEC);    // CONFIDENCE
  Serial.print(" ");
  Serial.print(leftWW, DEC);       // left wheel ticks
  Serial.print(" ");
  Serial.print(rightWW, DEC);    // right wheel ticks
  Serial.print(" ");
  Serial.print(rffIR, DEC);    // right front facing ir 
  Serial.print(" ");
  Serial.print(rfIR, DEC);    // right front ir 
  Serial.print(" ");
  Serial.print(cIR, DEC);    // center ir 
  Serial.print(" ");
  Serial.print(lfIR, DEC);    // left facing ir 
  Serial.print(" ");
  Serial.println(lffIR, DEC);    // left front facing ir
}

/*********************************************************************
 *                             SETUP & LOOP                           *
 *********************************************************************/

void setup()
{
  Serial.begin(115200);

  /* SETUP OUR GLOBAL VARIABLES */
  // Default value = turn right if the blob is not on screen
  onRight = true;
  reachedGoal = false;
  wallFollow = false;
  frontStuck = false;
  mode = 1;

  // Attach the wheel watchers
  attachInterrupt(INTERRUPT_1, _updateLeftEncoder, FALLING); 
  attachInterrupt(INTERRUPT_2, _updateRightEncoder, FALLING);  

  // Attach servos
  rightWheel.attach(10);
  leftWheel.attach(11);

  // CMU cam 2 init  
  cmucam.begin(9600);
  cmucam.print("RS"); 
  cmucam.print("\r");
  cmucam.print("RS"); 
  cmucam.print("\r");
  cmucam.listen();
  while( cmucam.available() > 0 ) 
  {
    cmucam.read();
  }
  delay(100);
  while(!cmucam2_set("RS", true));
  // End Init CMU Cam2

  // Turn OFF Auto Gain
  while(!cmucam2_set("CR 19 33", true));

  // Turn OFF Auto White Balance (this is unnecessary since it's the default)
  while(!cmucam2_set("CR 18 40", true));

  // Turn ON Poll Mode
  while(!cmucam2_set("PM 1", true));

  // Turn ON Raw Mode
  while(!cmucam2_set("RM 1", true));

}

void loop()
{
  SoftwareServo::refresh();
  // You must have this. This function needs to be called every 50ms in order to write new values to the servos.
  // If your loop function gets to big you will need to add more refresh()

  // To get a blob tracking packet from the camera,
  // we send the following command which says to look
  // for blobs whose colors are between 200 and 240 RED,
  // between 0 (really 16) and 40 BLUE, and between
  // 0 (really 16) and 40 GREEN. 
  // The data comes back as eight bytes describing a rectangular
  // bounding box around the blob.
  // Mean x, Mean y, Min x, Min y, Max x, Max y, 
  // num pixels in the rectangle which belong to the blob, 
  // and "confidence" (basically a measure how much of the
  // rectangle is filled by pixels belonging to the blob).

  // The camera is rotated 90 degrees, so use the Y value.
  // Low Y values indicate the blob is to the left of the
  // view, high Y values indicate the blob is to the right of the view.
  // You want a blob with a moderate confidence (perhaps over 20?)
  // Else it's probably just noise.

  cmucam2_get("TC 200 240 0 40 0 40", 'T', packet, false);

  // Read values from IR sensors
  rffIR = analogRead(RIGHT_FRONT_FACING_IR_PIN);
  rfIR = analogRead(RIGHT_FACING_IR_PIN);
  cIR = analogRead(CENTER_IR_PIN);
  lfIR = analogRead(LEFT_FACING_IR_PIN);
  lffIR = analogRead(LEFT_FRONT_FACING_IR_PIN);
  hBS = digitalRead(HIGH_BUMP_SENSOR_PIN);
  lBS = digitalRead(LOW_BUMP_SENSOR_PIN);

  switch (mode) {
  case 0: // Track Cone
    track_cone(packet, hBS);
    break;
  case 1: // Follow Wall
    if (!wallFollow) {
      if (lffIR > 300) {
        wallFollow = true;
      }
      else {
        drive(30,0);
      }
    }
    else { 
      if (cIR > 450 || hBS == 1 || lBS == 1 || lfIR > 450 || rffIR > 450)
        frontStuck = true;
      if (frontStuck)
        get_unstuck();
      else
        follow_wall(rffIR,rfIR,cIR,lfIR,lffIR,hBS); 
    }
    break;
  case 2: // Bug 0.5
    
    break;
  default:
    break;
  }
  // Here is some debugging code which will print out the packets
  // received.
  //debug_printer(packet, rffIR, rfIR, cIR, lfIR, lffIR);
}






