
// Plotclock
// cc - by Johannes Heberlein 2014
// v 1.02
// thingiverse.com/joo   wiki.fablab-nuernberg.de
// units: mm; microseconds; radians
// origin: bottom left of drawing surface
// time library see http://playground.arduino.cc/Code/time 
// RTC  library see http://playground.arduino.cc/Code/time 
//               or http://www.pjrc.com/teensy/td_libs_DS1307RTC.html  
// Change log:
// 1.01  Release by joo at https://github.com/9a/plotclock
// 1.02  Additional features implemented by Dave (https://github.com/Dave1001/):
//       - added ability to calibrate servofaktor seperately for left and right servos
//       - added code to support DS1307, DS1337 and DS3231 real time clock chips
//       - see http://www.pjrc.com/teensy/td_libs_DS1307RTC.html for how to hook up the real time clock 
// 1.03  Fixed the length bug at the servo2 angle calculation, other fixups
// 1.04  support DAISO WHITE BOARD MAKER by pokibon
// 1.05  optimeze parameters
// 2.00  for Original New Plotter

#include <TimeLib.h> // see http://playground.arduino.cc/Code/time  // 1.04
#include <Servo.h>

// delete or mark the next line as comment if you don't need these
//#define CALIBRATION      // enable XY calibration mode           // 1.04
//#define CALIBRATION2     // draw rect                               1.05
//#define CALIBRATION3     // draw mesh                               2.00
//#define CALIBRATION4     // enable Z calibration mode            // 2.00
#define REALTIMECLOCK    // enable real time clock

#define WISHY 12 // Offset of the Y coordinats of the plate-wisher

// When in calibration mode, adjust the following factors until the servos move exactly 90 degrees
#define SERVOFAKTORLEFT 570   // 1.04 org:600 to 560 620
#define SERVOFAKTORRIGHT 510  // 1.04 org:600 to 460 530

// Zero-position of left and right servo
// When in calibration mode, adjust the NULL-values so that the servo arms are at all times parallel
// either to the X or Y axis
#define SERVOLEFTNULL 1680    // 1.04 org:1950 1720
#define SERVORIGHTNULL 780    // 1.04 org: 815   740

#define SERVOPINLIFT  2       
#define SERVOPINLEFT  3
#define SERVOPINRIGHT 4

#define ZOFF 160         // 1.04 org:90   increase for up
// lift positions of lifting servo
#define LIFT2 1310 + ZOFF // going towards sweeper  2.00 LIFT0 <-> LIFT2
#define LIFT1 995 - 230 + ZOFF  // between numbers  2.00 org:995
#define LIFT0 660 + ZOFF  // drawing                1.05 org : 600 
#define LIFTE 720 + ZOFF  // eraseing               1.05 org : 720 

#define PEN_HOLDER_X  66    // 1.05 org:71.0
#define PEN_HOLDER_Y  44    // 1.03 org:46

#define LIFT_DRAW  0    // 1.04   draw
#define LIFT_SKIP  1    // 1.04   pen skip
#define LIFT_UP    2    // 1.04   pen move to erase position
#define LIFT_ERASE 3    // 2.00   erase positon

// speed of liftimg arm, higher is slower
#define LIFTSPEED 900   //1.04 org:2000

// length of arms
#define L1 35             // left servo arm
#define L2 55.1           // left servo pen holder
#define L3 13.2           // offset of pen hoder
#define L4 45             // right servo pen arm 1.05 org:45

// origin points of left and right servo 
#define O1X  25           // offset of left servo  1.05 org:24 to 29
#define O1Y -25           //                       2.00 org -25 to -25
#define O2X  51.5         // offset of right servo 1.05 org:49 to 55.5
#define O2Y -25           //                       2.00 org -25 to -25

#ifdef REALTIMECLOCK      
// for instructions on how to hook up a real time clock,
// see here -> http://www.pjrc.com/teensy/td_libs_DS1307RTC.html
// DS1307RTC works with the DS1307, DS1337 and DS3231 real time clock chips.
// Please run the SetTime example to initialize the time on new RTC chips and begin running.
#include <Wire.h>
#include <DS1307RTC.h> // see http://playground.arduino.cc/Code/time    
#endif

int servoLift = 1500;

Servo servo1;  // lift servo
Servo servo2;  // left servo
Servo servo3;  // right servo

volatile double lastX = 75;
volatile double lastY = 47.5;

int last_min = 0;
//================================
// Arduino Setup function
//================================
void setup() 
{   
  pinMode(7, INPUT_PULLUP);
#ifdef REALTIMECLOCK
  Serial.begin(115200);
  //while (!Serial) { ; } // wait for serial port to connect. Needed for Leonardo only

  // Set current time only the first to values, hh,mm are needed  
  tmElements_t tm;
  if (RTC.read(tm)) 
  {
    setTime(tm.Hour,tm.Minute,tm.Second,tm.Day,tm.Month,tm.Year);
    Serial.println("DS1307 time is set OK.");
  } 
  else 
  {
    if (RTC.chipPresent())
    {
      Serial.println("DS1307 is stopped.  Please run the SetTime example to initialize the time and begin running.");
    } 
    else 
    {
      Serial.println("DS1307 read error!  Please check the circuitry.");
    } 
    // Set current time only the first to values, hh,mm are needed

    setTime(13,25,0,0,0,0);
  }
#else  
  // Set current time only the first to values, hh,mm are needed
  setTime(13,25,0,0,0,0);
#endif

  drawTo(75.2, 47);
  lift(LIFT_DRAW);
  //servo1.attach(SERVOPINLIFT);  //  lifting servo
  //servo2.attach(SERVOPINLEFT);  //  left servo
  //servo3.attach(SERVOPINRIGHT);  //  right servo
  delay(1000);

} 


//================================
// Arduino main loop
//================================
int first_flag = 1;   // 1.04 for drawRect

void loop() 
{ 
#ifdef CALIBRATION        // setup left and right servo
   if (!servo1.attached()) servo1.attach(SERVOPINLIFT);
    if (!servo2.attached()) servo2.attach(SERVOPINLEFT);
    if (!servo3.attached()) servo3.attach(SERVOPINRIGHT);

  // Servohorns will have 90° between movements, parallel to x and y axis
  drawTo(-3, 29.2);
  delay(2000);            // 1.05 org:500
  drawTo(74.1, 28);
  delay(2000);            // 1.05 org:500

#elif defined(CALIBRATION2) // draw rectangle
  if (!servo1.attached()) servo1.attach(SERVOPINLIFT);
  if (!servo2.attached()) servo2.attach(SERVOPINLEFT);
  if (!servo3.attached()) servo3.attach(SERVOPINRIGHT);
  drawRect();

#elif defined(CALIBRATION3) // draw mesh
  if (!servo1.attached()) servo1.attach(SERVOPINLIFT);
  if (!servo2.attached()) servo2.attach(SERVOPINLEFT);
  if (!servo3.attached()) servo3.attach(SERVOPINRIGHT);
  drawMesh();
  
#elif defined(CALIBRATION4) // setup lift servo
   if (!servo1.attached()) servo1.attach(SERVOPINLIFT);
   if (!servo2.attached()) servo2.attach(SERVOPINLEFT);
   if (!servo3.attached()) servo3.attach(SERVOPINRIGHT);
  lift(0);
  delay(1000);            // 1.05 org:500
  lift(1);
  delay(1000);
  lift(2);
  delay(1000);
#else 

#define SCALE1 0.9    // 1.04
#define SCALE2 0.9    // 1.04
#define YPOS    28    // 1.03
#define XPOS     0    // 2.00
  int i = 0;
  if (last_min != minute()) {   // new time?
    if (!servo1.attached()) servo1.attach(SERVOPINLIFT);
    if (!servo2.attached()) servo2.attach(SERVOPINLEFT);
    if (!servo3.attached()) servo3.attach(SERVOPINRIGHT);
    lift(LIFT_DRAW);
    hour();
    while ((i+1)*10 <= hour())
    {
      i++;
    }
    number(3, 3, 111, 1);
    if (first_flag) {       // 1.04
  //    drawRect();         // 1.04
  //    drawMesh();         // 1.04
      first_flag = 0;       // 1.04
    }
    number(XPOS,  YPOS, i, SCALE1);              // 1.04
    number(XPOS + 14, YPOS, (hour()-i*10), SCALE1);  // 2.00 ORG:19
    number(XPOS + 23, YPOS, 11, SCALE1);             // 2.00 ORG:28

    i=0;
    while ((i+1)*10 <= minute())
    {
      i++;
    }
    number(XPOS + 29, YPOS, i, SCALE2);                // 2.00 ORG:34
    number(XPOS + 43, YPOS, (minute()-i*10), SCALE2);  // 2.00 ORG:48
    
    lift(LIFT_UP);
    drawTo(PEN_HOLDER_X, PEN_HOLDER_Y + 2);     // 2.00
    lift(LIFT_SKIP);
    drawTo(PEN_HOLDER_X, PEN_HOLDER_Y);         // 2.00 

    last_min = minute();
    delay(580);
    servo1.detach();
    servo2.detach();
    servo3.detach();
  }
#endif
} 

//================================
// drawRect : draw rectangle for calibration
//================================
#define START_X  0
#define START_Y 20
#define END_X   60
#define END_Y   45
#define STEP     5
void drawRect() {                               // 1.04 add drawRect
  lift(LIFT_UP);
  drawTo(START_X, START_Y);
  lift(LIFT_DRAW);
  delay(2000);            // 1.05 org:500
  drawTo(END_X, START_Y);
  delay(2000);            // 1.05 org:500
  drawTo(END_X, END_Y);
  delay(2000);            // 1.05 org:500
  drawTo(START_X, END_Y);
  delay(2000);            // 1.05 org:500
  drawTo(START_X, START_Y);
  delay(2000);            // 1.05 org:500
  lift(LIFT_SKIP);
}
//================================
// draw mesh for calibration
//================================
void drawMesh() {                                 // 1.04 add drawMesh
  int x, y;
  int stepx, stepy;

  stepx = (END_X - START_X) / STEP;
  stepy = (END_Y - START_Y) / STEP;
   
  lift(LIFT_UP);
  drawTo(START_X, START_Y);
  lift(LIFT_DRAW);

  for (y = START_Y; y <= END_Y; y += stepy) {
      lift(LIFT_SKIP);
      drawTo(START_X, y);
      lift(LIFT_DRAW);
      drawTo(END_X, y);
      lift(LIFT_SKIP);
  }
  drawTo(START_X, START_Y);
  for (x = START_X; x <= END_X; x += stepx) {
      lift(LIFT_SKIP);
      drawTo(x, START_Y);
      lift(LIFT_DRAW);
      drawTo(x, END_Y);
      lift(LIFT_SKIP);
  }
  lift(LIFT_UP);
}

//================================
// draw number
//================================
// Writing numeral with bx by being the bottom left originpoint. Scale 1 equals a 20 mm high font.
// The structure follows this principle: move to first startpoint of the numeral, lift down, draw numeral, lift up
void number(float bx, float by, int num, float scale) {

  switch (num) {

  case 0:
    drawTo(bx + 12 * scale, by + 6 * scale);
    lift(0);
    bogenGZS(bx + 7 * scale, by + 10 * scale, 10 * scale, -0.8, 6.7, 0.5);
    lift(1);
    break;
  case 1:

    drawTo(bx + 3 * scale, by + 15 * scale);
    lift(0);
    drawTo(bx + 10 * scale, by + 20 * scale);
    drawTo(bx + 10 * scale, by + 0 * scale);
    lift(1);
    break;
  case 2:
    drawTo(bx + 2 * scale, by + 12 * scale);
    lift(0);
    bogenUZS(bx + 8 * scale, by + 14 * scale, 6 * scale, 3, -0.8, 1);
    drawTo(bx + 1 * scale, by + 0 * scale);
    drawTo(bx + 12 * scale, by + 0 * scale);
    lift(1);
    break;
  case 3:
    drawTo(bx + 2 * scale, by + 17 * scale);
    lift(0);
    bogenUZS(bx + 5 * scale, by + 15 * scale, 5 * scale, 3, -2, 1);
    bogenUZS(bx + 5 * scale, by + 5 * scale, 5 * scale, 1.57, -3, 1);
    lift(1);
    break;
  case 4:
    drawTo(bx + 10 * scale, by + 0 * scale);
    lift(0);
    drawTo(bx + 10 * scale, by + 20 * scale);
    drawTo(bx + 2 * scale, by + 6 * scale);
    drawTo(bx + 12 * scale, by + 6 * scale);
    lift(1);
    break;
  case 5:
    drawTo(bx + 2 * scale, by + 5 * scale);
    lift(0);
    bogenGZS(bx + 5 * scale, by + 6 * scale, 6 * scale, -2.5, 2, 1);
    drawTo(bx + 5 * scale, by + 20 * scale);
    drawTo(bx + 12 * scale, by + 20 * scale);
    lift(1);
    break;
  case 6:
    drawTo(bx + 2 * scale, by + 10 * scale);
    lift(0);
    bogenUZS(bx + 7 * scale, by + 6 * scale, 6 * scale, 2, -4.4, 1);
    drawTo(bx + 11 * scale, by + 20 * scale);
    lift(1);
    break;
  case 7:
    drawTo(bx + 2 * scale, by + 20 * scale);
    lift(0);
    drawTo(bx + 12 * scale, by + 20 * scale);
    drawTo(bx + 2 * scale, by + 0);
    lift(1);
    break;
  case 8:
    drawTo(bx + 5 * scale, by + 10 * scale);
    lift(0);
    bogenUZS(bx + 5 * scale, by + 15 * scale, 5 * scale, 4.7, -1.6, 1);
    bogenGZS(bx + 5 * scale, by + 5 * scale, 5 * scale, -4.7, 2, 1);
    lift(1);
    break;

  case 9:
    drawTo(bx + 9 * scale, by + 11 * scale);
    lift(0);
    bogenUZS(bx + 7 * scale, by + 15 * scale, 5 * scale, 4, -0.5, 1);
    drawTo(bx + 5 * scale, by + 0);
    lift(1);
    break;

  case 111:

    lift(LIFT_ERASE);
    drawTo(65-WISHY, PEN_HOLDER_Y);
    drawTo(65-WISHY, 48);

    drawTo(65-WISHY, 48);
    drawTo(0, 48);
    drawTo(0, 44);
    drawTo(65-WISHY, 44);
    drawTo(65-WISHY, 40);

    drawTo(0, 40);
    drawTo(0, 35);
    drawTo(65-WISHY, 35);
    drawTo(65-WISHY, 30);

    drawTo(0, 30);
    drawTo(0, 26);
    drawTo(65-WISHY, 26);
    drawTo(65-WISHY, 22);

    drawTo(0, 22);
    drawTo(60-WISHY, 40);

    drawTo(PEN_HOLDER_X + 2, PEN_HOLDER_Y);       // 1.05 org:73.2, 44.0
    lift(2);

    break;

  case 11:      // colon
    drawTo(bx + 5 * scale, by + 15 * scale);
    lift(0);
    bogenGZS(bx + 5 * scale, by + 15 * scale, 0.1 * scale, 1, -1, 1);
    delay(10);
    lift(1);
    drawTo(bx + 5 * scale, by + 5 * scale);
    lift(0);
    bogenGZS(bx + 5 * scale, by + 5 * scale, 0.1 * scale, 1, -1, 1);
    delay(10);
    lift(1);
    break;

  }
}

//================================
// lift pen holder
//================================
void lift(char lift) {
  switch (lift) {
    // room to optimize  !

  case 0: //850

      if (servoLift >= LIFT0) {
      while (servoLift >= LIFT0) 
      {
        servoLift--;
        servo1.writeMicroseconds(servoLift);				
        delayMicroseconds(LIFTSPEED);
      }
    } 
    else {
      while (servoLift <= LIFT0) {
        servoLift++;
        servo1.writeMicroseconds(servoLift);
        delayMicroseconds(LIFTSPEED);

      }

    }

    break;

  case 1: //150

    if (servoLift >= LIFT1) {
      while (servoLift >= LIFT1) {
        servoLift--;
        servo1.writeMicroseconds(servoLift);
        delayMicroseconds(LIFTSPEED);

      }
    } 
    else {
      while (servoLift <= LIFT1) {
        servoLift++;
        servo1.writeMicroseconds(servoLift);
        delayMicroseconds(LIFTSPEED);
      }

    }

    break;

  case 2:

    if (servoLift >= LIFT2) {
      while (servoLift >= LIFT2) {
        servoLift--;
        servo1.writeMicroseconds(servoLift);
        delayMicroseconds(LIFTSPEED);
      }
    } 
    else {
      while (servoLift <= LIFT2) {
        servoLift++;
        servo1.writeMicroseconds(servoLift);				
        delayMicroseconds(LIFTSPEED);
      }
    }
    break;
    
  case 3:

    if (servoLift >= LIFTE) {
      while (servoLift >= LIFTE) {
        servoLift--;
        servo1.writeMicroseconds(servoLift);
        delayMicroseconds(LIFTSPEED);
      }
    } 
    else {
      while (servoLift <= LIFTE) {
        servoLift++;
        servo1.writeMicroseconds(servoLift);        
        delayMicroseconds(LIFTSPEED);
      }
    }
    break;
  }
}


void bogenUZS(float bx, float by, float radius, int start, int ende, float sqee) {
  float inkr = -0.05;
  float count = 0;

  do {
    drawTo(sqee * radius * cos(start + count) + bx,
    radius * sin(start + count) + by);
    count += inkr;
  } 
  while ((start + count) > ende);

}

void bogenGZS(float bx, float by, float radius, int start, int ende, float sqee) {
  float inkr = 0.05;
  float count = 0;

  do {
    drawTo(sqee * radius * cos(start + count) + bx,
    radius * sin(start + count) + by);
    count += inkr;
  } 
  while ((start + count) <= ende);
}

//================================
// move pen
//================================
void drawTo(double pX, double pY) {
  double dx, dy, c;
  int i;

  // dx dy of new point
  dx = pX - lastX;
  dy = pY - lastY;
  //path lenght in mm, times 4 equals 4 steps per mm
  c = floor(10 * sqrt(dx * dx + dy * dy));     // 2.00 org:7
//  c = floor(4 * sqrt(dx * dx + dy * dy));   // 1.05
  if (c < 1) c = 1;

  for (i = 0; i <= c; i++) {
    // draw line point by point
    set_XY(lastX + (i * dx / c), lastY + (i * dy / c));

  }

  lastX = pX;
  lastY = pY;
}

//================================
// calc angle
//================================
double return_angle(double a, double b, double c) {
  // cosine rule for angle between c and a
  return acos((a * a + c * c - b * b) / (2 * a * c));
}

//================================
// move pen
//================================
void set_XY(double Tx, double Ty) 
{
  delay(1);
  double dx, dy, c, a1, a2, Hx, Hy, sv2, sv3, d;

  // calculate triangle between pen, servoLeft and arm joint
  // cartesian dx/dy
  dx = Tx - O1X;
  dy = Ty - O1Y;

  // polar lemgth (c) and angle (a1)
  c = sqrt(dx * dx + dy * dy); // 
  a1 = atan2(dy, dx); //
  a2 = return_angle(L1, L2, c);
  sv2 = floor(((a2 + a1 - M_PI) * SERVOFAKTORLEFT) + SERVOLEFTNULL);
  servo2.writeMicroseconds((int)sv2);

  // calculate joinr arm point for triangle of the right servo arm
  a2 = return_angle(L2, L1, c);
//  d = 1.0 + cos((a1 - a2 + 0.621) + M_PI / 2);
//  Hx = Tx - L3 * d; //36,5°
  Hx = Tx + L3 * cos((a1 - a2 + 0.621) + M_PI); //36,5°
  Hy = Ty + L3 * sin((a1 - a2 + 0.621) + M_PI); 
/*
  Serial.print("d:");
  Serial.print(d);
  Serial.print("\tTx:");
  Serial.print(Tx);
  Serial.print("\tTy:");
  Serial.print(Ty);  
  Serial.print("\tHx:");
  Serial.print(Hx);
  Serial.print("\tHy:");
  Serial.println(Hy);
*/

  // calculate triangle between pen joint, servoRight and arm joint
  dx = Hx - O2X;
  dy = Hy - O2Y;

  c = sqrt(dx * dx + dy * dy);
  a1 = atan2(dy, dx);
  a2 = return_angle(L1, L4, c);
//  a2 = return_angle(L1, (L2 - L3), c);  // 1.05
  sv3 = floor(((a1 - a2) * SERVOFAKTORRIGHT) + SERVORIGHTNULL);
  servo3.writeMicroseconds((int)sv3);
/*
  Serial.print("sv2:");
  Serial.print(sv2);
  Serial.print("\tsv3:");
  Serial.println(sv3);
*/
}
