# WRO Future Engineer 2023

## By Yothinburana School Robot Club

Table of content


1. Engineering Information  <br>
  1.1 Movement Management <br>
  1.2 Power Management and Monitoring <br>
  1.3 Obstacle management <br>
  1.4. Engineering factor <br>
2. YouTube Link



Part 1: Robot Design

Designing a robot is not easy. You need to think ahead before you start building your robot. The reason is because

- Choosing the component for the robot
  -   Frame or Chassis – 90% of the parts we use to build the frame for our car are from LEGO MINDSTORMS Robot Inventor. There are various reasons that make us decide to use LEGO parts. One of the main reasons is that LEGO parts are durable and if it is broken you can easily replace it with a new one.


  - Driving Motor – LEGO Power Functions L-motor. It has high acceleration and it's practical. There are various types of motors, but we chose to use this one because of its design. With a LEGO frame, we can easily connect them to the other Lego part or easily connect to the Lego frame.

  - Servo – 2 Geek servos are used in our robot:
     - Geek servo 2kg 360 Degrees for the steering wheel so it can move left and right smoothly.
     -  GEEKSERVO-270 for rotating the ultrasonic sensor to make the robot can see the wall.



  - Color sensor – 2 ZX-03 Reflectors are used to help in the determination of the line at the floor that mark at the field so the robot can see where the red and blue line are.



  - Gyro sensor – GY-25 is the sensor we use to make the robot run in a straight line smoothly and if the bend to the left or right side it can automatically tune itself and run in a straight line.



  - Camera – PIXY 2.1: is used as the eye of the robot. We use it to see where the obstacle is so we can calculate exactly how far from the obstacle and when to dodge it. Placed we use it because it can detect the object that we need to dodge so fast, and it is not hard to tune it.



  - Ultrasonic Sensor – SEN0307 aids us in controlling the robot's proximity to the wall, preventing it from getting excessively close or far.



  - Arduino Uno – we use it to work as a main board for our robot.



  - Battery- Lipo Helicox 7.4V 2200 mAh served as the power supply for our robot. we use it because it is the basic power supply for the robot and the size of the battery is not that big so we can easily manage where to put them.



Engineering Factor

-in this topic, we need to take into account many factors here are some examples of the factor.

1)the motor torque

The torque this motor need is one of the factors because this motor has torque power of around 18N.cm so we can use it if we use the Lippo 7.4v battery and we need to use the power at least 20 or 30% if lower than that the motor will not move.

2)Lippo battery

Arduino uno use does not have a battery indicator that use to give the exact amount of battery that have left in the battery so we need to always check the battery and when we check it should not be lower than 7.7v because the program will be an error, so we need to always check the battery.

3)Gyro drift

It is the phenomenon of the gyroscope output it will slowly going up continuously and it will make the sensor error and it will make the whole program error. So how to fix this problem you need to unplug the sensor and attach it again or you need to put the robot down on the floor and open the robot and wait for 2-5 seconds.

4)Power overload

Sometimes the robot will reset itself if we use so much power so it may cause by the Arduino uno wire may be loose so it cannot transmit electricity to the Pixy camera or the ultrasonic sensor.

**Programming Part**

Require program

-Arduino IDE 2.1.1 - For Programming your Arduino UNO

-Pixymon V2 - For displaying and configuring your Pixy

**Required Libraries**

To be able to use the sensors, many libraries need to be downloaded and included in this project. The libraries that will need to be downloaded will depend on what sensors you use. Most of these libraries are very crucial for completing the Qualification rounds and Final rounds.
```c

#include "Mapf.h"

#include \<Servo.h\>

#include \<PID\_v2.h\>

#include \<Pixy2I2C.h\>

**Set up part()**

For the setup(), we have to include some libraries and initialize the sensors in order for the robot to work as intended.

---------------------------------------------------------

#include "Mapf.h"

#include \<Servo.h\>

#include \<PID\_v2.h\>

#include \<Pixy2I2C.h\>

Pixy2I2C pixy;

PID\_v2 compassPID(0.75, 0.001, 0.035, PID::Direct);

void setup() {

compassPID.Start(0, 0, 0);

compassPID.SetOutputLimits(-180, 180);

compassPID.SetSampleTime(10);

// pinMode(BUZZER, OUTPUT);

pinMode(ENB, OUTPUT);

pinMode(INB, OUTPUT);

pinMode(STEER\_SRV, OUTPUT);

pinMode(ULTRA\_SRV, OUTPUT);

pinMode(ULTRA\_PIN, INPUT);

pinMode(RED\_SEN, INPUT);

pinMode(BLUE\_SEN, INPUT);

pinMode(BUTTON, INPUT);

Serial.begin(115200);

pixy.init();

while (!Serial);

servo1.attach(STEER\_SRV, 500, 2500);

servo2.attach(ULTRA\_SRV, 500, 2500);

steering\_servo(0);

ultra\_servo(0, 'L');

// check\_leds();

while (analogRead(BUTTON) \> 500);

zeroYaw();

while (analogRead(BUTTON) \<= 500);

}
```

---------------------------------------------------------------

**Qualification Round Program explanation**

This program contain

Steering of the Robot - Turns the servo used for steering the robot to a specified degree.

Variables that affect the steering incudes:

  - The input degree [The direction we want to robot to steer to]
  - The direction of the robot measured by the compass sensor

[The direction that the robot is currently facing]

  - The distance between the wall and the robot measured by the ultrasonic sensor

[Preventing the robot from ramming into the wall]

**----------------------------------------------------------------------------**

```c
void loop() {

long countdown\_stop = millis();

while (analogRead(BUTTON) \> 500) {

getIMU();

line\_detection();

ultra\_servo(pvYaw, TURN);

int wall\_distance = getDistance();

motor\_and\_steer(-1 \* compassPID.Run(pvYaw + ((wall\_distance - 25) \* 1) \* ((float(TURN == 'R') - 0.5) \* 2)));

if (millis() - countdown\_stop \> 1200) {

// Stops everything

motor(0);

while (true);

}

if (lines\_detect\_num \< 12) {

countdown\_stop = millis();

}

}

while (analogRead(BUTTON) \<= 500);

...

}

**The driving Motor.**

Variables that affect the speed of the motor include:

-The direction of the robot is measured by the compass sensor If the robot is facing in the wrong way, reduce the speed

void motor\_and\_steer(int degree) {

degree = max(min(degree, 45), -45);

steering\_servo(degree);

motor(map(abs(degree), 0, 45, 80, 40));

}
```


**Turn the robot when a line is detected - There are 2 lines on each corner of the field. This program detects those lines and adjusts the robot heading.**

Variables that affect the turning of the robot include:

-The Reflected light values measured by the Light Sensors

```c
void line\_detection() {

int blue\_value = analogRead(BLUE\_SEN);

if (TURN == 'U') {

int red\_value = analogRead(RED\_SEN);

if (blue\_value \< 600 || red\_value \< 600) {

int lowest\_red\_sen = red\_value;

long timer\_line = millis();

while (millis() - timer\_line \< 100) {

int red\_value = analogRead(RED\_SEN);

if (red\_value \< lowest\_red\_sen) {

lowest\_red\_sen = red\_value;

}

}

if (lowest\_red\_sen \> 600) {

// Red

TURN = 'L';

compass\_offset += 90;

} else {

// Blue

TURN = 'R';

compass\_offset -= 90;

}

lines\_detect\_num++;

halt\_detect\_line\_timer = millis();

}

} else {

if (millis() - halt\_detect\_line\_timer \> 1000) {

if (blue\_value \< 600) {

if (TURN == 'R') {

compass\_offset -= 90;

} else {

compass\_offset += 90;

}

halt\_detect\_line\_timer = millis();

lines\_detect\_num++;

}

}

}

}
```
 Part 2 <br>
Youtube Link: <br>
Qualification Round: https://youtu.be/7kWcytFyiAU (Time 31 sec) <br>


**THE END**
