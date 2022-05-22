#include "Adafruit_VL6180X.h" //time of flight sensor
//#include <Adafruit_MPU6050.h> //gyroscope and accelerometer
#include <Wire.h>
#include <Vector.h>            //extra vector library
#include <Adafruit_MLX90614.h> //temperature sensor
#include <Adafruit_TSL2561_U.h>


#define stepsPerTile 1874          // set this as however many steps for exactly 30 cm rotation
#define TURNCONSTANT 1023          // this is the amount of steps for the robot to turn 90 deg
#define LUXTHRESHOLD 0            // threshold for the lux sensor to detect difference between black and not balck holes
#define SOLENOID_RELAY_PIN 52      // pin for the soledoid relay
#define VICTIM_LED 53              // pin for the victim led output
#define OBSTACLE_PIN 14            // pin that will be pulled high by obstacle bump pin
#define WALL_DISTANCE_THRESHOLD 150 // this is in millimeters

#define ERROR_LED_0 46
#define ERROR_LED_1 47
#define ERROR_LED_2 48
#define ERROR_LED_3 49

/*
ERROR CODES
WRRR
0001 = TOFR faliure to start
0010 = TOFF faliure to start
0011 = TOFL faliure to start
0100 = TOFB faliure to start
0101 = MPU faliure to start (no longer used)
0110 = Color Sensor faliure to start
0111 = Temp Right faliure to start
1000 = Temp Left faliure to start
*/

#define RSTEP 22 // pin to step right motor
#define RCC 23   // right motor change direction
#define LSTEP 24 // pin to step left motor
#define LCC 25   // left motor change direction

uint16_t AMBIENT_TEMP;

// Adafruit_MPU6050 mpu;
Adafruit_VL6180X tofFront;
Adafruit_VL6180X tofRight;
Adafruit_VL6180X tofLeft;
Adafruit_VL6180X tofBack;
Adafruit_MLX90614 tempRight;
Adafruit_MLX90614 tempLeft;
Adafruit_TSL2561_Unified luxSensor = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);


uint8_t orientation;

enum Directions
{
  UP,
  RIGHT,
  DOWN,
  LEFT
};

typedef Vector<Directions> Elements; //for vector

void rightStep(bool d)
{ // true for forward, false for backwards
  if (d)
  digitalWrite(RCC, HIGH);
  digitalWrite(RSTEP, HIGH);
  delay(1);
  digitalWrite(RSTEP, LOW);
  delay(1);
  digitalWrite(RCC, LOW);
}

void leftStep(bool d)
{
  if (!d)
  digitalWrite(LCC, HIGH);
  digitalWrite(LSTEP, HIGH);
  delay(1);
  digitalWrite(LSTEP, LOW);
  delay(1);
  digitalWrite(LCC, LOW);
}

void TCA(uint8_t bus) //send a byte to indicate which sensor to switch to
{
  Wire.beginTransmission(0x70); 
  Wire.write(1 << bus);         
  Wire.endTransmission();
}

struct Tile 
{
  bool state : 1;
  bool decision : 1;
};

void blinkIndicator() //blink all leds to signal victim
{
  //Serial.println("Blinking indicators!!");
  for (uint8_t i = 0; i < 5; i++)
  {
    digitalWrite(VICTIM_LED, HIGH);
    digitalWrite(ERROR_LED_0, HIGH);
    digitalWrite(ERROR_LED_1, HIGH);
    digitalWrite(ERROR_LED_2, HIGH);
    digitalWrite(ERROR_LED_3, HIGH);
    delay(500);
    digitalWrite(VICTIM_LED, LOW);
    digitalWrite(ERROR_LED_0, LOW);
    digitalWrite(ERROR_LED_1, LOW);
    digitalWrite(ERROR_LED_2, LOW);
    digitalWrite(ERROR_LED_3, LOW);
    delay(500);
  }
}

void dispenseN(uint8_t n)  //dispense medkits
{
  for (uint8_t i = 0; i < n; i++)
  {
    //Serial.println("dispensing...");
    digitalWrite(SOLENOID_RELAY_PIN, HIGH);
    delay(1000);
    digitalWrite(SOLENOID_RELAY_PIN, LOW);
    delay(1000);
  }
}
void turnLeft()
{
  //Serial.println("turning left");
  switch (orientation) //adjust orientation 
  {
  case UP:
    orientation = LEFT;
    break;
  case RIGHT:
    orientation = UP;
    break;
  case DOWN:
    orientation = RIGHT;
    break;
  case LEFT:
    orientation = DOWN;
    break;
  }

  for (uint16_t i = 0; i < TURNCONSTANT; i++)
  {
    leftStep(false);
    rightStep(true);
  }
}

void turnRight()
{
  //Serial.println("turning right");
  switch (orientation)
  {
  case UP:
    orientation = RIGHT;
    break;
  case RIGHT:
    orientation = DOWN;
    break;
  case DOWN:
    orientation = LEFT;
    break;
  case LEFT:
    orientation = UP;
    break;
  }

  for (uint16_t i = 0; i < TURNCONSTANT; i++)
  {
    leftStep(true);
    rightStep(false);
  }
}

/*
 * If the pi detects a victim, it will send a string into the serial buffer. this function reads if the serial buffer contains data and responds accordingly
*/
void victimDetect() 
{
  if (Serial.available() == 3 && (Serial.peek() == '1' || Serial.peek() == '2')) //check if serial buffer contains data
  {
    String data = Serial.readStringUntil('\n');
    //Serial.print("reading a : ");
    //Serial.println(data);

    //in the case that the pi detects a victim too far away, check the sides to make sure robot is adjacent to victim
    TCA(3);
    bool rightBlocked = (data[0] == '1' && tofLeft.readRange() < WALL_DISTANCE_THRESHOLD) ? true : false;
    TCA(1);
    bool leftBlocked = (data[0] == '2' && tofRight.readRange() < WALL_DISTANCE_THRESHOLD) ? true : false;

    if (rightBlocked || leftBlocked)
    {
      turnRight();
      turnRight();
      blinkIndicator();
      if (data[1] == 'h') //hurt 
      {
        dispenseN(3);
      }
      else if (data[1] == 's') //stable
      {
        dispenseN(2);
      }
      else if (data[1] == 'u') //unharmed
      {
        // dispense nothing
      }
      else if (data[1] == 'r') //red
      {
        dispenseN(1);
      }
      else if (data[1] == 'y') //yellow
      {
        dispenseN(1);
      }
      else if (data[1] == 'g') //green
      {
        // dispense nothing
      }
      turnLeft();
      turnLeft();
    }
  }
}

double readRightC()//wrapper functions to read temperatures
{
  TCA(6);
  return tempRight.readObjectTempC();
}
double readLeftC()
{
  TCA(7);
  return tempLeft.readObjectTempC();
}

class Bot
{
private:
  uint8_t x = 35; //robot's xy coordinates in the maze, read technical paper for explanation of pathfinding
  uint8_t y = 35;

  bool moveforward()
  { 
    //Serial.println("moving forward");
    sensors_event_t event;
    uint32_t i = 0;
    while (i < stepsPerTile)
    {
      rightStep(true);
      leftStep(true);

      // hole detection, run every 50 steps
      //look for changes in lux to determine if we are over a black tile
      if (i % 50 == 0) {
        luxSensor.getEvent(&event);
        if (event.light < LUXTHRESHOLD || digitalRead(OBSTACLE_PIN) == HIGH) 
        {
          //Serial.println("forward movemen failed");
          while (i >= 0) //reverse robot away from tile and return false to indicate that movement failed
          {
            rightStep(false);
            leftStep(false);
            delay(10);
            i--;
          }
          return false; 
        }
      }
      // check for temperature every 320 steps
      /*
      if (i % 320 == 0 && ((readRightC() - AMBIENT_TEMP) > 10 || (readLeftC() - AMBIENT_TEMP) > 10))
      {
        blinkIndicator();
        dispenseN(1);
      }*/

      victimDetect();
      i++;
    }
    //Serial.println("forward movement complete");
    return true;
  }
public:
  /*
  * because the algorithm checks cardinal directions on the maze and the robot is not always facing up, we need switches to adjust our orientation to make sure the robot is calling the correct sensors
  */
  bool checkUp()
  {
    switch (orientation)
    {
    case UP:
      TCA(2);
      if (tofFront.readRange() < WALL_DISTANCE_THRESHOLD){
        return false;}
      break;
    case RIGHT:
      TCA(3);
      if (tofLeft.readRange() < WALL_DISTANCE_THRESHOLD) {
        return false;}
      break;
    case DOWN:
      TCA(4);
      if (tofBack.readRange() < WALL_DISTANCE_THRESHOLD) {
        return false;}
      break;
    case LEFT:
      TCA(1);
      if (tofRight.readRange() < WALL_DISTANCE_THRESHOLD) {
        return false;}
      break;
    }
    //Serial.println("up is open");
    return true;
  }

  bool checkRight()
  {
    switch (orientation)
    {
    case UP:
      TCA(1);
      if (tofRight.readRange() < WALL_DISTANCE_THRESHOLD) {
        return false;}
      break;
    case RIGHT:
      TCA(2);
      if ( tofFront.readRange() < WALL_DISTANCE_THRESHOLD) {
        return false;}
      break;
    case DOWN:
      TCA(3);
      if ( tofLeft.readRange() < WALL_DISTANCE_THRESHOLD) {
        return false;}
      break;
    case LEFT:
      TCA(4);
      if ( tofBack.readRange() < WALL_DISTANCE_THRESHOLD) {
        return false;}
      break;
    }

    //Serial.println("right is open");
    return true;
  }

  bool checkDown()
  {
    switch (orientation)
    {
    case UP:
      TCA(4);
      if ( tofBack.readRange() < WALL_DISTANCE_THRESHOLD) {
        return false;}
      break;
    case RIGHT:
      TCA(1);
      if ( tofRight.readRange() < WALL_DISTANCE_THRESHOLD) {
        return false;}
      break;
    case DOWN:
      TCA(2);
      if ( tofFront.readRange() < WALL_DISTANCE_THRESHOLD) {
        return false;}
      break;
    case LEFT:
      TCA(3);
      if ( tofLeft.readRange() < WALL_DISTANCE_THRESHOLD) {
        return false;}
      break;
    }
    //Serial.println("down is open");
    return true;
  }

  bool checkLeft()
  {
    switch (orientation)
    {
    case UP:
      TCA(3);
      if ( tofLeft.readRange() < WALL_DISTANCE_THRESHOLD) {
        return false;}
      break;
    case RIGHT:
      TCA(4);
      if ( tofBack.readRange() < WALL_DISTANCE_THRESHOLD) {
        return false;}
      break;
    case DOWN:
      TCA(1);
      if ( tofRight.readRange() < WALL_DISTANCE_THRESHOLD) {
        return false;}
      break;
    case LEFT:
      TCA(2);
      if ( tofFront.readRange() < WALL_DISTANCE_THRESHOLD) {
        return false;}
      break;
    }
    //Serial.println("left is open");
    return true;
  }

  uint8_t getX() { return x; }

  uint8_t getY() { return y; }

  /*
  * translates absolute directions from algorithms into movement for the robot and update robot's position in maze
  */
  bool travel(Directions s)
  {
    if (s == UP)
    {
      switch (orientation)
      {
      case UP:
        if (!moveforward())
          return false;
        break;
      case RIGHT:
        turnLeft();
        if (!moveforward())
          return false;
        break;
      case DOWN:
        turnLeft();
        turnLeft();
        if (!moveforward())
          return false;
        break;
      case LEFT:
        turnRight();
        if (!moveforward())
          return false;
        break;
      }
      y -= 1;
    }
    else if (s == RIGHT)
    {
      switch (orientation)
      {
      case UP:
        turnRight();
        if (!moveforward())
          return false;
        break;
      case RIGHT:
        if (!moveforward())
          return false;
        break;
      case DOWN:
        turnLeft();
        if (!moveforward())
          return false;
        break;
      case LEFT:
        turnRight();
        turnRight();
        if (!moveforward())
          return false;
        break;
      }
      x += 1;
    }
    else if (s == LEFT)
    {
      switch (orientation)
      {
      case UP:
        turnLeft();
        if (!moveforward())
          return false;
        break;
      case RIGHT:
        turnLeft();
        turnLeft();
        if (!moveforward())
          return false;
        break;
      case DOWN:
        turnRight();
        if (!moveforward())
          return false;
        break;
      case LEFT:
        if (!moveforward())
          return false;
        break;
      }
      x -= 1;
    }
    else if (s == DOWN)
    {
      switch (orientation)
      {
      case UP:
        turnRight();
        turnRight();
        if (!moveforward())
          return false;
        break;
      case RIGHT:
        turnRight();
        if (!moveforward())
          return false;
        break;
      case DOWN:
        if (!moveforward())
          return false;
        break;
      case LEFT:
        turnLeft();
        if (!moveforward())
          return false;
        break;
      }
      y += 1;
    }
    return true;
  }
};

//over time error in movement will build up, this will adjust the robot's position to center it on a tile
void calibrate()
{
  //Serial.println("Calibrating distance from wall...");
  TCA(2);
  if (tofFront.readRangeStatus() == VL6180X_ERROR_NONE && tofFront.readRange() < 100)
  {
    while (tofFront.readRangeStatus() == VL6180X_ERROR_NONE && tofFront.readRange() > 50)
    {
      rightStep(true);
      leftStep(true);
      delay(100);
    }
  }
}

class Map
{
private:
  Bot bot;
  Tile map[70][70]; //70x70 mental map of the maze
  Directions storage_history[40]; //
  Directions storage_nextMove[5];
  Elements history;
  Elements nextMove;

  void fillMap()
  {
    for (uint8_t i = 0; i < 70; i++)
    {
      for (uint8_t j = 0; j < 70; j++)
      {
        map[i][j] = Tile();
        map[i][j].state = false;
        map[i][j].decision = false;
      }
    }
  }

  void backtrack()
  {
    do
    {
      Directions lastMove = history.back();
      history.pop_back();

      if (lastMove == LEFT)
        bot.travel(RIGHT);
      else if (lastMove == RIGHT)
        bot.travel(LEFT);
      else if (lastMove == UP)
        bot.travel(DOWN);
      else if (lastMove == DOWN)
        bot.travel(UP);

    } while (!(map[bot.getY()][bot.getX()].decision == true));
    map[bot.getY()][bot.getX()].decision = false;
  }

public:
  Map()
  {
    history.setStorage(storage_history);
    nextMove.setStorage(storage_nextMove);

    fillMap();
  }
  void move()
  {
    map[bot.getY()][bot.getX()].state = true; //explore current tile

    if (bot.checkRight() && map[bot.getY()][bot.getX() + 1].state == false)
    {
      nextMove.push_back(RIGHT);
      //Serial.println("Wants to move Right");
    }
    if (bot.checkUp() && map[bot.getY() - 1][bot.getX()].state == false)
    {
      nextMove.push_back(UP);
      //Serial.println("wants to move up");
    }
    if (bot.checkLeft() && map[bot.getY()][bot.getX() - 1].state == false)
    {
      nextMove.push_back(LEFT);
      //Serial.println("wants to move left");
    }
    if (bot.checkDown() && (map[bot.getY() + 1][bot.getX()].state == false))
    {
      nextMove.push_back(DOWN);
      //Serial.println("wants to move down");
    }

    if (nextMove.size() == 0)
    {
      backtrack();
      //Serial.println("backtracking...");
    }
    else if (nextMove.size() == 1)
    {
      history.push_back(nextMove[0]);
      //Serial.print("moving: ");
      //Serial.println(nextMove[0]);
      if (!bot.travel(nextMove[0]))
      {
        history.pop_back();
        switch (nextMove[0])//in the case that the robot fails to traverse to a tile, mark that tile as explored
        {
        case UP:
          map[bot.getY() - 1][bot.getX()].state = true;
          break;
        case RIGHT:
          map[bot.getY()][bot.getX() + 1].state = true;
          break;
        case DOWN:
          map[bot.getY() + 1][bot.getX()].state = true;
          break;
        case LEFT:
          map[bot.getY()][bot.getX() - 1].state = true;
        }
      }
    }
    else
    { 
      //Serial.print("moving :");
      //Serial.println(nextMove[0]);
      history.push_back(nextMove[0]);
      map[bot.getY()][bot.getX()].decision = true;
      if (!bot.travel(nextMove[0]))
      {
        history.pop_back();
        switch (nextMove[0])
        {
        case UP:
          map[bot.getY() - 1][bot.getX()].state = true;
          break;
        case RIGHT:
          map[bot.getY()][bot.getX() + 1].state = true;
          break;
        case DOWN:
          map[bot.getY() + 1][bot.getX()].state = true;
          break;
        case LEFT:
          map[bot.getY()][bot.getX() - 1].state = true;
        }
      }
    }
    calibrate();

    nextMove.clear();
  }
};

void setup()
{
  pinMode(ERROR_LED_0, OUTPUT);
  pinMode(ERROR_LED_1, OUTPUT);
  pinMode(ERROR_LED_2, OUTPUT);
  pinMode(ERROR_LED_3, OUTPUT);

  pinMode(RSTEP, OUTPUT);
  pinMode(RCC, OUTPUT);
  pinMode(LSTEP, OUTPUT);
  pinMode(LCC, OUTPUT);

  pinMode(SOLENOID_RELAY_PIN, OUTPUT);
  pinMode(VICTIM_LED, OUTPUT);
  pinMode(OBSTACLE_PIN, INPUT);

  Serial.begin(9600);
  while (!Serial)
  {
    delay(1);
  }
  
  //wait until the pi is ready
  while (Serial.available() > 0)
  {
    delay(1);
  }
  //clear the string buffer
  Serial.readStringUntil('\n');


  Wire.begin();

  TCA(1);
  if (!tofRight.begin())
  {
    digitalWrite(ERROR_LED_3, HIGH);
    while (true)
      delay(1);
  }
  TCA(2);
  if (!tofFront.begin())
  {
    digitalWrite(ERROR_LED_2, HIGH);
    while (true)
      delay(1);
  }

  TCA(3);
  if (!tofLeft.begin())
  {
    digitalWrite(ERROR_LED_3, HIGH);
    digitalWrite(ERROR_LED_2, HIGH);
    while (true)
      delay(1);
  }

  TCA(4);
  if (!tofBack.begin())
  {
    digitalWrite(ERROR_LED_1, HIGH);
    while (true)
      delay(1);
  }

  orientation = UP;


  if (!luxSensor.begin())
  {
    digitalWrite(ERROR_LED_1, HIGH);
    digitalWrite(ERROR_LED_2, HIGH);
    while (true)
      delay(1);
  }

  TCA(6);
  if (!tempRight.begin())
  {
    digitalWrite(ERROR_LED_1, HIGH);
    digitalWrite(ERROR_LED_2, HIGH);
    digitalWrite(ERROR_LED_3, HIGH);
    while (true)
      delay(1);
  }

  TCA(7);
  if (!tempLeft.begin())
  {
    digitalWrite(ERROR_LED_0, HIGH);
    while (true)
      delay(1);
  }

  TCA(6);
  AMBIENT_TEMP = tempRight.readAmbientTempC();
  //Serial.println("done with setup");

  delay(10000);
}

Map theMap;
void loop()
{
  //Serial.println("moving\n\n");
  theMap.move();
  delay(1000);
}
