#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <AccelStepper.h>
#include <Adafruit_MotorShield.h>
#include <Servo.h>

#define SERVO_PEN_UP_ANGLE 90 
#define SERVO_PEN_DOWN_ANGLE 20
#define WHEEL_BASE_INCHES 3.75

File myFile; // instance of a file

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *L_MOTOR = AFMS.getMotor(1);
Adafruit_DCMotor *R_MOTOR = AFMS.getMotor(2);

Servo myservo;

//our command string
#define COMMAND_SIZE 128
char Word[COMMAND_SIZE];
byte serial_count;
int no_data = 0;

//timer variables
unsigned long previousMillis = 0; 
unsigned long currentMillis = 0;

// our point structure to make things nice.
struct LongPoint {
  long x;
  long y;
  long z;
};

struct FloatPoint {
  float x;
  float y;
  float z;
};

FloatPoint current_units;
FloatPoint target_units;
FloatPoint current_angle;
FloatPoint target_angle;

boolean abs_mode = true;   //0 = incremental; 1 = absolute

float angleX, ratio, angleA, angleB, angle, radius, aX, aY, bX, bY, currentAngle, distance, feedrate, angleSteps, directionL, directionR, speedL, speedR, angleToMove, deltaAngle;

unsigned long intervalD, intervalA;

int steps, s, step, angle_steps;

bool Go = false;

void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(19200);
  
  // attaches the servo on pin 9 to the servo object
  myservo.attach(9);
 
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);

  // see if the card is present and can be initialized:
  if (!SD.begin(10)) 
    {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1) ;
    }
  delay(1000);
  Serial.println("card initialized.");
  delay(1000);
  // Open up the file we're going to log to!
  myFile = SD.open("csquares.txt");
  delay(1000);
  
  //other initialization.
  init_process_string();
  
  AFMS.begin(); // Start the top shield
   
  L_MOTOR->setSpeed(0);
  L_MOTOR->run(RELEASE);

  R_MOTOR->setSpeed(0);
  R_MOTOR->run(RELEASE);
}

void loop()
{
  char c;

  //read in characters if we got them.
  // button control here?
  if (myFile.available())
  {
    c = myFile.read();
    no_data = 0;
    
    //newlines are ends of commands.
    if (c != '\n')
    {
      Word[serial_count] = c;
      serial_count++;
    }
  }
  //mark no data.
  else
  {
    no_data++;
    delayMicroseconds(100);
  }

  //if theres a pause or we got a real command, do it
  if (serial_count && (c == '\n'|| no_data > 100))
  {
    //process our command!
    process_string(Word, serial_count);
   
    //clear command.
    init_process_string();
  } 
  if (no_data > 1000)
  {
  L_MOTOR->run(RELEASE);
  R_MOTOR->run(RELEASE);
  myservo.detach();
  }

}

//init our string processing
void init_process_string()
{
  //init our command
  for (byte i = 0; i < COMMAND_SIZE; i++)
    Word[i] = 0;
  serial_count = 0;

}

//Read the string and execute instructions
void process_string(char instruction[], int size)
{
  //the character / means delete block... used for comments and stuff.
  if (instruction[0] == '/')
  {
    Serial.println("ok");
    return;
  }

  //init baby!
  FloatPoint fp;
  fp.x = 0.0;
  fp.y = 0.0;
  fp.z = 0.0;

  FloatPoint fa;
  fa.x = 0.0;
  fa.y = 0.0;
  fa.z = 0.0;

  byte code = 0;;

  //what line are we at?
  if (has_command('G', instruction, size))
  {
    Serial.println(instruction);
  }

  //did we get a gcode?
  if (
    has_command('G', instruction, size) ||
    has_command('X', instruction, size) ||
    has_command('Y', instruction, size) ||
    has_command('Z', instruction, size)
  )
  {
    //which one?
    code = (int)search_string('G', instruction, size);

    // Get co-ordinates if required by the code type given
    switch (code)
    {
      case 0:
      case 1:
      case 2:
      case 3:
        if (abs_mode)
        {
          //we do it like this to save time. makes curves better.
          //eg. if only x and y are specified, we dont have to waste time looking up z.
          if (has_command('X', instruction, size))
          {
            fp.x = search_string('X', instruction, size);
            fa.x = fp.x;
          }
          else
          {
            fp.x = current_units.x;
            fa.x = 0;
          }

          if (has_command('Y', instruction, size))
          {
            fp.y = search_string('Y', instruction, size);
            fa.y = fp.y;
          }
          else
          {
            fp.y = current_units.y;
            fa.y = 0;
          }

          if (has_command('Z', instruction, size))
          {
            fp.z = search_string('Z', instruction, size);
            fa.z = fp.z;
          }
          else
          {
            fp.z = current_units.z;
            fa.z = 0;
          }
        }
        else
        {
          fp.x = search_string('X', instruction, size) + current_units.x;
          fp.y = search_string('Y', instruction, size) + current_units.y;
          fp.z = search_string('Z', instruction, size) + current_units.z;

        }

        break;
    }

    //do something!
    switch (code)
    {
      //Rapid Positioning
      //Linear Interpolation
      //these are basically the same thing.
      case 0:
      case 1:
        //set our target.
        set_target(fp.x, fp.y, fp.z);

        //do we have a set speed?
        if (has_command('G', instruction, size))
        {
          //adjust if we have a specific feedrate.
          if (code == 1)
          {
            //how fast do we move?
            feedrate = search_string('F', instruction, size);
          }
        }

        //find angle
        if (fa.x)
        {
          set_angle(fa.x, fa.y, fa.z);
          aX = (current_angle.x - target_angle.x);
          aY = (current_angle.y - target_angle.y);
          angleA = atan2(aY, aX) * (180 / M_PI);
          angle = (angleA - currentAngle);
          if (angle > 180)
          {
            angle = angle - 360;
          }
          if (angle < -180)
          {
            angle = 360 + angle;
          }
        }
        else
          angle = 0;


        angle = -angle; // make positive right turn, negative left turn

        angleSteps = angle * 5.1 ;//Multiplier;
        if (angleSteps > 0) // right turn
        {
          directionL = FORWARD;
          directionR = BACKWARD;
        }
        if (angleSteps < 0) // left turn
        {
          directionL = BACKWARD;
          directionR = FORWARD;
        }
        intervalA = abs(angleSteps);

        //find distance
        distance = sqrt(sq(current_units.x - target_units.x) + sq(current_units.y - target_units.y));
        intervalD = distance * 150; //Multiplier

        //raise/lower pen
        pen_up(current_units.z, target_units.z);

        //move!
        Serial.println(angle);
        Serial.println(intervalD);
        mover(intervalA, directionL, directionR, 50, 50);
        mover(intervalD, FORWARD, FORWARD, 50, 50);

        //set current angle to previous angle
        currentAngle = angleA;

        //set current unit to target unit
        current_units.x = target_units.x;
        current_units.y = target_units.y;
        current_units.z = target_units.z;

        current_angle.x = target_angle.x;
        current_angle.y = target_angle.y;

        break;

      //Clockwise arc
      case 2:
      //Counterclockwise arc
      case 3:
        set_target(fp.x, fp.y, fp.z);
        FloatPoint cent;

        // Centre coordinates are always relative
        cent.x = search_string('I', instruction, size) + current_units.x;
        cent.y = search_string('J', instruction, size) + current_units.y;


        aX = (current_units.x - cent.x);
        aY = (current_units.y - cent.y);
        bX = (fp.x - cent.x);
        bY = (fp.y - cent.y);

        if (code == 2) { // Clockwise
          angleA = atan2(bY, bX);
          angleB = atan2(aY, aX);
        } else { // Counterclockwise
          angleA = atan2(aY, aX);
          angleB = atan2(bY, bX);
        }

        // Make sure angleB is always greater than angleA
        // and if not add 2PI so that it is (this also takes
        // care of the special case of angleA == angleB,
        // ie we want a complete circle)
        if (angleB <= angleA) angleB += 2 * M_PI;
        angle = angleB - angleA;

        radius = sqrt(aX * aX + aY * aY);
        distance = radius * angle;

        //radians to degrees
        angleA = angleA * (180 / M_PI);
        angleB = angleB * (180 / M_PI);
        angle  = angle * (180 / M_PI);

        // Find the angle perpendicular to the starting cord.
        if (code == 2) { // Clockwise
          deltaAngle = angleB - 90;
        } else { // Counterclockwise
          deltaAngle = angleA - 90;
        }

        // find the diffence between the current Angle and the start position angle.
        angleToMove = currentAngle - deltaAngle;

        // get rid of extraneous moves
        if (
          (angleToMove < 361 && angleToMove > 359) ||
          (angleToMove > -361 && angleToMove < -359) ||
          (angleToMove == -180 || angleToMove == 180) ||
          (angleToMove > -1 && angleToMove < 1) 
        )
          angleToMove = 0;

        // something important? 
        if (angleToMove > 0)
            angleToMove = -(180 - angleToMove);
        


        angleSteps = angleToMove * 5.1; // multiplier
        intervalA = abs(angleSteps);



        //Serial.println(radius);
        //Serial.println(angleToMove);
        //Serial.println(currentAngle);
        //Serial.println(distance);
        //Serial.println(angle);
        //Serial.println(deltaAngle);
        //Serial.println(angleB);
        //Serial.println(angleA);


        if (angleSteps > 0) // right turn
        {
          directionL = FORWARD;
          directionR = BACKWARD;
        }
        if (angleSteps < 0) // left turn
        {
          directionL = BACKWARD;
          directionR = FORWARD;
        }

        //raise/lower pen
        pen_up(current_units.z, target_units.z);

        // move!! change angle to prepare for arc!
        mover(intervalA, directionL, directionR, 50, 50);


        if (code == 2) { // Clockwise
          if (radius > WHEEL_BASE_INCHES / 2)
          {
            directionL = FORWARD;
            directionR = FORWARD;
            speedL = 80;
            ratio = (radius - (WHEEL_BASE_INCHES / 2)) / (radius + (WHEEL_BASE_INCHES / 2));
            speedR = speedL * ratio;
            intervalD = distance * 120;
          }
          else
          {
            directionL = FORWARD;
            directionR = BACKWARD;
            speedR = 30;
            ratio = radius / (WHEEL_BASE_INCHES / 2);
            speedL = speedR / ratio;
            intervalD =  100 / ratio;
            Serial.println("small");
          }
          currentAngle = angleA - 90;

        } else { // Counterclockwise
          if (radius > WHEEL_BASE_INCHES / 2)
          {
            directionL = FORWARD;
            directionR = FORWARD;
            speedL = 30;
            ratio = (radius - (WHEEL_BASE_INCHES / 2)) / (radius + (WHEEL_BASE_INCHES / 2));
            speedR = speedL * ratio;
            intervalD =  distance * 150;
          }
          else
          {
            directionL = BACKWARD;
            directionR = FORWARD;
            speedL = 30;
            ratio = radius / (WHEEL_BASE_INCHES / 2);
            speedR = speedL / ratio;
            intervalD =  100 / ratio;
            Serial.println("small");
          }
          currentAngle = angleB - 90;
        }

        //Serial.println(angleSteps);
        //Serial.println(speedL);
        //Serial.println(speedR);
        //Serial.println(ratio);
        //Serial.println(currentAngle);
        //Serial.println(intervalD);

        //raise/lower the pen
        pen_up(current_units.z, target_units.z);

        //move!! draw arc!
        mover(intervalD, directionL, directionR, speedL, speedR);

        //set current units to target units
        current_units.x = target_units.x;
        current_units.y = target_units.y;
        current_units.z = target_units.z;

        break;

      //Dwell
      case 4:
        delay((int)search_string('P', instruction, size));
        break;

      //Absolute Positioning
      case 90:
        abs_mode = true;
        break;

      //Incremental Positioning
      case 91:
        abs_mode = false;
        break;

      //Set as home
      case 92:
        set_position(0.0, 0.0, 0.0);
        break;

      default:
        Serial.print("huh? G");
        Serial.println(code, DEC);
    }
  }

}
//look for the number that appears after the char key and return it
double search_string(char key, char instruction[], int string_size)
{
  char temp[10] = "";

  for (byte i = 0; i < string_size; i++)
  {
    if (instruction[i] == key)
    {
      i++;
      int k = 0;
      while (i < string_size && k < 10)
      {
        if (instruction[i] == 0 || instruction[i] == ' ')
          break;

        temp[k] = instruction[i];
        i++;
        k++;
      }
      return strtod(temp, NULL);
    }
  }

  return 0;
}

//look for the command if it exists.
bool has_command(char key, char instruction[], int string_size)
{
  for (byte i = 0; i < string_size; i++)
  {
    if (instruction[i] == key)
      return true;
  }

  return false;
}

void mover(unsigned long interval,int directionL,int directionR, float speedL, float speedR)
{
  
  L_MOTOR->setSpeed(speedL);
  R_MOTOR->setSpeed(speedR);
  previousMillis = millis();
  timer(interval);
  while (Go == true) 
  {
    //Serial.println("moving!!!");
    timer(interval);
    L_MOTOR->run(directionL);
    R_MOTOR->run(directionR);   
  }
   L_MOTOR->run(RELEASE);
   R_MOTOR->run(RELEASE);
 
}

void pen_up(int currentZ, int targetZ)
{

  if (targetZ > currentZ)
  {
    delay(400);
    myservo.write(SERVO_PEN_UP_ANGLE);
    delay(400);

    Serial.println("pen up!!");
  }
  if (targetZ < currentZ)
  {
    delay(400);
    myservo.write(SERVO_PEN_DOWN_ANGLE);
    delay(400);

    Serial.println("pen down!!");
  }
}

void set_target(float x, float y, float z)
{
  target_units.x = x;
  target_units.y = y;
  target_units.z = z;
}

void set_angle(float x, float y, float z)
{
  target_angle.x = x;
  target_angle.y = y;
  target_angle.z = z;
}

void set_position(float x, float y, float z)
{
  current_units.x = x;
  current_units.y = y;
  current_units.z = z;
}

bool timer(unsigned long interval)
{
  currentMillis = millis();
  if (currentMillis - previousMillis <= interval) {
    Go = true;
  }
  else {;
    Go = false;
    previousMillis = currentMillis;
  }
}
