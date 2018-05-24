#include <Balboa32U4.h>
#include <QTRSensors.h>
#include <avr/eeprom.h>

Balboa32U4ButtonA buttonA;
Balboa32U4ButtonB buttonB;
Balboa32U4ButtonC buttonC;
Balboa32U4Buzzer buzzer;
Balboa32U4Motors motors;
QTRSensorsRC lineSensors;
QTRSensorsRC sideSensors;

unsigned char lineSensorPins[]  = { A4, A3, A2 };
const unsigned char NumLineSensors = sizeof(lineSensorPins);
unsigned int lineSensorValues[NumLineSensors];

unsigned char sideSensorPins[]  = { 5, 12 };
const unsigned char NumSideSensors = sizeof(sideSensorPins);
unsigned int sideSensorValues[NumSideSensors];

unsigned int EEMEM storedMinimumOnLine[NumLineSensors];
unsigned int EEMEM storedMaximumOnLine[NumLineSensors];
unsigned int EEMEM storedMinimumOnSide[NumSideSensors];
unsigned int EEMEM storedMaximumOnSide[NumSideSensors];

void setup()
{
  bool doCalibrate = false;
  if (!checkStoredCal() || buttonC.isPressed()) { doCalibrate = true; }

  lineSensors.init(lineSensorPins, NumLineSensors, 2000, QTR_NO_EMITTER_PIN);
  sideSensors.init(sideSensorPins, NumSideSensors, 2000, QTR_NO_EMITTER_PIN);

  if (doCalibrate) { calibrate(); }

  lineSensors.calibrate(); // allocate storage
  sideSensors.calibrate(); // allocate storage
  readStoredCal();

  // Play a little welcome song
  buzzer.play(">g32>>c32");

  // Wait for button A to be pressed and released.
  buttonA.waitForButton();

  buzzer.play("c2");
  while(buzzer.isPlaying());
}

#define sign(x) ((x) < 0 ? -1 : 1)

void loop()
{
  const uint16_t MaxSpeed = 300;
  static int16_t lastError = 0;

  const uint8_t PrevErrorCount = 16;
  static int16_t prevError[PrevErrorCount] = {};
  static int32_t avgError;
  
  bool onLine = true;

  const uint16_t TurnPreTime = 250; // min time after marker for turn
  const uint16_t TurnTimeout = 1500; // max time after marker for turn
  static int8_t upcomingTurn = 0;
  static uint16_t turnMarkerTime = 0;
  
  // Get the position of the line.  Note that we *must* provide
  // the "lineSensorValues" argument to readLine() here, even
  // though we are not interested in the individual sensor
  // readings.
  int16_t position = readLineSensors(&onLine);

  // Our "error" is how far we are away from the center of the
  // line, which corresponds to position 2000.
  int16_t error = position - 1000;

  if (onLine)
  {
    avgError = 0;
 
    for (uint8_t i = (PrevErrorCount - 1); i > 0; i--)
    {
      prevError[i] = prevError[i - 1];
      avgError += prevError[i];
    }
    
    prevError[0] = error;
    avgError += error;
    avgError /= PrevErrorCount;
  }
  else if (abs(avgError) < 300)
  {
    buzzer.playNote(NOTE_A(4), 100, 15);
    error = 0;
  }
  
  const uint16_t SideThreshold = 800;

  sideSensors.readCalibrated(sideSensorValues);
  
  if (upcomingTurn == 0)
  {
    if (sideSensorValues[0] > SideThreshold && sideSensorValues[1] <= SideThreshold)
    {
      buzzer.playNote(NOTE_C(6), 100, 15);
      upcomingTurn = -1;
      turnMarkerTime = millis();
    }
    else if (sideSensorValues[1] > SideThreshold && sideSensorValues[0] <= SideThreshold)
    {
      buzzer.playNote(NOTE_G(6), 100, 15);
      upcomingTurn = 1;
      turnMarkerTime = millis();
    }
  }
  else if ((uint16_t)(millis() - turnMarkerTime) > TurnPreTime)
  {
    if ((sideSensorValues[0] > SideThreshold) && (sideSensorValues[1] > SideThreshold))
    {
      // in an intersection
      buzzer.playNote(NOTE_C(7), 100, 15);
      if (upcomingTurn == -1)
      {
        motors.setSpeeds(-100, 300);
      }
      else
      {
        motors.setSpeeds(300, -100);
      }
      delay(750);
      
      do
      {
        readLineSensors(&onLine);  
      }
      while (!onLine);
      
      upcomingTurn = 0;
    }
    else if ((uint16_t)(millis() - turnMarkerTime) > TurnTimeout)
    {
      // turn timed out
      buzzer.playNote(NOTE_E(5), 100, 15);
      upcomingTurn = 0;
    }
  }

  
  // Get motor speed difference using proportional and derivative
  // PID terms (the integral term is generally not very useful
  // for line following).  Here we are using a proportional
  // constant of 1/4 and a derivative constant of 6, which should
  // work decently for many Zumo motor choices.  You probably
  // want to use trial and error to tune these constants for your
  // particular Zumo and line course.
  int16_t speedDifference = error*5/8 + (error - lastError)*9/2 ;
  
  lastError = error;
  

  // Get individual motor speeds.  The sign of speedDifference
  // determines if the robot turns left or right.
  int16_t leftSpeed = (int16_t)MaxSpeed + speedDifference;
  int16_t rightSpeed = (int16_t)MaxSpeed - speedDifference;

  // Constrain our motor speeds to be between 0 and maxSpeed.
  // One motor will always be turning at maxSpeed, and the other
  // will be at maxSpeed-|speedDifference| if that is positive,
  // else it will be stationary.  For some applications, you
  // might want to allow the motor speed to go negative so that
  // it can spin in reverse.
  leftSpeed = constrain(leftSpeed, -200, (int16_t)MaxSpeed);
  rightSpeed = constrain(rightSpeed, -200, (int16_t)MaxSpeed);

  motors.setSpeeds(leftSpeed, rightSpeed);

/*  for (uint8_t i = 0; i < NumLineSensors; i++)
  {
    Serial.print(lineSensorValues[i]);
    Serial.print('\t');
  }
  Serial.print('\t');
  Serial.print(error);
  Serial.print('\t');
  Serial.print(leftSpeed);
  Serial.print('\t');
  Serial.print(rightSpeed);
  Serial.println();*/
}

void calibrate()
{
  buzzer.play("g16>c16");
  
  buttonC.waitForRelease();
  buttonC.waitForButton();
  buzzer.play(">e16");
    
  while(!buttonC.getSingleDebouncedRelease())
  {
    lineSensors.calibrate();
    sideSensors.calibrate();
  }
  
  writeStoredCal();

  buzzer.play(">c16g16");  
  
  // loop and wait for reset
  while(1) {}
}

bool checkStoredCal()
{
  return eeprom_read_word(&storedMaximumOnLine[NumLineSensors - 1]) != 0xFFFF;
}


bool writeStoredCal()
{
  for (uint8_t i = 0; i < NumLineSensors; i++)
  {
    eeprom_write_word(&storedMinimumOnLine[i], lineSensors.calibratedMinimumOn[i]);
    eeprom_write_word(&storedMaximumOnLine[i], lineSensors.calibratedMaximumOn[i]);
  }

  for (uint8_t i = 0; i < NumSideSensors; i++)
  {
    eeprom_write_word(&storedMinimumOnSide[i], sideSensors.calibratedMinimumOn[i]);
    eeprom_write_word(&storedMaximumOnSide[i], sideSensors.calibratedMaximumOn[i]);
  }
}

bool readStoredCal()
{
  for (uint8_t i = 0; i < NumLineSensors; i++)
  {
    lineSensors.calibratedMinimumOn[i] = eeprom_read_word(&storedMinimumOnLine[i]);
    lineSensors.calibratedMaximumOn[i] = eeprom_read_word(&storedMaximumOnLine[i]);
  }

  for (uint8_t i = 0; i < NumSideSensors; i++)
  {
    sideSensors.calibratedMinimumOn[i] = eeprom_read_word(&storedMinimumOnSide[i]);
    sideSensors.calibratedMaximumOn[i] = eeprom_read_word(&storedMaximumOnSide[i]);
  }
}

int16_t readLineSensors(bool * onLine)
{
  uint32_t avg; // this is for the weighted total, which is long
                // before division
  uint16_t sum; // this is for the denominator which is <= 64000

  static int16_t lastValue = 0;

  lineSensors.readCalibrated(lineSensorValues);

  avg = 0;
  sum = 0;
  *onLine = false;
  
  for (uint8_t i = 0; i < NumLineSensors; i++)
  {
    int value = lineSensorValues[i];

    // keep track of whether we see the line at all
    
    if (value > 200) { *onLine = true; }

    // only average in values that are above a noise threshold
    if (value > 50)
    {
      avg += (int32_t)value * (i * 1000);
      sum += value;
    }
  }

  if(*onLine == false)
  {
    // If it last read to the left of center, return 0.
    if(lastValue < (NumLineSensors - 1) * 500)
    {
      return 0;
    }
    // If it last read to the right of center, return the max.
    else
    {
      return (NumLineSensors - 1) * 1000;
    }
  }

  lastValue = avg / sum;

  return lastValue;
}
