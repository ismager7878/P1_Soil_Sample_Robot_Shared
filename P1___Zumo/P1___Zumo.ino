/*
 * read girscope data
 * Carlos Gomez Cubero
 * 
 * The gyroscope is calibrated at the beginning "turnSensorSetup();"
 * The calibration initialize the X, Y and Z axis of the girscopoe so place it 
 * in a flat surface.
 * 
 * When the yellow LED is off (few instants after start) you can press the A button to 
 * reset the rotation on the Z axis to zero and start measuring from there. The rotation
 * is sent thru the serial port.
 * 
 * 
 * Most of the code is extracted from pololu zumo32U4 example RotationResist
 * https://github.com/pololu/zumo-32u4-arduino-library/blob/master/examples/RotationResist/RotationResist.ino
 */

#include <Wire.h>
#include <Zumo32U4.h>

#define PI 3.14159265359
#define left 270
#define right 90


Zumo32U4OLED oled;
//is was here
Zumo32U4IMU imu;
Zumo32U4ButtonA buttonA;
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;


/* turnAngle is a 32-bit unsigned integer representing the amount
the robot has turned since the last time turnSensorReset was
called.  This is computed solely using the Z axis of the gyro, so
it could be inaccurate if the robot is rotated about the X or Y
axes.

Our convention is that a value of 0x20000000 represents a 45
degree counter-clockwise rotation.  This means that a uint32_t
can represent any angle between 0 degrees and 360 degrees.  If
you cast it to a signed 32-bit integer by writing
(int32_t)turnAngle, that integer can represent any angle between
-180 degrees and 180 degrees. */
uint32_t turnAngle = 0;

// turnRate is the current angular rate of the gyro, in units of
// 0.07 degrees per second.
int16_t turnRate;

// This is the average reading obtained from the gyro's Z axis
// during calibration.
int16_t gyroOffset;

// This variable helps us keep track of how much time has passed
// between readings of the gyro.
uint16_t gyroLastUpdate = 0;

float wheelCirc = 12.7930476960;
double wheelCircCal;

double realDistance = 94.5;
float realTicks = 7422;

int speed = 100;

const int threshold = 450;  // the threshold variable determens when the line sensors hit the line
#define NUM_SENSORS 3  // this determens that we only use 3 linesensors, all er in the front
uint16_t lineSensorValues[NUM_SENSORS];
int stage = 0;

float lastDistance = 0;

const float factor = 1.007;
const float revFactor = 0.99;


//Obstacle dectection
const uint8_t sensorThreshold = 6;
const int evasionLenght1 = 30;
const int evasionLenght2 = 10;
const int totalEvasion = evasionLenght1 + evasionLenght2;

// gridVariables
int ySize = 0;                             // ySize variable stores the ySize of the "field"
int xSize = 0;                             // xSize variable stores the xSize of our "field"
const int gridSpacing = 30;                 // Spacing between main grid points
const int offsetSpacing = gridSpacing / 2;  // Offset for staggered rows (half of gridSpacing)
const int sampleOffset = 5;
int centeringOffsetX;
int centeringOffsetY;  // Max random offset distance for subsampling (e.g., within 10 feet)
const int zumoOffset = 5;

// Variables for grid points and subsamples
const int maxNumSamplePoints = 35;
const int numSamplesPerPoint = 3;  // Number of subsamples per grid point

int numSamplesPoints = 0;

struct Point {
  int x;
  int y;
  int sample = 0;

  void takeSample(){
    sample = random(1,11);
  }

  void setXY(int newX, int newY) {
    x = newX;
    y = newY;
  }
};


struct SamplePoint {
  Point point;
  Point subSamples[numSamplesPerPoint];
  float sampleValue = 0;

  void calcSample() {
    int temp = 0;
    for (int i = 0; i < numSamplesPerPoint; i++) {
      temp += subSamples[i].sample;
    }
    sampleValue = temp/numSamplesPerPoint;
  }

  void setValues(Point mainPoint, Point newSubSamples[numSamplesPerPoint]) {
    point = mainPoint;
    for (int i = 0; i < numSamplesPerPoint; i++) {
      subSamples[i] = newSubSamples[i];
    }
  }
};

struct vec2D {
  float a;
  float b;

  float length() {
    return sqrt(pow(a, 2) + pow(b, 2));
  }
};

//Navigation data
int origoAngle = 0;
int targetAngle = 0;

vec2D currentPos = {zumoOffset,0};

SamplePoint samplePoints[maxNumSamplePoints];


void setup() {
  Serial.begin(9600);
  calibrateDistance();
  lineSensors.initThreeSensors();
  proxSensors.initFrontSensor();
  turnSensorSetup();

  delay(500);

  turnSensorReset();
  MeasurePerimmiter();
  delay(20);
  generateGrid();
  delay(500);
  navigateSamples();
  printSamples();
}

void printToOLED(String a = "", String b = "") {
  oled.clear();
  oled.gotoXY(0, 0);
  oled.print(a);
  oled.gotoXY(0, 1);
  oled.print(b);
}

void awaitStart() {
  printToOLED("Press A", "To Start");
  while (!buttonA.getSingleDebouncedRelease()) {
  }
  oled.clear();
}

void setupSerial() {
  while (!Serial);
  delay(1000);
}

void generateGrid() {
  randomSeed(millis());
  if(xSize % gridSpacing == 0){
    centeringOffsetX = 0;
  }else if (xSize % gridSpacing < offsetSpacing) {
    //Serial.println("Even");
    centeringOffsetX = gridSpacing - (xSize % gridSpacing + gridSpacing) / 2;
  } else {
    //Serial.println("Uneven");
    centeringOffsetX = offsetSpacing - (xSize % gridSpacing + offsetSpacing) / 2;
    //
  }
  centeringOffsetY = gridSpacing - ((ySize % gridSpacing == 0 ? gridSpacing : ySize % gridSpacing) + gridSpacing)  / 2;

  //Serial.println((String)centeringOffsetX + ", " + (String)centeringOffsetY);
  bool offsetRow = false;                                   // Flag to alternate rows
  for (int y = gridSpacing; y < ySize; y += gridSpacing) {  // Start at gridSpacing, end at yMax - gridSpacing
    for (int x = (offsetRow ? offsetSpacing : gridSpacing); x < xSize; x += gridSpacing) {
      // Generate subsamples for this grid point
      Point mainPoint;
      mainPoint.setXY(x - centeringOffsetX, y - centeringOffsetY);
      generateSubsamples(mainPoint);
      numSamplesPoints++;
    }
    offsetRow = !offsetRow;  // Alternate rows
  }
  filterSamplePoints();
}

void generateSubsamples(Point mainPoint) {
  Point subSamples[numSamplesPerPoint];

  for (int i = 0; i < numSamplesPerPoint; i++) {
    // Generate random offsets for subsampling
    int offsetX = random(-sampleOffset, sampleOffset + 1);
    int offsetY = random(-sampleOffset, sampleOffset + 1);

    // Ensure subsamples stay within bounds
    int sampleX = constrain(mainPoint.x + offsetX, 0, xSize);
    int sampleY = constrain(mainPoint.y + offsetY, 0, ySize);
    subSamples[i].setXY(sampleX, sampleY);
  }
  samplePoints[numSamplesPoints].setValues(mainPoint, subSamples);
}

void filterSamplePoints() {
  int filterIndex = 0; 
  for (int i = 0; i < numSamplesPoints; i++) {
    // Check if the main point's x and y are not both 0
    if (samplePoints[i].point.x != 0 || samplePoints[i].point.y != 0) {

      samplePoints[filterIndex++] = samplePoints[i];
    }
  }
  // Update the count of valid sample points
  numSamplesPoints = filterIndex;
}

void printSamples() {
  setupSerial();
  
  String mapColumns = "CDEFGHIJK";
  String tableColumns = "LMNOPQRSTUVW";

  int mapRow = 2;
  int mapCol = 1;

  int tableRow = 14;
  for (int i = 0; i < numSamplesPoints; i++) {
    for(int j = 0; j < numSamplesPerPoint; j++){
      samplePoints[i].subSamples[j].takeSample();
    }
    samplePoints[i].calcSample();

    if(i != 0){
      if(samplePoints[i].point.y != samplePoints[i-1].point.y){
        mapRow++;
        mapCol = (mapCol%2 == 0 ? 1 : 0);
      }
    }

    Serial.print("CELL,SET,");
    Serial.print(mapColumns[mapCol] + (String)mapRow + ",");
    Serial.println((String)samplePoints[i].sampleValue + ",");
    Serial.println("");
    for(int j = 0; j < 3; j++){
      Serial.print("CELL,SET,");
      Serial.print(tableColumns[j] + (String)tableRow + ",");
      switch(j){
        case 0:
          Serial.println((String)samplePoints[i].point.x + ",");
          break;
        case 1:
          Serial.println((String)samplePoints[i].point.y + ",");
          break;
        case 2:
          Serial.println((String)samplePoints[i].sampleValue + ",");
          break;
      }
      delay(100);
    }
    for (int j = 0; j < numSamplesPerPoint; j++) {
      for(int k = 0; k < 3; k++){
        Serial.print("CELL,SET,");
        Serial.print(tableColumns[3*(j+1) + k] + (String)tableRow + ",");
        switch(k){
        case 0:
          Serial.println((String)samplePoints[i].subSamples[j].x + ",");
          break;
        case 1:
          Serial.println((String)samplePoints[i].subSamples[j].y + ",");
          break;
        case 2:
          Serial.println((String)samplePoints[i].subSamples[j].sample + ",");
          break;
        }
      }
      delay(100);
    }
    tableRow++;
    mapCol += 2;
    Serial.println();
  }
}



// Prints a line with all the sensor readings to the serial
// monitor.
void printReadingsToSerial() {
  char buffer[80];
  sprintf(buffer, "%4d %4d %4d\n",
          lineSensorValues[0],
          lineSensorValues[1],
          lineSensorValues[2]);
  //Serial.print(buffer);
}

// the uncalibrated line sensor reading are between 0 (very bright) and 2000 (very dark)
void readLineSensors() {
  lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);
  printReadingsToSerial();
}

void calibrateDistance() {
  wheelCircCal = realDistance / (realTicks / 900);
}

uint32_t getTurnAngleInDegrees() {
  turnSensorUpdate();
  return (((uint32_t)turnAngle >> 16) * 360) >> 16;
}

void loop() {
  printToOLED((String)xSize, (String)ySize);
}

bool lineNotDetected() {
  readLineSensors();
  return !(lineSensorValues[1] > threshold);
}


void driveWhile(bool (&con)(), int direction) {
  ResetDistance();
  unsigned long currentTime = millis();
  motors.setSpeeds(speed, speed);
  while ((millis() - currentTime) < 1000) {
    updateMotorSpeeds(direction);
  }
  while (con()) {

    updateMotorSpeeds(direction);
  }
  lastDistance = 0;
  stop();
}

void driveMs(int time, int direction) {
  ResetDistance();
  unsigned long currentTime = millis();
  motors.setSpeeds(speed, speed);
  while ((millis() - currentTime) < time) {
    updateMotorSpeeds(direction);
  }
  lastDistance = 0;
  stop();
}

float getCurrentDistance(){
  return getDistance() + lastDistance;
}
void driveDistance(int distance, int direction, bool obs = true){
  ResetDistance();
  motors.setSpeeds(speed, speed);
  while (getCurrentDistance() < distance) {
    updateMotorSpeeds(direction);
    if(obs){
      obstacleDetection();
    }
  }
  lastDistance = 0;
  stop();
}

void updateMotorSpeeds(int direction) {

  int countLeft = encoders.getCountsLeft() * direction;
  int countRight = encoders.getCountsRight() * direction;

  if (countLeft > 30000 || countRight > 30000) {
    lastDistance = getDistance();
    encoders.getCountsAndResetLeft();
    encoders.getCountsAndResetRight();
  }

  int currentSpeedRight = speed + (countLeft - countRight * (direction > 0 ? factor : revFactor));
  motors.setSpeeds(speed * direction, currentSpeedRight * direction);
}
// This is the new function that measures the perimiter, this function should both save a
// lenght and xSize variable. But it should also be able to drive to the corner.
void MeasurePerimmiter() {
  //Drive backward until line
  driveWhile(lineNotDetected, -1);
  delay(500);

  //Drive drive forward until line and measure lenght
  ResetDistance();
  driveWhile(lineNotDetected, 1);
  setDistance('l');
  delay(500);

  //Drive away from line and turn and revserse to other line
  driveMs(1000, -1);
  turnByAngle(left);
  driveWhile(lineNotDetected, -1);
  delay(500);

  //Drive toward line and measure xSize
  ResetDistance();
  driveWhile(lineNotDetected, 1);
  setDistance('w');
  delay(500);

  //Find corner
  findCorner();
}

void findCorner() {
  turnByAngle(left);
  printToOLED("Finding Corner");
  delay(500);
  driveWhile(lineNotDetected, -1);
}

void stop() {
  motors.setSpeeds(0, 0);
}

void setDistance(char value) {
  int distance = getCurrentDistance();
  switch (value) {
    case 'l':
      ySize = distance;
      break;
    case 'w':
      xSize = distance;
      break;
  }
}

void obstacleDetection()
{
  proxSensors.read();
  uint8_t leftValue = proxSensors.countsFrontWithLeftLeds();
  uint8_t rightValue = proxSensors.countsFrontWithRightLeds();
  printToOLED((String)leftValue, (String)rightValue);
  if (leftValue >= sensorThreshold)
  {
    stop();
    avoidObstacle('r');
  }
  else if (rightValue >= sensorThreshold) 
  {
    stop();
    avoidObstacle('l');
  }

}

void avoidObstacle(char dir){
  //Calculate angles and catherters --------------------------
  float currentDistance = getCurrentDistance();

  printToOLED("Now:", (String)currentDistance);
  delay(1000);

  int angleB = turnAwayFromObstacleAndGetAngle(dir);
  delay(500);

  float sideC =  evasionLenght1 /cos(angleToRad(angleB));

  float sideB = sqrt(pow(totalEvasion,2)+pow(sideC,2)-2*totalEvasion*sideC*cos(angleToRad(angleB)));

  int angleC = round(inDegrees(acos(evasionLenght2/sideB)));

  int angleA = 180 - angleB - angleC;

  printToOLED("A: "+ (String)angleA, "B: " + (String)angleC);
  delay(1000);
  printToOLED((String)sideC);
  //Naviagte triangle----------------------------------------------

  //Kører den første kortside af trekanten
  driveDistance(sideC, 1, false);
  delay(500);

  //Drejer vinkel A og resetencoders for at kunne køre den næste kortside
  if(dir == 'r'){
    turnByAngle(180 - angleA);
  }else{
    turnByAngle(360-(180-angleA));
  }

  printToOLED((String)sideB);

  delay(500);
  //Kører den næste kortside af trekanten
  driveDistance(sideB, 1, false);
  delay(500);
     
  //Drejer tilbage til den vinkel den kørte før den fandt en forhindring
  turnByAngle(dir == 'r' ? 360 - angleC : angleC);
  delay(500);

  ResetDistance();
  lastDistance = currentDistance + totalEvasion;
  printToOLED("left:", (String)lastDistance);
  delay(1000);
}

int turnAwayFromObstacleAndGetAngle(char dir)
{
  turnSensorReset();
  proxSensors.read();
  while((dir == 'r' ? proxSensors.countsFrontWithRightLeds() : proxSensors.countsFrontWithLeftLeds()) >= 4 )//sensorThreshold)
  {
    //Turn away from obstacle
    if(dir == 'r'){
      motors.setSpeeds(speed, -speed);
    }else{
      motors.setSpeeds(-speed, speed);
    }
    turnSensorUpdate();
    //This block of code is for the proxSensors
    proxSensors.read();
    //delay(10);
  }
  stop();
  return dir == 'r' ? 360 - getTurnAngleInDegrees() : getTurnAngleInDegrees();;
}

float angleToRad(float angle)
{
  return angle * (PI / 180.0); // Konverter vinkel A til radianer
}
//------------------------------------------------------------------------------------------------------------------------------------------

float inDegrees(float rads) {
  return rads * 180 / PI;
}

void turnByAngle(int angle) {
  printToOLED("Angle:", (String)angle);
  turnSensorReset();
  if(angle > 180){
    motors.setSpeeds(speed, -speed);
    delay(20);
    while(getTurnAngleInDegrees() > angle){

    }
  }else{
    motors.setSpeeds(-speed, speed);
    delay(20);
    while(getTurnAngleInDegrees() < angle){

    }
  }
  stop();
}

vec2D getTravelVector(vec2D target){
  int newX = round(target.a - currentPos.a);
  int newY = round(target.b - currentPos.b);

  return {newX, newY};
}

vec2D getVectorData(float x, float y) {
  vec2D target = {x,y};
  // ZumoData er de værdier vores Zumo skal køre efter. Felt (1, 1) er vektorens længde. Felt (2, 1) er Gyroens grader.
  vec2D vector = getTravelVector(target);

  // Her finder vi længden af vektoren, ved at tage trække zumoens nuværende position fra og herfeter tage kvadratroden af x^2 + y^2.
  float vectorLength = vector.length();

  vec2D yVector = { 0, 1 };

  float yVectorLength = yVector.length();
  // Her beregner vi vinklen mellem vores vektor og y-aksen. For at kunne dette skal vi flytte vores vektor ned i origo (0, 0).
  // Dette gøres ved at trække den nuværende Zumo position fra vores vektor vi gerne vil følge.

  // Nu kan vi beregne vinklen mellem vores vektor og vektoren for y-aksen. For at gøre det mere overskuelig har vi delt regnestykket op. commandoen "acos" virker måske ligesom cos^-1.
  float dotProduct = (yVector.a * vector.a) + (yVector.b * vector.b);

  // Her beregner vi vinklen mellem vektoren og y-aksen.
  float angle = acos(dotProduct / (vectorLength * yVectorLength));

  // Da vores gyro er positiv mod urets retning, skal vi trække vores resulat fra 360 grader.
  float vectorAngle = vector.a < 0 ? inDegrees(angle) : 360 - inDegrees(angle);

  // De to resultater indsættes i en matrix og herefter retuneres.
  currentPos = target;

  return {vectorLength, vectorAngle};;
}

void navigateSamples() {
  origoAngle = 0;
  for (int i = 0; i < numSamplesPoints; i++) {
    SamplePoint nextPoint = samplePoints[i];
    goToPoint(nextPoint.point.x, nextPoint.point.y);

    for(int j = 0; j < numSamplesPerPoint; j++){
      samplePoints[i].subSamples[j].takeSample();
    }

    samplePoints[i].calcSample();
    delay(500);
    //Serial.println("Next point:" + (String)navVector.a + ", " + (String)navVector.b);
  }
  goToPoint(0,0);
}

void goToPoint(int x, int y){
  vec2D navVector = getVectorData(x, y);
  int turnAngle = navVector.b - origoAngle;
  if(turnAngle < 0){
    turnAngle = 360 + turnAngle;
  }
  turnByAngle(turnAngle);
  driveDistance(navVector.a, 1);
  origoAngle = navVector.b;
}
//------------------------------------------------------------------------------------------------------------------------------------------

/* This should be called in setup() to enable and calibrate the
gyro.  It uses the oled, yellow LED, and button A.  While the oled
is displaying "Gyro cal", you should be careful to hold the robot
still.

The digital zero-rate level of the gyro can be as high as
25 degrees per second, and this calibration helps us correct for
that. */
float getDistance() {
  int countsL = encoders.getCountsLeft();
  int countsR = encoders.getCountsRight();

  float distanceL = countsL / 900.0 * wheelCircCal;
  float distanceR = countsR / 900.0 * wheelCircCal;

  //Serial.println("Move my wheels.... counts L " + (String)countsL + " R " + (String)countsR);
  //Serial.println("Move my wheels.... cm L " + (String)distanceL + " R " + (String)distanceR);

  //delay(1000);
  return (distanceL + distanceR) / 2;
}

void ResetDistance() {
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  lastDistance = 0;
}


void turnSensorSetup() {
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.configureForTurnSensing();

  oled.clear();
  oled.print(F("Gyro cal"));

  // Turn on the yellow LED in case the oled is not available.
  ledYellow(1);

  // Delay to give the user time to remove their finger.
  delay(500);

  // Calibrate the gyro.
  int32_t total = 0;
  for (uint16_t i = 0; i < 1024; i++) {
    // Wait for new data to be available, then read it.
    while (!imu.gyroDataReady()) {}
    imu.readGyro();

    // Add the Z axis reading to the total.
    total += imu.g.z;
  }
  ledYellow(0);
  gyroOffset = total / 1024;

  // Display the angle (in degrees from -180 to 180) until the
  // user presses A.
  oled.clear();
  turnSensorReset();
  while (!buttonA.getSingleDebouncedRelease()) {
    turnSensorUpdate();
    oled.gotoXY(0, 0);
    // do some math and pointer magic to turn angle in seconds to angle in degrees
    oled.print((((int32_t)turnAngle >> 16) * 360) >> 16);
    oled.print(F("   "));
  }
  oled.clear();
}

// This should be called to set the starting point for measuring
// a turn.  After calling this, turnAngle will be 0.
void turnSensorReset() {
  gyroLastUpdate = micros();
  turnAngle = 0;
}

// Read the gyro and update the angle.  This should be called as
// frequently as possible while using the gyro to do turns.
void turnSensorUpdate() {
  // Read the measurements from the gyro.
  imu.readGyro();
  turnRate = imu.g.z - gyroOffset;

  // Figure out how much time has passed since the last update (dt)
  uint16_t m = micros();
  uint16_t dt = m - gyroLastUpdate;
  gyroLastUpdate = m;

  // Multiply dt by turnRate in order to get an estimation of how
  // much the robot has turned since the last update.
  // (angular change = angular velocity * time)
  int32_t d = (int32_t)turnRate * dt;

  // The units of d are gyro digits times microseconds.  We need
  // to convert those to the units of turnAngle, where 2^29 units
  // represents 45 degrees.  The conversion from gyro digits to
  // degrees per second (dps) is determined by the sensitivity of
  // the gyro: 0.07 degrees per second per digit.
  //
  // (0.07 dps/digit) * (1/1000000 s/us) * (2^29/45 unit/degree)
  // = 14680064/17578125 unit/(digit*us)
  turnAngle += (int64_t)d * 14680064 / 17578125;
}