#include <Wire.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h>
#include <MatrixMath.h>


bool TestReset = false;
int MasterFaultTime = 5 * 60 * 1000;
int lastMasterCom = 0;
int resets = 0;
int manualTimeoutS = 10 * 1000;
bool imuWorking = true;


class commandBuffer {
  public:
    int commandStack[200][2];
    int openSpot;
    commandBuffer() {
      commandStack[200][2] = { -1};
      openSpot = 0;
    }
    void print() {
      int i = 0;
      Serial.print("cBuf = [");
      int endT = millis() + manualTimeoutS;
      while (i < 200 && millis() < endT) {
        if (commandStack[i][0] == -1 && commandStack[i][1] == -1) {
          break;
        }
        Serial.print(commandStack[i][0]);
        Serial.print(":");
        Serial.print(commandStack[i][1]);
        Serial.print("|");
        i++;
      }
      Serial.println("]");
    }
};
commandBuffer cBuf;

class floatTuple
{
  public:
    float x;
    float y;
    float z;

    floatTuple(float a, float b, float c) {
      x = a;
      y = b;
      z = c;
    }
    void print() {
      Serial.print(x); Serial.print(" ");
      Serial.print(y); Serial.print(" ");
      Serial.print(z); Serial.println(" ");
    }
};
floatTuple MagVectorError = floatTuple(1, 1, 1);

////////////////////////////////
////////////////////////////////
////////////////////////////////

//IMU

////////////////////////////////
////////////////////////////////
////////////////////////////////
////////////////////////////////

////Parser Functions
void buildBuffer(String com)
{
  int commandData;
  int commandType;
  String comRemaining = com;
  bool loop = true;
  while (loop) {
    commandType = (com.substring(0, com.indexOf(","))).toInt();
    commandData = (com.substring(com.indexOf(",") + 1, com.indexOf("."))).toInt();
    cBuf.commandStack[cBuf.openSpot][0] = commandType;
    cBuf.commandStack[cBuf.openSpot][1] = commandData;
    if (com.indexOf(".") == com.length() - 1) {
      loop = false;
      Serial.println("Finished Adding Commands");
    } else {
      com = com.substring(com.indexOf(".") + 1);
    }
    cBuf.openSpot++;
  }
}

boolean isInputValid(String input) {
  //Check if incoming command is valid
  int lastPunc = 0; //1 if ",", 2 if ".", 0 Otherwise
  bool valid = true;
  int q = 0;
  int l = input.length();
  int endT = manualTimeoutS + millis();
  while (q < l) {
    char currentChar = input[q];
    q++;

    if (millis() > endT) {
      valid = false;
      break;
    }

    if (isPunct(currentChar)) {
      if (currentChar == (',')) {
        //Check if last was a period
        //Serial.println("Comma Found");
        if (lastPunc == 0 || lastPunc == 2) {
          //Serial.println("Comma OK");
          lastPunc = 1;
        } else {
          //Serial.println("2 Commas");
          valid = false;
          break;
        }
      } else if (currentChar == ('.')) {
        //Serial.println("Period Found");
        if (lastPunc == 1) {
          //Serial.println("Period ok");
          lastPunc = 2;
        } else {
          //Serial.println("2 Periods or No prior comma");
          valid = false;
          break;
        }
      } else {
        //Serial.println("Invalid Punc");
        valid = false;
        break;
      }
    } else if (isAlpha(currentChar)) {
      //Serial.println("Alpha");
      valid = false;
      break;
    } else if (isSpace(currentChar)) {
      //Serial.println("Space");
      valid = false;
      break;
    }

    //Detect no ending period
    if (q == input.length() - 1) {
      if (input[q] != '.') {
        //Serial.println("No Ending");
        valid = false;
        break;
      }
    }
    //Null Character in the middle
    if (currentChar == '\0' && q != input.length() - 1) {
      valid = false;
      break;
    }
  }
  return valid;
}

void popCommand() {
  //Process an Incoming Command
  Serial.println ("Executing Command:");
  if (cBuf.openSpot > 0) {
    int currentCommand[2] = {cBuf.commandStack[cBuf.openSpot - 1][0],
                             cBuf.commandStack[cBuf.openSpot - 1][1]
                            };
    Serial.print(currentCommand[0]);
    Serial.print(":");
    Serial.println(currentCommand[1]);
    cBuf.commandStack[cBuf.openSpot - 1][0] = -1;
    cBuf.commandStack[cBuf.openSpot - 1][1] = -1;
    cBuf.openSpot --;


  } else {
    Serial.println("No Command");
  }
}


////////////////////////////////
////////////////////////////////
////////////////////////////////
////////////////////////////////

//Slave Pinouts
const int MasterReset = A4;
const int CamRx = 0; //Camera Serial into SCom
const int CamTx = 1; //Camera Serial out of SCom
const int SDApin = 20; //I2C Data
const int SCLpin = 21; //I2C Clock
const int TempS = A0;  //Analog Temp Sensor
const int LightS = A1; //Analog Light Sensor

const int CX1 = 22; //Coil X Input 1
const int CX2 = 23; //Coil X Input 2
const int CY1 = 5; //Coil Y Input 1
const int CY2 = 6; //Coil Y Input 2
const int CZ1 = 9; //Coil Z Input 1
const int CZ2 = 10; //Coil Z Input 2

const int CX_PWM = 11; //Coil X PWM Control
const int CY_PWM = 12; //Coil Y PWM Control
const int CZ_PWM = 13; //Coil Z PWM Control
const int CXY_Enable = A5; //Coil X and Y Enable
const int CZ_Enable = 24; //Coil Z Enable




int getTempDegrees(int TempPin) {
  // Returns the temperature of the sensor at pin senseTemp
  int temp = analogRead(TempPin);
  temp = map(temp, 0, 543, -50, 125);
  return temp;
}
int getLightLvl(int LightPin) {
  //Returns the lighting of the sensor at pin senselight (10k resistor)
  int light = analogRead(LightPin);
  light = map(light, 0, 775, 100, 0);
  //2.5 / 3.3 *1023
  return light;
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void commandParser(int howMany) {
  String command = "";
  while (1 <= Wire.available()) { // loop through all but the last
    char c = Wire.read(); // receive byte as a character
    command += c;        // print the character
  }
  //Parse Command
  if (isInputValid(command)) {
    Serial.println("Command is Valid");
    buildBuffer(command);
    Serial.println("Built Command Buffer Successfully");
  } else {
    Serial.println("Invalid Command");
  }
}

String buildStatusString() {
  String res = "";
  res += "{MRsts: " + String(resets);
  res += ",ErrX: " + String(MagVectorError.x);
  res += ",ErrY: " + String(MagVectorError.y);
  res += ",Errz: " + String(MagVectorError.z);
  res += "}||||";
  return res;
}

void requestEvent() {
  Serial.println("Data Request");
  String r = buildStatusString();
  char response[r.length()];
  r.toCharArray(response, r.length());
  Wire.write(response);
  lastMasterCom = millis();
}

void initalizePinOut() {
  const int MasterReset = A4; pinMode(MasterReset, OUTPUT); //Reset Master Core
  const int CamRx = 0; //Camera Serial into SCom
  const int CamTx = 1; //Camera Serial out of SCom
  const int SDApin = 20; //I2C Data
  const int SCLpin = 21; //I2C Clock
  const int TempS = A0; pinMode(TempS, INPUT); //Analog Temp Sensor
  const int LightS = A1; pinMode(LightS, INPUT); //Analog Light Sensor

  const int CX1 = 22; pinMode(CX1, OUTPUT); //Coil X Input 1
  const int CX2 = 23; pinMode(CX2, OUTPUT); //Coil X Input 2
  const int CY1 = 5; pinMode(CY1, OUTPUT); //Coil Y Input 1
  const int CY2 = 6; pinMode(CY2, OUTPUT); //Coil Y Input 2
  const int CZ1 = 9; pinMode(CZ1, OUTPUT); //Coil Z Input 1
  const int CZ2 = 10; pinMode(CZ2, OUTPUT); //Coil Z Input 2

  const int CX_PWM = 11; pinMode(CX_PWM, OUTPUT); //Coil X PWM Control
  const int CY_PWM = 12; pinMode(CY_PWM, OUTPUT); //Coil Y PWM Control
  const int CZ_PWM = 13; pinMode(CZ_PWM, OUTPUT); //Coil Z PWM Control
  const int CXY_Enable = A5; pinMode(CXY_Enable, OUTPUT); //Coil X and Y Enable
  const int CZ_Enable = 24; pinMode(CZ_Enable, OUTPUT); //Coil Z Enable

  const int LED = 13;  pinMode(LED, OUTPUT);
}


void forcedStall(){
  digitalWrite(8,HIGH);
  int test[5] = {4};
  int x = test[78];
  Serial.println(x);
  Serial.println("Didnt crash1");
  int y = 12/0.0;
  Serial.println(y);
  Serial.println("Didnt crash2");
  int z;
  int crash = z + 4;
  Serial.println(z);
  Serial.println("Didnt crash3");
  //while(1);
  digitalWrite(8,LOW); 
  
}

void setup() {
  Serial.begin(9600);
  delay(1000);

  initalizePinOut();

  //digitalWrite(MasterReset, HIGH); //Enable Master
  Wire.begin(8);
  // join i2c bus with address #8
  //Wire.onReceive(commandParser);
  Wire.onRequest(requestEvent);

  //int endT = millis() + manualTimeoutS;
  //while (!imu.begin() && millis() < endT);
  //if (!imu.begin()) {
  //  imuWorking = false;
  //}

  //Reset Indication
  pinMode(8, OUTPUT);
  for (int j = 0; j < 10; j++) {
    digitalWrite(8, HIGH);
    delay(140);
    digitalWrite(8, LOW);
    delay(140);
  }

  //Forced Stall
  pinMode(12, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(12), forcedStall, LOW);
  
}

int ledState = HIGH;
int ledLastTime = 0;
void loop() {
  //Blinker for Testing
  if (millis() - ledLastTime >= 477) {
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
    digitalWrite(13, ledState);
    Serial.print("S Running: ");
    Serial.println(millis() - ledLastTime);
    ledLastTime = millis();
  }
  //popCommand();

  //Test IMU Slave Link
  //floatTuple mag = getMagData(imu, waitTime);
  //floatTuple gyro = getGyroData(imu, waitTime);
  //Serial.print("Mag: "); mag.print();
  //Serial.print("Gyro: "); gyro.print();


  //Reset Master if No Communication for 5 min
  //  if (TestReset && (millis() - lastMasterCom > MasterFaultTime)) {
  //    digitalWrite(MasterReset, LOW);
  //    resets++;
  //    delay(100);
  //    digitalWrite(MasterReset, HIGH);
  //    lastMasterCom = millis(); //
  //  }
}


////////////////////////////////////////////////////////////////////////////////


