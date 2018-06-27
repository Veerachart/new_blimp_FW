#include <CurieBLE.h>
#include <CurieIMU.h>
#include <CurieTimerOne.h>
#include <MadgwickAHRS.h>
#include <stdint.h>

#define CURETIMER_USED 0
#define NOINTERRUPTS 1
#define DRIVE_LIMIT 200

int SAMPLE_RATE = 25; //25, 50, 100, 200, 400, 800, 1600 Hz
int LEDBLINK_MS = 1000;
int LEDBLINK_PIN = 13;

int powerPins[] = {9,3,5,6};
int phasePins[] = {8,2,4,7};
short int powers[] = {0,0,0,0};
short int commands[] = {0,0,0,0};

/*BLEService batteryService("180F"); // BLE Battery Service
BLEUnsignedCharCharacteristic batteryLevelCharacteristic("2A19",  // standard 16-bit characteristic UUID
    BLERead | BLENotify);     // remote clients will be able to
BLEDescriptor batteryDescriptor("2905", "battery");*/

BLEService orientationService("301c9b40-a61b-408a-a8bf-5efcd95a3486");
BLECharacteristic orientationCharacteristic("301c9b44-a61b-408a-a8bf-5efcd95a3486", BLERead | BLENotify, 2);
BLEDescriptor            orientationDescriptor("2903", "gyro");

BLEService commandService("301c9b20-a61b-408a-a8bf-5efcd95a3486");
BLECharacteristic commandCharacteristic("301c9b21-a61b-408a-a8bf-5efcd95a3486", BLERead | BLEWrite, 8);
BLEDescriptor            commandDescriptor("2901", "switch");

const unsigned char *cmd;

static volatile int accelometerValues[3];
static volatile int gyroValues[3];
static volatile short yawValue[1];
static volatile bool valueUpdated = false;
long int t = 0;

bool onOff = false;

int aix, aiy, aiz;
int gix, giy, giz;
float ax, ay, az;
float gx, gy, gz;
float roll, pitch, heading;
float offset = -360., angle_wrap = 360.;

Madgwick filter;
float accelScale, gyroScale;
float accelRange, gyroRange;

// PID for yaw
float Kp = 1.8, Ki = 0., Kd = 1.5;
float plant_state;
int control_effort;
float setpoint = 0.;
unsigned long microsPrevious_pid = 0, delta_t;
float error_integral = 0;
float P = 0;
float I = 0;
float D = 0;
float tan_filt = 1.;
int upper_limit = 255, lower_limit = -255;
float windup_limit = 255.;
float error[3] = {0.,0.,0.}, filtered_error[3] = {0.,0.,0.}, error_deriv[3] = {0.,0.,0.}, filtered_error_deriv[3] = {0.,0.,0.};

//short int oldBatteryLevel = 0;  // last battery level reading from analog input
//unsigned long microsPrevious_batt = 0;  // last time the battery level was checked, in us

void timerSampleValuesIsr()
{
  if (valueUpdated) return;
  if (!CurieIMU.dataReady()) return;
  
  t = millis();
  CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

  ax = convertRawAcceleration(aix);
  ay = convertRawAcceleration(aiy);
  az = convertRawAcceleration(aiz);
  gx = convertRawGyro(gix);
  gy = convertRawGyro(giy);
  gz = convertRawGyro(giz);

  // update the filter, which computes orientation
  filter.updateIMU(gx, gy, gz, ax, ay, az);

  roll = filter.getRoll();
  pitch = filter.getPitch();
  heading = filter.getYaw();
  if (offset == -360.) {
    offset = heading;
  }
  heading -= offset;
  while (heading <= -angle_wrap/2.0)
    heading += angle_wrap;
  while (heading > angle_wrap/2.0)
    heading -= angle_wrap;

  yawValue[0] = short(round(heading));

  get_control_effort(heading);
  driveMotors();
  
  valueUpdated = true;
}

void driveMotors() {
  for (int i=0; i<4; i++) {
    if (powers[i] >= 0) {
      digitalWrite(phasePins[i], HIGH);
      analogWrite(powerPins[i], powers[i]);
    }
    else {
      digitalWrite(phasePins[i], LOW);
      analogWrite(powerPins[i], -powers[i]);
    }
  }
}

void get_control_effort(float yaw) {
  plant_state = yaw;
  error[2] = error[1];
  error[1] = error[0];
  filtered_error[2] = filtered_error[1];
  filtered_error[1] = filtered_error[0];
  error_deriv[2] = error_deriv[1];
  error_deriv[1] = error_deriv[0];
  filtered_error_deriv[2] = filtered_error_deriv[1];
  filtered_error_deriv[1] = filtered_error_deriv[0];

  if (microsPrevious_pid != 0) {    // Not the first time
    unsigned long t_now = micros();
    delta_t = t_now - microsPrevious_pid;
    microsPrevious_pid = t_now;
    if (delta_t == 0)
      return;
  }
  else {
    microsPrevious_pid = micros();
    return;
  }
  
  error[0] = setpoint - plant_state;

  while (error[0] <= -angle_wrap/2.0) {
    error[0] += angle_wrap;
    //error[1] = 0.;
    //error[2] = 0.;
  }
  while (error[0] > angle_wrap/2.0) {
    error[0] -= angle_wrap;
    //error[1] = 0.;
    //error[2] = 0.;
  }
  if (fabs(error[0] - error[1]) > angle_wrap/2.0) {
    // The error cross the line opposite to the setpoint
    Serial.println("Reset");
    // filter
    if(error[0] > error[1]) {
      error_deriv[0] = (error[0] - error[1] - angle_wrap)/delta_t * 1.0e6;
    }
    else {
      error_deriv[0] = (error[0] - error[1] + angle_wrap)/delta_t * 1.0e6;
    }
    error[1] = 0;
    error[2] = 0;
    error_deriv[2] = 0;
    error_deriv[1] = 0;
    filtered_error[2] = 0;
    filtered_error[1] = 0;
    filtered_error_deriv[2] = 0;
    filtered_error_deriv[1] = 0;
    error_integral = 0;

    filtered_error[0] = (1/3.414)*(error[2]+2*error[1]+error[0]-(0.586)*filtered_error[2]);
    filtered_error_deriv[0] = (1/(3.414))*(error_deriv[2]+2*error_deriv[1]+error_deriv[0]-(0.586)*filtered_error_deriv[2]);
  }
  else {
    filtered_error[0] = (1/3.414)*(error[2]+2*error[1]+error[0]-(0.586)*filtered_error[2]);
    error_deriv[0] = (error[0] - error[1])/delta_t * 1.0e6;
    filtered_error_deriv[0] = (1/(3.414))*(error_deriv[2]+2*error_deriv[1]+error_deriv[0]-(0.586)*filtered_error_deriv[2]);
  }
  //error[2] = 0.;
  //error[1] = 0.;
  //error_integral = 0.;        // Still feel weird here

  error_integral += error[0] * delta_t*1.0e-6;   // microsec
  error_integral = min(max(error_integral, -windup_limit), windup_limit);

  
  // Control effort
  P = Kp * filtered_error[0];
  I = Ki * error_integral;
  D = Kd * filtered_error_deriv[0];
  float temp_control = P + I + D;
  
  temp_control = min(max(temp_control, lower_limit), upper_limit);
  control_effort = round(temp_control);
  //Serial.print(yaw);
  //Serial.print(":\t");
  //Serial.print(P);
  //Serial.print(" ");
  //Serial.print(D);
  //Serial.print(" ");
  Serial.println(temp_control);

  powers[0] = max(min(-commands[0] + control_effort, DRIVE_LIMIT),-DRIVE_LIMIT);
  powers[1] = max(min(commands[0] + control_effort, DRIVE_LIMIT),-DRIVE_LIMIT);
}

void commandCharacteristicWritten() {
  cmd = commandCharacteristic.value();
  for (int i=0; i<4; i++) {
    commands[i] = cmd[2*i] << 8 | cmd[2*i+1];
    //Serial.print(commands[i]);
    //Serial.print(" ");
  }
  //Serial.println();

  setpoint = float(commands[1]);
  powers[0] = max(min(-commands[0] + control_effort, 255),-255);
  powers[1] = max(min(commands[0] + control_effort, 255),-255);
  powers[2] = commands[2];
  powers[3] = commands[3];
  //power1 = frontCommandChar.value();
  //power2 = frontCommandChar.value();
}

void setup() {
  pinMode(LEDBLINK_PIN, OUTPUT);
  for (int i=0; i<4; i++) {
    pinMode(phasePins[i], OUTPUT);
    pinMode(powerPins[i], OUTPUT);
    analogWriteFrequency(powerPins[i],25000);
  }
  Serial.begin(9600);

  BLE.begin();
  BLE.setLocalName("BLIMP");

  /*BLE.setAdvertisedServiceUuid(batteryService.uuid());
  batteryLevelCharacteristic.addDescriptor(batteryDescriptor);
  batteryService.addCharacteristic(batteryLevelCharacteristic);
  BLE.addService(batteryService);*/
  
  BLE.setAdvertisedServiceUuid(commandService.uuid());
  commandCharacteristic.addDescriptor(commandDescriptor);
  commandService.addCharacteristic(commandCharacteristic);
  BLE.addService(commandService);
  
  BLE.setAdvertisedServiceUuid(orientationService.uuid());
  orientationCharacteristic.addDescriptor(orientationDescriptor);
  orientationService.addCharacteristic(orientationCharacteristic);
  BLE.addService(orientationService);
  
  //const uint8_t initialValue[AccelometerCharacteristicSize] = { 0 };
  const uint8_t initialValue[2] = { 0 };

  //accelometerCharacteristic.setValue(initialValue, AccelometerCharacteristicSize);
  orientationCharacteristic.setValue(initialValue, 2);

  BLE.advertise();
  //blePeripheral.begin();

  accelRange = 2;
  gyroRange = 250;
  // Initialize IMU
  CurieIMU.begin();

  CurieIMU.setAccelerometerRange(accelRange);  // Max: 2G
  CurieIMU.setGyroRange(gyroRange);     // Max: 250 [deg/s]

  CurieIMU.setAccelerometerRate(SAMPLE_RATE);
  CurieIMU.setGyroRate(SAMPLE_RATE);
  filter.begin(SAMPLE_RATE);

  CurieIMU.setAccelerometerOffset(X_AXIS,97.50);
  CurieIMU.setAccelerometerOffset(Y_AXIS,97.50);
  CurieIMU.setAccelerometerOffset(Z_AXIS,70.20);
  CurieIMU.setGyroOffset(X_AXIS,1.52);
  CurieIMU.setGyroOffset(Y_AXIS,0.00);
  CurieIMU.setGyroOffset(Z_AXIS,-0.37);
  #if CURETIMER_USED == 1 
    CurieTimerOne.start(1000000u/SAMPLE_RATE, timerSampleValuesIsr);
  #endif
}

void loop() {
  #if CURETIMER_USED == 0
    noInterrupts();
    timerSampleValuesIsr();
    interrupts();
  #endif
  if (valueUpdated)
    valueUpdated = false;
  //BLECentral central = blePeripheral.central();
  ledBlink();
  BLEDevice central = BLE.central();
  if (central) {
    LEDBLINK_MS = 250;

    while (central.connected()) {
      #if CURETIMER_USED == 0
      noInterrupts();
      timerSampleValuesIsr();
      interrupts();
      #endif
      //unsigned long microsNow = micros();

      if (valueUpdated)
      {
        //accelometerCharacteristic.setValue(reinterpret_cast<uint8_t*>(const_cast<int*>(accelometerValues)), AccelometerCharacteristicSize);
        orientationCharacteristic.setValue(reinterpret_cast<uint8_t*>(const_cast<short*>(yawValue)), 2);

        valueUpdated = false;
      }

      if (commandCharacteristic.written()) {
        commandCharacteristicWritten();
        /*Serial.print("Received ");
        const unsigned char* nums = commandCharacteristic.value();
        for (int i = 0; i < 4; i++) {
          short num = nums[2*i] << 8 | nums[2*i+1];
          Serial.print(num);
          Serial.print(" ");
        }
        Serial.println();
        if (onOff)
          digitalWrite(13, HIGH);
        else
          digitalWrite(13, LOW);
        onOff = !onOff;*/
      }

      ledBlink();

      /*// if 200ms have passed, check the battery level:
      if (microsNow - microsPrevious_batt >= 200000) {
        microsPrevious_batt = microsNow;
        updateBatteryLevel();
      }*/
    }
    LEDBLINK_MS = 1000;
    // Reset all commands (still have yaw maintaining control)
    for (int i = 0; i < 4; i++)
      commands[i] = 0;
    setpoint = float(commands[1]);
    powers[0] = max(min(-commands[0] + control_effort, 255),-255);
    powers[1] = max(min(commands[0] + control_effort, 255),-255);
    powers[2] = commands[2];
    powers[3] = commands[3];
    driveMotors();
    /*#if CURETIMER_USED == 1
    CurieTimerOne.stop();
    #endif*/
  }
}

/*void updateBatteryLevel() {
  // Read the current voltage level on the A0 analog input pin.
  //   This is used here to simulate the charge level of a battery.
  //
  int battery = analogRead(A0);
  short int batteryLevel = map(battery, 800, 1023, 0, 100);

  if (batteryLevel != oldBatteryLevel) {      // if the battery level has changed
    batteryLevelCharacteristic.setValue(batteryLevel);  // and update the battery level characteristic
    oldBatteryLevel = batteryLevel;           // save the level for next comparison
    if (batteryLevel < 15) {
      // Low battery, below 25%
      LEDBLINK_MS = 100;
    }
  }
}*/

//
// LED Heartbeat routine by Allen C. Huffman (www.appleause.com)
//
void ledBlink()
{
  static unsigned int  ledStatus = LOW;  // Last set LED mode.
  static unsigned long ledBlinkTime = 0; // LED blink time.

  // LED blinking heartbeat. Yes, we are alive.
  // For explanation, see:
  // http://playground.arduino.cc/Code/TimingRollover
  if ( (long)(millis()-ledBlinkTime) >= 0 )
  {
    // Toggle LED.
    ledStatus = (ledStatus==HIGH ? LOW : HIGH);

    // Set LED pin status.
    digitalWrite(LEDBLINK_PIN, ledStatus);

    // Reset "next time to toggle" time.
    ledBlinkTime = millis()+LEDBLINK_MS;
  }
}

float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
  
  float a = (aRaw * accelRange) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  
  float g = (gRaw * gyroRange) / 32768.0;
  return g;
}


