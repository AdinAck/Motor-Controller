#include "Arduino.h"
#include <Wire.h>
#include <SimpleFOC.h>
#include <Math.h>
#include "encoders/as5047/MagneticSensorAS5047.h"
#include "drivers/drv8316/drv8316.h"

void printDRV8316Status();
void printSensorStatus();

// for human use or not
#define HUMAN true

// for use with multiple motors
#define MOTOR_ID 0

// pin definitions
#define SENSOR_nCS 30
#define DRIVER_nCS 31
#define DRIVER_UH  5
#define DRIVER_VH  6
#define DRIVER_WH  9
#define DRIVER_OFF 11
#define nFAULT     SDA
#define USB_ONLY   SCL
#define DISABLE    16
#define LED        13

MagneticSensorSPIConfig_s AS5047 = {
  .spi_mode = SPI_MODE1,
  .clock_speed = 1000000,
  .bit_resolution = 14,
  .angle_register = 0x3FFF,
  .data_start_bit = 13,
  .command_rw_bit = 14,
  .command_parity_bit = 15
};

MagneticSensorAS5047 sensor(SENSOR_nCS, false, SPISettings(4000000, BitOrder::MSBFIRST, SPI_MODE1));
BLDCMotor motor(11);
DRV8316Driver3PWM driver(DRIVER_UH, DRIVER_VH, DRIVER_WH, DRIVER_nCS, false);
Commander command(Serial);

void onMotor(char *cmd) { command.motor(&motor, cmd); }

void onDRV(char *cmd) {
  DRV8316Status status = driver.getStatus();
  switch (cmd[0]) {
    case 'F': // fault
      Serial.println(status.isFault());
    default:
      break;
  }
}

void onID(char *cmd) {
  Serial.println(MOTOR_ID);
}

void setup()
{
  pinMode(nFAULT, INPUT);
  pinMode(USB_ONLY, INPUT);
  pinMode(DISABLE, OUTPUT);
  pinMode(LED, OUTPUT);

  digitalWrite(DISABLE, LOW);

  sensor.init();
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 12;
  driver.init();

  motor.linkDriver(&driver);
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::angle;

  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0;
  motor.LPF_velocity.Tf = 0.01;
  motor.P_angle.P = 20;

  motor.voltage_limit = 3;
  // motor.velocity_limit = 5;

  Serial.begin(115200);
  motor.useMonitoring(Serial);

  if (HUMAN) {
    while (!Serial)
      _delay(1000);
  
    Serial.println(digitalRead(USB_ONLY) ?
      F("USB is power source, motor disabled.") :
      F("External power supply connected, motor enabled.")
    );
  }

  motor.init();
  // motor.initFOC(0.09, CW); // Change this for the motor you use
  motor.initFOC();
  // motor.disable();

  command.add('M', onMotor);
  command.add('D', onDRV);
  command.add('I', onID);

  if (HUMAN) {
    printDRV8316Status();
    printSensorStatus();
  } else {
    command.verbose = VerboseMode::on_request;
  }

  _delay(1000);
}

void loop()
{
  motor.loopFOC();

  motor.move(motor.target);
  // motor.monitor();
  command.run();
  // Serial.println(sensor.getCurrentAngle());
}

void printSensorStatus()
{
  AS5047Diagnostics diag = sensor.readDiagnostics();
  Serial.println(F("AS5047 Status:"));
  Serial.print(F("Error: "));
  Serial.println(sensor.isErrorFlag());
  Serial.print(F("COF: "));
  Serial.println(diag.cof);
  float currentAngle = sensor.getCurrentAngle();
  Serial.print(F("Angle: "));
  Serial.println(currentAngle);
  uint16_t magnitude = sensor.readMagnitude();
  Serial.print(F("Magnitude: "));
  Serial.println(magnitude);
  Serial.print(F("Error: "));
  Serial.println(sensor.isErrorFlag());
  if (sensor.isErrorFlag())
  {
    AS5047Error err = sensor.clearErrorFlag();
    Serial.print(F("Command invalid: "));
    Serial.println(err.commandInvalid);
    Serial.print(F("Framing error: "));
    Serial.println(err.framingError);
    Serial.print(F("Parity error: "));
    Serial.println(err.parityError);
  }
  Serial.println();
}

void printDRV8316Status()
{
  DRV8316Status status = driver.getStatus();
  Serial.println(F("DRV8316 Status:"));
  Serial.print(F("Fault: "));
  Serial.println(status.isFault());
  Serial.print(F("Buck Error: "));
  Serial.print(status.isBuckError());
  Serial.print(F("  Undervoltage: "));
  Serial.print(status.isBuckUnderVoltage());
  Serial.print(F("  OverCurrent: "));
  Serial.println(status.isBuckOverCurrent());
  Serial.print(F("Charge Pump UnderVoltage: "));
  Serial.println(status.isChargePumpUnderVoltage());
  Serial.print(F("OTP Error: "));
  Serial.println(status.isOneTimeProgrammingError());
  Serial.print(F("OverCurrent: "));
  Serial.print(status.isOverCurrent());
  Serial.print(F("  Ah: "));
  Serial.print(status.isOverCurrent_Ah());
  Serial.print(F("  Al: "));
  Serial.print(status.isOverCurrent_Al());
  Serial.print(F("  Bh: "));
  Serial.print(status.isOverCurrent_Bh());
  Serial.print(F("  Bl: "));
  Serial.print(status.isOverCurrent_Bl());
  Serial.print(F("  Ch: "));
  Serial.print(status.isOverCurrent_Ch());
  Serial.print(F("  Cl: "));
  Serial.println(status.isOverCurrent_Cl());
  Serial.print(F("OverTemperature: "));
  Serial.print(status.isOverTemperature());
  Serial.print(F("  Shutdown: "));
  Serial.print(status.isOverTemperatureShutdown());
  Serial.print(F("  Warning: "));
  Serial.println(status.isOverTemperatureWarning());
  Serial.print(F("OverVoltage: "));
  Serial.println(status.isOverVoltage());
  Serial.print(F("PowerOnReset: "));
  Serial.println(status.isPowerOnReset());
  Serial.print(F("SPI Error: "));
  Serial.print(status.isSPIError());
  Serial.print(F("  Address: "));
  Serial.print(status.isSPIAddressError());
  Serial.print(F("  Clock: "));
  Serial.print(status.isSPIClockFramingError());
  Serial.print(F("  Parity: "));
  Serial.println(status.isSPIParityError());
  if (status.isFault())
    driver.clearFault();
  delayMicroseconds(1); // ensure 400ns delay
  DRV8316_PWMMode val = driver.getPWMMode();
  Serial.print(F("PWM Mode: "));
  Serial.println(val);
  delayMicroseconds(1); // ensure 400ns delay
  bool lock = driver.isRegistersLocked();
  Serial.print(F("Lock: "));
  Serial.println(lock);
  Serial.println();
}