/*

    This is simple STM32 based 1000W BLDC motor(with Hall sensors) controller.
    https://github.com/techn0man1ac/SimpleBLDC32

*/


#include <SimpleFOC.h> // https://docs.simplefoc.com/installation

#define PWM_U PA8
#define PWM_V PA9
#define PWM_W PA10
#define IN_U PB13
#define IN_V PB14
#define IN_W PB15
#define HAL_U PB5
#define HAL_V PB4
#define HAL_W PB3
#define Vbatt PA0

// BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(pole pair number);
BLDCMotor motor = BLDCMotor(15, 0.4, 20.0);
// BLDC driver instance
BLDCDriver3PWM driver = BLDCDriver3PWM(PWM_U, PWM_V, PWM_W, IN_U, IN_V, IN_W);

// Hall sensor instance
// HallSensor(int hallA, int hallB , int cpr, int index)
//  - hallA, hallB, hallC    - HallSensor A, B and C pins
//  - pp                     - pole pairs
HallSensor sensor = HallSensor(HAL_U, HAL_V, HAL_W, 15);

// Interrupt routine intialisation
// channel A and B callbacks
void doA() {
  sensor.handleA();
}
void doB() {
  sensor.handleB();
}
void doC() {
  sensor.handleC();
}


// velocity set point variable
float target_velocity = 0;
// instantiate the commander
Commander command = Commander(Serial);

void doTarget(char* cmd) {
  command.scalar(&target_velocity, cmd);
}

void doLimit(char* cmd) {
  command.scalar(&motor.voltage_limit, cmd);
}

void setup() {

  // use monitoring with serial
  Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);
  pinMode(PWM_U, OUTPUT);
  pinMode(PWM_V, OUTPUT);
  pinMode(PWM_W, OUTPUT);
  pinMode(IN_U, OUTPUT);
  pinMode(IN_V, OUTPUT);
  pinMode(IN_W, OUTPUT);
  pinMode(HAL_U, INPUT);
  pinMode(HAL_V, INPUT);
  pinMode(HAL_W, INPUT);

  // initialise encoder hardware
  sensor.init();
  // hardware interrupt enable
  sensor.enableInterrupts(doA, doB, doC);

  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // pwm frequency to be used [Hz]
  // for atmega328 fixed to 32kHz
  // esp32/stm32/teensy configurable
  driver.pwm_frequency = 20000;
  // power supply voltage [V]
  driver.voltage_power_supply = getBatteryVoltage();
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  // aligning voltage [V]
  motor.voltage_sensor_align = 3;

  // set motion control loop to be used
  motor.controller = MotionControlType::velocity;

  // contoller configuration
  // default parameters in defaults.h

  // velocity PI controller parameters
  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 2;
  motor.PID_velocity.D = 0;
  // default voltage_power_supply
  motor.voltage_limit = 15;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;

  // velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.01f;

  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  // add target command T
  command.add('T', doTarget, "target voltage");
  command.add('L', doLimit, "voltage limit");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));
  _delay(1000);
}


void loop() {
  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
  motor.loopFOC();
  float battVoltage = getBatteryVoltage();
  Serial.println(battVoltage, 2);
  driver.voltage_power_supply = battVoltage;
  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(target_velocity);

  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  motor.monitor();

  // user communication
  command.run();
}

float getBatteryVoltage() {
  return analogRead(Vbatt) * (69.3 / 4095.0);
}

