# PID-control-for-motor-using-arduino without using RMCS 2303 Library
#include <Wire.h>
#include <Adafruit_INA219.h>

// PID Constants for Motor 1 and Motor 2
double Kp1 = 2.0, Ki1 = 0.5, Kd1 = 1.0;
double Kp2 = 2.0, Ki2 = 0.5, Kd2 = 1.0;

// Desired RPMs
double setRPM1 = 1000; 
double setRPM2 = 1000; 

// Measured RPMs
double currentRPM1 = 0, currentRPM2 = 0; 

// PWM output values
double motorSpeed1 = 0, motorSpeed2 = 0; 

// Errors for PID
double error1 = 0, lastError1 = 0, integral1 = 0, derivative1 = 0;
double error2 = 0, lastError2 = 0, integral2 = 0, derivative2 = 0;

// Encoder pulse counts
volatile unsigned int pulseCount1 = 0;
volatile unsigned int pulseCount2 = 0;

// Timing
unsigned long lastTime1 = 0, lastTime2 = 0;

// INA219 instances
Adafruit_INA219 ina219_1(0x40); // Address 0x40 for Motor 1
Adafruit_INA219 ina219_2(0x41); // Address 0x41 for Motor 2

void setup() {
  Serial.begin(9600);

  // Initialize INA219s
  if (!ina219_1.begin() || !ina219_2.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1);
  }
  ina219_1.setCalibration_32V_2A();
  ina219_2.setCalibration_32V_2A();

  // Motor PWM pins
  pinMode(9, OUTPUT);  // Motor 1 PWM
  pinMode(10, OUTPUT); // Motor 2 PWM

  // Encoder input pins
  pinMode(2, INPUT_PULLUP); // Motor 1 Encoder
  pinMode(3, INPUT_PULLUP); // Motor 2 Encoder
  attachInterrupt(digitalPinToInterrupt(2), encoderISR1, RISING);
  attachInterrupt(digitalPinToInterrupt(3), encoderISR2, RISING);

  lastTime1 = millis();
  lastTime2 = millis();
}

void loop() {
  unsigned long currentTime = millis();

  // Motor 1 RPM and PID
  if (currentTime - lastTime1 >= 100) {
    noInterrupts();
    currentRPM1 = (pulseCount1 * 600) / 20; // Adjust encoder pulses/revolution if needed
    pulseCount1 = 0;
    interrupts();

  error1 = setRPM1 - currentRPM1;
    integral1 += error1 * (currentTime - lastTime1) / 1000.0;
    derivative1 = (error1 - lastError1) / ((currentTime - lastTime1) / 1000.0);
    motorSpeed1 += Kp1 * error1 + Ki1 * integral1 + Kd1 * derivative1;
    motorSpeed1 = constrain(motorSpeed1, 0, 255);
    analogWrite(9, motorSpeed1);

  lastError1 = error1;
    lastTime1 = currentTime;

    // INA219 for Motor 1
  float voltage1 = ina219_1.getBusVoltage_V();
    float current1 = ina219_1.getCurrent_mA();
    Serial.print("Motor 1 | Set RPM: ");
    Serial.print(setRPM1);
    Serial.print(" | Current RPM: ");
    Serial.print(currentRPM1);
    Serial.print(" | PWM: ");
    Serial.print(motorSpeed1);
    Serial.print(" | Voltage: ");
    Serial.print(voltage1);
    Serial.print(" V | Current: ");
    Serial.print(current1);
    Serial.println(" mA");
  }

  // Motor 2 RPM and PID
  if (currentTime - lastTime2 >= 100) {
    noInterrupts();
    currentRPM2 = (pulseCount2 * 600) / 20; // Adjust encoder pulses/revolution if needed
    pulseCount2 = 0;
    interrupts();

  error2 = setRPM2 - currentRPM2;
    integral2 += error2 * (currentTime - lastTime2) / 1000.0;
    derivative2 = (error2 - lastError2) / ((currentTime - lastTime2) / 1000.0);
    motorSpeed2 += Kp2 * error2 + Ki2 * integral2 + Kd2 * derivative2;
    motorSpeed2 = constrain(motorSpeed2, 0, 255);
    analogWrite(10, motorSpeed2);

  lastError2 = error2;
    lastTime2 = currentTime;

    // INA219 for Motor 2
  float voltage2 = ina219_2.getBusVoltage_V();
    float current2 = ina219_2.getCurrent_mA();
    Serial.print("Motor 2 | Set RPM: ");
    Serial.print(setRPM2);
    Serial.print(" | Current RPM: ");
    Serial.print(currentRPM2);
    Serial.print(" | PWM: ");
    Serial.print(motorSpeed2);
    Serial.print(" | Voltage: ");
    Serial.print(voltage2);
    Serial.print(" V | Current: ");
    Serial.print(current2);
    Serial.println(" mA");
  }
}

// Encoder ISR for Motor 1
void encoderISR1() {
  pulseCount1++;
}

// Encoder ISR for Motor 2
void encoderISR2() {
  pulseCount2++;
}
