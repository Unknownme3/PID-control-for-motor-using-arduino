#include <RMCS2303drive.h>

RMCS2303 rmcs; // Create RMCS-2303 library object

// Define Slave IDs for two motors
byte slave_id_motor1 = 1;
byte slave_id_motor2 = 2;

// Target RPM for both motors
int targetRPM_motor1 = 1000; 
int targetRPM_motor2 = 1000;

// PID variables for Motor 1
double Kp1 = 2.0, Ki1 = 0.5, Kd1 = 1.0;
double error1 = 0, lastError1 = 0, integral1 = 0, derivative1 = 0;

// PID variables for Motor 2
double Kp2 = 2.0, Ki2 = 0.5, Kd2 = 1.0;
double error2 = 0, lastError2 = 0, integral2 = 0, derivative2 = 0;

unsigned long lastUpdate = 0;

void setup() {
  Serial.begin(9600); // Start Serial Monitor for debugging

  // Initialize RMCS-2303 library for Software Serial
  rmcs.begin(&Serial, 9600);

  // Set both motors to Digital Speed Control Mode
  rmcs.Enable_Digital_Mode(slave_id_motor1, 0); // Motor 1 Forward
  rmcs.Enable_Digital_Mode(slave_id_motor2, 0); // Motor 2 Forward
}

void loop() {
  unsigned long currentTime = millis();

  // PID Update every 100ms
  if (currentTime - lastUpdate >= 100) {
    lastUpdate = currentTime;

    // Motor 1 PID control
    long currentRPM_motor1 = rmcs.Speed_Feedback(slave_id_motor1); // Get current RPM
    error1 = targetRPM_motor1 - currentRPM_motor1;
    integral1 += error1 * 0.1; // Integral term
    derivative1 = (error1 - lastError1) / 0.1; // Derivative term
    int speedCommand1 = targetRPM_motor1 + (Kp1 * error1 + Ki1 * integral1 + Kd1 * derivative1);
    speedCommand1 = constrain(speedCommand1, 0, 65535); // Constrain speed
    rmcs.Speed(slave_id_motor1, speedCommand1); // Update Motor 1 speed
    lastError1 = error1;

    // Motor 2 PID control
    long currentRPM_motor2 = rmcs.Speed_Feedback(slave_id_motor2); // Get current RPM
    error2 = targetRPM_motor2 - currentRPM_motor2;
    integral2 += error2 * 0.1; // Integral term
    derivative2 = (error2 - lastError2) / 0.1; // Derivative term
    int speedCommand2 = targetRPM_motor2 + (Kp2 * error2 + Ki2 * integral2 + Kd2 * derivative2);
    speedCommand2 = constrain(speedCommand2, 0, 65535); // Constrain speed
    rmcs.Speed(slave_id_motor2, speedCommand2); // Update Motor 2 speed
    lastError2 = error2;

    // Debugging output
    Serial.print("Motor 1 | Target RPM: ");
    Serial.print(targetRPM_motor1);
    Serial.print(" | Current RPM: ");
    Serial.print(currentRPM_motor1);
    Serial.print(" | Speed Command: ");
    Serial.println(speedCommand1);

    Serial.print("Motor 2 | Target RPM: ");
    Serial.print(targetRPM_motor2);
    Serial.print(" | Current RPM: ");
    Serial.print(currentRPM_motor2);
    Serial.print(" | Speed Command: ");
    Serial.println(speedCommand2);
  }
}
