#include <RPLidar.h>
#define RPLIDAR_MOTOR 3 // The PWM pin for control the speed of RPLIDAR's motor (MOTOCTRL).
#include <math.h>

const float TOLERANCE = 10.0; // Adjust as needed
const float LASER_SCAN_HEIGHT = 10; // Set appropriate height for your setup

RPLidar lidar;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);  // For RPLidar
  lidar.begin(Serial1);
  pinMode(RPLIDAR_MOTOR, OUTPUT);  // set pin modes
}

void loop() {
  if (IS_OK(lidar.waitPoint())) {
    float distance = lidar.getCurrentPoint().distance;  // Distance from LIDAR
    float angle = lidar.getCurrentPoint().angle;  // Angle from LIDAR in degrees

    // Convert angle from degrees to radians for trigonometric calculations
    float angle_rad = angle * (M_PI / 180.0); 

    if (lidar.getCurrentPoint().startBit) {
       // A new scan started
       Serial.println("New scan started...");
    }

    // Skip if angle is greater than 90 degrees (as per Python logic)
    if (abs(angle_rad) >= M_PI / 2) {
      return;
    }

    // Calculate the desired distance based on the laser scan height
    float desired = LASER_SCAN_HEIGHT / cos(angle_rad);
    float difference = abs(desired - distance);

    // Store the detected holes (edges) using the same logic as in Python
    static float last_distance = 0;
    static bool hole_detected = false;

    if (difference > desired * (TOLERANCE / 100.0)) {
      if (!hole_detected) {
        // A hole is detected, print the last distance as a hole's edge
        Serial.print("Hole detected at distance: ");
        Serial.println(last_distance);
        hole_detected = true;  // Avoid duplicate hole detection
      }
    } else {
      // Update the last valid distance when no hole is detected
      last_distance = abs(tan(angle_rad) * LASER_SCAN_HEIGHT);
      hole_detected = false;  // Reset hole detection
    }
  }
  else {
    analogWrite(RPLIDAR_MOTOR, 0); // Stop the RPLIDAR motor
    // Try to detect RPLIDAR
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
       // Detected
       lidar.startScan();
       analogWrite(RPLIDAR_MOTOR, 255);
       delay(1000);
    }
  }
}
