#include <RPLidar.h>
#define RPLIDAR_MOTOR 3 // The PWM pin for control the speed of RPLIDAR's motor (MOTOCTRL).
#define MAX_POINTS 1000   // Maximum number of points per scan, assuming 1 degree per point.

RPLidar lidar;

float angles_arr[MAX_POINTS];     // Array to store angles
float distances_arr[MAX_POINTS];  // Array to store corresponding distances
int points_count = 0;             // Counter to track number of points in the current scan

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);  // For RPLidar
  lidar.begin(Serial1);
  pinMode(RPLIDAR_MOTOR, OUTPUT);  // Set pin modes
}

void loop() {
  if (IS_OK(lidar.waitPoint())) {
    float distance = lidar.getCurrentPoint().distance;
    float angle = lidar.getCurrentPoint().angle;  // 0-360 degrees

    if (lidar.getCurrentPoint().startBit) {
      // A new scan started: process and print the previous scan's data
      if (points_count > 0) {
        Serial.println("New scan started...");
        processAndPrintData();
      }
    }

    // Collect valid data points into arrays
    if (distance > 0 && points_count < MAX_POINTS) {
      angles_arr[points_count] = angle;
      distances_arr[points_count] = distance;
      points_count++;
    }
  } else {
    analogWrite(RPLIDAR_MOTOR, 0); // Stop the RPLIDAR motor
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
      // Detected, restart scan
      lidar.startScan();
      analogWrite(RPLIDAR_MOTOR, 255);
      delay(1000);
    }
  }
}

void processAndPrintData() {
  // Sort the arrays based on angles
  for (int i = 0; i < points_count - 1; i++) {
    for (int j = i + 1; j < points_count; j++) {
      if (angles_arr[i] > angles_arr[j]) {
        // Swap angles
        float temp_angle = angles_arr[i];
        angles_arr[i] = angles_arr[j];
        angles_arr[j] = temp_angle;

        // Swap corresponding distances
        float temp_distance = distances_arr[i];
        distances_arr[i] = distances_arr[j];
        distances_arr[j] = temp_distance;
      }
    }
  }

  // Print sorted data
  for (int i = 0; i < points_count; i++) {
    Serial.print("Angle: ");
    Serial.print(angles_arr[i]);
    Serial.print("  Distance: ");
    Serial.println(distances_arr[i]);
  }

  Serial.println("Scan completed.\n");
}
