/*
Date: 18.10.2024
Author: Siddharth A. Patel
GitHub: https://github.com/siddharthpatelde?tab=overview&from=2024-10-01&to=2024-10-17

In this version, I haven't made any major changes. I just added an if condition to print the distance and angle values only when they are in the first quadrant, 
so we can store them in an array later.

Also, during this experiment, I noticed that when an object is placed closer than 5 cm to the LIDAR, it doesn't print the distance and angle values. 
Keep that in mind.
*/


#include <RPLidar.h>
#define RPLIDAR_MOTOR 3 // The PWM pin for control the speed of RPLIDAR's motor (MOTOCTRL).

// #define MAX_POINTS 1000   // Maximum number of points per scan, assuming 1 degree per point.

// float angles_arr[MAX_POINTS];     // Array to store angles
// float distances_arr[MAX_POINTS];  // Array to store corresponding distances

int points_count = 0;             // Counter to track number of points in the current scan


RPLidar lidar;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);  // For RPLidar
  lidar.begin(Serial1);
  pinMode(RPLIDAR_MOTOR, OUTPUT);  // set pin modes
}

void loop() {
  if (IS_OK(lidar.waitPoint())) {
    //perform data processing here... 
    float distance = lidar.getCurrentPoint().distance;
    float angle = lidar.getCurrentPoint().angle;  // 0-360 deg

    if (lidar.getCurrentPoint().startBit) {
      // printDataArray(angles_arr,distances_arr);

      // A new scan started
       Serial.print("Total number of elements in previous array: ");
       Serial.println(points_count);
       Serial.println("New scan started...");
       points_count = 0;
    }    

      /*
      I edited this logic to only include angle and distance values from the first quadrant.
      I also noticed that if the distance is less than 5 cm, it doesn't receive the correct distance value. 
      Since we are using `if (distance > 0)`, it simply skips printing the angle and distance in these cases.
      */

    if (distance > 0) {  // Ensure distance is valid
      if (angle > 270 && angle < 330) {  // 30 below from horizontal level

      //this is for debug purpose 
      printData(angle, distance, points_count);

      // //to store the angle and distance in an array so that we can use them as any function argument
      // angles_arr[points_count] = angle;
      // distances_arr[points_count] = distance;

      points_count++;
      }
    }
  }
  else {
    analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor
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

// void printDataArray(float angles_arr[], float distances_arr[]) {

//   int arraySize = sizeof(angles_arr) / sizeof(angles_arr[0]);

//   for (int i = 0; i < arraySize; i++) {
//     Serial.print("Element number: ");
//     Serial.print(i);
//     Serial.print("    dist_from_array: ");
//     Serial.print(distances_arr[i]);
//     Serial.print("    angle_from_array: ");
//     Serial.println(angles_arr[i]);
//   }
// }



void printData(float angle, float distance, int element) {
  Serial.print("Element number: ");
  Serial.print(element);
  Serial.print("    dist: ");
  Serial.print(distance);
  Serial.print("    angle: ");
  Serial.println(angle);
}
