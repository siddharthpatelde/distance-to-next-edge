/*
Date: 25.10.2024
Author: Siddharth A. Patel
GitHub: https://github.com/siddharthpatelde?tab=overview&from=2024-10-01&to=2024-10-17

This is the final attempt to filter data using the Filters library.

The Filters library used can be found here:
https://github.com/edargelies/arduino_eq/tree/master/libraries/Filters

The RPlidar library used can be found here:
https://github.com/robopeak/rplidar_arduino/tree/master
*/

#include <Filters.h>

#include <RPLidar.h>
#define RPLIDAR_MOTOR 3 //The PWM pin for control the speed of RPLIDAR's motor (MOTOCTRL).

#define lidar_height 25  //lidar position 25 cm from the ground
#define tolerance 2 //define the tolerance factor for edge detection logic

int points_count = 0; //Counter to track number of points in the current scan
float alfas_when_edge[5]; // Define an array to hold 10 float alfa values when edge is detected values

FilterOnePole lowpassFilter(LOWPASS, 0.1); // Smoothing filter
float smoothedValue = 0;

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
    float distance_non_filted = 0.0;

    if (lidar.getCurrentPoint().startBit) {
      //Serial.println(points_count);
      //Serial.println("New scan started...");
      points_count = 0;
    }    

    if (distance > 0) {  // Ensure distance is valid
      if (angle > 270 && angle < 350) {  // 30 below from horizontal level


      distance_non_filted = get_next_holes_from_laserscan_non_filtered(angle, distance,lidar_height);
      if (distance_non_filted != 0.0) {
        // Print both non-filtered and filtered values
        smoothedValue = lowpassFilter.input(distance_non_filted);
        
        // Output both values in a format suitable for the Serial Plotter
        Serial.print(distance_non_filted);    // Print the non-filtered value
        Serial.print("\t");                   // Tab separator
        Serial.println(smoothedValue);        // Print the filtered (smoothed) value
      }

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

float get_next_holes_from_laserscan_non_filtered(float angle_lidar, float distance_lidar, int scan_height){
  float alfa_degrees = 0.0; //define the alfa angle for math model (see the Diagramm)
  float alfa_radianse = 0.0; //define the alfa angle for in radianse
  float distance_cm = 0.0; //define the associated distance with alfa in cm
  float distance_calculated_cm = 0.0; //define the calculated distance value in cm

  float distance_to_next_edge = 0.0;

  alfa_degrees = angle_lidar - 270; //geeting alfa angle from angles values from lidar
  alfa_radianse = alfa_degrees * PI / 180; // Convert degrees to radians
  distance_cm = distance_lidar / 10 ; // Convert distacnce in mm to cm
  distance_calculated_cm = scan_height / cos(alfa_radianse); //getting theoretical value of distance

  if(distance_cm > tolerance * distance_calculated_cm){
    
    for (int i = 0; i < 5; i++) {
      alfas_when_edge[i] = alfa_radianse;
    }
    float min_alfa_Value = findMin(alfas_when_edge, 5);
    distance_to_next_edge = tan(min_alfa_Value)*scan_height;
  }

 return distance_to_next_edge;
}

void printData(float angle, float distance, int element) {
  Serial.print("Element number: ");
  Serial.print(element);
  Serial.print("    dist: ");
  Serial.print(distance);
  Serial.print("    angle: ");
  Serial.println(angle);
}

// Function to find the minimum value from an array of floats
float findMin(float arr[], int size) {
  float minValue = arr[0];
  for (int i = 1; i < size; i++) {
    if (arr[i] < minValue) {
      minValue = arr[i];
    }
  }
  return minValue;
}
