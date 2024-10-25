/*
Date: 20.10.2024
Author: Siddharth A. Patel
GitHub: https://github.com/siddharthpatelde?tab=overview&from=2024-10-01&to=2024-10-17

In this version, I edited the code from "Distance from Next Edge v1" to adjust the quadrant value and array size. In the next version, we will apply a low-pass filter for better distance accuracy.
(View here: https://github.com/Solar-Clean/robot_arduino/blob/main/rplidar-with-pico/lidar_continue_data_stream_v2/lidar_continue_data_stream_v2.ino)
This version implements the edge detection logic first, allowing us to later calculate the distance from the edge.
*/




#include <RPLidar.h>
#define RPLIDAR_MOTOR 3 //The PWM pin for control the speed of RPLIDAR's motor (MOTOCTRL).

#define lidar_height 25  //lidar position 25 cm from the ground
#define tolerance 2 //define the tolerance factor for edge detection logic

int points_count = 0; //Counter to track number of points in the current scan
float alfas_when_edge[5]; // Define an array to hold 10 float alfa values when edge is detected values

static bool new_scan_started;

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
      Serial.println(points_count);
      start_new_scan();
      //Serial.println("New scan started...");
      points_count = 0;
    }    

    if (distance > 0) {  // Ensure distance is valid
      if (angle > 270 && angle < 350) {  // 30 below from horizontal level


      distance_non_filted = get_next_holes_from_laserscan_non_filtered(angle, distance,lidar_height);

      if(distance_non_filted != 0.0){
        float distance_filted = filter_minimum_per_scan(distance_non_filted);
      // Serial.print("edge id detected at ");
      // Serial.println(distance_non_filted);
      Serial.println(distance_filted);
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

    //Serial.print("edge id detected at ");
    //Serial.println(distance_to_next_edge);
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


// Function to filter and retrieve the minimum value per scan
float filter_minimum_per_scan(float current_distance) {
  static bool new_scan_started = true;  // flag to track the start of a new scan
  static float min_value = 0.0;  // stores the minimum value for each scan
  
  if (new_scan_started) {
    // New scan started, initialize the minimum value with the first distance
    min_value = current_distance;
    new_scan_started = false;
  }

  // If a new edge is detected that is smaller, update the minimum value
  if (current_distance < min_value) {
    min_value = current_distance;
  }

  return min_value;  // Return the minimum value for the current scan
}


// Reset the scan when a new scan is detected
void start_new_scan() {
  Serial.println("New scan started...");
  new_scan_started = true;  // Set flag to true to initialize the minimum for the next scan
}