// /*
// Date: 17.10.2024
// Author: Siddharth A. Patel
// GitHub: https://github.com/siddharthpatelde?tab=overview&from=2024-10-01&to=2024-10-17

// I took help from the RPlidar.h library available here: https://github.com/robopeak/rplidar_arduino/tree/master
// and modified the example code to achieve my goal.

// In this code, I have created arrays for distances and angles per scan, so I can later use them to process the next hole or edge distance.
// */




// #include <RPLidar.h>
// #define RPLIDAR_MOTOR 3 // The PWM pin for control the speed of RPLIDAR's motor (MOTOCTRL).

// // Because i need to save the angle and distances in array and that will be of size 360;
// const int MAX_POINTS = 360; // Max points per scan (depends on your LIDAR resolution) 
// float distances_arr[MAX_POINTS];
// float angles_arr[MAX_POINTS];
// int currentIndex = 0;


// RPLidar lidar;

// void setup() {
//   Serial.begin(115200);
//   Serial1.begin(115200);  // For RPLidar
//   lidar.begin(Serial1);
//   pinMode(RPLIDAR_MOTOR, OUTPUT);  // set pin modes
// }


// void loop() {
//   if (IS_OK(lidar.waitPoint())) {
//     //perform data processing here... 
//     float distance = lidar.getCurrentPoint().distance;
//     float angle = lidar.getCurrentPoint().angle;  // 0-360 deg



//     if (lidar.getCurrentPoint().startBit) {
//        // A new scan started
//        //Serial.println("New scan started...");
//        // Reset the index for a new scan
//        currentIndex = 0;
//     }

//     // Store the distance and angle in arrays if valid
//     if (distance > 0 && currentIndex < MAX_POINTS) {
//       distances_arr[currentIndex] = distance;
//       angles_arr[currentIndex] = angle;
//       currentIndex++;
//     }

//     for (int i = 0; i < MAX_POINTS; i++) {
//     // Process each point here
//     Serial.print("Angle: ");
//     Serial.print(angles_arr[i]);
//     Serial.print(", Distance: ");
//     Serial.println(distances_arr[i]);
//   }

//     // // Once the scan completes, process the data
//     // if (currentIndex >= MAX_POINTS) {
//     //   get_next_holes_from_laserscan(distances_arr, angles_arr);
//     //   currentIndex = 0; // Reset for the next scan
//     // }

//   }


//   else {
//     analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor
//     // Try to detect RPLIDAR
//     rplidar_response_device_info_t info;
//     if (IS_OK(lidar.getDeviceInfo(info, 100))) {
//        // Detected
//        lidar.startScan();
//        analogWrite(RPLIDAR_MOTOR, 255);
//        delay(1000);
//     }
//   }
// }



// // float get_next_holes_from_laserscan(float distance[], float angle[]) {
// //   // Your implementation for processing distance and angle arrays

// //   for (int i = 0; i < MAX_POINTS; i++) {
// //     // Process each point here
// //     Serial.print("Angle: ");
// //     Serial.print(angle[i]);
// //     Serial.print(", Distance: ");
// //     Serial.println(distance[i]);
// //   }
// //   return 0.0; // Modify this return as per your function logic
// // }


#include <RPLidar.h>
#define RPLIDAR_MOTOR 3 // The PWM pin for control the speed of RPLIDAR's motor (MOTOCTRL).

const int MAX_POINTS = 360; // Max points per scan (depends on your LIDAR resolution) 
float distances_arr[MAX_POINTS];
float angles_arr[MAX_POINTS];
int currentIndex = 0;

RPLidar lidar;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);  // For RPLidar
  lidar.begin(Serial1);
  pinMode(RPLIDAR_MOTOR, OUTPUT);  // set pin modes
  analogWrite(RPLIDAR_MOTOR, 255); // Start motor
  Serial.println("Starting LIDAR...");
}

void loop() {
  // Check if the LIDAR point is valid
  if (IS_OK(lidar.waitPoint())) {
    float distance = lidar.getCurrentPoint().distance;
    float angle = lidar.getCurrentPoint().angle;  // 0-360 deg

    if (lidar.getCurrentPoint().startBit) {
      // New scan started
      Serial.println("New scan started...");
    }

    // Store the distance and angle in arrays if valid
    if (distance > 0 && currentIndex < MAX_POINTS) {
      if(angle >= 0 && angle <= 360){
        distances_arr[currentIndex] = distance;
        angles_arr[currentIndex] = angle;
      currentIndex++;
      }

      // // Print the current point to verify readings
      // Serial.print("Angle: ");
      // Serial.print(angle);
      // Serial.print(", Distance: ");
      // Serial.println(distance);
      //Serial.println(currentIndex);
    }

    // Once the scan completes, print all data
    if (currentIndex >= MAX_POINTS) {
      Serial.println("Scan complete. Printing all data:");
      for (int i = 0; i < MAX_POINTS; i++) {
        Serial.print("Angle: ");
        Serial.print(angles_arr[i]);
        Serial.print(", Distance: ");
        Serial.println(distances_arr[i]);
      }
      currentIndex = 0; // Reset for the next scan
    }
  } else {
    Serial.println("No valid data from LIDAR, stopping motor.");
    analogWrite(RPLIDAR_MOTOR, 0); // Stop the rplidar motor

    // Try to detect RPLIDAR
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
      // Detected
      Serial.println("LIDAR detected, starting scan...");
      lidar.startScan();
      analogWrite(RPLIDAR_MOTOR, 255); // Restart motor
      delay(1000);
    } else {
      Serial.println("Failed to detect LIDAR. Retrying...");
    }
  }
}


