/*
Date: 25.10.2024
Author: Siddharth A. Patel
GitHub: https://github.com/siddharthpatelde?tab=overview&from=2024-10-01&to=2024-10-17

This is the final attempt to filter data without using the Filters library.

Added function: JSON serialization string for utilizing data

The RPlidar library used can be found here:
https://github.com/robopeak/rplidar_arduino/tree/master

Angle Cases:
Case 1: 0° to 90°
Case 2: 90° to 180°
Case 3: 180° to 270°
Case 4: 270° to 360°
*/

#include <ArduinoJson.h>
#include <RPLidar.h>

#define RPLIDAR_MOTOR 3 //The PWM pin for control the speed of RPLIDAR's motor (MOTOCTRL).

#define lidar_height 22  //lidar position 25 cm from the ground
#define tolerance 1.3 //define the tolerance factor for edge detection logic

// Parameters for data collection
const unsigned long COLLECTION_PERIOD = 1500; // T = 1 seconds (adjust as needed)

RPLidar lidar;

void setup() {
  Serial.begin(115200);  // Initialize the USB serial port for JASON string
  Serial1.begin(115200);  // For RPLidar
  lidar.begin(Serial1);
  pinMode(RPLIDAR_MOTOR, OUTPUT);  // set pin modes
}

// Example usage in your loop():
void loop() {

  if (IS_OK(lidar.waitPoint())) {
    //perform data processing here... 
    float distance = lidar.getCurrentPoint().distance;
    float angle = lidar.getCurrentPoint().angle;  // 0-360 deg
    float distance_filted = 0.0;

          distance_filted = get_next_holes_from_laserscan_non_filtered(angle, distance,lidar_height,4);
          if (distance_filted != 0.0) {
            if (!isnan(distance_filted)) {
            // Create a JSON document to serialize
            StaticJsonDocument<200> doc;
            doc["lidar_height"] = lidar_height;
            doc["Distance"] = distance_filted;

            // Serialize JSON to Serial
            serializeJson(doc, Serial);
            Serial.println(); // Print newline for readability
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
       analogWrite(RPLIDAR_MOTOR, 175);
       delay(1000);
    }
  }

  
}

float get_next_holes_from_laserscan_non_filtered(float angle_lidar, float distance_lidar, int scan_height, int case_num) {
    float alfa_degrees = 0.0;
    float alfa_radianse = 0.0;
    float distance_cm = 0.0;
    float distance_calculated_cm = 0.0;
    float distance_to_next_edge = 0.0;
    float minDistance = 0.0;
    
    // Define angle bounds based on case number
    float angle_lower_bound = 0.0;
    float angle_upper_bound = 0.0;
    
    switch (case_num) {
        case 1: // 0° to 90°
            angle_lower_bound = 0.0;
            angle_upper_bound = 90.0;
            alfa_degrees = -(angle_lidar - 90);
            break;
        case 2: // 90° to 180°
            angle_lower_bound = 90.0;
            angle_upper_bound = 180.0;
            alfa_degrees = angle_lidar - 90.0;
            break;
        case 3: // 180° to 270°
            angle_lower_bound = 180.0;
            angle_upper_bound = 270.0;
            alfa_degrees = angle_lidar - 180.0;
            break;
        case 4: // 270° to 360°
            angle_lower_bound = 270.0;
            angle_upper_bound = 360.0;
            alfa_degrees = angle_lidar - 270.0;
            break;
        default:
            return 0.0; // Invalid case
    }

    alfa_radianse = alfa_degrees * PI / 180;
    distance_cm = distance_lidar / 10; // Convert distance in mm to cm
    distance_calculated_cm = scan_height / cos(alfa_radianse);

    if (distance_cm > tolerance * distance_calculated_cm) {
        distance_to_next_edge = tan(alfa_radianse) * scan_height;
    }
    
    // Process data only within the selected angle range
    if (angle_lidar > angle_lower_bound && angle_lidar < angle_upper_bound && distance_lidar > 0) {
        float filtered_distance = distance_to_next_edge;
        
        if (filtered_distance != 0.0) {
            // Get minimum over the defined period
            minDistance = getMinValueOverTime(filtered_distance, COLLECTION_PERIOD);
        }
    }
    
    return minDistance;
}



float getMinValueOverTime(float currentValue, unsigned long collectionTime) {
  // Settings
  const int MAX_SAMPLES = 500;  // Maximum array size (adjust as needed)
  static float samples[MAX_SAMPLES];
  static int sampleCount = 0;
  static unsigned long periodStart = 0;
  
  // Initialize on first run or after completion
  if (sampleCount == 0) {
    periodStart = millis();
  }
  
  // Add current value to array (if there's space)
  if (sampleCount < MAX_SAMPLES) {
    samples[sampleCount] = currentValue;
    sampleCount++;
  }
  
  // Check if collection period is complete
  if (millis() - periodStart >= collectionTime && sampleCount > 0) {
    // Find minimum value in collected samples
    float minVal = samples[0];
    for (int i = 1; i < sampleCount; i++) {
      if (samples[i] < minVal) {
        minVal = samples[i];
      }
    }
    
    // Reset for next collection period
    sampleCount = 0;
    return minVal;
  }
  
  return NAN; // Still collecting data
}


  
