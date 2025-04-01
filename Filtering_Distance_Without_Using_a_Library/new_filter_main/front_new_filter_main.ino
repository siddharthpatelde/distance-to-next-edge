
#include <Filters.h>
#include <ArduinoJson.h>
#include <RPLidar.h>

#define RPLIDAR_MOTOR 3 //The PWM pin for control the speed of RPLIDAR's motor (MOTOCTRL).

#define lidar_height 22  //lidar position 25 cm from the ground
#define tolerance 1.3 //define the tolerance factor for edge detection logic

// Parameters for data collection
const unsigned long COLLECTION_PERIOD = 1000; // T = 1 seconds (adjust as needed)
const unsigned long SAMPLE_INTERVAL = 100;    // Sample every 10ms (adjust as needed) const int MAX_SAMPLES = COLLECTION_PERIOD / SAMPLE_INTERVAL; // X = number of samples



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
    float distance_non_filted = 0.0;

      if (distance > 0) {  // Ensure distance is valid
        if (angle > 270 && angle < 350) {  // 30 below from horizontal level
          
          distance_non_filted = get_next_holes_from_laserscan_non_filtered(angle, distance,lidar_height);
          if (distance_non_filted != 0.0) {

              
            // Call the function (Sensor data,  interval,  collection period)
            float minDistance = getMinValueOverTime(distance_non_filted, SAMPLE_INTERVAL, COLLECTION_PERIOD);
            
            if (!isnan(minDistance)) {
              // Serial.print("Minimum distance: ");
              // Serial.println(minDistance);

            // Create a JSON document to serialize
            StaticJsonDocument<200> doc;
            doc["lidar_height"] = lidar_height;
            doc["Distance"] = minDistance;

            // Serialize JSON to Serial
            serializeJson(doc, Serial);
            Serial.println(); // Print newline for readability
            }
          }
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

    distance_to_next_edge = tan(alfa_radianse)*scan_height;
  }

 return distance_to_next_edge;
}



// Function to collect samples and find minimum value over time period
float getMinValueOverTime(float currentValue, unsigned long sampleInterval, unsigned long collectionPeriod) {
  // Static variables maintain state between function calls
  static float minValue;
  static unsigned long lastSampleTime = 0;
  static unsigned long collectionStartTime = 0;
  static bool isFirstSample = true;
  
  unsigned long currentTime = millis();
  
  if (isFirstSample) {
    minValue = currentValue; // Initialize with first value
    collectionStartTime = currentTime;
    lastSampleTime = currentTime;
    isFirstSample = false;
    return NAN; // No result yet
  }
  
  // Check if collection period is complete
  if (currentTime - collectionStartTime >= collectionPeriod) {
    float result = minValue; // Store the result
    // Reset for next collection period
    isFirstSample = true;
    return result;
  }
  
  // Take samples at regular intervals
  if (currentTime - lastSampleTime >= sampleInterval) {
    if (currentValue < minValue) {
      minValue = currentValue; // Update minimum
    }
    lastSampleTime = currentTime;
  }
  
  return NAN; // Still collecting data
}
