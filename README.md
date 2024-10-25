# 2D LIDAR Edge Detection using Raspberry Pi Pico

This project focuses on building a logic to calculate the distance to the next edge when a robot equipped with a 2D LIDAR sensor is placed on a table. The project leverages the **RPlidar.h** library and a **Raspberry Pi Pico** to work with the LIDAR sensor.

## Project Overview

I am currently in the process of refining this project, and so far, I've conducted several experiments to achieve the final goal—calculating the distance to the next edge detected by the LIDAR sensor.

For now, this project emphasizes:

- Building the logic for edge detection
- Working with the **RPlidar.h** library to process data from the 2D LIDAR sensor
- Using **Raspberry Pi Pico** as the hardware platform for the implementation

### Library Sources

The **RPlidar.h** library can be found here: [RPLidar Arduino Library](https://github.com/robopeak/rplidar_arduino/tree/master)

The **Filters** library for data filtering can be found here: [Arduino Filters Library](https://github.com/edargelies/arduino_eq/tree/master/libraries/Filters)

## Project Workflow

Here’s the workflow for this project and the current status of each task:

### - **Continuous Data Retrieval from LIDAR (Important)** – **DONE** (click to play the video) 

[![Proof_Lidar_Data_Stream](image/1.png)](https://drive.google.com/file/d/1rfpvJtDQD67CvJ5mutH_rplSHh9WAc60/view?usp=drive_link)

**Circuit Diagram:**

![Circuit](image/circuit.png)

### - **LIDAR Hole Processor (Not so important)** – **DONE** (click to play the video) 

[![Proof_Hole_Processor](image/2.png)](https://drive.google.com/file/d/1ZKafViBrdYzuwkXnPgtSrBZSwxWRYnGK/view?usp=drive_link)

[![Sorted_Lidar_Array](image/3.png)](https://drive.google.com/file/d/1vgB4sXewlVeRfvCux_HYK_wVaKtTB3Rp/view?usp=drive_link)

[![Lidar_Output](image/4.png)](https://drive.google.com/file/d/1PlDPItqsNulzSENrVX-oHoexpWoK3kjX/view?usp=drive_link)

### - **Edge Detection using LIDAR (Important)** – **DONE** (click to play the video) 

[![Proof_Edge_Detection](image/5.png)](https://drive.google.com/file/d/1PsGvN44Rhcd295yuS7_6knRiBt5i1g9i/view?usp=drive_link)

### - **Calculating Distance to the Next Edge (Non-filtered)** – **DONE** (click to play the video)

[![Proof 1](image/proof1.png)](https://drive.google.com/file/d/1WEArDvpSgvIyKT1jKW4AdApa847UnFev/view?usp=sharing)

[![Proof 2](image/proof2.png)](https://drive.google.com/file/d/1mivxFB5GDWuHXXUXX1iQcaiiQBgAlYqu/view?usp=sharing)

### - **Calculating Distance to the Next Edge (filtered)** – **DONE** (click to play the video)

[![Proof_Filtered_Distance](image/filtered_proof.png)](https://drive.google.com/file/d/1sBQNz1nN8M1QA3F-Lf8BnC2cx3LwU1n8/view?usp=sharing)

## Author

**Siddharth A. Patel**  
GitHub: [siddharthpatelde](https://github.com/siddharthpatelde?tab=overview&from=2024-10-01&to=2024-10-17)
