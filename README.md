# 2D LIDAR Edge Detection using Raspberry Pi Pico

This project focuses on building a logic to calculate the distance to the next edge when a robot equipped with a 2D LIDAR sensor is placed on a table. The project leverages the **RPlidar.h** library and a **Raspberry Pi Pico** to work with the LIDAR sensor.

## Project Overview

I am currently in the process of refining this project, and so far, I've conducted several experiments to achieve the final goal—calculating the distance to the next edge detected by the LIDAR sensor.

For now, this project emphasizes:

- Building the logic for edge detection
- Working with the **RPlidar.h** library to process data from the 2D LIDAR sensor
- Using **Raspberry Pi Pico** as the hardware platform for the implementation

### Library Source

The **RPlidar.h** library can be found here: [RPLidar Arduino Library](https://github.com/robopeak/rplidar_arduino/tree/master)

## Project Workflow

Here’s the workflow for this project and the current status of each task:

### - **Continuous Data Retrieval from LIDAR (Important)** – **DONE** (click to play the video) 



[![Proof_Lidar_Data_Stream](1.png)](https://drive.google.com/file/d/1rfpvJtDQD67CvJ5mutH_rplSHh9WAc60/view?usp=drive_link)

**Circuit Diagram:**

![Circuit](circuit.png)

### - **LIDAR Hole Processor (Not so important)** – **DONE** (click to play the video) 


[![Proof_Hole_Processor](2.png)](https://drive.google.com/file/d/1ZKafViBrdYzuwkXnPgtSrBZSwxWRYnGK/view?usp=drive_link)

[![Sorted_Lidar_Array](3.png)](https://drive.google.com/file/d/1vgB4sXewlVeRfvCux_HYK_wVaKtTB3Rp/view?usp=drive_link)

[![Lidar_Output](4.png)](https://drive.google.com/file/d/1PlDPItqsNulzSENrVX-oHoexpWoK3kjX/view?usp=drive_link)

### - **Edge Detection using LIDAR (Important)** – **DONE** (click to play the video) 


[![Proof_Edge_Detection](5.png)](https://drive.google.com/file/d/1PsGvN44Rhcd295yuS7_6knRiBt5i1g9i/view?usp=drive_link)
- **Calculating Distance to the Next Edge** – **REMAINING** (click to play the video) 


## Next Steps

I am currently working on the logic for calculating the distance to the next edge detected by the LIDAR sensor. Once this step is completed, the project will provide a robust solution for edge detection and distance calculation using a 2D LIDAR sensor and a Raspberry Pi Pico.

Feel free to explore the code and contribute if you'd like!

## Author

**Siddharth A. Patel**  
GitHub: [siddharthpatelde](https://github.com/siddharthpatelde?tab=overview&from=2024-10-01&to=2024-10-17)
