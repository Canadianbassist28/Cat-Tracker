# Cat-Tracker
Team project for CS3560
### Goal: Use Intel Realsense D435i camera to do indoor mapping

### Image Processing
//add info on image processing

### Mapping
//add info on mapping


### Motion
To succesfully map the 3D coordinates from the camera, We need to accuratly measure the position and orientation of the camera as it moves through a space.

To do so, we plan to use the D435i's internal IMU (acceleromter and gyroscope) to calculate positon and orientation. While this approach can work using the d435i's IMU, the t265 camera from intel offers a superior IMU, as it calculates pose data (position and orientation) internaly, additionaly offers more accurate data, with less drift.

###### Aproach:
Located in src/Motion/realsenseMotion.py, an object that will perform processing on motion data.
1. Calculate orientation and rotiation matrix:
    1. Uses gyroscope data, integrated over time, to calculate Eular angle orientation (pitch, yaw, roll) in radians 
    2. Calculate rotiation matrix from eular angle orientation
2. Use rotiation matrix to remove gravity from accelerometer data to find __Linear Acceleration__
    1. Multiply Rotation matrix by the gravity vector (0,9.81,0)
    2. Subtracting the accelerometer data by this resulting vector will give the Linear Acceleration
3. Intagrating this Linear Accleration will give the velocity, then again will give the relative positition of the camera in (x,y,z)


##### Work to be done:
* filter data to return more smooth/constistant results
* develop method to transform 3D coordinates from the camera to the relative postition
