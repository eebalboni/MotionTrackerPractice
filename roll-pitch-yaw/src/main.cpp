// SPDX-FileCopyrightText: 2020 Kattni Rembor for Adafruit Industries
//
// SPDX-License-Identifier: MIT

// Basic demo for accelerometer, lsm_gyro, and magnetometer readings
// from the following Adafruit ST Sensor combo boards:
// * LSM6DSOX + LIS3MDL FeatherWing : https://www.adafruit.com/product/4565
// * ISM330DHCX + LIS3MDL FeatherWing https://www.adafruit.com/product/4569
// * LSM6DSOX + LIS3MDL Breakout : https://www.adafruit.com/product/4517
// * LSM6DS33 + LIS3MDL Breakout Lhttps://www.adafruit.com/product/4485

//#include <Adafruit_LSM6DSOX.h>
//Adafruit_LSM6DSOX lsm6ds;

// To use with the LSM6DS33+LIS3MDL breakout, uncomment these two lines
// and comment out the lines referring to the LSM6DSOX above
//#include <Adafruit_LSM6DS33.h>
//Adafruit_LSM6DS33 lsm6ds;

// To use with the ISM330DHCX+LIS3MDL Feather Wing, uncomment these two lines
// and comment out the lines referring to the LSM6DSOX above
//#include <Adafruit_ISM330DHCX.h>
//Adafruit_ISM330DHCX lsm6ds;

// To use with the LSM6D3TR-C+LIS3MDL breakout, uncomment these two lines
// and comment out the lines referring to the LSM6DSOX above
#include <Adafruit_LSM6DS3TRC.h>
Adafruit_LSM6DS3TRC lsm6ds;

#include <Adafruit_LIS3MDL.h>
Adafruit_LIS3MDL lis3mdl;

#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>

Adafruit_ICM20948 icm;
uint16_t measurement_delay_us = 65535; // Delay between measurements for testing

#include <MahonyAHRS.h>
Mahony filter;

unsigned long lastTime = millis();
float icm_thetaG=0;
float icm_phiG=0;
unsigned long oldTime;
float dt;
bool done = false;
float start_lsm_gx;
float start_lsm_gy;
float start_lsm_gz;
float icm_PthetaA = 0;
float icm_PphiA = 0;
float icm_theta = 0;
float icm_phi = 0;
float angleT = 0;
float angleP = 0;

void LSM_Initialization(){
  // lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  Serial.print("Accelerometer range set to: ");
  switch (lsm6ds.getAccelRange()) {
  case LSM6DS_ACCEL_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case LSM6DS_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case LSM6DS_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case LSM6DS_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  // lsm6ds.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Accelerometer data rate set to: ");
  switch (lsm6ds.getAccelDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  // lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS );
  Serial.print("Gyro range set to: ");
  switch (lsm6ds.getGyroRange()) {
  case LSM6DS_GYRO_RANGE_125_DPS:
    Serial.println("125 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  case ISM330DHCX_GYRO_RANGE_4000_DPS:
    Serial.println("4000 degrees/s");
    break;
  }
  // lsm6ds.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Gyro data rate set to: ");
  switch (lsm6ds.getGyroDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  // You can check the datarate by looking at the frequency of the DRDY pin
  Serial.print("Magnetometer data rate set to: ");
  switch (lis3mdl.getDataRate()) {
    case LIS3MDL_DATARATE_0_625_HZ: Serial.println("0.625 Hz"); break;
    case LIS3MDL_DATARATE_1_25_HZ: Serial.println("1.25 Hz"); break;
    case LIS3MDL_DATARATE_2_5_HZ: Serial.println("2.5 Hz"); break;
    case LIS3MDL_DATARATE_5_HZ: Serial.println("5 Hz"); break;
    case LIS3MDL_DATARATE_10_HZ: Serial.println("10 Hz"); break;
    case LIS3MDL_DATARATE_20_HZ: Serial.println("20 Hz"); break;
    case LIS3MDL_DATARATE_40_HZ: Serial.println("40 Hz"); break;
    case LIS3MDL_DATARATE_80_HZ: Serial.println("80 Hz"); break;
    case LIS3MDL_DATARATE_155_HZ: Serial.println("155 Hz"); break;
    case LIS3MDL_DATARATE_300_HZ: Serial.println("300 Hz"); break;
    case LIS3MDL_DATARATE_560_HZ: Serial.println("560 Hz"); break;
    case LIS3MDL_DATARATE_1000_HZ: Serial.println("1000 Hz"); break;
  }

  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  Serial.print("Range set to: ");
  switch (lis3mdl.getRange()) {
    case LIS3MDL_RANGE_4_GAUSS: Serial.println("+-4 gauss"); break;
    case LIS3MDL_RANGE_8_GAUSS: Serial.println("+-8 gauss"); break;
    case LIS3MDL_RANGE_12_GAUSS: Serial.println("+-12 gauss"); break;
    case LIS3MDL_RANGE_16_GAUSS: Serial.println("+-16 gauss"); break;
  }

  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  Serial.print("Magnetometer performance mode set to: ");
  switch (lis3mdl.getPerformanceMode()) {
    case LIS3MDL_LOWPOWERMODE: Serial.println("Low"); break;
    case LIS3MDL_MEDIUMMODE: Serial.println("Medium"); break;
    case LIS3MDL_HIGHMODE: Serial.println("High"); break;
    case LIS3MDL_ULTRAHIGHMODE: Serial.println("Ultra-High"); break;
  }

  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  Serial.print("Magnetometer operation mode set to: ");
  // Single shot mode will complete conversion and go into power down
  switch (lis3mdl.getOperationMode()) {
    case LIS3MDL_CONTINUOUSMODE: Serial.println("Continuous"); break;
    case LIS3MDL_SINGLEMODE: Serial.println("Single mode"); break;
    case LIS3MDL_POWERDOWNMODE: Serial.println("Power-down"); break;
  }

  lis3mdl.setIntThreshold(500);
  lis3mdl.configInterrupt(false, false, true, // enable z axis
                          true, // polarity
                          false, // don't latch
                          true); // enabled!
}

void ICM_Initialization(){
  Serial.println("ICM20948 Found!");
  // icm.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
  Serial.print("Accelerometer range set to: ");
  switch (icm.getAccelRange()) {
  case ICM20948_ACCEL_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case ICM20948_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case ICM20948_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case ICM20948_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  Serial.println("OK");

  // icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
  Serial.print("Gyro range set to: ");
  switch (icm.getGyroRange()) {
  case ICM20948_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  }

  //  icm.setAccelRateDivisor(4095);
  uint16_t accel_divisor = icm.getAccelRateDivisor();
  float accel_rate = 1125 / (1.0 + accel_divisor);

  Serial.print("Accelerometer data rate divisor set to: ");
  Serial.println(accel_divisor);
  Serial.print("Accelerometer data rate (Hz) is approximately: ");
  Serial.println(accel_rate);

  //  icm.setGyroRateDivisor(255);
  uint8_t gyro_divisor = icm.getGyroRateDivisor();
  float gyro_rate = 1100 / (1.0 + gyro_divisor);

  Serial.print("Gyro data rate divisor set to: ");
  Serial.println(gyro_divisor);
  Serial.print("Gyro data rate (Hz) is approximately: ");
  Serial.println(gyro_rate);

  // icm.setMagDataRate(AK09916_MAG_DATARATE_10_HZ);
  Serial.print("Magnetometer data rate set to: ");
  switch (icm.getMagDataRate()) {
  case AK09916_MAG_DATARATE_SHUTDOWN:
    Serial.println("Shutdown");
    break;
  case AK09916_MAG_DATARATE_SINGLE:
    Serial.println("Single/One shot");
    break;
  case AK09916_MAG_DATARATE_10_HZ:
    Serial.println("10 Hz");
    break;
  case AK09916_MAG_DATARATE_20_HZ:
    Serial.println("20 Hz");
    break;
  case AK09916_MAG_DATARATE_50_HZ:
    Serial.println("50 Hz");
    break;
  case AK09916_MAG_DATARATE_100_HZ:
    Serial.println("100 Hz");
    break;
  }
  Serial.println();
}

float roundToFirstDecimal(float num) {
  return round(num * 10.0) / 10.0;
}

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  ///*
  Serial.println("Adafruit ICM20948 test!");
  // Try to initialize!
  if (!icm.begin_I2C(0x69,&Wire1)) {
    // if (!icm.begin_SPI(ICM_CS)) {
    // if (!icm.begin_SPI(ICM_CS, ICM_SCK, ICM_MISO, ICM_MOSI)) {

    Serial.println("Failed to find ICM20948 chip");
    while (1) {
      delay(10);
    }
  }else {
      Serial.println("ICM Found!!");
  }
  //*/


  /*
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit LSM6DS+LIS3MDL test!");

  bool lsm6ds_success, lis3mdl_success;

  // hardware I2C mode, can pass in address & alt Wire

  lsm6ds_success = lsm6ds.begin_I2C(0x6A, &Wire1);
  lis3mdl_success = lis3mdl.begin_I2C(0x1C, &Wire1);

  if (!lsm6ds_success){
    Serial.println("Failed to find LSM6DS chip");
  }
  if (!lis3mdl_success){
    Serial.println("Failed to find LIS3MDL chip");
  }
  if (!(lsm6ds_success && lis3mdl_success)) {
    while (1) {
      delay(10);
    }
  }

  Serial.println("LSM6DS and LIS3MDL Found!");
  */
  ICM_Initialization();
  //LSM_Initialization();
  
  filter.begin(100);
  oldTime = millis();
}

void loop() {

  sensors_event_t lsm_accel, lsm_gyro, lsm_mag, lsm_temp;
  sensors_event_t icm_accel, icm_gyro, icm_mag, icm_temp;


  //  /* Get new normalized sensor events */
  lsm6ds.getEvent(&lsm_accel, &lsm_gyro, &lsm_temp);
  lis3mdl.getEvent(&lsm_mag);

  icm.getEvent(&icm_accel, &icm_gyro, &icm_temp, &icm_mag);
 /*
  float lsm_ax = lsm_accel.acceleration.x;
  float lsm_ay = lsm_accel.acceleration.y;
  float lsm_az = lsm_accel.acceleration.z;
  float lsm_gx = lsm_gyro.gyro.x;
  float lsm_gy = lsm_gyro.gyro.y;
  float lsm_gz = lsm_gyro.gyro.z;
  float lsm_mx = lsm_mag.magnetic.x;
  float lsm_my = lsm_mag.magnetic.y;
  float lsm_mz = lsm_mag.magnetic.z;

  float lsm_thetaA = atan(lsm_ax/lsm_az) * (180/(M_PI)); // Theta is about the y axis
  float lsm_phiA = atan(lsm_ay/lsm_az) * (180/(M_PI)); // phi is about the x access

  if (done == false){
    start_lsm_gx = lsm_gx;
    start_lsm_gy = lsm_gy;
    start_lsm_gz = lsm_gz;
    done=true;
  }

  dt = (millis()-oldTime)/1000.0;
  Serial.println(dt, 5);
  oldTime = millis();
  lsm_thetaG= (lsm_thetaG + (lsm_gy-start_lsm_gy)*dt); // Theta is about the y axis
  lsm_phiG = (lsm_phiG + (lsm_gx-start_lsm_gx)*dt); // phi is about the x access
*/

  float icm_ax = icm_accel.acceleration.x;
  float icm_ay = icm_accel.acceleration.y;
  float icm_az = icm_accel.acceleration.z;
  float icm_gx = icm_gyro.gyro.x;
  float icm_gy = icm_gyro.gyro.y;
  float icm_gz = icm_gyro.gyro.z;
  float icm_mx = icm_mag.magnetic.x;
  float icm_my = icm_mag.magnetic.y;
  float icm_mz = icm_mag.magnetic.z;
  
  /*
  if (icm_ax >= 0 && icm_az >= 0){
    angle1 = 0;
  } else if (icm_ax < 0 && icm_az > 0){
    angle1 = 270;
  } else if (icm_ax < 0 && icm_az < 0){
    angle1 = 180;
  } else {
    angle1 = 90;
  } 
  
  if (icm_ay >= 0 && icm_az >= 0){
    angle2 = 0;
  } else if (icm_ay <= 0 && icm_az >= 0){
    angle2 = -90;
  } else if (icm_ay <= 0 && icm_az <= 0){
    angle2 = 180;
  } else {
    angle2 = -270;
  }
  */


  // Calibration
  // Set the gyro on several different surfaces and read
  // See if thers a constant it goes to

  if (icm_ax >= 0 && icm_az >= 0){
    angleT = 0;
  } else if (icm_ax >= 0 && icm_az <= 0){
    angleT = -180;
  } else if (icm_ax <= 0 && icm_az <= 0){
    angleT = 180;
  } else {
    angleT = -360;
  }

  if (icm_ay <= 0 && icm_az >= 0){
    angleP = 0;
  } else if (icm_ay <= 0 && icm_az <= 0){
    angleP = -180;
  } else if (icm_ay >= 0 && icm_az <= 0){
    angleP = 180;
  } else {
    angleP = -360;
  }

  float icm_thetaA = abs(angleT + abs(atan(icm_ax/icm_az)*(180/(M_PI))) ); // Theta is about the y axis
  float icm_phiA = abs(angleP + abs(atan(icm_ay/icm_az)*(180/(M_PI)))); // phi is about the x access

  float icm_FthetaA = icm_PthetaA*0.9 + icm_thetaA*0.1; // Theta is about the y axis
  float icm_FphiA   = icm_PphiA*0.9 + icm_phiA*0.1; // phi is about the x access
  

  dt = (millis()-oldTime)/1000.0;
  oldTime = millis();
  icm_thetaG= (icm_thetaG + roundToFirstDecimal(icm_gy)*dt); // Theta is about the y axis
  icm_phiG = (icm_phiG + roundToFirstDecimal(icm_gx)*dt); // phi is about the x access

  icm_PthetaA = icm_FthetaA;
  icm_PphiA = icm_FphiA;

  icm_theta = (icm_theta + (roundToFirstDecimal(icm_gy)*dt)*(180.0/(M_PI)))*.95 + icm_thetaA*.05;
  icm_phi = (icm_phi + (roundToFirstDecimal(icm_gx)*dt)*(180.0/(M_PI)))*.95 + icm_phiA*.05;

  float icm_thetaRad = icm_theta/(180.0/(M_PI));
  float icm_phiRad = icm_phi/(180.0/(M_PI));


  float icm_proj_mx = icm_mx*cos(icm_thetaRad) + icm_my*sin(icm_phiRad)*sin(icm_thetaRad) - icm_mz*cos(icm_phiRad)*sin(icm_thetaRad);
  float icm_proj_my = icm_my*cos(icm_phiRad) + icm_mz*sin(icm_phiRad);

  float yaw = atan(icm_proj_my/icm_proj_mx)*(180.0/(M_PI));

  /*
  if (done == false){
    start_icm_gx = icm_gx;
    start_icm_gy = icm_gy;
    start_icm_gz = icm_gz;
    done=true;
  }

  dt = (millis()-oldTime)/1000.0;
  Serial.println(dt, 5);
  oldTime = millis();
  icm_thetaG= (icm_thetaG + (icm_gy-start_icm_gy)*dt); // Theta is about the y axis
  icm_phiG = (icm_phiG + (icm_gx-start_icm_gx)*dt); // phi is about the x access
  */

  /* Display the results LSM (acceleration is measured in m/s^2) 
  Serial.print("\t\tLSM Accel X: ");
  Serial.print(lsm_ax, 4);
  Serial.print(" \tY: ");
  Serial.print(lsm_ay, 4);
  Serial.print(" \tZ: ");
  Serial.print(lsm_az, 4);
  Serial.println(" \tm/s^2 ");*/

  /* Display the results LSM (velocity is measured in m/s) 
  Serial.print("\t\tLSM Gyro X: ");
  Serial.print(icm_gx, 4);
  Serial.print(" \tY: ");
  Serial.print(icm_gy, 4);
  Serial.print(" \tZ: ");
  Serial.print(icm_gz, 4);
  Serial.println(" \tm/s ");*/

  //filter.updateIMU(icm_gx, icm_gy, icm_gz, icm_ax, icm_ay, icm_az);
  // Get yaw, pitch, and roll from the filter
  //float yaw = filter.getYaw();
  //float pitch = filter.getPitch();
  //float roll = filter.getRoll();

  if (millis()-lastTime>=1){
    /*
    // Display results
    Serial.print("Theta (according to accel): "); //DONT DO THIS NOT RELIABLE
    Serial.print(icm_FthetaA);
    Serial.print("\t\t");
    Serial.print("Phi (according to accel): ");
    Serial.println(icm_FphiA);
    */

    /*
    Serial.print("Theta (according to gyro): ");
    Serial.print(icm_thetaG*(180.0/(M_PI)));
    Serial.print("\t\t");
    Serial.print("Phi (according to gyro): ");
    Serial.println(icm_phiG*(180.0/(M_PI)));
    */

    /*
    Serial.print("Theta: ");
    Serial.print(icm_theta);
    Serial.print("\t\t");
    Serial.print("Phi: ");
    Serial.println(icm_phi);
    */

    /*
    Serial.print("mag x: ");
    Serial.print(icm_mx);
    Serial.print("\t\t");
    Serial.print("mag y: ");
    Serial.print(icm_my);
    Serial.print("\t\t");
    Serial.print("mag z: ");
    Serial.println(icm_mz);
    */
    /*
    Serial.print("acc x: ");
    Serial.print(icm_ax);
    Serial.print("\t\t");
    Serial.print("acc y: ");
    Serial.print(icm_ay);
    Serial.print("\t\t");
    Serial.print("acc z: ");
    Serial.println(icm_az);
    */

    //Serial.print("Tilt: ");
    Serial.println(yaw);

    lastTime = millis();
  }
  





/* Display the results ICM (acceleration is measured in m/s^2) 
  Serial.print("\t\tICM Accel X: ");
  Serial.print(icm_ax, 4);
  Serial.print(" \tY: ");
  Serial.print(icm_ay, 4);
  Serial.print(" \tZ: ");
  Serial.print(icm_az, 4);
  Serial.println(" \tm/s^2 ");

  /* Display the results LSM (velocity is measured in m/s) 
  Serial.print("\t\tLSM Gyro X: ");
  Serial.print(lsm_gx, 4);
  Serial.print(" \tY: ");
  Serial.print(lsm_gy, 4);
  Serial.print(" \tZ: ");
  Serial.print(lsm_gz, 4);
  Serial.println(" \tm/s ");*/
  
}
