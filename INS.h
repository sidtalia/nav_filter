#pragma once

#include "uNavAHRS.h"
#include "MPU9250.h"
#include "EEPROM.h"
#include "estimator_ekf.h"
#include "GPS.h"
#include "SparkFunBME280.h"

#define GLOBAL_DT 0.0025

MPU9250 Imu(SPI,10);
BME280 Baro;
GPS gps;

uNavAHRS Filter;
estimator_ekf state_filter;

// EEPROM buffer and variables to load accel and mag bias 
// and scale factors from CalibrateMPU9250.ino
uint8_t EepromBuffer[48];
float axb, axs, ayb, ays, azb, azs;
float hxb, hxs, hyb, hys, hzb, hzs;
float AHRS[5];
// timers to measure performance
unsigned long tstart, tstop;
float start_alt = 0;

void caliberateIMU()
{
  // calibrating accelerometer
  Imu.calibrateAccel(); //level
  delay(5000);
  Imu.calibrateAccel(); //left
  delay(5000);
  Imu.calibrateAccel(); //right
  delay(5000);
  Imu.calibrateAccel(); //front
  delay(5000);
  Imu.calibrateAccel(); //rear
  delay(5000);
  Imu.calibrateAccel(); //on it's back
  delay(5000);
  // calibrating magnetometer
  float value;
  Imu.calibrateMag(); //rotate in all directions.
  // saving to EEPROM
  value = Imu.getAccelBiasX_mss();
  memcpy(EepromBuffer,&value,4);
  value = Imu.getAccelScaleFactorX();
  memcpy(EepromBuffer+4,&value,4);
  value = Imu.getAccelBiasY_mss();
  memcpy(EepromBuffer+8,&value,4);
  value = Imu.getAccelScaleFactorY();
  memcpy(EepromBuffer+12,&value,4);
  value = Imu.getAccelBiasZ_mss();
  memcpy(EepromBuffer+16,&value,4);
  value = Imu.getAccelScaleFactorZ();
  memcpy(EepromBuffer+20,&value,4);
  value = Imu.getMagBiasX_uT();
  memcpy(EepromBuffer+24,&value,4);
  value = Imu.getMagScaleFactorX();
  memcpy(EepromBuffer+28,&value,4);
  value = Imu.getMagBiasY_uT();
  memcpy(EepromBuffer+32,&value,4);
  value = Imu.getMagScaleFactorY();
  memcpy(EepromBuffer+36,&value,4);
  value = Imu.getMagBiasZ_uT();
  memcpy(EepromBuffer+40,&value,4);
  value = Imu.getMagScaleFactorZ();
  memcpy(EepromBuffer+44,&value,4);
  for (size_t i=0; i < sizeof(EepromBuffer); i++) {
    EEPROM.write(i,EepromBuffer[i]);
  }
}

void INS_setup()
{
	  // start communication with IMU 
	int status = Imu.begin();
	// load and set accel and mag bias and scale
	// factors from CalibrateMPU9250.ino 
	for (size_t i=0; i < sizeof(EepromBuffer); i++) {
		EepromBuffer[i] = EEPROM.read(i);
	}
	memcpy(&axb,EepromBuffer+0,4);
	memcpy(&axs,EepromBuffer+4,4);
	memcpy(&ayb,EepromBuffer+8,4);
	memcpy(&ays,EepromBuffer+12,4);
	memcpy(&azb,EepromBuffer+16,4);
	memcpy(&azs,EepromBuffer+20,4);
	memcpy(&hxb,EepromBuffer+24,4);
	memcpy(&hxs,EepromBuffer+28,4);
	memcpy(&hyb,EepromBuffer+32,4);
	memcpy(&hys,EepromBuffer+36,4);
	memcpy(&hzb,EepromBuffer+40,4);
	memcpy(&hzs,EepromBuffer+44,4);

	Imu.setAccelCalX(axb,axs);
	Imu.setAccelCalY(ayb,ays);
	Imu.setAccelCalZ(azb,azs);

	Imu.setMagCalX(hxb,hxs);
	Imu.setMagCalY(hyb,hys);
	Imu.setMagCalZ(hzb,hzs);

	//  Serial.print(gps.initialize());
	Baro.beginSPI(9);
	Baro.readTempC();
	Baro.readFloatHumidity();
	start_alt = Baro.readFloatAltitudeMeters();

	gps.initialize(); // initialize gps
	Imu.readSensor();
	while(!Filter.update(Imu.getGyroX_rads(),Imu.getGyroY_rads(),Imu.getGyroZ_rads(),Imu.getAccelX_mss(),Imu.getAccelY_mss(),Imu.getAccelZ_mss(),Imu.getMagX_uT(),Imu.getMagY_uT(),Imu.getMagZ_uT()))
	{
		Imu.readSensor();    
	} //this initializes the IMU
  
}

void INS_run(bool reset_system)
{
	uint32_t time_stamp_ms = millis();
	uint64_t time_stamp_us = micros();
	tstart = micros();
	// read the sensor
	Imu.readSensor();
	// update the filter
	Filter.update(Imu.getGyroX_rads(),Imu.getGyroY_rads(),Imu.getGyroZ_rads(),Imu.getAccelX_mss(),Imu.getAccelY_mss(),Imu.getAccelZ_mss(),Imu.getMagX_uT(),Imu.getMagY_uT(),Imu.getMagZ_uT());
	// order in which info is provided doesn't matter as long as you do it before calling "run_filter"
	float A[3], G[3], M[3],zeros[3]={0,0,0}, flowData[7],AHRS[5]={Filter.getRoll_rad(),Filter.getPitch_rad(),Filter.getHeading_rad(), Filter.getGyroBiasX_rads(), Filter.getGyroBiasY_rads()};
	float RngFinderDist =0;
	Imu.getSensorRead(A,G,M);
	float alt = Baro.readFloatAltitudeMeters();
	state_filter.setIMUData(A, G, GLOBAL_DT, time_stamp_ms, time_stamp_us);
	state_filter.setMagData(M, zeros, Filter.magUpdated_ );

	state_filter.setGPSData(gps.latitude, gps.longitude, gps.hMSL, gps.gSpeed, gps.headMot, gps.VelNED, gps.hAcc, gps.hAcc, gps.sAcc, gps.magDec, gps.magAcc, gps.delta_t, gps.fixType, gps.tick);

	gps.tick = false; //reset tick to be false after data has been used.
	state_filter.setAirData(0,0,0,false);
	state_filter.setFlowData(flowData, GLOBAL_DT, false);
	state_filter.setAhrsData(AHRS);
	state_filter.setDistData(RngFinderDist,0,false);
	state_filter.run_filter(reset_system);
	reset_system = false;
	tstop = micros();
}