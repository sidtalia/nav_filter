#pragma once
#include "estimator_22states.h"


#define MAG_SCALING float(1/49.0) // strength of earth's mag. field w/o interference.

class estimator_ekf
{
private:
	uint32_t msecVelDelay ;
	uint32_t msecPosDelay ;
	uint32_t msecHgtDelay ;
	uint32_t msecRngDelay ;
	uint32_t msecMagDelay;
	uint32_t msecTasDelay ;
	uint32_t msecOptFlowDelay;
	uint32_t msecOdomDelay;

	// IMU input data variables
	float imuIn;
	Vector3f lastAngRate;
	Vector3f lastAccel;
	float IMUtimestamp;
	uint32_t IMUmsec;
	uint64_t IMUusec;
	float cov_pred_dt;
	// Magnetometer input data variables
	float magIn;
	float tempMag[8];
	float tempMagPrev[8];
	float posNED[3];
	float MAGtimestamp;
	uint32_t MAGmsec;
	uint32_t lastMAGmsec;
	bool newDataMag;

	// AHRS input data variables
	float ahrsIn;
	float tempAhrs[7];
	float tempAhrsPrev[7];
	float AHRStimestamp;
	uint32_t AHRSmsec;
	uint32_t lastAHRStime;
	float ahrsEul[3];
	float ahrsErrorRP;
	float ahrsErrorYaw;
	float eulerEst[3]; // Euler angles calculated from filter states
	float eulerDif[3]; // difference between Euler angle estimated by EKF and the AHRS solution
	float gpsRaw[7];
	// ADS input data variables
	float adsIn;
	float tempAds[10];
	float tempAdsPrev[10];
	float ADStimestamp;
	uint32_t ADSmsec;
	uint32_t lastADSmsec;
	float Veas;
	bool newAdsData;
	bool new_ASData;
	bool newDataGps;
	bool newOptFlowData;

	bool newDataOdom;

	float onboardTimestamp;
	uint32_t onboardMsec;
	uint32_t lastOnboardMsec;
	bool newOnboardData;

	float onboardPosNED[3];
	float onboardVelNED[3];
	float onboardLat;
	float onboardLon;
	float onboardHgt;

	uint32_t flowMsec;
	uint32_t lastFlowMsec;
	bool newFlowData;

	float flowTimestamp;      // in seconds
	float flowRawPixelX;       // in pixels
	float flowRawPixelY;       // in pixels
	float flowDistance;        // in meters
	float flowQuality;   // distance normalized between 0 and 1
	float flowSensorId;        // sensor ID
	float flowGyroX;
	float flowGyroY;
	float flowGyroBiasX;
	float flowGyroBiasY;

	float flowRadX;
	float flowRadY;

	float flowRawGroundSpeedX;
	float flowRawGroundSpeedY;

	uint32_t distMsec;
	uint32_t lastDistMsec;
	bool newDistData;

	float distTimestamp;
	bool distValid;
	float distGroundDistance;
	float distLastValidReading;

	// input data timing
	uint64_t msecAlignTime;
	uint64_t msecStartTime;
	uint64_t msecEndTime;

	float gpsGndSpd;
	float gpsCourse;
	float gpsVelD;
	int gps_status;

public:
	AttPosEKF *_ekf;
	float rpy[3];
	float tilt_margin;
	bool constrain_AG;
	byte mag_number;
	byte odom_number;
	Vector3f bodyVelPred; // this is the body frame velocity vector
	Vector3f Accel_compensation;
	estimator_ekf()
	{
		// Estimated time delays (msec)
		msecVelDelay = 100;
		msecPosDelay = 100;
		msecHgtDelay = 50;
		msecRngDelay = 100;
		msecMagDelay = 25;
		msecTasDelay = 100;
		msecOptFlowDelay = 0;
		msecOdomDelay = 20; // 20 ms assumed delay

		// IMU input data variables
		IMUmsec = 0;
		IMUusec = 0;
		lastAngRate.zero();
		lastAccel.zero();
		cov_pred_dt = 0;

		// Magnetometer input data variables
		MAGtimestamp = 0;
		MAGmsec = 0;
		lastMAGmsec = 0;
		newDataMag = false;

		// AHRS input data variables
		AHRStimestamp = 0;
		AHRSmsec = 0;
		lastAHRStime = 0;
		// ADS input data variables
		ADStimestamp = 0;
		ADSmsec = 0;
		lastADSmsec = 0;
		newAdsData = false;
		newDataGps = false;
		newOptFlowData = false;

		onboardTimestamp = 0;
		onboardMsec = 0;
		lastOnboardMsec = 0;
		newOnboardData = false;

		flowMsec = 0;
		lastFlowMsec = 0;
		newFlowData = false;

		flowGyroX = 0.0f;
		flowGyroY = 0.0f;
		flowGyroBiasX = 0.0f;
		flowGyroBiasY = 0.0f;

		distMsec = 0;
		lastDistMsec = 0;
		newDistData = false;

		distTimestamp = 0.0f;
		distValid = false;

		_ekf = new AttPosEKF();
		_ekf->setIsFixedWing(true);

		tilt_margin = 0;
		constrain_AG = false;
		mag_number = 0;
		odom_number = 0;
	}

	void SelectFlowFusion()
	{
	// Fuse optical flow measurements
		if(newDataGps)
		{
			newFlowData = false; // don't use stale data
			return; //don't fuse if we have just received gps 
		}
		if (newFlowData && _ekf->useOpticalFlow && flowQuality > 0.5 && _ekf->Tnb.z.z > 0.71f)
		{
		    // recall states and angular rates stored at time of measurement after adjusting for delays
		    _ekf->RecallStates(_ekf->statesAtFlowTime, (IMUmsec - msecOptFlowDelay));
		    _ekf->RecallOmega(_ekf->omegaAcrossFlowTime, (IMUmsec - 2*msecOptFlowDelay));

		    // Calculate bias errorsfor flow sensor internal gyro
		    flowGyroBiasX = 0.999f * flowGyroBiasX + 0.001f * (flowGyroX - _ekf->omegaAcrossFlowTime[0]);
		    flowGyroBiasY = 0.999f * flowGyroBiasY + 0.001f * (flowGyroY - _ekf->omegaAcrossFlowTime[1]);

		    //use sensor internal rates corrected for bias errors
		    _ekf->omegaAcrossFlowTime[0] = flowGyroX - flowGyroBiasX;
		    _ekf->omegaAcrossFlowTime[1] = flowGyroY - flowGyroBiasY;

		    // calculate rotation matrix
		    // Copy required states to local variable names
		    float q0 = _ekf->statesAtFlowTime[0];
		    float q1 = _ekf->statesAtFlowTime[1];
		    float q2 = _ekf->statesAtFlowTime[2];
		    float q3 = _ekf->statesAtFlowTime[3];
		    float q00 = sq(q0);
		    float q11 = sq(q1);
		    float q22 = sq(q2);
		    float q33 = sq(q3);
		    float q01 = q0 * q1;
		    float q02 = q0 * q2;
		    float q03 = q0 * q3;
		    float q12 = q1 * q2;
		    float q13 = q1 * q3;
		    float q23 = q2 * q3;
		    _ekf->Tnb_flow.x.x = q00 + q11 - q22 - q33;
		    _ekf->Tnb_flow.y.y = q00 - q11 + q22 - q33;
		    _ekf->Tnb_flow.z.z = q00 - q11 - q22 + q33;
		    _ekf->Tnb_flow.y.x = 2*(q12 - q03);
		    _ekf->Tnb_flow.z.x = 2*(q13 + q02);
		    _ekf->Tnb_flow.x.y = 2*(q12 + q03);
		    _ekf->Tnb_flow.z.y = 2*(q23 - q01);
		    _ekf->Tnb_flow.x.z = 2*(q13 - q02);
		    _ekf->Tnb_flow.y.z = 2*(q23 + q01);

		    // scale from raw pixel flow rate to radians/second
		    //float scaleFactor = 0.03f; // best value for quad106.zip data using the 16 mm lens
		    //float scaleFactor = 0.06f; // best value for InputFilesPX4_flow.zip data
		    //float scaleFactor = 0.882f; // best value for quad123.zip data which outputs flow rates that have already been scaled to rad/sec
		    float scaleFactor = 1.000f; // best value for quadOptFlowLogData.zip data which outputs flow rates that have already been scaled to rad/sec
		    flowRadX = flowRawPixelX * scaleFactor;
		    flowRadY = flowRawPixelY * scaleFactor;

		    // calculate motion compensated angular flow rates used for fusion in the main nav filter
		    _ekf->flowRadXYcomp[0] = flowRadX/_ekf->flowStates[0] + _ekf->omegaAcrossFlowTime[0];
		    _ekf->flowRadXYcomp[1] = flowRadY/_ekf->flowStates[0] + _ekf->omegaAcrossFlowTime[1];

		    // these flow rates are not motion compensated and are used for focal length scale factor estimation
		    _ekf->flowRadXY[0] = flowRadX;
		    _ekf->flowRadXY[1] = flowRadY;

		    // perform optical flow fusion
		    _ekf->fuseOptFlowData = true;
		    _ekf->fuseRngData = false;

		    // don't try to estimate focal length scale factor if GPS is not being used or there is no range finder.
		    if (_ekf->useGPS && _ekf->useRangeFinder) {
		        _ekf->OpticalFlowEKF();
		    }
		    _ekf->FuseOptFlow();
		    _ekf->fuseOptFlowData = false;

		    // estimate speed over ground for cross-check of data (debugging only)
		    float tempQuat[4];
		    float euler[3];
		    for (uint8_t j=0; j<4; j++) tempQuat[j] = _ekf->states[j];
		    _ekf->quat2eul(euler, tempQuat);
		    float bx = (flowRadX - _ekf->angRate.x) * distLastValidReading;
		    float by = (flowRadY - _ekf->angRate.y) * distLastValidReading;
		    flowRawGroundSpeedY = cos(euler[2]) * bx + -sin(euler[2]) * by;
		    flowRawGroundSpeedX = -(sin(euler[2]) * bx + cos(euler[2]) * by);
		} 
		else 
		{
		    _ekf->fuseOptFlowData = false;
		}
		if (newDistData && _ekf->useRangeFinder)
		{
			if (distValid > 0.0f && _ekf->Tnb.z.z > 0.9f)
			{
				distLastValidReading = distGroundDistance;
				_ekf->rngMea = distGroundDistance;
				_ekf->fuseRngData = true;
				_ekf->fuseOptFlowData = false;
				_ekf->RecallStates(_ekf->statesAtRngTime, (IMUmsec - msecRngDelay));
				_ekf->OpticalFlowEKF();
				_ekf->fuseRngData = false;
			}
		}
	}

	void SelectPVFusion()
	{
		if (newDataGps)
		{
		    // calculate a position offset which is applied to NE position and velocity wherever it is used throughout code to allow GPS position jumps to be accommodated gradually
		    _ekf->decayGpsOffset();
		    calcposNED(posNED, _ekf->gpsLat, _ekf->gpsLon, _ekf->gpsHgt, _ekf->latRef, _ekf->lonRef, _ekf->hgtRef);
		    _ekf->posNE[0] = posNED[0];
		    _ekf->posNE[1] = posNED[1];
		     // set fusion flags
		    _ekf->fuseVelData = true;
		    _ekf->fusePosData = true;
		    _ekf->fuseHgtData = false; // height fusion is done when baro reading arrives.
		    
		    _ekf->fusionModeGPS = 1; // don't fuse velD from GPS unless
		    if(_ekf->gpsvAcc <= 2.0) // vAcc < 2.5 m
		    {
		    	_ekf->fusionModeGPS = 0;
		    }
		    // recall states stored at time of measurement after adjusting for delays
		    _ekf->RecallStates(_ekf->statesAtVelTime, (IMUmsec - msecVelDelay));
		    _ekf->RecallStates(_ekf->statesAtPosTime, (IMUmsec - msecPosDelay));
		    // run the fusion step
		    _ekf->FuseVelposNED();
		}
		else
		{
		    _ekf->fuseVelData = false;
		    _ekf->fusePosData = false;
		    _ekf->fuseHgtData = false;
		}
	}

	void SelectAspdFusion()
	{
		// Fuse Airspeed Measurements
		if (newAdsData)
		{
			if (_ekf->VtasMeas > 8.0f && _ekf->useAirspeed && new_ASData and !newDataGps)
			{
			    _ekf->fuseVtasData = true;
			    _ekf->RecallStates(_ekf->statesAtVtasMeasTime, (IMUmsec - msecTasDelay)); // assume 100 msec avg delay for airspeed data
			    _ekf->FuseAirspeed();
			}
			else
			{
				new_ASData = false;
			    _ekf->fuseVtasData = false;
			}
		    // what would ardupilot do?
		    // _ekf->hgtMea = _ekf->baroHgt - _ekf->baroHgtOffset - _ekf->hgtRef;  //test this somehow?
			float coeff = _ekf->BaroSigma/(_ekf->gpsvAcc + _ekf->BaroSigma);
			if(not newDataGps) //prevent old gps reading from messing up height measurements
			{
				coeff = 0;
			}
			_ekf->hgtMea = (1.0f - coeff)*(_ekf->baroHgt - _ekf->baroHgtOffset - _ekf->hgtRef) + coeff*(_ekf->gpsHgt - _ekf->hgtRef);
			if(coeff>0)
				_ekf->R_hgt = _ekf->BaroSigma*coeff;
			else
				_ekf->R_hgt = _ekf->BaroSigma;
			_ekf->fuseVelData = false;
			_ekf->fusePosData = false;
			_ekf->fuseHgtData = true;
			// recall states stored at time of measurement after adjusting for delays
			_ekf->RecallStates(_ekf->statesAtHgtTime, (IMUmsec - msecHgtDelay));
			// run the fusion step
			_ekf->FuseVelposNED();
		//                printf("time = %e \n", IMUtimestamp);
		}
		else
		{
		    _ekf->fuseVelData = false;
		    _ekf->fusePosData = false;
		    _ekf->fuseHgtData = false;
		}
	}

	void SelectMagFusion()
	{
		if(newDataGps or newAdsData or newFlowData or newDataOdom)
		{
			newDataMag = false;
			return; // don't fuse mag data on the cycle where you have received anything else
		}
		// Fuse Magnetometer Measurements
		#if 1
		if (newDataMag && _ekf->useCompass)
		{
		    _ekf->fuseMagData = true;
		    _ekf->RecallStates(_ekf->statesAtMagMeasTime, (IMUmsec - msecMagDelay)); // Assume 50 msec avg delay for magnetometer data
		    _ekf->magstate.obsIndex = (mag_number++)%3; //set to 0 and uncomment the other 2 lines if you want all 3 axes to be fused on the same cycle.
		    _ekf->inhibitMagStates = false;
		    _ekf->FuseMagnetometer(); // The overall mag update takes 3 mag cycles. The system doesn't require fusion of all 3 axes at the same time anyway.
		}
		#else
		if (newDataMag && _ekf->useCompass)
		{
		    _ekf->fuseMagData = true;
		    _ekf->RecallStates(_ekf->statesAtMagMeasTime, (IMUmsec - msecMagDelay)); // Assume 50 msec avg delay for magnetometer data
		    _ekf->magstate.obsIndex = 0; //set to 0 and uncomment the other 2 lines if you want all 3 axes to be fused on the same cycle.
		    _ekf->inhibitMagStates = false;
		    _ekf->FuseMagnetometer(); //X axis.
		    _ekf->FuseMagnetometer(); //Y axis
		    _ekf->FuseMagnetometer(); //Z axis
		}
		#endif
		else
		{
		    _ekf->fuseMagData = false;
		}
	}

	void SelectOdomFusion()
	{
		if(newDataGps or newAdsData or newFlowData or newDataMag)
		{
			return; // don't fuse mag data on the cycle where you have received anything else
		}
		if (newDataOdom)
		{
		    _ekf->RecallStates(_ekf->statesAtOdomTime, (IMUmsec - msecOdomDelay)); // Assume 50 msec avg delay for magnetometer data
		    _ekf->FuseBodyVel(odom_number%3); // %3 is for safety
		    odom_number += 1;
		    if(odom_number==3)
		    {
		    	odom_number = 0;
		    	newDataOdom = false;
		    }
		}
	}

	void run_filter(bool reset) 
	{
		if(reset)
		{
			_ekf->statesInitialised = false;
		}
		if(!_ekf->statesInitialised and (_ekf->GPSstatus >= 3) and _ekf->gpshAcc < 2.5 and _ekf->pDOP <= 2.5)// if GPS is available
		{ 
			_ekf->InitialiseFilter(_ekf->velNED, _ekf->gpsLat, _ekf->gpsLon, _ekf->gpsHgt, 0.0f); // last thing is declination
			tilt_margin = 0;
		} 
		else if (_ekf->statesInitialised) 
		{
		    // Run the strapdown INS equations every IMU update
		    _ekf->UpdateStrapdownEquationsNED();
		    float tempQuat[4];
		    for (uint8_t j=0; j<4; j++) tempQuat[j] = _ekf->states[j];
		    _ekf->quat2eul(rpy, tempQuat);
			rpy[0] -= ahrsEul[0];
			rpy[1] -= ahrsEul[1];
			rpy[2] -= ahrsEul[2];
			rpy[0] *= 57.3;
			rpy[1] *= 57.3;
			rpy[2] *= 57.3;
			if((fabs(rpy[0])>tilt_margin or fabs(rpy[1])>tilt_margin or fabs(rpy[2])>tilt_margin) and _ekf->new_AHRS_Data and _ekf->accNavMag < 2.0f ) // this needs an acceleration related argument
			{
				_ekf->staticMode = false;
				_ekf->InitialiseFilter(_ekf->velNED, _ekf->latRef, _ekf->lonRef, _ekf->hgtRef, 0.0f); // this is mostly for the initial tilt alignment.
			}
			tilt_margin = _ekf->ConstrainFloat(tilt_margin + 1000*_ekf->dtIMU,0,1000);

		    for (uint8_t j=0; j<4; j++) tempQuat[j] = _ekf->states[j];
		    _ekf->quat2eul(rpy, tempQuat);
			rpy[0] *= 57.3;
			rpy[1] *= 57.3;
			rpy[2] *= 57.3;

		    // store the predicted states for subsequent use by measurement fusion
		    _ekf->StoreStates(IMUmsec); //use internal clock. clock rollover happens after 49 days. We don't have a plane that will fly for that long.
		    // Check if on ground - status is used by covariance prediction
		    bool onground = false;//(((sq(_ekf->velNED[0]) + sq(_ekf->velNED[1]) + sq(_ekf->velNED[2])) < 2.0f));// && (_ekf->VtasMeas < 2.0f));
		    _ekf->staticMode = false;
		    _ekf->setOnGround(onground);
		    // sum delta angles and time used by covariance prediction
		    _ekf->summedDelAng = _ekf->summedDelAng + _ekf->correctedDelAng;
		    _ekf->summedDelVel = _ekf->summedDelVel + _ekf->dVelIMU;
		    cov_pred_dt += _ekf->dtIMU;
		    // perform a covariance prediction
		    if(!(newDataGps) and !(newAdsData) and !(newDataMag) and !(newFlowData) and !(newDistData))
		    {
				_ekf->CovariancePrediction(cov_pred_dt);
				_ekf->summedDelAng.zero();
				_ekf->summedDelVel.zero();
				cov_pred_dt = 0;
		    }
		    // Set global time stamp used by EKF processes
		    _ekf->globalTimeStamp_ms = IMUmsec;

		    //Mag Fusion
		    SelectMagFusion();
		    //Airspeed fusion 
		    SelectAspdFusion();
		    // Fuse Optical Flow data
			SelectFlowFusion();
			// Fuse Body Odometry
			SelectOdomFusion();		
		    // Fuse GPS Measurements
			SelectPVFusion();

			get_centrifugal_correction(); // this is for external stuff-> only calculates accel compensation and body frame velocity (position is already world frame and accel is available in body frame already so)
		}
		else
		{
		    _ekf->fuseVelData = false;
		    _ekf->fusePosData = false;
		    _ekf->fuseHgtData = false;
		}
	}

	void setIMUData(float A[3], float G[3], float delta_T, float time_stamp_ms, float time_stamp_us)
	{
		_ekf->dtIMU = delta_T;
		IMUmsec = time_stamp_ms;
		IMUusec = time_stamp_us;
		_ekf->angRate.x = G[0];
		_ekf->angRate.y = G[1];
		_ekf->angRate.z = G[2];
		_ekf->accel.x   = A[0];
		_ekf->accel.y   = A[1];
		_ekf->accel.z   = A[2];
		_ekf->dAngIMU = 0.5f*(_ekf->angRate + lastAngRate)*_ekf->dtIMU;
		lastAngRate = _ekf->angRate;
		_ekf->dVelIMU = 0.5f*(_ekf->accel + lastAccel)*_ekf->dtIMU;
		lastAccel = _ekf->accel;
		// test these constraint methods; I have a feeling this will backfire because it may just adjust the gyro biases. 
		if(constrain_AG) //constrain sensor error growth on GPS loss; this way it doesn't backfire because the biases aren't being corrected no more son.
		{
			Vector3f dOmega = _ekf->angRate - lastAngRate;
			Vector3f _gravityNED(0.0f, 0.0f, GRAVITY_MSS);
			Vector3f gravity_comp = _ekf->Tnb*_gravityNED*delta_T;
			_ekf->dVelIMU.y = _ekf->ConstrainFloat(_ekf->dVelIMU.y + gravity_comp.y, -2*dOmega.z, 2*dOmega.z) - gravity_comp.y;
			_ekf->dVelIMU.z = _ekf->ConstrainFloat(_ekf->dVelIMU.z + gravity_comp.z, -2*dOmega.y, 2*dOmega.y) - gravity_comp.z;
		}
	}

	void setOdomData(float bodyVel[4], bool new_data)
	{
		if(odom_number > 0) //wait for all the axes to be fused in.
		{
			return;
		}
		for(int i=0;i<3;i++)
		{
			bodyVel[i] = fabs(bodyVel[i]) < 0.05 ? 0: bodyVel[i];
		}
		_ekf->body_Odom_vel.x = bodyVel[0];
		_ekf->body_Odom_vel.y = bodyVel[1];
		_ekf->body_Odom_vel.z = bodyVel[2];
		_ekf->body_Odom_velErr = bodyVel[3];
		newDataOdom = new_data;
	}

	void get_centrifugal_correction()
	{
		// alternate approach: use some source of body frame velocity (say optical flow), and then measure the change in body frame velocity's orientation (in body frame itself).
		// this would avoid the added errors introduced by errors in orientation estimates used for transforming from NED to body frame.
		Accel_compensation = Vector3f(0,0,0);
		// return;

		Vector3f VelocityNED_fused_now = Vector3f(_ekf->states[4],_ekf->states[5],_ekf->states[6]);

		_ekf->RecallStates(_ekf->statesAtRotTime, (IMUmsec - msecVelDelay));
		Vector3f VelocityNED_fused_past = Vector3f(_ekf->statesAtRotTime[4],_ekf->statesAtRotTime[5],_ekf->statesAtRotTime[6]);

		// this may be replaced by body frame velocity measurements in the future, should they be available.
		Vector3f bodyVelNow = (_ekf->Tnb * VelocityNED_fused_now);
		bodyVelPred = bodyVelNow; // used later
		bodyVelNow = bodyVelNow/(bodyVelNow.length()); //normalize

		Vector3f bodyVelPast = (_ekf->Tnb * VelocityNED_fused_past);
		bodyVelPast = bodyVelPast/(bodyVelPast.length()); //normalize

		float angle = acosf(bodyVelNow.x*bodyVelPast.x + bodyVelNow.y*bodyVelPast.y + bodyVelNow.z*bodyVelPast.z);
		Vector3f axis = bodyVelPast%bodyVelNow;
		float magnitude = axis.length();
		if(magnitude <= 0.01)
		{
			return;
		}
		axis = axis/magnitude; //gotta get that unit vector babe.
		Vector3f BF_Vel_rotation = 10.0f*angle*axis;
		if(_ekf->GPSstatus>=3 and _ekf->gpssAcc <= 0.5 and gpsGndSpd >= 2.5)
		{
	        Accel_compensation = BF_Vel_rotation % bodyVelPred; // body frame centripetal force.
		}
		return;
	}

	void setGPSData(double Lat,double Lon,float Hgt, float GndSpd, float Course, float velNED[3], float hAcc, float vAcc, float sAcc, float magDec, float magAcc, float gpsDt, float status, float pDOP, bool new_data)
	{
		_ekf->updateDtGpsFilt(gpsDt);

		_ekf->GPSstatus = status;

		gpsCourse = deg2rad*Course;
		gpsGndSpd = GndSpd;
		gpsVelD = velNED[2];
		_ekf->gpsLat = deg2rad*Lat;
		_ekf->gpsLon = deg2rad*Lon - M_PI;
		_ekf->gpsHgt = Hgt;
		_ekf->velNED[0] = velNED[0];
		_ekf->velNED[1] = velNED[1];
		_ekf->velNED[2] = velNED[2];
		_ekf->gpsmagDec = magDec;
		_ekf->gpshAcc = _ekf->ConstrainFloat(hAcc,0.5,20);
		_ekf->gpsvAcc = _ekf->ConstrainFloat(vAcc,0.1,20);;
		_ekf->gpssAcc = _ekf->ConstrainFloat(sAcc,0.5,20);;
		_ekf->gpsmagAcc = magAcc;
		_ekf->pDOP = pDOP;
		newDataGps = new_data;
	}

	void setMagData(float M[3], float MB[3], bool new_data)
	{
		_ekf->magData.x =  0.2*M[0]*MAG_SCALING + (0.8*_ekf->magData.x);
		_ekf->magBias.x = -MB[0]*MAG_SCALING;
		_ekf->magData.y =  0.2*M[1]*MAG_SCALING + (0.8*_ekf->magData.y);
		_ekf->magBias.y = -MB[1]*MAG_SCALING;
		_ekf->magData.z =  0.2*M[2]*MAG_SCALING + (0.8*_ekf->magData.z);
		_ekf->magBias.z = -MB[2]*MAG_SCALING;
		_ekf->magTotal = sqrtf(sq(M[0]) + sq(M[1]) + sq(M[2]));
		newDataMag = new_data;
	}

	void setAirData(float AS, float baroHgt, float BaroSigma, float hgtDt, bool new_data, bool new_Airspeed_data)
	{
		_ekf->updateDtHgtFilt(hgtDt);
		_ekf->VtasMeas = _ekf->EAS2TAS*AS;
		_ekf->baroHgt = baroHgt;
		_ekf->BaroSigma = BaroSigma;
		newAdsData = !newAdsData; // flip
		new_ASData = new_Airspeed_data;
	}

	void setAhrsData(float AHRS[5]) // roll pitch yaw err_rp, err_y
	{
		for (uint8_t j=0; j<3; j++)
		{
		    ahrsEul[j] = AHRS[j];
			if(ahrsEul[j] > M_PI)
				ahrsEul[j] -= 2*M_PI;
			else if(ahrsEul[j] < -M_PI)
				ahrsEul[j] += 2*M_PI;
			_ekf->ahrsEul[j] = ahrsEul[j];
		}
		_ekf->ahrsEul[3] = AHRS[3];
		_ekf->ahrsEul[4] = AHRS[4];
		_ekf->new_AHRS_Data = true;
	}

	void setTimingData(uint32_t timeArray[5], float EAS2TAS)
	{
		msecVelDelay  = timeArray[0];
		msecPosDelay  = timeArray[1];
		msecHgtDelay  = timeArray[2];
		msecMagDelay  = timeArray[3];
		msecTasDelay  = timeArray[4];
		_ekf->EAS2TAS = EAS2TAS;
	}

	void setFlowData(float flow[7], uint32_t flowDt,bool new_data)
	{
		// timestamp, rawx, rawy, distance, quality, sensor id, flowGyroX, flowGyroY
		flowRawPixelX = flow[0];        // in pixels
		flowRawPixelY = flow[1];        // in pixels
		flowDistance  = flow[2];        // in meters from rangefinder (?) 
		// catch glitches in logged data
		if (flowRawPixelX > 200 || flowRawPixelY > 200 || flowRawPixelX < -200 || flowRawPixelY < -200)
		{
			flowQuality = 0.0f;    // quality normalized between 0 and 1
		}
		else
		{
			flowQuality = flow[3] / 255;    // quality normalized between 0 and 1
		}
		flowSensorId = flow[4];         // sensor ID
		flowGyroX = flow[5];
		flowGyroY = flow[6];
		// assume 1/2 interval effective delay associated with averaging inside the sensor
		msecOptFlowDelay = flowDt/2;
		newFlowData = new_data;
	}

	void setDistData(float distance_read, float read_valid, bool new_data)
	{
		// timestamp, distance, flags
		distGroundDistance = distance_read;   // in meters
		distValid = (read_valid > 0.0f);   // reading is valid
    	newDistData = new_data;
	}

};




	// State vector:
	// 0-3: quaternions (q0, q1, q2, q3)
	// 4-6: Velocity - m/sec (North, East, Down)
	// 7-9: Position - m (North, East, Down)
	// 10-12: Delta Angle bias - rad (X,Y,Z)
	// 13: Delta Velocity Z bias -m/s
	// 14-15: Wind Vector  - m/sec (North,East)
	// 16-18: Earth Magnetic Field Vector - milligauss (North, East, Down)
	// 19-21: Body Magnetic Field Vector - milligauss (X,Y,Z)
	// 22: Terrain Vertical Offset - m

	// printf("\n");
	// printf("dtIMU: %8.6f, dt: %8.6f, imuMsec: %u\n", _ekf->dtIMU, dt, IMUmsec);
	// printf("posNED: %8.4f, %8.4f, %8.4f, velNED: %8.4f, %8.4f, %8.4f\n", (double)_ekf->posNED[0], (double)_ekf->posNED[1], (double)_ekf->posNED[2],
	//     (double)_ekf->velNED[0], (double)_ekf->velNED[1], (double)_ekf->velNED[2]);
	// printf("vTAS: %8.4f baro alt: %8.4f\n", _ekf->VtasMeas, _ekf->hgtMea);
	// printf("mag: %8.4f, %8.4f, %8.4f\n", (double)_ekf->magData.x, (double)_ekf->magData.y, (double)_ekf->magData.z);
	// printf("states (quat)        [1-4]: %8.4f, %8.4f, %8.4f, %8.4f\n", (double)_ekf->states[0], (double)_ekf->states[1], (double)_ekf->states[2], (double)_ekf->states[3]);
	// printf("states (vel m/s)     [5-7]: %8.4f, %8.4f, %8.4f\n", (double)_ekf->states[4], (double)_ekf->states[5], (double)_ekf->states[6]);
	// printf("states (pos m)      [8-10]: %8.4f, %8.4f, %8.4f\n", (double)_ekf->states[7], (double)_ekf->states[8], (double)_ekf->states[9]);
	// printf("states (delta ang) [11-13]: %8.4f, %8.4f, %8.4f\n", (double)_ekf->states[10], (double)_ekf->states[11], (double)_ekf->states[12]);
	// printf("states (delta vel) [14]: %8.4ff\n", (double)_ekf->states[13]);
	// printf("states (wind)      [15-16]: %8.4f, %8.4f\n", (double)_ekf->states[14], (double)_ekf->states[15]);
	// printf("states (earth mag) [17-19]: %8.4f, %8.4f, %8.4f\n", (double)_ekf->states[16], (double)_ekf->states[17], (double)_ekf->states[18]);
	// printf("states (body mag)  [20-22]: %8.4f, %8.4f, %8.4f\n", (double)_ekf->states[19], (double)_ekf->states[20], (double)_ekf->states[21]);
	// printf("states (terain offset) [23]: %8.4ff\n", (double)_ekf->states[22]);
	// printf("states: %s %s %s %s %s %s %s %s %s\n",
	//     (_ekf->statesInitialised) ? "INITIALIZED" : "NON_INIT",
	//     (_ekf->onGround) ? "ON_GROUND" : "AIRBORNE",
	//     (_ekf->fuseVelData) ? "FUSE_VEL" : "INH_VEL",
	//     (_ekf->fusePosData) ? "FUSE_POS" : "INH_POS",
	//     (_ekf->fuseHgtData) ? "FUSE_HGT" : "INH_HGT",
	//     (_ekf->fuseMagData) ? "FUSE_MAG" : "INH_MAG",
	//     (_ekf->fuseVtasData) ? "FUSE_VTAS" : "INH_VTAS",
	//     (_ekf->useAirspeed) ? "USE_AIRSPD" : "IGN_AIRSPD",
	//     (_ekf->useCompass) ? "USE_COMPASS" : "IGN_COMPASS");
