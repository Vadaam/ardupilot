/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>

#if HAL_CPU_CLASS >= HAL_CPU_CLASS_150

#include "AP_NavEKF2.h"
#include "AP_NavEKF2_core.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>

#include <stdio.h>

extern const AP_HAL::HAL& hal;

/********************************************************
*                   RESET FUNCTIONS                     *
********************************************************/
// resets position states to last vision measurement
void NavEKF2_core::ResetVisionPosition(void)
{
	stateStruct.quat.rotation_matrix(Tbn_vision);

	worldVisionPos = markerposNED - Tbn_vision*vpDataNew.visionPosition;

    stateStruct.position.x = worldVisionPos.x;
    stateStruct.position.y = worldVisionPos.y;

    vpDataDelayed.visionPosition.x = vpDataNew.visionPosition.x;
    vpDataDelayed.visionPosition.y = vpDataNew.visionPosition.y;
}
/********************************************************
*                   FUSE MEASURED_DATA                  *
********************************************************/
// determine when to perform fusion of vision measurements
void NavEKF2_core::SelectVisionPositionFusion()
{
    // start performance timer
	hal.util->perf_begin(_perf_FuseVisionPosNED);

    // Check if the visual position data is still valid
    visionPositionDataValid = ((imuSampleTime_ms - visionPosValidMeaTime_ms) < 1000);

    // Check if the fusion has timed out (vision measurements have been rejected for too long)
    bool visionFusionTimeout = ((imuSampleTime_ms - prevVisionFuseTime_ms) > 5000);

    // determine if conditions are right to start a new fusion cycle
    bool dataReady = statesInitialised && newDataVisionPosition && useVisionPosition() && visionPositionDataValid;
    if (dataReady) {

    	if (visionFusionTimeout)
    	{
    		//It means that vision target is re-acquired, so we should reset position to last vision positions.
    		//Otherwise the step in innovation may cause problems with transients in other states.
    		ResetVisionPosition();
    	}
    	else
    	{
			// fuse vision data
			FuseVisionPosNED();
			// reset flag to indicate that no new vision position data is available for fusion
			newDataVisionPosition = false;
			// indicate that flow fusion has been performed. This is used for load spreading.
			visionPosFusePerformed = true;
    	}
    }

    // stop performance timer
    hal.util->perf_end(_perf_FuseVisionPosNED);
}


// fuse vision position measurements
void NavEKF2_core::FuseVisionPosNED()
{
    // start performance timer
	hal.util->perf_begin(_perf_FuseVisionPosNED);

	stateStruct.quat.rotation_matrix(Tbn_vision);

	worldVisionPos = markerposNED - Tbn_vision*vpDataDelayed.visionPosition;

    uint8_t stateIndex;
    uint8_t obsIndex;

    // declare variables used by state and covariance update calculations
    Vector3 R_OBS; // Measurement variances used for fusion
    Vector3 observation;
    float SK;

    observation[0] = worldVisionPos.x;
    observation[1] = worldVisionPos.y;
    observation[2] = worldVisionPos.z;

    R_OBS[0] = sq(constrain_float(frontend->_visionHorizPosNoise, 0.0f, 5.0f));
    R_OBS[1] = R_OBS[0];
    R_OBS[2] = sq(constrain_float(frontend->_visionVerticalPosNoise,  0.0f, 5.0f));

    for (obsIndex=0; obsIndex<=2; obsIndex++) {
    	// calculate the measurement innovation, using states from a different time coordinate
    	innovVisionPos[obsIndex] = stateStruct.position[obsIndex] - observation[obsIndex];
    }

    float maxVisionPosInnov2 = sq(frontend->_visionPosInnovGate * frontend->_visionHorizPosNoise);

    float visionPosTestRatio = (sq(innovVisionPos[0]) + sq(innovVisionPos[1])) / maxVisionPosInnov2;
    visionPosHealth = ((visionPosTestRatio < 1.0f));

    if (visionPosHealth)
    {
    	prevVisionFuseTime_ms = imuSampleTime_ms;
    	// fuse measurements sequentially
		for (obsIndex=0; obsIndex<=2; obsIndex++) {

			stateIndex = 7 + obsIndex;


			// calculate the Kalman gain and calculate innovation variances
			varInnovVisionPos[obsIndex] = P[stateIndex][stateIndex] + R_OBS[obsIndex];
			SK = 1.0f/varInnovVisionPos[obsIndex];

			for (uint8_t i= 0; i<=21; i++) {
				Kfusion[i] = 0.0f;
			}

			for (uint8_t i= 7; i<=9; i++) {
				Kfusion[i] = P[i][stateIndex]*SK;
			}

			// calculate state corrections
			for (uint8_t i = 7; i<=9; i++) {
				statesArray[i] = statesArray[i] - Kfusion[i] * innovVisionPos[obsIndex];

			}

			// update the covariance - take advantage of direct observation of a single state at index = stateIndex to reduce computations
			// this is a numerically optimised implementation of standard equation P = (I - K*H)*P;
			for (uint8_t i= 0; i<=21; i++) {
				for (uint8_t j= 0; j<=21; j++)
				{
					KHP[i][j] = Kfusion[i] * P[stateIndex][j];
				}
			}
			for (uint8_t i= 0; i<=21; i++) {
				for (uint8_t j= 0; j<=21; j++) {
					P[i][j] = P[i][j] - KHP[i][j];
				}
			}
		}


		// force the covariance matrix to be symmetrical and limit the variances to prevent ill-condiioning.
		ForceSymmetry();
		ConstrainVariances();
    }

    // stop performance timer
    hal.util->perf_end(_perf_FuseVelPosNED);
}

/********************************************************
*                   MISC FUNCTIONS                      *
********************************************************/

#endif // HAL_CPU_CLASS
