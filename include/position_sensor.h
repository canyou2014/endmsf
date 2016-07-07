#ifndef POSITION_SENSOR_H
#define POSITION_SENSOR_H

#include "measurement.h"
//#include <ssf_updates/PositionWithCovarianceStamped.h>

class PositionSensorHandler: public ssf_core::MeasurementHandler
{
private:
	// measurements
//  Eigen::Quaternion<double> z_q_; /// attitude measurement camera seen from world - here we do not have an attitude measurement
	Eigen::Matrix<double, 3, 1> z_p_; /// position measurement camera seen from world
	double n_zp_ /*, n_zq_*/; /// position and attitude measurement noise - here we do not have an attitude measurement

	//ros::Subscriber subMeasurement_;

	bool measurement_world_sensor_; ///< defines if the pose of the sensor is measured in world coordinates (true, default) or vice versa (false, e.g. PTAM)
	bool use_fixed_covariance_; ///< use fixed covariance set by dynamic reconfigure

	//void subscribe();
	void measurementCallback(const Eigen::Vector3d& msg, unsigned char idx);
	//void noiseConfig(ssf_core::SSF_CoreConfig& config, uint32_t level);

public:
	PositionSensorHandler();
	PositionSensorHandler(ssf_core::Measurements* meas);
};

#endif /* POSITION_SENSOR_H */
