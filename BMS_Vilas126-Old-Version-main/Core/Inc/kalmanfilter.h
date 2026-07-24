#ifndef INC_KALMANFILTER_H_
#define INC_KALMANFILTER_H_

typedef struct {
	float x; 											// State estimate
	float P; 											// Estimate error covariance
	float Q; 											// Process noise covariance
	float R; 											// Measurement noise covariance
} KalmanFilter;

void 	kalman_filter_init				(KalmanFilter *kf, float initial_state_estimate, float initial_estimate_error_cov, float measurement_noise_cov);
float kalman_filter_update			(KalmanFilter *kf, float z, float measurement_noise_cov);

#endif /* INC_KALMANFILTER_H_ */
