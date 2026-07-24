#include "kalmanfilter.h"

#if IS_BOOTLOADER == 0

void kalman_filter_init					(KalmanFilter *kf, float initial_state_estimate, float initial_estimate_error_cov, float measurement_noise_cov) 
{
    kf->x 	= initial_state_estimate;
    kf->P 	= initial_estimate_error_cov;
		kf->Q 	= 0.00000005f; 												
    kf->R 	= measurement_noise_cov;  						
}

float kalman_filter_update			(KalmanFilter *kf, float z, float measurement_noise_cov)
{
    kf->R 	= measurement_noise_cov;
    kf->P   = kf->P + kf->Q; 
    float denominator = kf->P + kf->R;
    if (denominator < 1e-6f) denominator = 1e-6f; 
    float K = kf->P / denominator;
    kf->x 	= kf->x + K * (z - kf->x);
    if (kf->x > 100.0f) kf->x = 100.0f;
    if (kf->x < 0.0f)   kf->x = 0.0f;    
    kf->P 	= (1.0f - K) * kf->P;  
    if (kf->P < 0) kf->P = 0;
    return kf->x;
}

#endif
