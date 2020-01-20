#pragma once

#include <matrix/matrix/math.hpp>
#include <ecl/geo/geo.h>

class ZeroOrderHoverThrustEkf
{
public:
	ZeroOrderHoverThrustEkf() = default;
	~ZeroOrderHoverThrustEkf() = default;

	void predict(float _dt);
	void fuseAccZ(float acc_z, float thrust);
	void setProcessNoiseStdDev(float process_noise) { _Q = process_noise * process_noise; }
	void setMeasurementNoiseStdDev(float measurement_noise) { _R = measurement_noise * measurement_noise; }

	float getHoverThrustEstimate() const { return _hover_thr; }
private:
	float _hover_thr{0.5f};

	float _gate_size{3.f};
	float _P{0.2f};
	float _Q{0.01f};
	float _R{0.5f};
	float _dt{0.02f};

	float computeH(float thrust);
	float computeInnovVar(float H);
	float computePredictedAccZ(float thrust);
	float computeInnov(float acc_z, float thrust);
	float computeKalmanGain(float H, float innov_var);
	float computeInnovTestRatio(float innov, float innov_var);
	bool isTestRatioPassing(float innov_test_ratio);

	void updateState(float K, float innov);
	void updateStateCovariance(float K, float H);
	void updateMeasurementNoise(float residual, float H);

	static constexpr float noise_learning_time_constant = 0.5f; // in seconds
};
