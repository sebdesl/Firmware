#include "ZeroOrderHoverThrustEkf.hpp"
#include <mathlib/mathlib.h>

void ZeroOrderHoverThrustEkf::predict(const float dt)
{
	// State is constant
	// Predict state covariance only
	_P += _Q * dt;
	_dt = dt;
}

void ZeroOrderHoverThrustEkf::fuseAccZ(const float acc_z, const float thrust)
{
	const float H = computeH(thrust);
	const float innov_var = computeInnovVar(H);
	const float predicted_acc_z = computePredictedAccZ(thrust);
	const float innov = computeInnov(acc_z, predicted_acc_z);
	const float K = computeKalmanGain(H, innov_var);
	const float innov_test_ratio = computeInnovTestRatio(innov, innov_var);
	printf("AZ = %.3f\tthr = %.3f\tH = %.3f\tinnov_var = %.3f\tpredicted_acc_z = %.3f\tinnov = %.3f\tK = %.3f\ttest_ratio = %.3f\n",
	       (double)acc_z,
	       (double)thrust,
	       (double)H,
	       (double)innov_var,
	       (double)predicted_acc_z,
	       (double)innov,
	       (double)K,
	       (double)innov_test_ratio);

	if (isTestRatioPassing(innov_test_ratio)) {
		updateState(K, innov);
		updateStateCovariance(K, H);
		printf("estimated hover thrust: %.3f\t std_dev = %.3f\n", (double)_hover_thr, (double)sqrtf(_P));
	}
}

float ZeroOrderHoverThrustEkf::computeH(const float thrust)
{
	return -9.81f * thrust / (_hover_thr * _hover_thr);
}

float ZeroOrderHoverThrustEkf::computeInnovVar(const float H)
{
	return math::max(H * _P * H + _R, _R);
}

float ZeroOrderHoverThrustEkf::computePredictedAccZ(const float thrust)
{
	return 9.81f * thrust / _hover_thr - 9.81f;
}

float ZeroOrderHoverThrustEkf::computeInnov(const float acc_z, const float predicted_acc_z)
{
	return acc_z - predicted_acc_z;
}

inline float ZeroOrderHoverThrustEkf::computeKalmanGain(const float H, const float innov_var)
{
	return _P * H / innov_var;
}

inline float ZeroOrderHoverThrustEkf::computeInnovTestRatio(const float innov, const float innov_var)
{
	return innov * innov / (_gate_size * _gate_size * innov_var);
}

inline bool ZeroOrderHoverThrustEkf::isTestRatioPassing(const float innov_test_ratio)
{
	return innov_test_ratio < 1.f;
}

inline void ZeroOrderHoverThrustEkf::updateState(const float K, const float innov)
{
	_hover_thr = math::constrain(_hover_thr + K * innov, 0.1f, 0.8f);
}

inline void ZeroOrderHoverThrustEkf::updateStateCovariance(const float K, const float H)
{
	_P = math::max((1.f - K * H) * _P, 1e-10f);
}
