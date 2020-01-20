#include "HoverThrustEstimator.hpp"

void HoverThrustEstimator::reset()
{
	_thrust = 0.f;
	_acc_z = 0.f;
}

void HoverThrustEstimator::handleParameterUpdate()
{
	_hte.setProcessNoiseStdDev(_param_hte_ht_noise.get());
	_hte.setMeasurementNoiseStdDev(_param_hte_acc_noise.get());
}

void HoverThrustEstimator::update(const float dt)
{
	_hte.predict(dt);
	_hte.fuseAccZ(_acc_z, _thrust);
}
