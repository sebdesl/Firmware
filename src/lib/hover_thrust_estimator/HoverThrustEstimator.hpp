#include <px4_platform_common/module_params.h>
#include "ZeroOrderHoverThrustEkf.hpp"

class HoverThrustEstimator : public ModuleParams
{
public:
	HoverThrustEstimator() :
		ModuleParams(nullptr)
	{
	}
	~HoverThrustEstimator() = default;

	void reset();

	void handleParameterUpdate();
	void update(float dt);
	void setThrust(float thrust) { _thrust = -thrust; };
	void setAccel(float accel) { _acc_z = -accel; };

	float getHoverThrustEstimate() const { return _hte.getHoverThrustEstimate(); }

private:
	ZeroOrderHoverThrustEkf _hte{};
	float _acc_z{};
	float _thrust{};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::HTE_HT_NOISE>) _param_hte_ht_noise,
		(ParamFloat<px4::params::HTE_ACC_NOISE>) _param_hte_acc_noise
	)
};
