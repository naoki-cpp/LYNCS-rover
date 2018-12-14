#pragma once
namespace lyncs
{
 class PIDController
{
  private:
	const double kProportionGain_;
	const double kIntegralGain_;
	const double kDifferentialGain_;
 	double integral_;
	double prev_error_;
	double manipulative_variable_;
 	PIDController();
  public:
	PIDController(const double kProportionGain, const double kIntegralGain, const double kDifferentialGain);
	~PIDController();
	void InputPID(double data, double goal, double dt);
	const double GetPID() const;
};
 PIDController::PIDController(const double kProportionGain, const double kIntegralGain, const double kDifferentialGain)
	: kProportionGain_(kProportionGain),
	  kIntegralGain_(kIntegralGain),
	  kDifferentialGain_(kDifferentialGain),
	  integral_(0),
	  prev_error_(0),
	  manipulative_variable_(0)
{
 }
 PIDController::~PIDController()
{
}
 void PIDController::InputPID(double data, double goal, double dt)
{
	double error = goal - data;
	integral_ = error * dt;
	manipulative_variable_ = kProportionGain_ * error ;//+ kIntegralGain_ * integral_ + kDifferentialGain_ * (error - prev_error_) / dt;
	prev_error_ = error;
}
 const double PIDController::GetPID() const{
	return manipulative_variable_;
}
 } // namespace lyncs 