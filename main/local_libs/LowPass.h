#pragma once
namespace lyncs
{
class LowPass
{
  private:
	double data_;
	const double kRate_;

  public:
	LowPass(double rate);
	~LowPass();
	void InputData(const double kInput);
	const double GetData() const;
};

void LowPass::InputData(const double kInput){
	data_=kRate_*kInput+(1-kRate_)*data_;
}

const double LowPass::GetData() const{
	return data_;
}
LowPass::LowPass(double rate)
	:kRate_(rate),
	data_(0)
{
}

LowPass::~LowPass()
{
}

} // namespace lyncs