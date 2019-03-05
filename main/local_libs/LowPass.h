#pragma once
namespace lyncs
{
template <typename T>

class LowPass
{
  private:
	T data_;
	const T kRate_;

  public:
	LowPass(T rate);
	~LowPass();
	void InputData(const T kInput);
	const T GetData() const;
};

template <typename T>
void LowPass<T>::InputData(const T kInput)
{
	data_ = kRate_ * kInput + (1 - kRate_) * data_;
}

template <typename T>
const T LowPass<T>::GetData() const
{
	return data_;
}

template <typename T>
LowPass<T>::LowPass(T rate)
	: kRate_(rate),
	  data_(0)
{
}

template <typename T>
LowPass<T>::~LowPass()
{
}
} // namespace lyncs