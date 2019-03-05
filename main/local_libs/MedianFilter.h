#pragma once
namespace lyncs
{
template <typename _T, unsigned int _N>
class MedianFilter
{
  private:
	void bubbleSort(_T list[_N]);
	_T data_array_[_N];

  public:
	MedianFilter();
	~MedianFilter();
	void InputData(_T newdata);
	_T GetData() const;
};

template <typename _T, unsigned int _N>
MedianFilter<_T, _N>::MedianFilter()
{
}

template <typename _T, unsigned int _N>
MedianFilter<_T, _N>::~MedianFilter()
{
}

template <typename _T, unsigned int _N>
void MedianFilter<_T, _N>::bubbleSort(_T list[_N])
{
	for (int i = 0; i < _N; i++)
	{
		for (int j = _N - 1; j > i; j--)
		{
			if (list[j] < list[j - 1])
			{
				unsigned long temp = list[j];
				list[j] = list[j - 1];
				list[j - 1] = temp;
			}
		}
	}
}

template <typename _T, unsigned int _N>
void MedianFilter<_T, _N>::InputData(_T newdata)
{
	for (int i = 0; i < _N - 1; i++)
	{
		data_array_[i] = data_array_[i + 1];
	}
	data_array_[_N - 1] = newdata;
}

template <typename _T, unsigned int _N>
_T MedianFilter<_T, _N>::GetData() const
{
	_T sorted[_N];
	for (int i = 0; i < _N; i++)
	{
		sorted[i] = data_array_[i];
	}
	this->bubbleSort(sorted);
	return (sorted[_N / 2 - 1] + sorted[_N / 2]) / 2;
}

} // namespace lyncs