/*=========================================================
	===		Matrix struct
			2018 Naoki Yano
			Usage::
				construct::
				////
				Matrix<double,3,3> matrix {{{1,0,1},{0,1,0},[0,0,1]}}
				////
===========================================================*/
#pragma once
namespace lyncs
{

template <typename T, const unsigned int COLUMN, const unsigned int ROW>
struct Matrix
{
	T matrix_[COLUMN][ROW];

  public:
	~Matrix();

	const T GetElement(const unsigned int column, const unsigned int row) const;
	const Matrix<T, 1, ROW> GetColumn(const unsigned int column) const;
	const Matrix<T, COLUMN, 1> GetRow(const unsigned int row) const;

	const void WriteElement(unsigned int column, unsigned int row, T element);
	//operators
	const Matrix<T, COLUMN, ROW> operator+(const Matrix<T, COLUMN, ROW> &matrix) const;
	template <const unsigned int ROW2>
	const Matrix<T, COLUMN, ROW2> operator*(const Matrix<T, ROW, ROW2> &matrix) const;
	const Matrix<T, COLUMN, ROW> &operator=(Matrix<T, COLUMN, ROW> matrix);
};

template <typename T, const unsigned int COLUMN, const unsigned int ROW>
Matrix<T, COLUMN, ROW>::~Matrix()
{
}
template <typename T, const unsigned int COLUMN, const unsigned int ROW>
const T Matrix<T, COLUMN, ROW>::GetElement(const unsigned int column, const unsigned int row) const //TODO:(naoki-cpp) column,jが無効の場合の処理をうまいことやる.
{
	if (column < COLUMN && row < ROW)
	{
		return matrix_[column][row];
	}
	return 0;
}

template <typename T, const unsigned int COLUMN, const unsigned int ROW>
const Matrix<T, 1, ROW> Matrix<T, COLUMN, ROW>::GetColumn(const unsigned int column) const //TODO:(naoki-cpp) iが無効の場合の処理をうまいことやる.
{
	Matrix<T, 1, ROW> column_vect;
	if (column < COLUMN)
	{
		for (int row = 0; row < ROW; row++)
		{
			column_vect.WriteElement(1, row, this->GetElement(column, row));
		}
	}
	return column_vect;
}

template <typename T, const unsigned int COLUMN, const unsigned int ROW>
const Matrix<T, COLUMN, 1> Matrix<T, COLUMN, ROW>::GetRow(const unsigned int row) const //TODO:(naoki-cpp) iが無効の場合の処理をうまいことやる.
{
	Matrix<T, COLUMN, 1> row_vect;
	if (row < ROW)
	{
		for (int column = 0; column < COLUMN; column++)
		{
			row_vect.WriteElement(column, 1, this->GetElement(column, row));
		}
	}
	return row_vect;
}

template <typename T, const unsigned int COLUMN, const unsigned int ROW>
const void Matrix<T, COLUMN, ROW>::WriteElement(unsigned int column, unsigned int row, T element)
{
	if (column < COLUMN && row < ROW)
	{
		matrix_[column][row] = element;
	}
}

template <typename T, const unsigned int COLUMN, const unsigned int ROW>
const Matrix<T, COLUMN, ROW> &Matrix<T, COLUMN, ROW>::operator=(Matrix<T, COLUMN, ROW> matrix)
{
	for (unsigned int column = 0; column < COLUMN; column++)
	{
		for (unsigned int row = 0; row < ROW; row++)
		{
			this->WriteElement(column, row, matrix.GetElement(column, row));
		}
	}
	return *this;
}

template <typename T, const unsigned int COLUMN, const unsigned int ROW>
const Matrix<T, COLUMN, ROW> Matrix<T, COLUMN, ROW>::operator+(const Matrix<T, COLUMN, ROW> &matrix) const
{
	Matrix<T, COLUMN, ROW> sum;
	for (int column = 0; column < COLUMN; column++)
	{
		for (int row = 0; row < ROW; row++)
		{
			sum.WriteElement(column, row, matrix_[column][row] + matrix.GetElement(column, row));
		}
	}
	return sum;
}
template <typename T, const unsigned int COLUMN, const unsigned int ROW>
template <unsigned int ROW2>
const Matrix<T, COLUMN, ROW2> Matrix<T, COLUMN, ROW>::operator*(const Matrix<T, ROW, ROW2> &matrix) const
{
	Matrix<T, COLUMN, ROW> product;
	for (int column = 0; column < COLUMN; column++)
	{
		for (int row = 0; row < ROW2; row++)
		{
			T sum(0);
			for (int u = 0; u < ROW; u++)
			{
				sum += matrix_[column][u] * matrix.GetElement(u, row);
			}
			product.WriteElement(column, row, sum);
		}
	}
	return product;
}

template <typename T, const unsigned int COLUMN, const unsigned int ROW>
const Matrix<T, ROW, COLUMN> TransposeMatrix(const Matrix<T, COLUMN, ROW> matrix)
{
	Matrix<T, ROW, COLUMN> transposed;
	for (int column = 0; column < COLUMN; column++)
	{
		for (int row = 0; row < ROW; row++)
		{
			transposed.WriteElement(row, column, matrix.GetElement(column, row));
		}
	}
	return transposed;
}

} // namespace lyncs