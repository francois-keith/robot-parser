// Simple implementation of matrix 3x3
#ifndef _AF_MATRIX3_H_
#define _AF_MATRIX3_H_

#include <iostream>

#include <cmath>
#include <cassert>

#include "vector3.h"

namespace afbase
{
	template<class T>
	class matrix3
	{
	public:

		typedef T type_t;
		typedef unsigned int size_t;

	public:

		matrix3() {}
		explicit matrix3(
			T m11, T m12, T m13,
			T m21, T m22, T m23, T m31,
			T m32, T m33) {
				m_[0] = m11; m_[1] = m12; m_[2] = m13;
				m_[3] = m21; m_[4] = m22; m_[5] = m23;
				m_[6] = m31; m_[7] = m32; m_[8] = m33;
		}
		explicit matrix3(T el) {
			m_[0] = el; m_[1] = el; m_[2] = el;
			m_[3] = el; m_[4] = el; m_[5] = el;
			m_[6] = el; m_[7] = el; m_[8] = el;
		}
		matrix3(const matrix3<T>& rh) {
			m_[0] = rh.m_[0]; m_[1] = rh.m_[1]; m_[2] = rh.m_[2];
			m_[3] = rh.m_[3]; m_[4] = rh.m_[4]; m_[5] = rh.m_[5];
			m_[6] = rh.m_[6]; m_[7] = rh.m_[7]; m_[8] = rh.m_[8];
		}

		template<class Y>
		explicit matrix3(matrix3<Y>& rh) {
			m_[0] = rh.m_[0]; m_[1] = rh.m_[1]; m_[2] = rh.m_[2];
			m_[3] = rh.m_[3]; m_[4] = rh.m_[4]; m_[5] = rh.m_[5];
			m_[6] = rh.m_[6]; m_[7] = rh.m_[7]; m_[8] = rh.m_[8];
		}

		template<class Y>
		matrix3<T>& operator=(const matrix3<Y>& rh) {
			m_[0] = rh[0]; m_[1] = rh[1]; m_[2] = rh[2];
			m_[3] = rh[3]; m_[4] = rh[4]; m_[5] = rh[5];
			m_[6] = rh[6]; m_[7] = rh[7]; m_[8] = rh[8];
			return *this;
		}

		matrix3<T>& operator=(const matrix3<T>& rh) {
			m_[0] = rh.m_[0]; m_[1] = rh.m_[1]; m_[2] = rh.m_[2];
			m_[3] = rh.m_[3]; m_[4] = rh.m_[4]; m_[5] = rh.m_[5];
			m_[6] = rh.m_[6]; m_[7] = rh.m_[7]; m_[8] = rh.m_[8];
			return *this;
		}

	public:

		void print(std::ostream& os) const {
			os << m_[0] << " " << m_[1] << " " << m_[2] << std::endl;
			os << m_[3] << " " << m_[4] << " " << m_[5] << std::endl;
			os << m_[6] << " " << m_[7] << " " << m_[8];
		}
		void read(std::istream& is) {
			is>>m_[0]>>m_[1]>>m_[2]>>m_[3]>>m_[4]>>m_[5]>>m_[6]>>m_[7]>>m_[8];
		}

		void clear() {
			m_[0] = 0; m_[1] = 0; m_[2] = 0;
			m_[3] = 0; m_[4] = 0; m_[5] = 0;
			m_[6] = 0; m_[7] = 0; m_[8] = 0;
		}

		template<class Y>
		void fill(Y el) {
			m_[0] = el; m_[1] = el; m_[2] = el;
			m_[3] = el; m_[4] = el; m_[5] = el;
			m_[6] = el; m_[7] = el; m_[8] = el;
		}

		void setIdentity() {
			m_[0] = 1; m_[1] = 0; m_[2] = 0;
			m_[3] = 0; m_[4] = 1; m_[5] = 0;
			m_[6] = 0; m_[7] = 0; m_[8] = 1;
		}

		bool isIdentity() const {
			return (
				m_[0] == 1 && m_[1] == 0 && m_[2] == 0 && 
				m_[3] == 0 && m_[4] == 1 && m_[5] == 0 && 
				m_[6] == 0 && m_[7] == 0 && m_[8] == 1
				);
		}

		T det() const {
			T res1 = m_[4]*m_[8];
			res1 -= m_[7]*m_[5];
			res1 *= m_[0];

			T res2 = m_[7]*m_[2];
			res2 -= m_[1]*m_[8];
			res2 *= m_[3];

			T res = m_[1]*m_[5];
			res -= m_[4]*m_[2];
			res *= m_[6];
			res += res1;
			res += res2;

			return res;
		}

		const matrix3<T> inverse(double tol=1e-24) const {
			T delta = det();
			assert((delta >= tol) && "matrix is not invertible");
			delta = 1/delta;

			matrix3<T> res(
				m_[4]*m_[8], m_[7]*m_[2], m_[1]*m_[5],
				m_[6]*m_[5], m_[0]*m_[8], m_[3]*m_[2],
				m_[3]*m_[7], m_[6]*m_[1], m_[0]*m_[4]
			);

			res.m_[0] -= m_[7]*m_[5]; res.m_[1] -= m_[1]*m_[8]; res.m_[2] -= m_[4]*m_[2];
			res.m_[3] -= m_[3]*m_[8]; res.m_[4] -= m_[6]*m_[2]; res.m_[5] -= m_[0]*m_[5];
			res.m_[6] -= m_[6]*m_[4]; res.m_[7] -= m_[0]*m_[7]; res.m_[8] -= m_[3]*m_[1];
			res *= delta;

			return res;
		}

		const matrix3<T> transpose() const {
			return matrix3<T>(
				m_[0], m_[3], m_[6],
				m_[1], m_[4], m_[7],
				m_[2], m_[5], m_[8]
			);
		}

	public:

		const matrix3<T> operator-() const {
			return matrix3<T>(
				-m_[0], -m_[1], -m_[2],
				-m_[1], -m_[4], -m_[5],
				-m_[2], -m_[7], -m_[8]
			);
		}

		template<class Y>
		void operator+=(Y el) {
			m_[0] += el; m_[1] += el; m_[2] += el;
			m_[3] += el; m_[4] += el; m_[5] += el;
			m_[6] += el; m_[7] += el; m_[8] += el;
		}

		template<class Y>
		void operator+=(const matrix3<Y>& rh) {
			m_[0] += rh.m_[0]; m_[1] += rh.m_[1]; m_[2] += rh.m_[2];
			m_[3] += rh.m_[3]; m_[4] += rh.m_[4]; m_[5] += rh.m_[5];
			m_[6] += rh.m_[6]; m_[7] += rh.m_[7]; m_[8] += rh.m_[8];
		}

		template<class Y>
		void operator-=(Y el) {
			m_[0] -= el; m_[1] -= el; m_[2] -= el;
			m_[3] -= el; m_[4] -= el; m_[5] -= el;
			m_[6] -= el; m_[7] -= el; m_[8] -= el;
		}

		template<class Y>
		void operator-=(const matrix3<Y>& rh) {
			m_[0] -= rh.m_[0]; m_[1] -= rh.m_[1]; m_[2] -= rh.m_[2];
			m_[3] -= rh.m_[3]; m_[4] -= rh.m_[4]; m_[5] -= rh.m_[5];
			m_[6] -= rh.m_[6]; m_[7] -= rh.m_[7]; m_[8] -= rh.m_[8];
		}

		template<class Y>
		void operator*=(Y el) {
			m_[0] *= el; m_[1] *= el; m_[2] *= el;
			m_[3] *= el; m_[4] *= el; m_[5] *= el;
			m_[6] *= el; m_[7] *= el; m_[8] *= el;
		}

		template<class Y>
		void operator/=(Y el) {
			m_[0] /= el; m_[1] /= el; m_[2] /= el;
			m_[3] /= el; m_[4] /= el; m_[5] /= el;
			m_[6] /= el; m_[7] /= el; m_[8] /= el;
		}

		T operator()(size_t i, size_t j) const {
			return m_[3*i+j];
		}
		T& operator()(size_t i, size_t j) {
			return m_[3*i+j];
		}
		T operator[](size_t i) const {
			return m_[i];
		}
		T& operator[](size_t i) {
			return m_[i];
		}

	private:

		T m_[9];

		template<class W, class Y>
		friend const matrix3<W> operator+(Y el, const matrix3<W>& rh);

		template<class W, class Y>
		friend const matrix3<W> operator+(const matrix3<W>& rh, Y el);

		template<class W>
		friend const matrix3<W> operator+(const matrix3<W>& m1, const matrix3<W>& m2);

		template<class W, class Y>
		friend const matrix3<W> operator-(Y el, const matrix3<W>& rh);

		template<class W, class Y>
		friend const matrix3<W> operator-(const matrix3<W>& rh, Y el);

		template<class W>
		friend const matrix3<W> operator-(const matrix3<W>& m1, const matrix3<W>& m2);

		template<class W, class Y>
		friend const matrix3<W> operator*(Y el, const matrix3<W>& rh);

		template<class W, class Y>
		friend const matrix3<W> operator*(const matrix3<W>& rh, Y el);

		template<class W, class Y>
		friend const vector3<W> operator*(const matrix3<W>& rh, const vector3<Y>& v);

		template<class W, class Y>
		friend const matrix3<W> operator*(const matrix3<W>& rh1, const matrix3<Y>& rh2);

		template<class W, class Y>
		friend const matrix3<W> operator/(const matrix3<W>& rh, Y el);
	};


	template<class T, class Y>
	inline const matrix3<T> operator+(Y el, const matrix3<T>& rh)
	{
		return matrix3<T>(
			rh.m_[0]+el, rh.m_[1]+el, rh.m_[2]+el,
			rh.m_[3]+el, rh.m_[4]+el, rh.m_[5]+el,
			rh.m_[6]+el, rh.m_[7]+el, rh.m_[8]+el
			);
	}

	template<class T, class Y>
	inline const matrix3<T> operator+(const matrix3<T>& rh, Y el)
	{
		return matrix3<T>(
			rh.m_[0]+el, rh.m_[1]+el, rh.m_[2]+el,
			rh.m_[3]+el, rh.m_[4]+el, rh.m_[5]+el,
			rh.m_[6]+el, rh.m_[7]+el, rh.m_[8]+el
			);
	}

	template<class T>
	inline const matrix3<T> operator+(const matrix3<T>& m1, const matrix3<T>& m2)
	{
		return matrix3<T>(
			m1.m_[0]+m2.m_[0], m1.m_[1]+m2.m_[1], m1.m_[2]+m2.m_[2],
			m1.m_[3]+m2.m_[3], m1.m_[4]+m2.m_[4], m1.m_[5]+m2.m_[5],
			m1.m_[6]+m2.m_[6], m1.m_[7]+m2.m_[7], m1.m_[8]+m2.m_[8]
		);
	}

	template<class T, class Y>
	inline const matrix3<T> operator-(Y el, const matrix3<T>& rh)
	{
		return matrix3<T>(
			el-rh.m_[0], el-rh.m_[1], el-rh.m_[2],
			el-rh.m_[3], el-rh.m_[4], el-rh.m_[5],
			el-rh.m_[6], el-rh.m_[7], el-rh.m_[8]
		);
	}

	template<class T, class Y>
	inline const matrix3<T> operator-(const matrix3<T>& rh, Y el)
	{
		return matrix3<T>(
			rh.m_[0]-el, rh.m_[1]-el, rh.m_[2]-el,
			rh.m_[3]-el, rh.m_[4]-el, rh.m_[5]-el,
			rh.m_[6]-el, rh.m_[7]-el, rh.m_[8]-el
			);
	}

	template<class T>
	inline const matrix3<T> operator-(const matrix3<T>& m1, const matrix3<T>& m2)
	{
		return matrix3<T>(
			m1.m_[0]-m2.m_[0], m1.m_[1]-m2.m_[1], m1.m_[2]-m2.m_[2],
			m1.m_[3]-m2.m_[3], m1.m_[4]-m2.m_[4], m1.m_[5]-m2.m_[5],
			m1.m_[6]-m2.m_[6], m1.m_[7]-m2.m_[7], m1.m_[8]-m2.m_[8]
		);
	}

	template<class T, class Y>
	inline const matrix3<T> operator*(Y el, const matrix3<T>& rh)
	{
		return matrix3<T>(
			rh.m_[0]*el, rh.m_[1]*el, rh.m_[2]*el,
			rh.m_[3]*el, rh.m_[4]*el, rh.m_[5]*el,
			rh.m_[6]*el, rh.m_[7]*el, rh.m_[8]*el
			);
	}

	template<class T, class Y>
	inline const matrix3<T> operator*(const matrix3<T>& rh, Y el)
	{		
		return matrix3<T>(
			rh.m_[0]*el, rh.m_[1]*el, rh.m_[2]*el,
			rh.m_[3]*el, rh.m_[4]*el, rh.m_[5]*el,
			rh.m_[6]*el, rh.m_[7]*el, rh.m_[8]*el
			);
	}

	template<class T, class Y>
	inline const vector3<T> operator*(const matrix3<T>& rh, const vector3<Y>& v)
	{
		return vector3<T>(
			rh.m_[0]*v.x() + rh.m_[1]*v.y() + rh.m_[2]*v.z(),
			rh.m_[3]*v.x() + rh.m_[4]*v.y() + rh.m_[5]*v.z(),
			rh.m_[6]*v.x() + rh.m_[7]*v.y() + rh.m_[8]*v.z()
			);
	}

	template<class T, class Y>
	inline const matrix3<T> operator*(const matrix3<T>& rh1, const matrix3<Y>& rh2)
	{
		return matrix3<T>(
			rh1[0]*rh2[0]+rh1[1]*rh2[3]+rh1[2]*rh2[6],
			rh1[0]*rh2[1]+rh1[1]*rh2[4]+rh1[2]*rh2[7],
			rh1[0]*rh2[2]+rh1[1]*rh2[5]+rh1[2]*rh2[8],

			rh1[3]*rh2[0]+rh1[4]*rh2[3]+rh1[5]*rh2[6],
			rh1[3]*rh2[1]+rh1[4]*rh2[4]+rh1[5]*rh2[7],
			rh1[3]*rh2[2]+rh1[4]*rh2[5]+rh1[5]*rh2[8],

			rh1[6]*rh2[0]+rh1[7]*rh2[3]+rh1[8]*rh2[6],
			rh1[6]*rh2[1]+rh1[7]*rh2[4]+rh1[8]*rh2[7],
			rh1[6]*rh2[2]+rh1[7]*rh2[5]+rh1[8]*rh2[8]
		);
	}

	template<class T, class Y>
	inline const matrix3<T> operator/(const matrix3<T>& rh, Y el)
	{
		return matrix3<T>(
			rh.m_[0]/el, rh.m_[1]/el, rh.m_[2]/el,
			rh.m_[3]/el, rh.m_[4]/el, rh.m_[5]/el,
			rh.m_[6]/el, rh.m_[7]/el, rh.m_[8]/el
			);
	}

	template<class T>
	inline std::ostream& operator<<(std::ostream& os, const matrix3<T>& rh)
	{
		rh.print(os);
		return os;
	}

	template<class T>
	inline std::istream& operator>>(std::istream& is, matrix3<T>& rh)
	{
		rh.read(is);
		return is;
	}

	typedef matrix3<double> matrix3d;
	typedef matrix3<float> matrix3f;
	typedef matrix3<int> matrix3i;
}

#endif
