#ifndef _AF_VECTOR3_H_
#define _AF_VECTOR3_H_

#include <iostream>

#include <cmath>
#include <cassert>

// simple implementation of vector3d
namespace afbase
{
	template<class T>
	class vector3
	{
	public:

		typedef T type_t;
		typedef unsigned int size_t;

	public:

		vector3() {}
		explicit vector3(T sx, T sy, T sz): x_(sx), y_(sy), z_(sz) {}
		explicit vector3(T el): x_(el), y_(el), z_(el) {}
		vector3(const vector3<T>& v): x_(v.x()), y_(v.y()), z_(v.z()) {}

		template<class Y>
		explicit vector3(vector3<Y>& v): x_(v.x()), y_(v.y()), z_(v.z()) {}

		template<class Y>
		vector3<T>& operator=(const vector3<Y>& v) { x_ = v.x(); y_ = v.y(); z_ = v.z(); return *this; }

		vector3<T>& operator=(const vector3<T>& v) { x_ = v.x_; y_ = v.y_; z_ = v.z_; return *this; }

	public:

		void print(std::ostream& os) const { os << x_ << " " << y_ << " " << z_; }
		void read(std::istream& is) { is >> x_ >> y_ >> z_; }

		void clear() {
			x_ = 0;
			y_ = 0;
			z_ = 0;
		}

		T norm() const {
			T r2 = x_*x_;
			r2 += y_*y_;
			r2 += z_*z_;
			return sqrt(r2);
		}

		T square_norm() const {
			T r2 = x_*x_;
			r2 += y_*y_;
			r2 += z_*z_;
			return r2;
		}

		void normalize(double tol=1e-24) {
			T l = norm();
			assert((l >= tol) && "can not normalize null vector");
			x_ /= l; y_ /= l; z_ /= l;
		}

		template<class Y>
		void fill(Y el) { x_ = el; y_ = el; z_ = el; }

		T x() const { return x_; }
		T y() const { return y_; }
		T z() const { return z_; }

		T& x() { return x_; }
		T& y() { return y_; }
		T& z() { return z_; }

		T dot_product(const vector3<T>& v) const {
			T res = x_ * v.x_;
			res += y_ * v.y_;
			res += z_ * v.z_;
			return res;
		}

		vector3<T> elt_product(const vector3<T>& v) const {
			return vector3<T>(x_ * v.x_, y_ * v.y_, z_ * v.z_);
		}

		vector3<T> cross_product(const vector3<T>& v) const {
			T nx = y_ * v.z_; nx -= z_ * v.y_;
			T ny = z_ * v.x_; ny -= x_ * v.z_;
			T nz = x_ * v.y_; nz -= y_ * v.x_;
			return vector3<T>(nx, ny, nz);
		}

	public:

		bool operator==(const vector3<T> &v) const
		{
			return (
				(v.x_==x_) &&
				(v.y_==y_) && 
				(v.z_==z_));
		};

		const vector3<T> operator-() const { return vector3<T>(-x_, -y_, -z_); }

		template<class Y>
		void operator+=(Y el) {
			x_ += el; y_ += el; z_ += el;
		}

		template<class Y>
		void operator+=(const vector3<Y>& v) {
			x_ += v.x(); y_ += v.y(); z_ += v.z();
		}

		template<class Y>
		void operator-=(Y el) {
			x_ -= el; y_ -= el; z_ -= el;
		}

		template<class Y>
		void operator-=(const vector3<Y>& v) {
			x_ -= v.x(); y_ -= v.y(); z_ -= v.z();
		}

		template<class Y>
		void operator*=(Y el) {
			x_ *= el; y_ *= el; z_ *= el;
		}

		template<class Y>
		void operator/=(Y el) {
			x_ /= el; y_ /= el; z_ /= el;
		}

		/*
		* Access to the elements: the body is the same for all the methods, thus
		* this macro to avoid code duplication. The double slash (//) at the end
		* of the macro allows the caller to finish the line by a semicolumn (;) to
		* avoid auto-indent issues with some IDEs.
		*/
#define ACCESS \
	switch(i) \
		{ \
	case 0: return x_; break; \
	case 1: return y_; break; \
	case 2: return z_; break; \
		} //


		T operator()(size_t i) const {
			ACCESS;
			return x_; // to avoid warnings from paranoid compilers
		}
		T& operator()(size_t i) {
			ACCESS;
			return x_; // to avoid warnings from paranoid compilers
		}
		T operator[](size_t i) const {
			ACCESS;
			return x_; // to avoid warnings from paranoid compilers
		}
		T& operator[](size_t i) {
			ACCESS;
			return x_; // to avoid warnings from paranoid compilers
		}


#undef ACCESS

	private:

		T x_;
		T y_;
		T z_;

		template<class W, class Y>
		friend const vector3<W> operator+(Y el, const vector3<W>& v);

		template<class W, class Y>
		friend const vector3<W> operator+(const vector3<W>& v, Y el);

		template<class W>
		friend const vector3<W> operator+(const vector3<W>& v1, const vector3<W>& v2);

		template<class W, class Y>
		friend const vector3<W> operator-(Y el, const vector3<W>& v);

		template<class W, class Y>
		friend const vector3<W> operator-(const vector3<W>& v, Y el);

		template<class W>
		friend const vector3<W> operator-(const vector3<W>& v1, const vector3<W>& v2);

		template<class W, class Y>
		friend const vector3<W> operator*(Y el, const vector3<W>& v);

		template<class W, class Y>
		friend const vector3<W> operator*(const vector3<W>& v, Y el);

		template<class W, class Y>
		friend const vector3<W> operator/(const vector3<W>& v, Y el);
	};

	template<class T, class Y>
	inline const vector3<T> operator+(Y el, const vector3<T>& v)
	{
		return vector3<T>(v.x_ + el, v.y_ + el, v.z_ + el);
	}

	template<class T, class Y>
	inline const vector3<T> operator+(const vector3<T>& v, Y el)
	{
		return vector3<T>(v.x_ + el, v.y_ + el, v.z_ + el);
	}

	template<class T>
	inline const vector3<T> operator+(const vector3<T>& v1, const vector3<T>& v2)
	{
		return vector3<T>(v1.x_ + v2.x_, v1.y_ + v2.y_, v1.z_ + v2.z_);
	}

	template<class T, class Y>
	inline const vector3<T> operator-(Y el, const vector3<T>& v)
	{
		return vector3<T>(el - v.x_, el - v.y_, el - v.z_);
	}

	template<class T, class Y>
	inline const vector3<T> operator-(const vector3<T>& v, Y el)
	{
		return vector3<T>(v.x_ - el, v.y_ - el, v.z_ - el);
	}

	template<class T>
	inline const vector3<T> operator-(const vector3<T>& v1, const vector3<T>& v2)
	{
		return vector3<T>(v1.x_ - v2.x_, v1.y_ - v2.y_, v1.z_ - v2.z_);
	}

	template<class T, class Y>
	inline const vector3<T> operator*(Y el, const vector3<T>& v)
	{
		return vector3<T>(v.x_ * el, v.y_ * el, v.z_ * el);
	}

	template<class T, class Y>
	inline const vector3<T> operator*(const vector3<T>& v, Y el)
	{
		return vector3<T>(v.x_ * el, v.y_ * el, v.z_ * el);
	}

	template<class T, class Y>
	inline const vector3<T> operator/(const vector3<T>& v, Y el)
	{
		return vector3<T>(v.x_ / el, v.y_ / el, v.z_ / el);
	}

	template<class T>
	inline std::ostream& operator<<(std::ostream& os, const vector3<T>& v)
	{
		v.print(os);
		return os;
	}

	template<class T>
	inline std::istream& operator>>(std::istream& is, vector3<T>& v)
	{
		v.read(is);
		return is;
	}

	typedef vector3<double> vector3d;
	typedef vector3<float> vector3f;
	typedef vector3<int> vector3i;
}

#endif
