// simple tools for parsing.

#ifndef MATRIX_OPERATION_HXX
#define MATRIX_OPERATION_HXX

//// Matrix operations

template<class T>
inline T sign(T val)
{
	if(val < 0){ return static_cast<T>(-1.0); }
	return static_cast<T>(1.0);
}

// Compute the euler angles corresponding to the given rotation matrix.
template<class T>
inline afbase::vector3<T> computeEulerFromRotationMatrix ( const afbase::matrix3<T> & rotation )
{
	afbase::vector3<T> euler(0,0,0);
	double rotationMatrix00 = rotation(0,0);
	double rotationMatrix10 = rotation(1,0);
	double rotationMatrix20 = rotation(2,0);
	double rotationMatrix01 = rotation(0,1);
	double rotationMatrix11 = rotation(1,1);
	double rotationMatrix21 = rotation(2,1);
	double rotationMatrix02 = rotation(0,2);
	double rotationMatrix12 = rotation(1,2);
	double rotationMatrix22 = rotation(2,2);

	double cosTheta = sqrt(0.5 * ( rotationMatrix00*rotationMatrix00 +  rotationMatrix10*rotationMatrix10 + rotationMatrix21*rotationMatrix21 + rotationMatrix22*rotationMatrix22));
	double sinTheta = -rotationMatrix20;
	euler[1] = atan2 (sinTheta, cosTheta);

	double cosTheta_cosPhi = 0.5 * (rotationMatrix22 * rotationMatrix11 - rotationMatrix21 * rotationMatrix12);
	double cosTheta_sinPhi = 0.5 * (rotationMatrix21 * rotationMatrix02 - rotationMatrix22 * rotationMatrix01);
	double cosTheta_cosPsi = 0.5 * (rotationMatrix00 * rotationMatrix11 - rotationMatrix01 * rotationMatrix10);
	double cosTheta_sinPsi = 0.5 * (rotationMatrix10 * rotationMatrix02 - rotationMatrix00 * rotationMatrix12);

	//if cosTheta == 0
	if (fabs(cosTheta) < 1e-9 )
	{
		if (sinTheta > 0.5) // sinTheta ~= 1
		{
			//phi_psi = phi - psi
			double phi_psi = atan2(- rotationMatrix10, rotationMatrix11);
			double psi = euler[2];

			double phi = phi_psi + psi;
			euler[0] = phi;
		}
		else  //sinTheta  ~= -1
		{
			//phi_psi = phi + psi
			double phi_psi = atan2(- rotationMatrix10,  rotationMatrix11);

			double psi = euler[2];

			double phi = phi_psi;
			euler[0] = phi - psi;
		}
	}
	else
	{
		double cosPsi = cosTheta_cosPsi / cosTheta;
		double sinPsi = cosTheta_sinPsi / cosTheta;
		euler[0] = atan2 (sinPsi, cosPsi);

		double cosPhi = cosTheta_cosPhi / cosTheta;
		double sinPhi = cosTheta_sinPhi / cosTheta;
		euler[2] = atan2 (sinPhi, cosPhi);
	}

	return euler;
}


template<class T>
inline afbase::matrix3<T> computeRotationMatrixFromEuler ( const afbase::vector3<T> & angle)
{
	// We use the ZYX rotation convention with Euler angles
	// See: http://en.wikipedia.org/wiki/Euler_angles
	afbase::matrix3<T> result;

	T cx = cos(angle[0u]);
	T cy = cos(angle[1u]);
	T cz = cos(angle[2u]);
	T sx = sin(angle[0u]);
	T sy = sin(angle[1u]);
	T sz = sin(angle[2u]);
	T szcx = sz * cx;
	T czcx = cz * cx;

	result[0] = cz * cy;	result[1] = -szcx + cz * sy * sx;	result[2] = sz * sx + czcx * sy;
	result[3] = sz * cy;	result[4] = czcx + sz * sy * sx;	result[5] = -cz * sx + szcx * sy;
	result[6] = -sy;		result[7] = cy * sx;				result[8] = cy * cx;

	return result;
}

template<class T, class VecT, class MatT>
inline void matrixToUTheta(VecT& u, T& theta, const MatT& R)
{
	double nz_ay = static_cast<double>(R[5] - R[7]);
	double ax_sz = static_cast<double>(R[6] - R[2]);
	double sy_nx = static_cast<double>(R[1] - R[3]);

	double ctheta = 0.5 * (static_cast<double>(R[0] + R[4] + R[8]) - 1.0);
	double stheta = 0.5 * sqrt(nz_ay*nz_ay + ax_sz*ax_sz + sy_nx*sy_nx);
	theta = static_cast<T>(atan2(stheta, ctheta));

	double denom = 1.0 - ctheta;
	if(denom > 1e-24) {
		u[0] = -sign(nz_ay) * sqrt(std::abs((static_cast<double>(R[0]) - ctheta) / denom));
		u[1] = -sign(ax_sz) * sqrt(std::abs((static_cast<double>(R[4]) - ctheta) / denom));
		u[2] = -sign(sy_nx) * sqrt(std::abs((static_cast<double>(R[8]) - ctheta) / denom));
	}
	else {
		theta = 0.0;
		u[0] = 1.0;
		u[1] = 0.0;
		u[2] = 0.0;
	}
}

template<class T, class VecT, class MatT>
void uThetaToMatrix(MatT& R, const VecT& v, const T& theta)
{
	double c = cos(theta);
	double s = sin(theta);
	double t = 1.0 - c;

	R[0] = c + v.x()*v.x()*t;
	R[4] = c + v.y()*v.y()*t;
	R[8] = c + v.z()*v.z()*t;

	double tmp1 = v.x()*v.y()*t;
	double tmp2 = v.z()*s;
	R[3] = tmp1 + tmp2;
	R[1] = tmp1 - tmp2;

	tmp1 = v.x()*v.z()*t;
	tmp2 = v.y()*s;
	R[6] = tmp1 - tmp2;
	R[2] = tmp1 + tmp2;

	tmp1 = v.y()*v.z()*t;
	tmp2 = v.x()*s;
	R[7] = tmp1 + tmp2;
	R[5] = tmp1 - tmp2;
}
#endif // MATRIX_OPERATION_HXX

