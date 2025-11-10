/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#ifndef TF3SIMD_QUADWORD_H
#define TF3SIMD_QUADWORD_H

#include "Scalar.h"
#include "MinMax.h"


#if defined (__CELLOS_LV2) && defined (__SPU__)
#include <altivec.h>
#endif

namespace tf3
{
/**@brief The QuadWord class is base class for Vector3 and Quaternion. 
 * Some issues under PS3 Linux with IBM 2.1 SDK, gcc compiler prevent from using aligned quadword.
 */
#ifndef USE_LIBSPE2
ATTRIBUTE_ALIGNED16(class) QuadWord
#else
class QuadWord
#endif
{
protected:

#if defined (__SPU__) && defined (__CELLOS_LV2__)
	union {
		vec_float4 mVec128;
		tf3Scalar	m_floats[4];
	};
public:
	vec_float4	get128() const
	{
		return mVec128;
	}
protected:
#else //__CELLOS_LV2__ __SPU__
	tf3Scalar	m_floats[4];
#endif //__CELLOS_LV2__ __SPU__

	public:
  

  /**@brief Return the x value */
		TF3SIMD_FORCE_INLINE const tf3Scalar& getX() const { return m_floats[0]; }
  /**@brief Return the y value */
		TF3SIMD_FORCE_INLINE const tf3Scalar& getY() const { return m_floats[1]; }
  /**@brief Return the z value */
		TF3SIMD_FORCE_INLINE const tf3Scalar& getZ() const { return m_floats[2]; }
  /**@brief Set the x value */
		TF3SIMD_FORCE_INLINE void	setX(tf3Scalar x) { m_floats[0] = x;};
  /**@brief Set the y value */
		TF3SIMD_FORCE_INLINE void	setY(tf3Scalar y) { m_floats[1] = y;};
  /**@brief Set the z value */
		TF3SIMD_FORCE_INLINE void	setZ(tf3Scalar z) { m_floats[2] = z;};
  /**@brief Set the w value */
		TF3SIMD_FORCE_INLINE void	setW(tf3Scalar w) { m_floats[3] = w;};
  /**@brief Return the x value */
		TF3SIMD_FORCE_INLINE const tf3Scalar& x() const { return m_floats[0]; }
  /**@brief Return the y value */
		TF3SIMD_FORCE_INLINE const tf3Scalar& y() const { return m_floats[1]; }
  /**@brief Return the z value */
		TF3SIMD_FORCE_INLINE const tf3Scalar& z() const { return m_floats[2]; }
  /**@brief Return the w value */
		TF3SIMD_FORCE_INLINE const tf3Scalar& w() const { return m_floats[3]; }

	//TF3SIMD_FORCE_INLINE tf3Scalar&       operator[](int i)       { return (&m_floats[0])[i];	}      
	//TF3SIMD_FORCE_INLINE const tf3Scalar& operator[](int i) const { return (&m_floats[0])[i]; }
	///operator tf3Scalar*() replaces operator[], using implicit conversion. We added operator != and operator == to avoid pointer comparisons.
	TF3SIMD_FORCE_INLINE	operator       tf3Scalar *()       { return &m_floats[0]; }
	TF3SIMD_FORCE_INLINE	operator const tf3Scalar *() const { return &m_floats[0]; }

	TF3SIMD_FORCE_INLINE	bool	operator==(const QuadWord& other) const
	{
		return ((m_floats[3]==other.m_floats[3]) && (m_floats[2]==other.m_floats[2]) && (m_floats[1]==other.m_floats[1]) && (m_floats[0]==other.m_floats[0]));
	}

	TF3SIMD_FORCE_INLINE	bool	operator!=(const QuadWord& other) const
	{
		return !(*this == other);
	}

  /**@brief Set x,y,z and zero w 
   * @param x Value of x
   * @param y Value of y
   * @param z Value of z
   */
		TF3SIMD_FORCE_INLINE void 	setValue(const tf3Scalar& x, const tf3Scalar& y, const tf3Scalar& z)
		{
			m_floats[0]=x;
			m_floats[1]=y;
			m_floats[2]=z;
			m_floats[3] = 0.f;
		}

/*		void getValue(tf3Scalar *m) const 
		{
			m[0] = m_floats[0];
			m[1] = m_floats[1];
			m[2] = m_floats[2];
		}
*/
/**@brief Set the values 
   * @param x Value of x
   * @param y Value of y
   * @param z Value of z
   * @param w Value of w
   */
		TF3SIMD_FORCE_INLINE void	setValue(const tf3Scalar& x, const tf3Scalar& y, const tf3Scalar& z,const tf3Scalar& w)
		{
			m_floats[0]=x;
			m_floats[1]=y;
			m_floats[2]=z;
			m_floats[3]=w;
		}
  /**@brief No initialization constructor */
		TF3SIMD_FORCE_INLINE QuadWord()
		//	:m_floats[0](tf3Scalar(0.)),m_floats[1](tf3Scalar(0.)),m_floats[2](tf3Scalar(0.)),m_floats[3](tf3Scalar(0.))
		{
		}
 
  /**@brief Three argument constructor (zeros w)
   * @param x Value of x
   * @param y Value of y
   * @param z Value of z
   */
		TF3SIMD_FORCE_INLINE QuadWord(const tf3Scalar& x, const tf3Scalar& y, const tf3Scalar& z)		
		{
			m_floats[0] = x, m_floats[1] = y, m_floats[2] = z, m_floats[3] = 0.0f;
		}

/**@brief Initializing constructor
   * @param x Value of x
   * @param y Value of y
   * @param z Value of z
   * @param w Value of w
   */
		TF3SIMD_FORCE_INLINE QuadWord(const tf3Scalar& x, const tf3Scalar& y, const tf3Scalar& z,const tf3Scalar& w) 
		{
			m_floats[0] = x, m_floats[1] = y, m_floats[2] = z, m_floats[3] = w;
		}

  /**@brief Set each element to the max of the current values and the values of another QuadWord
   * @param other The other QuadWord to compare with 
   */
		TF3SIMD_FORCE_INLINE void	setMax(const QuadWord& other)
		{
			tf3SetMax(m_floats[0], other.m_floats[0]);
			tf3SetMax(m_floats[1], other.m_floats[1]);
			tf3SetMax(m_floats[2], other.m_floats[2]);
			tf3SetMax(m_floats[3], other.m_floats[3]);
		}
  /**@brief Set each element to the min of the current values and the values of another QuadWord
   * @param other The other QuadWord to compare with 
   */
		TF3SIMD_FORCE_INLINE void	setMin(const QuadWord& other)
		{
			tf3SetMin(m_floats[0], other.m_floats[0]);
			tf3SetMin(m_floats[1], other.m_floats[1]);
			tf3SetMin(m_floats[2], other.m_floats[2]);
			tf3SetMin(m_floats[3], other.m_floats[3]);
		}



};

}
#endif //TF3SIMD_QUADWORD_H
