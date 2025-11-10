/*
Copyright (c) 2003-2009 Erwin Coumans  http://bullet.googlecode.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/



#ifndef TF3_SCALAR_H
#define TF3_SCALAR_H

#ifdef TF3_MANAGED_CODE
//Aligned data types not supported in managed code
#pragma unmanaged
#endif


#include <math.h>
#include <stdlib.h>//size_t for MSVC 6.0
#include <cstdlib>
#include <cfloat>
#include <float.h>

#if defined(DEBUG) || defined (_DEBUG)
#define TF3_DEBUG
#endif


#ifdef _WIN32

		#if defined(__MINGW32__) || defined(__CYGWIN__) || (defined (_MSC_VER) && _MSC_VER < 1300)

			#define TF3SIMD_FORCE_INLINE inline
			#define ATTRIBUTE_ALIGNED16(a) a
			#define ATTRIBUTE_ALIGNED64(a) a
			#define ATTRIBUTE_ALIGNED128(a) a
		#else
			//#define TF3_HAS_ALIGNED_ALLOCATOR
			#pragma warning(disable : 4324) // disable padding warning
//			#pragma warning(disable:4530) // Disable the exception disable but used in MSCV Stl warning.
//			#pragma warning(disable:4996) //Turn off warnings about deprecated C routines
//			#pragma warning(disable:4786) // Disable the "debug name too long" warning

			#define TF3SIMD_FORCE_INLINE __forceinline
			#define ATTRIBUTE_ALIGNED16(a) __declspec(align(16)) a
			#define ATTRIBUTE_ALIGNED64(a) __declspec(align(64)) a
			#define ATTRIBUTE_ALIGNED128(a) __declspec (align(128)) a
		#ifdef _XBOX
			#define TF3_USE_VMX128

			#include <ppcintrinsics.h>
 			#define TF3_HAVE_NATIVE_FSEL
 			#define tf3Fsel(a,b,c) __fsel((a),(b),(c))
		#else


		#endif//_XBOX

		#endif //__MINGW32__

		#include <assert.h>
#ifdef TF3_DEBUG
		#define tf3Assert assert
#else
		#define tf3Assert(x)
#endif
		//tf3FullAssert is optional, slows down a lot
		#define tf3FullAssert(x)

		#define tf3Likely(_c)  _c
		#define tf3Unlikely(_c) _c

#else
	
#if defined	(__CELLOS_LV2__)
		#define TF3SIMD_FORCE_INLINE inline
		#define ATTRIBUTE_ALIGNED16(a) a __attribute__ ((aligned (16)))
		#define ATTRIBUTE_ALIGNED64(a) a __attribute__ ((aligned (64)))
		#define ATTRIBUTE_ALIGNED128(a) a __attribute__ ((aligned (128)))
		#ifndef assert
		#include <assert.h>
		#endif
#ifdef TF3_DEBUG
		#define tf3Assert assert
#else
		#define tf3Assert(x)
#endif
		//tf3FullAssert is optional, slows down a lot
		#define tf3FullAssert(x)

		#define tf3Likely(_c)  _c
		#define tf3Unlikely(_c) _c

#else

#ifdef USE_LIBSPE2

		#define TF3SIMD_FORCE_INLINE __inline
		#define ATTRIBUTE_ALIGNED16(a) a __attribute__ ((aligned (16)))
		#define ATTRIBUTE_ALIGNED64(a) a __attribute__ ((aligned (64)))
		#define ATTRIBUTE_ALIGNED128(a) a __attribute__ ((aligned (128)))
		#ifndef assert
		#include <assert.h>
		#endif
#ifdef TF3_DEBUG
		#define tf3Assert assert
#else
		#define tf3Assert(x)
#endif
		//tf3FullAssert is optional, slows down a lot
		#define tf3FullAssert(x)


		#define tf3Likely(_c)   __builtin_expect((_c), 1)
		#define tf3Unlikely(_c) __builtin_expect((_c), 0)
		

#else
	//non-windows systems

		#define TF3SIMD_FORCE_INLINE inline
		///@todo: check out alignment methods for other platforms/compilers
		///#define ATTRIBUTE_ALIGNED16(a) a __attribute__ ((aligned (16)))
		///#define ATTRIBUTE_ALIGNED64(a) a __attribute__ ((aligned (64)))
		///#define ATTRIBUTE_ALIGNED128(a) a __attribute__ ((aligned (128)))
		#define ATTRIBUTE_ALIGNED16(a) a
		#define ATTRIBUTE_ALIGNED64(a) a
		#define ATTRIBUTE_ALIGNED128(a) a
		#ifndef assert
		#include <assert.h>
		#endif

#if defined(DEBUG) || defined (_DEBUG)
		#define tf3Assert assert
#else
		#define tf3Assert(x)
#endif

		//tf3FullAssert is optional, slows down a lot
		#define tf3FullAssert(x)
		#define tf3Likely(_c)  _c
		#define tf3Unlikely(_c) _c

#endif // LIBSPE2

#endif	//__CELLOS_LV2__
#endif


///The tf3Scalar type abstracts floating point numbers, to easily switch between double and single floating point precision.
typedef double tf3Scalar;
//this number could be bigger in double precision
#define TF3_LARGE_FLOAT 1e30


#define TF3_DECLARE_ALIGNED_ALLOCATOR() \
   TF3SIMD_FORCE_INLINE void* operator new(size_t sizeInBytes)   { return tf3AlignedAlloc(sizeInBytes,16); }   \
   TF3SIMD_FORCE_INLINE void  operator delete(void* ptr)         { tf3AlignedFree(ptr); }   \
   TF3SIMD_FORCE_INLINE void* operator new(size_t, void* ptr)   { return ptr; }   \
   TF3SIMD_FORCE_INLINE void  operator delete(void*, void*)      { }   \
   TF3SIMD_FORCE_INLINE void* operator new[](size_t sizeInBytes)   { return tf3AlignedAlloc(sizeInBytes,16); }   \
   TF3SIMD_FORCE_INLINE void  operator delete[](void* ptr)         { tf3AlignedFree(ptr); }   \
   TF3SIMD_FORCE_INLINE void* operator new[](size_t, void* ptr)   { return ptr; }   \
   TF3SIMD_FORCE_INLINE void  operator delete[](void*, void*)      { }   \



		
TF3SIMD_FORCE_INLINE tf3Scalar tf3Sqrt(tf3Scalar x) { return sqrt(x); }
TF3SIMD_FORCE_INLINE tf3Scalar tf3Fabs(tf3Scalar x) { return fabs(x); }
TF3SIMD_FORCE_INLINE tf3Scalar tf3Cos(tf3Scalar x) { return cos(x); }
TF3SIMD_FORCE_INLINE tf3Scalar tf3Sin(tf3Scalar x) { return sin(x); }
TF3SIMD_FORCE_INLINE tf3Scalar tf3Tan(tf3Scalar x) { return tan(x); }
TF3SIMD_FORCE_INLINE tf3Scalar tf3Acos(tf3Scalar x) { if (x<tf3Scalar(-1))	x=tf3Scalar(-1); if (x>tf3Scalar(1))	x=tf3Scalar(1); return acos(x); }
TF3SIMD_FORCE_INLINE tf3Scalar tf3Asin(tf3Scalar x) { if (x<tf3Scalar(-1))	x=tf3Scalar(-1); if (x>tf3Scalar(1))	x=tf3Scalar(1); return asin(x); }
TF3SIMD_FORCE_INLINE tf3Scalar tf3Atan(tf3Scalar x) { return atan(x); }
TF3SIMD_FORCE_INLINE tf3Scalar tf3Atan2(tf3Scalar x, tf3Scalar y) { return atan2(x, y); }
TF3SIMD_FORCE_INLINE tf3Scalar tf3Exp(tf3Scalar x) { return exp(x); }
TF3SIMD_FORCE_INLINE tf3Scalar tf3Log(tf3Scalar x) { return log(x); }
TF3SIMD_FORCE_INLINE tf3Scalar tf3Pow(tf3Scalar x,tf3Scalar y) { return pow(x,y); }
TF3SIMD_FORCE_INLINE tf3Scalar tf3Fmod(tf3Scalar x,tf3Scalar y) { return fmod(x,y); }


#define TF3SIMD_2_PI         tf3Scalar(6.283185307179586232)
#define TF3SIMD_PI           (TF3SIMD_2_PI * tf3Scalar(0.5))
#define TF3SIMD_HALF_PI      (TF3SIMD_2_PI * tf3Scalar(0.25))
#define TF3SIMD_RADS_PER_DEG (TF3SIMD_2_PI / tf3Scalar(360.0))
#define TF3SIMD_DEGS_PER_RAD  (tf3Scalar(360.0) / TF3SIMD_2_PI)
#define TF3SIMDSQRT12 tf3Scalar(0.7071067811865475244008443621048490)

#define tf3RecipSqrt(x) ((tf3Scalar)(tf3Scalar(1.0)/tf3Sqrt(tf3Scalar(x))))		/* reciprocal square root */


#define TF3SIMD_EPSILON      DBL_EPSILON
#define TF3SIMD_INFINITY     DBL_MAX

TF3SIMD_FORCE_INLINE tf3Scalar tf3Atan2Fast(tf3Scalar y, tf3Scalar x) 
{
	tf3Scalar coeff_1 = TF3SIMD_PI / 4.0f;
	tf3Scalar coeff_2 = 3.0f * coeff_1;
	tf3Scalar abs_y = tf3Fabs(y);
	tf3Scalar angle;
	if (x >= 0.0f) {
		tf3Scalar r = (x - abs_y) / (x + abs_y);
		angle = coeff_1 - coeff_1 * r;
	} else {
		tf3Scalar r = (x + abs_y) / (abs_y - x);
		angle = coeff_2 - coeff_1 * r;
	}
	return (y < 0.0f) ? -angle : angle;
}

TF3SIMD_FORCE_INLINE bool      tf3FuzzyZero(tf3Scalar x) { return tf3Fabs(x) < TF3SIMD_EPSILON; }

TF3SIMD_FORCE_INLINE bool	tf3Equal(tf3Scalar a, tf3Scalar eps) {
	return (((a) <= eps) && !((a) < -eps));
}
TF3SIMD_FORCE_INLINE bool	tf3GreaterEqual (tf3Scalar a, tf3Scalar eps) {
	return (!((a) <= eps));
}


TF3SIMD_FORCE_INLINE int       tf3IsNegative(tf3Scalar x) {
    return x < tf3Scalar(0.0) ? 1 : 0;
}

TF3SIMD_FORCE_INLINE tf3Scalar tf3Radians(tf3Scalar x) { return x * TF3SIMD_RADS_PER_DEG; }
TF3SIMD_FORCE_INLINE tf3Scalar tf3Degrees(tf3Scalar x) { return x * TF3SIMD_DEGS_PER_RAD; }

#define TF3_DECLARE_HANDLE(name) typedef struct name##__ { int unused; } *name

#ifndef tf3Fsel
TF3SIMD_FORCE_INLINE tf3Scalar tf3Fsel(tf3Scalar a, tf3Scalar b, tf3Scalar c)
{
	return a >= 0 ? b : c;
}
#endif
#define tf3Fsels(a,b,c) (tf3Scalar)tf3Fsel(a,b,c)


TF3SIMD_FORCE_INLINE bool tf3MachineIsLittleEndian()
{
   long int i = 1;
   const char *p = (const char *) &i;
   if (p[0] == 1)  // Lowest address contains the least significant byte
	   return true;
   else
	   return false;
}



///tf3Select avoids branches, which makes performance much better for consoles like Playstation 3 and XBox 360
///Thanks Phil Knight. See also http://www.cellperformance.com/articles/2006/04/more_techniques_for_eliminatin_1.html
TF3SIMD_FORCE_INLINE unsigned tf3Select(unsigned condition, unsigned valueIfConditionNonZero, unsigned valueIfConditionZero) 
{
    // Set testNz to 0xFFFFFFFF if condition is nonzero, 0x00000000 if condition is zero
    // Rely on positive value or'ed with its negative having sign bit on
    // and zero value or'ed with its negative (which is still zero) having sign bit off 
    // Use arithmetic shift right, shifting the sign bit through all 32 bits
    unsigned testNz = (unsigned)(((int)condition | -(int)condition) >> 31);
    unsigned testEqz = ~testNz;
    return ((valueIfConditionNonZero & testNz) | (valueIfConditionZero & testEqz)); 
}
TF3SIMD_FORCE_INLINE int tf3Select(unsigned condition, int valueIfConditionNonZero, int valueIfConditionZero)
{
    unsigned testNz = (unsigned)(((int)condition | -(int)condition) >> 31);
    unsigned testEqz = ~testNz; 
    return static_cast<int>((valueIfConditionNonZero & testNz) | (valueIfConditionZero & testEqz));
}
TF3SIMD_FORCE_INLINE float tf3Select(unsigned condition, float valueIfConditionNonZero, float valueIfConditionZero)
{
#ifdef TF3_HAVE_NATIVE_FSEL
    return (float)tf3Fsel((tf3Scalar)condition - tf3Scalar(1.0f), valueIfConditionNonZero, valueIfConditionZero);
#else
    return (condition != 0) ? valueIfConditionNonZero : valueIfConditionZero; 
#endif
}

template<typename T> TF3SIMD_FORCE_INLINE void tf3Swap(T& a, T& b)
{
	T tmp = a;
	a = b;
	b = tmp;
}


//PCK: endian swapping functions
TF3SIMD_FORCE_INLINE unsigned tf3SwapEndian(unsigned val)
{
	return (((val & 0xff000000) >> 24) | ((val & 0x00ff0000) >> 8) | ((val & 0x0000ff00) << 8)  | ((val & 0x000000ff) << 24));
}

TF3SIMD_FORCE_INLINE unsigned short tf3SwapEndian(unsigned short val)
{
	return static_cast<unsigned short>(((val & 0xff00) >> 8) | ((val & 0x00ff) << 8));
}

TF3SIMD_FORCE_INLINE unsigned tf3SwapEndian(int val)
{
	return tf3SwapEndian((unsigned)val);
}

TF3SIMD_FORCE_INLINE unsigned short tf3SwapEndian(short val)
{
	return tf3SwapEndian((unsigned short) val);
}

///tf3SwapFloat uses using char pointers to swap the endianness
////tf3SwapFloat/tf3SwapDouble will NOT return a float, because the machine might 'correct' invalid floating point values
///Not all values of sign/exponent/mantissa are valid floating point numbers according to IEEE 754. 
///When a floating point unit is faced with an invalid value, it may actually change the value, or worse, throw an exception. 
///In most systems, running user mode code, you wouldn't get an exception, but instead the hardware/os/runtime will 'fix' the number for you. 
///so instead of returning a float/double, we return integer/long long integer
TF3SIMD_FORCE_INLINE unsigned int  tf3SwapEndianFloat(float d)
{
    unsigned int a = 0;
    unsigned char *dst = (unsigned char *)&a;
    unsigned char *src = (unsigned char *)&d;

    dst[0] = src[3];
    dst[1] = src[2];
    dst[2] = src[1];
    dst[3] = src[0];
    return a;
}

// unswap using char pointers
TF3SIMD_FORCE_INLINE float tf3UnswapEndianFloat(unsigned int a) 
{
    float d = 0.0f;
    unsigned char *src = (unsigned char *)&a;
    unsigned char *dst = (unsigned char *)&d;

    dst[0] = src[3];
    dst[1] = src[2];
    dst[2] = src[1];
    dst[3] = src[0];

    return d;
}


// swap using char pointers
TF3SIMD_FORCE_INLINE void  tf3SwapEndianDouble(double d, unsigned char* dst)
{
    unsigned char *src = (unsigned char *)&d;

    dst[0] = src[7];
    dst[1] = src[6];
    dst[2] = src[5];
    dst[3] = src[4];
    dst[4] = src[3];
    dst[5] = src[2];
    dst[6] = src[1];
    dst[7] = src[0];

}

// unswap using char pointers
TF3SIMD_FORCE_INLINE double tf3UnswapEndianDouble(const unsigned char *src) 
{
    double d = 0.0;
    unsigned char *dst = (unsigned char *)&d;

    dst[0] = src[7];
    dst[1] = src[6];
    dst[2] = src[5];
    dst[3] = src[4];
    dst[4] = src[3];
    dst[5] = src[2];
    dst[6] = src[1];
    dst[7] = src[0];

	return d;
}

// returns normalized value in range [-TF3SIMD_PI, TF3SIMD_PI]
TF3SIMD_FORCE_INLINE tf3Scalar tf3NormalizeAngle(tf3Scalar angleInRadians) 
{
	angleInRadians = tf3Fmod(angleInRadians, TF3SIMD_2_PI);
	if(angleInRadians < -TF3SIMD_PI)
	{
		return angleInRadians + TF3SIMD_2_PI;
	}
	else if(angleInRadians > TF3SIMD_PI)
	{
		return angleInRadians - TF3SIMD_2_PI;
	}
	else
	{
		return angleInRadians;
	}
}

///rudimentary class to provide type info
struct tf3TypedObject
{
	tf3TypedObject(int objectType)
		:m_objectType(objectType)
	{
	}
	int	m_objectType;
	inline int getObjectType() const
	{
		return m_objectType;
	}
};
#endif //TF3SIMD___SCALAR_H
