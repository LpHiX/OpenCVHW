//======================================================================================================
// Copyright 2012, NaturalPoint Inc.
//======================================================================================================
#pragma once

// System includes
#include <string.h> //== for memcpy
#include <math.h>
#include <float.h>
#include <iostream>
#include <algorithm>

// Local includes
#include "Platform.h"

#include "Core/EulerTypes.h"

namespace Core
{
    template<typename T>
    class cVector3;

    template<typename T, bool AutoNormalize>
    class cQuaternion;

    // 4 x 4 Row Major Matrix
    template<typename T>
    class cMatrix4
    {
    public:
        cMatrix4();
        cMatrix4( const cMatrix4& other );
        cMatrix4( T t11, T t12, T t13, T t14, T t21, T t22, T t23, T t24, T t31, T t32, T t33, T t34, T t41, T t42, T t43, T t44 );
        cMatrix4( const T vals[16] );

        /// <summary>Set the vector values.</summary>
        void            SetValues( T t11, T t12, T t13, T t14, T t21, T t22, T t23, T t24, T t31, T t32, T t33, T t34, T t41, T t42, T t43, T t44 );

        /// <summary>Set the vector values.</summary>
        void            SetValues( const T vals[16] );

        /// <summary>Set the value at specified location.</summary>
        void            SetValue( int row, int col, const T val );

        /// <summary>Returns the value at row(0 to 3) and col(0 to 3).</summary>
        T               Value( int row, int col ) const; 

        /// <summary>Returns the value at row(0 to 3) and col(0 to 3).</summary>
        T&              Value( int row, int col );

        /// <summary>Copyes values from source.</summary>
        void            CopyValues( const cMatrix4<T> &src, int start, int count );

        /// <summary>Access to the data array.</summary>
        const T*        Data() const;

        /// <summary>Translation matrix.</summary>
        void            Translate( T x, T y, T z );

        /// <summary>Translation matrix.</summary>
        void            Translate( const cVector3<T>& v );

        /// <summary>Return the translation component of the matrix.</summary>
        cVector3<T>     Translation() const;

        /// <summary>Set this to an X axis rotation matrix, with angle given in radians.</summary>
        void            SetRotateX( T angle );

        /// <summary>Set this to a Y axis rotation matrix, with angle given in radians.</summary>
        void            SetRotateY( T angle );

        /// <summary>Set this to a Z axis rotation matrix, with angle given in radians.</summary>
        void            SetRotateZ( T angle );

        /// <summary>Set this matrix to a Euler rotation with specified rotation order, angles in radians.</summary>
        void            SetRotation( T x, T y, T z, int rotationOrder = EulOrdXYZr );

        /// <summary>Set this matrix to a rotation matrix from a quaternion.</summary>
        void            SetRotation( const cQuaternion<T, false> &q );

        /// <summary>Return the rotation component of the matrix.</summary>
        cQuaternion<T, true> Rotation() const;

        /// <summary>Set the scale component of the matrix.</summary>
        void            SetScale( T x, T y, T z );

        /// <summary>Set the scale component of the matrix.</summary>
        void            SetScale( const cVector3<T>& v );

        /// <summary>Returns the scale component of the matrix.</summary>
        cVector3<T>     Scale() const;

        /// <summary>Combined rotation and translation matrix.</summary>
        void            RotateTranslate( const cQuaternion<T, false>& q, const cVector3<T>& v );

        /// <summary>Combined scale, rotation and translation matrix.</summary>
        void            ScaleRotateTranslate( const cVector3<T>& s, const cQuaternion<T, false>& q, const cVector3<T>& v );

        /// <summary>Returns the determinant of the matrix.<summary>
        T               Determinant() const;

        /// <summary>Replaces the contents of the matrix with its inverse.</summary>
        void            Invert();

        /// <summary>Returns the inverse of the matrix.<summary>
        cMatrix4        Inverse() const;

        /// <summary>Replaces the contents of this matrix with its transpose.<summary>
        void            Transpose();

        /// <summary>Builds perspective view projection matrix for left handed coordinate.</summary>
        void            PerspectiveFovLH( float fovY, float aspect, float nearClipPlane, float farClipPlane );

        /// <summary>Builds perspective view projection matrix for right handed coordinate.</summary>
        void            PerspectiveFovRH( float fovY, float aspect, float nearClipPlane, float farClipPlane );

        /// <summary>Builds perspective view projection matrix off centered for left handed coordinate.</summary>
        void            PerspectiveOffCenterLH( float left, float right, float bottom, float top, float nearClipPlane, float farClipPlane );

        /// <summary>Builds perspective view projection matrix off centered for right handed coordinate.</summary>
        void            PerspectiveOffCenterRH( float left, float right, float bottom, float top, float nearClipPlane, float farClipPlane );

        /// <summary>Builds view matrix for left handed coordinate.</summary>
        void            LookAtLH( const cVector3<T> &eye, const cVector3<T> &lookAt, const cVector3<T> &up );

        /// <summary>Builds view matrix for right handed coordinate.</summary>
        void            LookAtRH( const cVector3<T> &eye, const cVector3<T> &lookAt, const cVector3<T> &up );

        /// <summary>Builds orthogonal view projection matrix for left handed coordinate.</summary>
        void            OrthoLH( float w, float h, float nearClipPlane, float farClipPlane );

        /// <summary>Builds orthogonal view projection matrix for right handed coordinate.</summary>
        void            OrthoRH( float w, float h, float nearClipPlane, float farClipPlane );

        /// <summary>Builds orthogonal view projection matrix off centered for left handed coordinate.</summary>
        void            OrthoOffCenterLH( float left, float right, float bottom, float top, float nearClipPlane, float farClipPlane );

        /// <summary>Builds orthogonal view projection matrix off centered for right handed coordinate.</summary>
        void            OrthoOffCenterRH( float left, float right, float bottom, float top, float nearClipPlane, float farClipPlane );

        //====================================================================================
        // Type conversion helpers
        //====================================================================================

        cMatrix4<float> ToFloat() const;

        cMatrix4<double> ToDouble() const;

        //====================================================================================
        // Operators
        //====================================================================================

        bool            operator==( const cMatrix4& rhs ) const;
        bool            operator!=( const cMatrix4& rhs ) const;
        cMatrix4&       operator*=( const cMatrix4& rhs );
        cMatrix4        operator*( const cMatrix4& rhs ) const;

        //====================================================================================
        // Helper constants
        //====================================================================================
        
		static const cMatrix4 kZero;
		static const cMatrix4 kIdentity;

    private:
                    /*  0  1  2  3 */
                    /*  4  5  6  7 */
                    /*  8  9 10 11 */
                    /* 12 13 14 15 */
        T               mVals[16];

        void            Transpose( const cMatrix4& m );
        void            Invert( const cMatrix4& m );
        void            Multiply( const cMatrix4& m1, const cMatrix4& m2 );
    };

    //=========================================================================
    // Stream I/O operators
    //=========================================================================
    template< typename T>
    std::wostream& operator<<( std::wostream& os, const cMatrix4<T>& m )
    {
        os << L"|" << m.Value( 0, 0 ) << L"," << m.Value( 0, 1 ) << L"," << m.Value( 0, 2 ) << L"," << m.Value( 0, 3 ) << L"|\n";
        os << L"|" << m.Value( 1, 0 ) << L"," << m.Value( 1, 1 ) << L"," << m.Value( 1, 2 ) << L"," << m.Value( 1, 3 ) << L"|\n";
        os << L"|" << m.Value( 2, 0 ) << L"," << m.Value( 2, 1 ) << L"," << m.Value( 2, 2 ) << L"," << m.Value( 2, 3 ) << L"|\n";
        os << L"|" << m.Value( 3, 0 ) << L"," << m.Value( 3, 1 ) << L"," << m.Value( 3, 2 ) << L"," << m.Value( 3, 3 ) << L"|\n";
        return os;
    }   

    template< typename T>
    std::ostream& operator<<( std::ostream& os, const cMatrix4<T>& m )
    {
		for ( int i = 0; i < 4; ++i )
		{
			for ( int j = 0; j < 4; ++j )
			{
				T value = m.Value( i, j );
				os.write( (char *)&value, sizeof( T ) );
			}
		}
       
        return os;
    }

	template<typename T>
	std::istream& operator >> ( std::istream& is, cMatrix4<T>& m )
	{
		for ( int i = 0; i < 4; ++i )
		{
			for ( int j = 0; j < 4; ++j )
			{
				T value;
				is.read( (char *)&value, sizeof( T ) );
				m.SetValue( i, j, value );
			}
		}
		return is;
	}

    template <typename T>
    const cMatrix4<T> cMatrix4<T>::kZero( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
    template <typename T>
    const cMatrix4<T> cMatrix4<T>::kIdentity( 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 );

    typedef cMatrix4<float> cMatrix4f;
    typedef cMatrix4<double> cMatrix4d;
}

#include "Core/Matrix4.inl"



