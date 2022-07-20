//======================================================================================================
// Copyright 2013, NaturalPoint Inc.
//======================================================================================================
#pragma once

#include "Platform.h"

// System includes (this is needed for VS2005/2008 compiles of the API)
#include <math.h>
#include <float.h>
#include <exception>
#include <iostream>

// Local includes
#include "Core/BuildConfig.h"
#include "Vector3.h"
#include "Matrix4.h"
#include "Core/DebugSystem.h"

namespace Core
{
    template<typename T, bool AutoNormalize>
    class cQuaternion
    {
    public:
        cQuaternion() : mX( 0 ), mY( 0 ), mZ( 0 ), mW( 1 )
        {
        }
        
        cQuaternion( T x, T y, T z, T w ) : mX( x ), mY( y ), mZ( z ), mW( w )
        {
            if( AutoNormalize )
            {
                Normalize();
            }
        }
        
        cQuaternion( const T vals[4] )
        {
            ::memcpy( &mX, vals, 4 * sizeof( T ) );

            if( AutoNormalize )
            {
                Normalize();
            }
        }
        template<typename U> 
        cQuaternion( const U vals[4] )
        {
            mX = (T) vals[0];
            mY = (T) vals[1];
            mZ = (T) vals[2];
            mW = (T) vals[3];

            if( AutoNormalize )
            {
                Normalize();
            }
        }

        template <bool DoAuto>
        cQuaternion( const cQuaternion<T,DoAuto> &val )
        {
            ::memcpy( &mX, val.Data(), 4 * sizeof( T ) );

            if( AutoNormalize )
            {
                Normalize();
            }
        }

        template <typename U, bool DoAuto>
        cQuaternion( const cQuaternion<U,DoAuto>& val )
        {
            mX = T( val.X() );
            mY = T( val.Y() );
            mZ = T( val.Z() );
            mW = T( val.W() );

            if( AutoNormalize )
            {
                Normalize();
            }
        }

        /// <summary>Set the quaternion values.</summary>
        void            SetValues( T x, T y, T z, T w )
        { 
            mX = x; 
            mY = y; 
            mZ = z;
            mW = w;

            if( AutoNormalize )
            {
                Normalize();
            }
        }

        /// <summary>Set the quaternion values from an array.</summary>
        template<typename U> 
        void            SetValues( const U vals[4] )
        {
            mX = vals[0];
            mY = vals[1];
            mZ = vals[2];
            mW = vals[3];

            if( AutoNormalize )
            {
                Normalize();
            }
        }

        /// <summary>Set the real part (i.e. XYZ components) of the quaternion.</summary>
        void            SetRealPart( const cVector3<T> &vec )
        {
            mX = vec.X();
            mY = vec.Y();
            mZ = vec.Z();

            if( AutoNormalize )
            {
                Normalize();
            }
        }

        /// <summary>Set the X quaternion values.</summary>
        template<typename U> 
        void            SetX( const U val )
        {
            mX = val;

            if( AutoNormalize )
            {
                Normalize();
            }
        }

        /// <summary>Set the Y quaternion values.</summary>
        template<typename U> 
        void            SetY( const U val )
        {
            mY = val;

            if( AutoNormalize )
            {
                Normalize();
            }
        }

        /// <summary>Set the Z quaternion values.</summary>
        template<typename U> 
        void            SetZ( const U val )
        {
            mZ = val;

            if( AutoNormalize )
            {
                Normalize();
            }
        }

        /// <summary>Set the W quaternion values.</summary>
        template<typename U> 
        void            SetW( const U val )
        {
            mW = val;

            if( AutoNormalize )
            {
                Normalize();
            }
        }

        /// <summary>Retrieve the real part (i.e. XYZ components) of the quaternion.</summary>
        cVector3<T>     RealPart() const { return cVector3<T>( mX, mY, mZ ); }

        T               X() const { return mX; }
        T               Y() const { return mY; }
        T               Z() const { return mZ; }
        T               W() const { return mW; }

        T&              X() { return mX; }
        T&              Y() { return mY; }
        T&              Z() { return mZ; }
        T&              W() { return mW; }

        T               operator[]( int idx ) const 
        { 
            switch(idx)
            {
            case 0:
                return mX;
            case 1:
                return mY;
            case 2:
                return mZ;
            case 3:
                return mW;
            }
            
            ASSERT( false ); // Invalid index specified
            throw std::out_of_range( "Quaternion index out of range. 0 <= index < 4" );
        }

        T&               operator[]( int idx )
        {
            switch ( idx )
            {
            case 0:
                return mX;
            case 1:
                return mY;
            case 2:
                return mZ;
            case 3:
                return mW;
            }

            ASSERT( false ); // Invalid index specified
            throw std::out_of_range( "Quaternion index out of range. 0 <= index < 4" );
        }
        
        /// <summary>Access to the data array.</summary>
        const T*        Data() const { return &mX; }

        // assignment operators
        cQuaternion&    operator+=( const cQuaternion& q )
        {
            mX += q.mX;
            mY += q.mY;
            mZ += q.mZ;
            mW += q.mW;

            if( AutoNormalize )
            {
                Normalize();
            }
            return *this;
        }

        cQuaternion&    operator-=( const cQuaternion& q )
        {
            mX -= q.mX;
            mY -= q.mY;
            mZ -= q.mZ;
            mW -= q.mW;

            if( AutoNormalize )
            {
                Normalize();
            }
            return *this;
        }

        cQuaternion&    operator*=( const cQuaternion& q )
        {
            *this = operator*( q );

            if( AutoNormalize )
            {
                Normalize();
            }
            return *this;
        }

        cQuaternion&    operator*=( T f )
        {
            if( !AutoNormalize ) // Don't try to scale a quaternion that is meant to be unit length
            {
                mX *= f;
                mY *= f;
                mZ *= f;
                mW *= f;
            }
            return *this;
        }

        cQuaternion&    operator/=( T f )
        {
            if( !AutoNormalize ) // Don't try to scale a quaternion that is meant to be unit length
            {
                T fInv = ((T)1.0) / f;
                mX *= fInv;
                mY *= fInv;
                mZ *= fInv;
                mW *= fInv;
            }
            return *this;
        }

        // unary operators
        cQuaternion     operator+() const
        {
            return *this;
        }

        cQuaternion     operator-() const
        {
            return cQuaternion(-mX, -mY, -mZ, -mW);
        }

        // binary operators
        cQuaternion     operator+( const cQuaternion& q ) const
        {
            return cQuaternion(mX + q.mX, mY + q.mY, mZ + q.mZ, mW + q.mW);
        }

        cQuaternion     operator-( const cQuaternion& q ) const
        {
            return cQuaternion(mX - q.mX, mY - q.mY, mZ - q.mZ, mW - q.mW);
        }

        cQuaternion     operator*( const cQuaternion& q ) const
        {
#if 1
            // Left to right quaternion multiplication
            // q = q1 * q2 * q3 
            // multiplies q1 and q2 first then q3

            return cQuaternion(
                mW * q.mX + mX * q.mW + mZ * q.mY - mY * q.mZ,
                mW * q.mY + mY * q.mW + mX * q.mZ - mZ * q.mX,
                mW * q.mZ + mZ * q.mW + mY * q.mX - mX * q.mY,
                mW * q.mW - mX * q.mX - mY * q.mY - mZ * q.mZ );
#else
            // Right to left quaternion multiplication
            // q = q1 * q2 * q3 
            // multiplies q3 and q2 first then q1
            
            return cQuaternion(
                mW * q.mX + mX * q.mW + mY * q.mZ - mZ * q.mY,
                mW * q.mY - mX * q.mZ + mY * q.mW + mZ * q.mX,
                mW * q.mZ + mX * q.mY - mY * q.mX + mZ * q.mW,
                mW * q.mW - mX * q.mX - mY * q.mY - mZ * q.mZ );
#endif
        }

        cQuaternion     operator*( T f ) const
        {
            if( AutoNormalize ) // Don't try to scale a quaternion that is meant to be unit length
            {
                return *this;
            }
            else
            {
                return cQuaternion(mX * f, mY * f, mZ * f, mW * f);
            }
        }

        cQuaternion     operator/( T f ) const
        {
            if( AutoNormalize ) // Don't try to scale a quaternion that is meant to be unit length
            {
                return *this;
            }
            else
            {
                T fInv = ((T)1.0) / f;
                return cQuaternion(mX * fInv, mY * fInv, mZ * fInv, mW * fInv);
            }
        }

        bool            operator==( const cQuaternion& q ) const
        {
            return ( mX == q.mX && mY == q.mY && mZ == q.mZ && mW == q.mW );
        }

        bool            operator!=( const cQuaternion& q ) const
        {
            return ( mX != q.mX || mY != q.mY || mZ != q.mZ || mW != q.mW );
        }

        /// <summary>Less-than comparitor, used mostly for sorting.</summary>
        bool            operator<( const cQuaternion& q ) const
        {
            return ( mX < q.mX && mY < q.mY && mZ < q.mZ && mW < q.mW );
        }

        /// <summary>Compare two quaternions to within a tolerance.</summary>
        bool            Equals( const cQuaternion& q, T tolerance ) const
        {
            if( fabs( mW - q.mW ) <= tolerance )
            {
                return ( fabs( mX - q.mX ) <= tolerance && fabs( mY - q.mY ) <= tolerance
                    && fabs( mZ - q.mZ ) <= tolerance );
            }
            // May be equivalent by having all opposite signs.
            return ( fabs( mX + q.mX ) <= tolerance && fabs( mY + q.mY ) <= tolerance
                && fabs( mZ + q.mZ ) <= tolerance && fabs( mW + q.mW ) <= tolerance );
        }

        /// <summary>Returns the dot product with another quaternion.</summary>
        inline T        Dot( const cQuaternion &q ) const
        {
            return (mX*q.mX + mY*q.mY + mZ*q.mZ + mW*q.mW);
        }

        /// <summary>When called on a quaternion object q, returns the norm of q.</summary>
        inline T        Norm() const
        {
            return (mX*mX + mY*mY + mZ*mZ + mW*mW);
        }

        /// <summary>When called on a quaternion object q, returns the magnitude q.</summary>
        inline T        Magnitude() const
        {
            return sqrt( Norm() );
        }

        /// <summary>When called on a quaternion object q, returns the unit quaternion.</summary>
        inline cQuaternion Normalized() const
        {
            if( AutoNormalize )
            {
                return *this;
            }
            else
            {
                return Scaled(1/Magnitude());
            }
        }

        /// <summary>Normalize to a unit quaternion.</summary>
        inline void     Normalize()
        {
            T       mag = Magnitude();

            if( mag > 0 && mag != 1 )
            {
                InternalScale( 1 / mag );
            }
        }

        /// <summary>When called on a quaternion object q, returns the conjugate.</summary>
        inline cQuaternion Conjugate() const
        {
            return cQuaternion( -mX, -mY, -mZ, mW, true );
        }

        /// <summary>When called on a quaternion object q, returns the inverse.</summary>
        inline cQuaternion Inverse() const
        {
            cQuaternion result( *this );

            result.Invert();

            return result;
        }

        /// <summary>Invert the quaternion in place.</summary>
        inline void     Invert()
        {
            *this = Conjugate();
        }

        /// <summary>When called on a quaternion object q, returns a quaternion scaled by s.</summary>
        inline cQuaternion Scaled( T s ) const
        {
            if( AutoNormalize )
            {
                return *this; // Can't scale a unit quaternion
            }
            else
            {
                return cQuaternion(mX*s, mY*s, mZ*s, mW*s);
            }
        }
        
        /// <summary>Scales the quaternion in place.</summary>
        inline void     Scale( T s )
        {
            if( !AutoNormalize ) // Can't scale a unit quaternion
            {
                mX *= s;
                mY *= s;
                mZ *= s;
                mW *= s;
            }
        }

        /// <summary>Rotate the given vector through the rotation of this quaternion.</summary>
        inline cVector3<T> Rotate( const cVector3<T> &vec ) const
        {
            cQuaternion<T,false> result( *this );
            cQuaternion<T,false> vecQuat( vec.X(), vec.Y(), vec.Z(), T( 0.0 ) );

            result.Invert();
            result *= vecQuat;
            result *= *this;

            return result.RealPart();
        }

        /// <summary>Calculate the angle (in radians) between this quaternion and the one given</summary>
        T               AngleTo( const cQuaternion& q ) const
        {
            T       innerProduct = Dot( q );
            T       val = innerProduct * innerProduct * 2 - 1;
            if( val > T( 1.0 ) )
            {
                val = T( 1.0 );
            }
            else if( val < T( -1.0 ) )
            {
                val = T( -1.0 );
            }
            return (T) acos( val );
        }

        /// <summary>
        /// Calculate a rough parametric distance between two quaternions. When the angle between the two
        /// quaternions is zero, the return value will be zero. When it is 180 degrees, the return value will be one.
        /// </summary>
        T               ParametricDistance( const cQuaternion& q ) const
        {
            T       innerProduct = Norm();

            return ( 1 - innerProduct * innerProduct );
        }

        /// <summary>
        /// Returns a linear interpolated quaternion some percentage 't' between two quaternions.
        /// The parameter t is usually in the range [0,1], but this method can also be used to extrapolate
        /// rotations beyond that range.
        /// </summary>
        inline static cQuaternion Lerp( const cQuaternion& q1, const cQuaternion& q2, T t )
        {
            // In order for slerp to behave correctly, the input quaternions need to be unit length.
            // The caller needs to ensure that.

            T cosOmega = q1.Dot( q2 );
            T mult = 1;

            if( cosOmega < 0 )
            {
                // Input quaternions are on the opposite side of the hypersphere from each other.
                // Negate one to ensure they interpolate along the shortest path.
                mult = -1;
            }

            T k1 = 1 - t;
            T k2 = t;

            T x = k1 * q1.mX + k2 * q2.mX * mult;
            T y = k1 * q1.mY + k2 * q2.mY * mult;
            T z = k1 * q1.mZ + k2 * q2.mZ * mult;
            T w = k1 * q1.mW + k2 * q2.mW * mult;

            cQuaternion result( x, y, z, w, false );
            result.Normalize();

            return result;
        }
        inline cQuaternion Lerp( const cQuaternion &q, T t )
        {
            return Lerp( *this, q, t );
        }

        /// <summary>
        /// Returns a spherical linear interpolated quaternion some percentage 't' between two quaternions.
        /// The parameter t is usually in the range [0,1], but this method can also be used to extrapolate
        /// rotations beyond that range.
        /// </summary>
        inline static cQuaternion Slerp( const cQuaternion &q1, const cQuaternion &q2, T t )
        {
            // In order for slerp to behave correctly, the input quaternions need to be unit length.
            // The caller needs to ensure that.

            T cosOmega = q1.Dot( q2 );
            T mult = 1;

            if( cosOmega < 0 )
            {
                // Input quaternions are on the opposite side of the hypersphere from each other.
                // Negate one to ensure they interpolate along the shortest path.
                mult = -1;
                cosOmega = -cosOmega;
            }

            T k1;
            T k2;

            // Lerp and slerp will differ at machine precision level up to very near 1 for cosOmega.
            if( cosOmega > T( 0.999999 ) )
            {
                // Just use simple lerp to simplify calculations when the angle is very small, and to
                // avoid divide by zero in the slerp calcs.
                k1 = 1 - t;
                k2 = t;
            }
            else
            {
                T sinOmega = sqrt( 1 - cosOmega * cosOmega );
                T omega = atan2( sinOmega, cosOmega );
                T fDiv = 1 / omega; // We ensured omega would not be zero by checking if cosOmega is near 1
                k1 = sin( (1-t) * omega ) * fDiv;
                k2 = sin( t*omega ) * fDiv;
            }

            T x = k1 * q1.mX + k2 * q2.mX * mult;
            T y = k1 * q1.mY + k2 * q2.mY * mult;
            T z = k1 * q1.mZ + k2 * q2.mZ * mult;
            T w = k1 * q1.mW + k2 * q2.mW * mult;

            cQuaternion result( x, y, z, w, false );
            result.Normalize();

            return result;
        }
        inline cQuaternion Slerp( const cQuaternion &q, T t )
        {
            return Slerp( *this, q, t );
        }

        /// <summary>Create a quaternion from 2 vectors./summary>
        inline static cQuaternion FromVectors( cVector3<T> vec1, cVector3<T> vec2 )
        {
            cVector3<T> w = vec1.Cross(vec2);
            cQuaternion q(1+vec1.Dot(vec2), w.X(), w.Y(), w.Z());
            return q.Normalized();
        }

        /// <summary>Create quaternion from 2 orthogonal vectors</summary>
        inline static cQuaternion FromOrthogonalVectors( cVector3<T> vec1, cVector3<T> vec2 )
        {
            cVector3<T> vec3;
            
            vec3 = vec1.Cross(vec2).Normalized();
            vec2 = vec1.Cross(vec3).Normalized();
            vec1.Normalize();

            //== vec1, vec2, and vec3 are orthogonal vectors ==--
            // TODO : Check for degenerate cases (i.e. when vec1 and vec2 are colinear)

            T m[9] = { vec1.X(), vec2.X(), vec3.X(),
                       vec1.Y(), vec2.Y(), vec3.Y(),
                       vec1.Z(), vec2.Z(), vec3.Z() };

            cQuaternion q;

            T trace = (T) (m[0] + m[4] + m[8]); 
            if( trace > 0 )
            {
                T s = ((T)0.5) / sqrt(trace+ 1);

                q.mW = ((T)0.25) / s;
                q.mX = ( m[7] - m[5] ) * s;
                q.mY = ( m[2] - m[6] ) * s;
                q.mZ = ( m[3] - m[1] ) * s;
            }
            else
            {
                if ( m[0] > m[4] && m[0] > m[8] ) 
                {
                    T s = ((T)2.0) * sqrt( ((T)1.0) + m[0] - m[4] - m[8]);
                    q.mW = (m[7] - m[5] ) / s;
                    q.mX = 0.25f * s;
                    q.mY = (m[1] + m[3] ) / s;
                    q.mZ = (m[2] + m[6] ) / s;
                } 
                else 
                {
                    if (m[4] > m[8]) 
                    {
                        T s = ((T)2.0) * sqrt( ((T)1.0) + m[4] - m[0] - m[8]);
                        q.mW = (m[2] - m[6] ) / s;
                        q.mX = (m[1] + m[3] ) / s;
                        q.mY = ((T)0.25) * s;
                        q.mZ = (m[5] + m[7] ) / s;
                    } 
                    else
                    {
                        T s = ((T)2.0) * sqrt( ((T)1.0) + m[8] - m[0] - m[4] );
                        q.mW = (m[3] - m[1] ) / s;
                        q.mX = (m[2] + m[6] ) / s;
                        q.mY = (m[5] + m[7] ) / s;
                        q.mZ = ((T)0.25) * s;
                    }
                }
            }
            return q;
        }

        /// <summary>Create quaternion that rotates a vector1 to vector2</summary>
        inline void VectorToVector( cVector3<T> vec1, cVector3<T> vec2 )
        {
            cVector3<T> v1( vec1 ), v2( vec2 ), vHalf, vCross;

            v1.Normalize();
            v2.Normalize();
            vHalf = v1 + v2;

            if( vHalf.LengthSquared() < 0.00001f )
            {
                // v1 & v2 are pointing opposite from each other, use any orthogonal vector to v1
                T x = std::abs( v1.X() );
                T y = std::abs( v1.Y() );
                T z = std::abs( v1.Z() );

                if( x > y && x > z )
                    vHalf.SetValues( v1.Y(),-v1.X(), v1.Z() );
                else if( y > z && y > x )
                    vHalf.SetValues( v1.X(), v1.Z(),-v1.Y() );
                else
                    vHalf.SetValues(-v1.Z(), v1.Y(), v1.X() );
            }

            vHalf.Normalize();
            vCross = vHalf.Cross( v2 );

            T w = vHalf.Dot( v2 );
            T x = vCross.X();
            T y = vCross.Y();
            T z = vCross.Z();

            if( w >= 1 )
                w = 1;
            else if( w <= -1 )
                w = -1;

            if( x >= 1 )
                x = 1;
            else if( x <= -1 )
                x = -1;

            if( y >= 1 )
                y = 1;
            else if( y <= -1 )
                y = -1;

            if( z >= 1 )
                z = 1;
            else if( z <= -1 )
                z = -1;

            mX = x;
            mY = y;
            mZ = z;
            mW = w;
        }

        /// <summary>Set Axis Angle. Angle is encoded in vector's length</summary>
        void            SetAxisAngle( const cVector3<T>& aa )
        {
            T radians = aa.Length();
            
            if( fabs( radians ) > FLT_EPSILON )
            {
                mW = cos( radians / T( 2.0 ) );

                // Divide the length out.
                cVector3<T> v = aa / radians * sin( radians / T( 2.0 ) );

                mX = v.X();
                mY = v.Y();
                mZ = v.Z();
            }
            else
            {
                mX = mY = mZ = T( 0.0 );
                mW = T( 1.0 );
            }
        }

        /// <summary>Set Axis Angle by specifying separate axis and angle. Axis may be un-normalized.</summary>
        void            SetAxisAngle( const cVector3<T>& axis, T angle )
        {
            if( fabs( angle ) > FLT_EPSILON )
            {
                cVector3<T> v( axis.Normalized() );

                mW = cos( angle / T( 2.0 ) );

                v *= sin( angle / T( 2.0 ) );

                mX = v.X();
                mY = v.Y();
                mZ = v.Z();
            }
            else
            {
                mX = mY = mZ = T( 0.0 );
                mW = T( 1.0 );
            }
        }

        /// <summary>Get Axis Angle. Angle is encoded in vector's length</summary>
        inline cVector3<T> AxisAngle() const
        {
            T   x = mX, y = mY, z = mZ, w = mW;

            if( mW < 0 )
            {
                x = -x;
                y = -y;
                z = -z;
                w = -w;
            }
            
            cVector3<T> v( x, y, z );
            T radians = T( 2.0 ) * acos( w ) ;

            v.Normalize();
            v *= radians;
            
            return v;
        }

        //====================================================================================
        // Type conversion helpers
        //====================================================================================

        inline Core::cQuaternion<float, AutoNormalize> ToFloat() const
        {
            return Core::cQuaternion<float, AutoNormalize>( (float) X(), (float) Y(), (float) Z(), (float) W() );
        }

        inline Core::cQuaternion<double, AutoNormalize> ToDouble() const
        {
            return Core::cQuaternion<double, AutoNormalize>( (double) X(), (double) Y(), (double) Z(), (double) W() );
        }

        //== everything below this line is subject to coordinate system handedness and is work in progress ==--

        /// <summary>Initialize quaternion to a rotation in X.</summary>
        template<typename U> 
        inline void SetRotationX( U angle )
        {
            T f = (T) (angle / 2.0);
            T s = sin(f);
            mW = cos(f);
            mX = s;
            mY = 0;
            mZ = 0;
        }

        /// <summary>Initialize quaternion to a rotation in Y.</summary>
        template<typename U> 
        inline void SetRotationY( U angle )
        {
            T f = (T) (angle / 2.0);
            T s = sin(f);
            mW = cos(f);
            mX = 0;
            mY = s;
            mZ = 0;
        }

        /// <summary>Initialize quaternion to a rotation in Z.</summary>
        template<typename U> 
        inline void SetRotationZ( U angle )
        {
            T f = (T) ( angle / 2.0 );
            T s = sin(f);
            mW = cos(f);
            mX = 0;
            mY = 0;
            mZ = (T) s;
        }
        
        /// <summary>Create quaternion orientation from a 3x3 rotation matrix array.</summary>
        template<typename U>
        void FromOrientationMatrix( const U *m )
        {
            cQuaternion &q = *this;

            T trace = (T) (m[0] + m[4] + m[8]); 
            if( trace > 0 )
            {
                T s = ((T)0.5) / sqrt(trace+ 1);

                q.mW = (T) 0.25 / s;
                q.mX = (T) ( m[7] - m[5] ) * s;
                q.mY = (T) ( m[2] - m[6] ) * s;
                q.mZ = (T) ( m[3] - m[1] ) * s;
            }
            else
            {
                if ( m[0] > m[4] && m[0] > m[8] ) 
                {
                    T s = (T) ( 2.0 * sqrt( 1.0 + m[0] - m[4] - m[8] ) );
                    q.mW = (T) ( m[7] - m[5] ) / s;
                    q.mX = (T) 0.25 * s;
                    q.mY = (T) ( m[1] + m[3] ) / s;
                    q.mZ = (T) ( m[2] + m[6] ) / s;
                } 
                else 
                {
                    if (m[4] > m[8]) 
                    {
                        T s = (T) ( 2.0 * sqrt( 1.0 + m[4] - m[0] - m[8] ) );
                        q.mW = (T) ( m[2] - m[6] ) / s;
                        q.mX = (T) ( m[1] + m[3] ) / s;
                        q.mY = (T) 0.25 * s;
                        q.mZ = (T) ( m[5] + m[7] ) / s;
                    } 
                    else
                    {
                        T s = (T) ( 2.0 * sqrt( 1.0 + m[8] - m[0] - m[4] ) );
                        q.mW = (T) ( m[3] - m[1] ) / s;
                        q.mX = (T) ( m[2] + m[6] ) / s;
                        q.mY = (T) ( m[5] + m[7] ) / s;
                        q.mZ = (T) 0.25 * s;
                    }
                }
            }
        }
           
        /// <summary>Output quaternion to a 3x3 rotation matrix array.</summary>
        template<typename U> 
        void ToOrientationMatrix( U* m ) const
        {
            const cQuaternion<T,AutoNormalize> &q = *this;

            T sqw = (T)(q.mW*q.mW);
            T sqx = (T)(q.mX*q.mX);
            T sqy = (T)(q.mY*q.mY);
            T sqz = (T)(q.mZ*q.mZ);

            T invs = 1 / (sqx + sqy + sqz + sqw);

            m[0] = (( sqx - sqy - sqz + sqw)*invs);
            m[4] = ((-sqx + sqy - sqz + sqw)*invs);
            m[8] = ((-sqx - sqy + sqz + sqw)*invs);

            T tmp1 = (T)(q.mX*q.mY);
            T tmp2 = (T)(q.mZ*q.mW);

            m[3] = (2 * (tmp1 + tmp2)*invs);
            m[1] = (2 * (tmp1 - tmp2)*invs);

            tmp1 = (T)(q.mX*q.mZ);
            tmp2 = (T)(q.mY*q.mW);
            m[6] = (2 * (tmp1 - tmp2)*invs);
            m[2] = (2 * (tmp1 + tmp2)*invs);
            tmp1 = (T)(q.mY*q.mZ);
            tmp2 = (T)(q.mX*q.mW);
            m[7] = (2 * (tmp1 + tmp2)*invs);
            m[5] = (2 * (tmp1 - tmp2)*invs); 
        }

        /// <summary>Create quaternion orientation from a matrix.</summary>
        void FromMatrix( const cMatrix4<T>& m )
        {
            T rot[9];
            rot[0] = m.Value( 0, 0 );
            rot[1] = m.Value( 1, 0 );
            rot[2] = m.Value( 2, 0 );
            rot[3] = m.Value( 0, 1 );
            rot[4] = m.Value( 1, 1 );
            rot[5] = m.Value( 2, 1 );
            rot[6] = m.Value( 0, 2 );
            rot[7] = m.Value( 1, 2 );
            rot[8] = m.Value( 2, 2 );

            FromOrientationMatrix( rot );
        }

        //== Handy constants ==--

        /// <summary>Returns an identity quaternion. This typically means 'no rotation'.</summary>
        static const cQuaternion kIdentity;

    private:        
        //== member variables ==--

        T       mX;
        T       mY;
        T       mZ;
        T       mW;

        // An alternative constructor that does not pay attention to AutoNormalize
        cQuaternion( T x, T y, T z, T w, bool junk )
        {
            mX = x;
            mY = y;
            mZ = z;
            mW = w;
        }

        // Internal scaling method that does not respect AutoNormalize
        inline void     InternalScale( T s )
        {
            mX *= s;
            mY *= s;
            mZ *= s;
            mW *= s;
        }
    };

    template <typename T, bool AutoNormalize>
    const cQuaternion<T,AutoNormalize> cQuaternion<T,AutoNormalize>::kIdentity( 0, 0, 0, 1 );


    //=========================================================================
    // Stream I/O operators
    //=========================================================================
    template<typename T, bool AutoNormalize>
    std::wostream& operator<<( std::wostream& os, const cQuaternion<T, AutoNormalize>& q )
    {
        os << L"(" << q.X() << L"," << q.Y() << L"," << q.Z() << L"," << q.W() << L")";

        return os;
    }

    template<typename T, bool AutoNormalize>
    std::ostream& operator<<( std::ostream& os, const cQuaternion<T, AutoNormalize>& q )
    {
        T x = q.X();
        T y = q.Y();
        T z = q.Z();
        T w = q.W();
        os.write( (char *)&x, sizeof( T ) );
        os.write( (char *)&y, sizeof( T ) );
        os.write( (char *)&z, sizeof( T ) );
        os.write( (char *)&w, sizeof( T ) );
        return os;
    }

    template<typename T, bool AutoNormalize>
    std::istream& operator>>( std::istream& is, cQuaternion<T, AutoNormalize>& v )
    {
        T x, y, z, w;
        is.read( (char *)&x, sizeof( T ) );
        is.read( (char *)&y, sizeof( T ) );
        is.read( (char *)&z, sizeof( T ) );
        is.read( (char *)&w, sizeof( T ) );
        v.SetValues( x, y, z, w );
        return is;
    }

    //== class short-hand definitions ==--

    typedef cQuaternion<float,false>  cQuaternionf;
    typedef cQuaternion<double,false> cQuaterniond;
    typedef cQuaternion<float,true>  cRotationf;
    typedef cQuaternion<double,true> cRotationd;
}


