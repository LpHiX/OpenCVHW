//======================================================================================================
// Copyright 2017-2018 NaturalPoint Inc.
// Written by Colin Davidson
//======================================================================================================
#pragma once

#include <vector>
#include <cmath>
#include <map>
#include <set>
#include <algorithm>
#include <iterator> // std::back_inserter
#include "Core/DebugSystem.h"
#include "Core/Label.h"
#include "Core/UID.h"
#include "Core/Vector3.h"
#include "Core/Vector2.h"
#include <Eigen/Dense>

// Avoid Windows' definition of max, which overrides std::max
#undef max

namespace Core
{
	template<class T> struct sIndexDataPair
	{
		sIndexDataPair() : mIndex(-1) {}
		sIndexDataPair(int index, T data) : mIndex(index), mData(data) {}
		bool operator<(const sIndexDataPair &that) const { return mData < that.mData || (!(that.mData < mData) && mIndex < that.mIndex); }

		int mIndex;
		T   mData;
	};
	typedef sIndexDataPair<float> sIndexFloat;

	// TODO consider moving this to BinaryStreamReader, change R to cIReader
	template<class R, class VT> static void ReadVector(R &reader, std::vector<VT> &v)
	{
		int tmp = reader.ReadInt();
		ASSERT(tmp == sizeof(VT));
		v.resize(reader.ReadLongLong());
		if (v.size())
		{
			reader.ReadData(reinterpret_cast<unsigned char*>(v.data()), (tmp * v.size()));
		}
	}

	// TODO consider moving this to BinaryStreamWriter, change W to cIWriter
	template<class W, class VT> static void WriteVector(W &writer, const std::vector<VT> &v)
	{
		int tmp = sizeof(VT);
		writer.WriteInt(tmp);
		writer.WriteLongLong(v.size());
		if (v.size())
		{
			writer.WriteData(reinterpret_cast<const unsigned char *>(v.data()), (tmp * v.size()));
		}
	}

	/// <summary> cVec behaves like a std::vector but does not own the data (and never makes copies). </summary>
	template<class T> class cVec
	{
	public:
		typedef typename const T* const_iterator;
		typedef typename T* iterator;
		cVec(const std::vector<T> &v) : mBegin(const_cast<iterator>(v.data())), mEnd(mBegin + v.size()) {}
		cVec() : mBegin(nullptr), mEnd(nullptr) {}
		cVec(iterator begin, iterator end) : mBegin(begin), mEnd(end) {}
		iterator begin() { return mBegin; }
		const_iterator begin() const { return mBegin; }
		iterator end() { return mEnd; }
		const_iterator end() const { return mEnd; }
		T& operator[](int i) { return mBegin[i];  }
		const T& operator[](int i) const { return mBegin[i]; }
		int size() const { return int(mEnd - mBegin); }
		T& back() { return mEnd[-1]; }
		const T& back() const { return mEnd[-1]; }
		void pop_back() { --mEnd; }
		//void sort() { std::sort(mBegin, mEnd); }
		//void swap_del(iterator p) { std::swap(*p, *(--mEnd)); }
		std::vector<T>& CopyTo(std::vector<T> &v) const { v = std::vector<T>(mBegin, mEnd); return v;  }
	private:
		iterator mBegin, mEnd;
	};

	/// <summary>cMatrix is a useful vector of vectors. </summary>
	template<class T> class cMatrix
	{
		template<class U> friend class cUIDMatrix;
		template<class U> friend class cMatrix;
		template<class U> friend class cMat;
	public:
		// these methods are meant to behave similarly to the stl
		cMatrix() { mRowOffsets.push_back(0); }
		void reserve(size_t s) { mRowOffsets.reserve(s+1); mData.reserve(s); }
		void clear() { mRowOffsets.clear(); mData.clear(); mRowOffsets.push_back(0); }
		int size() const { return (int)mRowOffsets.size()-1; }
		// these helpers allow code using cMatrix<T> to look just like std::vector<std::vector<T> >
		cVec<T> operator[](int i) { return cVec<T>(mData.data() + mRowOffsets[i], mData.data() + mRowOffsets[i+1]); }
		const cVec<const T> operator[](int i) const { return cVec<const T>(mData.data() + mRowOffsets[i], mData.data() + mRowOffsets[i+1]); }
		bool IsEmptyRow(int i) const { return mRowOffsets[i] == mRowOffsets[i+1]; }
		cMatrix<T>& CopyTo(cMatrix<T> &m) const
		{
			m.mRowOffsets = mRowOffsets;
			m.mData = mData;
			return m;
		}

		void AddRowItem(const T &v) { mData.push_back(v); }
		void EndRow() { mRowOffsets.push_back((int)mData.size()); }
		const std::vector<T> &Data() const { return mData; } // the matrix as a flat vector
		std::vector<T> &Data() { return mData; } // the matrix as a flat vector
		void SortColumns()
		{
			for (int i = 0; i != size(); ++i)
				std::sort(mData.begin() + mRowOffsets[i], mData.begin() + mRowOffsets[i+1]);
		}
		template<class U>
		void SetShape(const cMatrix<U> &m, const T &d = T())
		{
			mRowOffsets = m.mRowOffsets;
			mData.resize(m.mData.size(), d);
		}

		template<class U>
		void SetShape(const cMat<U> &m, const T &d = T())
		{
			m.mRowOffsets.CopyTo(mRowOffsets);
			mData.resize(m.mData.size(), d);
		}

		void SetShape(int rows, int cols, const T &d = T())
		{
			mData.resize(rows * cols, d);
			mRowOffsets.resize(rows+1);
			for (int r = 0; r <= rows; ++r)
				mRowOffsets[r] = r*cols;
		}

		///<summary>This method depends on the type having a mIndex member (such as sIndexDataPair)</summary>
		int NumCols() const
		{
			int count = -1;
			for (const T &mit : mData) count = std::max(count, mit.mIndex);
			return count+1;
		}

		template<class W> void Save(W &writer) const
		{
			writer.WriteByte(4); // revision
			WriteVector(writer, mRowOffsets);
			WriteVector(writer, mData);
		}

		template<class R> bool Load(R &reader, int *_revision=nullptr)
		{
			unsigned char revision = reader.ReadByte();
			if (_revision != nullptr) *_revision = revision;
			if (revision < 4) return false; // from the past: ignore
			if (revision > 4) return false; // from the future; we don't know how to decode it
			if (revision == 4)
			{
				ReadVector(reader, mRowOffsets);
				ReadVector(reader, mData);
			}
			else return false;
			return true;
		}

	private:
		std::vector<int>      mRowOffsets;
		std::vector<T>        mData;
	};

	/// <summary> cMat behaves like a cMatrix, but does not own the data (and never makes copies). </summary>
	template<class T> class cMat
	{
		template<class U> friend class cMatrix;
	public:
		cMat() : mSize(0) {}
		cMat(const cMatrix<T> &mat) : mSize(mat.size()), mRowOffsets(mat.mRowOffsets), mData(mat.mData) {}
		int size() const { return mSize; }
		cVec<T> operator[](int i) { return cVec<T>(mData.begin() + mRowOffsets[i], mData.begin() + mRowOffsets[i+1]); }
		const cVec<const T> operator[](int i) const { return cVec<const T>(mData.begin() + mRowOffsets[i], mData.begin() + mRowOffsets[i+1]); }
		cMatrix<T>& CopyTo(cMatrix<T> &m) const
		{
			mRowOffsets.CopyTo(m.mRowOffsets);
			mData.CopyTo(m.mData);
			return m;
		}
		const cVec<T> &Data() const { return mData; } // the matrix as a flat vector
		int NumCols() const
		{
			int count = -1;
			for (const T &mit : mData) count = std::max(count, mit.mIndex);
			return count + 1;
		}
		bool IsEmptyRow(int i) const { return mRowOffsets[i] == mRowOffsets[i+1]; }
	private:
		int            mSize;
		cVec<int>      mRowOffsets;
		cVec<T>        mData;
	};

	/// <summary> cUIDMatrix is a cMatrix that additionally supports assiging a UID to each index and converting 
	/// back-and-forth between index and UID. it also supports labels. </summmary>
	template<class T>
	class cUIDMatrix : public cMatrix<T>
	{
	public:
		bool HasUID(cUID u)
		{
			return mUID2Idx.count(u) != 0;
		}
		void AddUID(cUID u)
		{
			mUID2Idx[u] = (int)mIdx2UID.size();
			mIdx2UID.push_back(u);
			mIdx2Label.push_back(cLabel::kInvalid);
		}
		void SetLabel(int index, cLabel label)
		{
			mIdx2Label[index] = label;
		}
		void SetLabel(cLabel label)
		{
			mIdx2Label.back() = label;
		}
		void AddRowVector(cUID u, const std::vector<T> &v)
		{
			AddUID(u);
			mData.insert(mData.end(), v.begin(), v.end());
			EndRow();
		}
		void AddFixedRow(cUID u, int n, const T &v)
		{
			AddUID(u);
			mData.insert(mData.end(), n, v);
			EndRow();
		}
		int Uid2Index(const cUID &u) const { return mUID2Idx.at(u); }
		const cUID & Index2Uid(int i) const { return mIdx2UID[i]; }
		const cLabel & Index2Label(int i) const { return mIdx2Label[i]; }

		template<class W> void Save(W &writer) const
		{
			cMatrix<T>::Save(writer);
			WriteVector(writer, mIdx2UID);
			WriteVector(writer, mIdx2Label);
		}

		template<class R> bool Load(R &reader)
		{
			int revision;
			bool ok = cMatrix<T>::Load(reader, &revision);
			if (!ok) return false;
			if (revision < 4) return false; // from the past: ignore
			if (revision > 4) return false; // from the future
			if (revision == 4)
			{
				ReadVector(reader, mIdx2UID);
				ReadVector(reader, mIdx2Label);
			}
			else return false;
			mUID2Idx.clear();
			for (int i = 0; i != mIdx2UID.size(); ++i) mUID2Idx[mIdx2UID[i]] = i;
			return true;
		}

		const std::vector<cUID> &GetIdx2UID() const { return mIdx2UID; }
		const std::vector<cLabel> &GetIdx2Label() const { return mIdx2Label; }

	private:
		std::vector<cUID>	  mIdx2UID;
		std::vector<cLabel>	  mIdx2Label;
		std::map<cUID, int>   mUID2Idx;
	};

	enum eChannelType
	{
		I = -6, LTZ = -5, LTY = -4, LTX = -3, LT = -2, L = -1,
		tx = 0, ty = 1, tz = 2, rx = 3, ry = 4, rz = 5
	};

	///<summary> cRT is a cMatrix34 type that only represents rotations and translations (no scale or skew). </summary>
	template<class T> class cRT
	{
		template <class T> friend class cP;
		template <class T> friend class cCameraSolver;
		template <class T> friend class cPointSolver;
	public:
		cRT(T m00 = 1, T m01 = 0, T m02 = 0, T m03 = 0, 
			T m10 = 0, T m11 = 1, T m12 = 0, T m13 = 0, 
			T m20 = 0, T m21 = 0, T m22 = 1, T m23 = 0)
		{
			m0[0] = m00; m0[1] = m01; m0[2] = m02; m0[3] = m03;
			m1[0] = m10; m1[1] = m11; m1[2] = m12; m1[3] = m13;
			m2[0] = m20; m2[1] = m21; m2[2] = m22; m2[3] = m23;
		}

		cRT(const cRT &m)
		{
			m0[0] = m.m0[0]; m0[1] = m.m0[1]; m0[2] = m.m0[2]; m0[3] = m.m0[3];
			m1[0] = m.m1[0]; m1[1] = m.m1[1]; m1[2] = m.m1[2]; m1[3] = m.m1[3];
			m2[0] = m.m2[0]; m2[1] = m.m2[1]; m2[2] = m.m2[2]; m2[3] = m.m2[3];
		}

		cRT& operator=(const cRT &m)
		{
			m0[0] = m.m0[0]; m0[1] = m.m0[1]; m0[2] = m.m0[2]; m0[3] = m.m0[3];
			m1[0] = m.m1[0]; m1[1] = m.m1[1]; m1[2] = m.m1[2]; m1[3] = m.m1[3];
			m2[0] = m.m2[0]; m2[1] = m.m2[1]; m2[2] = m.m2[2]; m2[3] = m.m2[3];
			return *this;
		}

		void Init(const double *m)
		{
			m0[0] = (T)m[0]; m0[1] = (T)m[1]; m0[2] = (T)m[2]; m0[3] = (T)m[3];
			m1[0] = (T)m[4]; m1[1] = (T)m[5]; m1[2] = (T)m[6]; m1[3] = (T)m[7];
			m2[0] = (T)m[8]; m2[1] = (T)m[9]; m2[2] = (T)m[10]; m2[3] = (T)m[11];
		}

		// Translate = multiply on right by [[1 0 0 tx],[0 1 0 ty],[0 0 1 tz],[0 0 0 1]]
		cRT<T>& Translate(const T tx, const T ty, const T tz)
		{
			m0[3] += m0[0] * tx + m0[1] * ty + m0[2] * tz;
			m1[3] += m1[0] * tx + m1[1] * ty + m1[2] * tz;
			m2[3] += m2[0] * tx + m2[1] * ty + m2[2] * tz;
			return *this;
		}

		cRT<T>& RotateZXY(const T rz, const T rx, const T ry)
		{
			return RotateY(ry).RotateX(rx).RotateZ(rz); // pan-tilt-roll
		}

		// TranslateX = multiply on right by [[1 0 0 tx],[0 1 0 0],[0 0 1 0],[0 0 0 1]]
		cRT<T>& TranslateX(const T tx)
		{
			m0[3] += m0[0] * tx;
			m1[3] += m1[0] * tx;
			m2[3] += m2[0] * tx;
			return *this;
		}
		// TranslateY = multiply on right by [[1 0 0 0],[0 1 0 ty],[0 0 1 0],[0 0 0 1]]
		cRT<T>& TranslateY(const T ty)
		{
			m0[3] += m0[1] * ty;
			m1[3] += m1[1] * ty;
			m2[3] += m2[1] * ty;
			return *this;
		}
		// TranslateZ = multiply on right by [[1 0 0 0],[0 1 0 0],[0 0 1 tz],[0 0 0 1]]
		cRT<T>& TranslateZ(const T tz)
		{
			m0[3] += m0[2] * tz;
			m1[3] += m1[2] * tz;
			m2[3] += m2[2] * tz;
			return *this;
		}
		// RotateX = multiply on right by [[1 0 0 0],[0 c -s 0],[0 s c 0],[0 0 0 1]]
		cRT<T>& RotateX(const T rx)
		{
			const T s(sin(rx)), c(cos(rx));
			{ const T tmp = m0[1]; m0[1] = c * tmp + s * m0[2]; m0[2] = c * m0[2] - s * tmp; }
			{ const T tmp = m1[1]; m1[1] = c * tmp + s * m1[2]; m1[2] = c * m1[2] - s * tmp; }
			{ const T tmp = m2[1]; m2[1] = c * tmp + s * m2[2]; m2[2] = c * m2[2] - s * tmp; }
			return *this;
		}
		// RotateY = multiply on right by [[c 0 s 0],[0 1 0 0],[-s 0 c 0],[0 0 0 1]]
		cRT<T>& RotateY(const T ry)
		{
			const T s(sin(ry)), c(cos(ry));
			{ const T tmp = m0[2]; m0[2] = c * tmp + s * m0[0]; m0[0] = c * m0[0] - s * tmp; }
			{ const T tmp = m1[2]; m1[2] = c * tmp + s * m1[0]; m1[0] = c * m1[0] - s * tmp; }
			{ const T tmp = m2[2]; m2[2] = c * tmp + s * m2[0]; m2[0] = c * m2[0] - s * tmp; }
			return *this;
		}
		// RotateZ = multiply on right by [[c -s 0 0],[s c 0 0],[0 0 1 0],[0 0 0 1]]
		cRT<T>& RotateZ(const T rz)
		{
			const T s(sin(rz)), c(cos(rz));
			{ const T tmp = m0[0]; m0[0] = c * tmp + s * m0[1]; m0[1] = c * m0[1] - s * tmp; }
			{ const T tmp = m1[0]; m1[0] = c * tmp + s * m1[1]; m1[1] = c * m1[1] - s * tmp; }
			{ const T tmp = m2[0]; m2[0] = c * tmp + s * m2[1]; m2[1] = c * m2[1] - s * tmp; }
			return *this;
		}

		// lim d->0 ((M.TranslateX(d)-M)*(M^-1(v)))/d
		cVector3<T> DTranslateX() const
		{
			// (M.TranslateX(d)-M)/d = M * [[0 0 0 1],[0 0 0 0],[0 0 0 0],[0 0 0 0]] = [0; mData[:,0]]
			return cVector3<T>(m0[0], m1[0], m2[0]);
		}
		// lim d->0 ((M.TranslateY(d)-M)*(M^-1(v)))/d
		cVector3<T> DTranslateY() const
		{
			// (M.TranslateY(d)-M)/d = M * [[0 0 0 0],[0 0 0 1],[0 0 0 0],[0 0 0 0]] = [0; mData[:,1]]
			return cVector3<T>(m0[1], m1[1], m2[1]);
		}
		// lim d->0 ((M.TranslateY(d)-M)*(M^-1(v)))/d
		cVector3<T> DTranslateZ() const
		{
			// (M.TranslateZ(d)-M)/d = M * [[0 0 0 0],[0 0 0 0],[0 0 0 1],[0 0 0 0]] = [0; mData[:,2]]
			return cVector3<T>(m0[2], m1[2], m2[2]);
		}
		// lim d->0 (M.RotateX(d)-M)*(M^-1(v))/d
		cVector3<T> DRotateX(const cVector3<T> &v) const
		{
			// (M.RotateX(d)-M)/d = M * [[0 0 0 0],[0 0 -1 0],[0 1 0 0],[0 0 0 0]] = [0; mData[:,2]; -mData[:,1]; 0]
			const T d0 = v[0] - m0[3], d1 = v[1] - m1[3], d2 = v[2] - m2[3];
			const T y = m0[1] * d0 + m1[1] * d1 + m2[1] * d2;
			const T z = m0[2] * d0 + m1[2] * d1 + m2[2] * d2;
			return cVector3<T>(m0[2]*y-m0[1]*z, m1[2]*y-m1[1]*z, m2[2]*y-m2[1]*z);
		}
		// lim d->0 (M.RotateY(d)-M)*(M^-1(v))/d
		cVector3<T> DRotateY(const cVector3<T> &v) const
		{
			// (M.RotateY(d)-M)/d = M * [[0 0 1 0],[0 0 0 0],[-1 0 0 0],[0 0 0 0]] = [-mData[:,2]; 0; mData[:,0]; 0]
			const T d0 = v[0] - m0[3], d1 = v[1] - m1[3], d2 = v[2] - m2[3];
			const T x = m0[0] * d0 + m1[0] * d1 + m2[0] * d2;
			const T z = m0[2] * d0 + m1[2] * d1 + m2[2] * d2;
			return cVector3<T>(m0[0]*z-m0[2]*x, m1[0]*z-m1[2]*x, m2[0]*z-m2[2]*x);
		}

		// lim d->0 (M.RotateZ(d)-M)*(M^-1(v))/d
		cVector3<T> DRotateZ(const cVector3<T> &v) const
		{
			// (M.RotateZ(d)-M)/d = M *[[0 -1 0 0],[1 0 0 0],[0 0 0 0],[0 0 0 0]] = [mData[:,1]; -mData[:,0]; 0; 0]
			const T d0 = v[0] - m0[3], d1 = v[1] - m1[3], d2 = v[2] - m2[3];
			const T x = m0[0] * d0 + m1[0] * d1 + m2[0] * d2;
			const T y = m0[1] * d0 + m1[1] * d1 + m2[1] * d2;
			return cVector3<T>(m0[1]*x-m0[0]*y, m1[1]*x-m1[0]*y, m2[1]*x-m2[0]*y);
		}

		cVector3<T> Origin() const
		{
			return cVector3<T>(m0[3], m1[3], m2[3]);
		}

		cVector3<T> InverseRot(const cVector3<T> &v) const
		{
			// R^T v (true in this class)
			const T d0 = v[0], d1 = v[1], d2 = v[2];
			const T x = m0[0] * d0 + m1[0] * d1 + m2[0] * d2;
			const T y = m0[1] * d0 + m1[1] * d1 + m2[1] * d2;
			const T z = m0[2] * d0 + m1[2] * d1 + m2[2] * d2;
			return cVector3<T>(x, y, z);
		}

		cVector3<T> Inverse(const cVector3<T> &v) const
		{
			// M = [R;T]; M u = v; R u + T = v; u = R^-1(v - T)
			return InverseRot(v - Origin());
		}

		cVector3<T> operator*(const cVector3<T> &v) const
		{
			const T v0 = v[0], v1 = v[1], v2 = v[2];
			return cVector3<T>(
				m0[0] * v0 + m0[1] * v1 + m0[2] * v2 + m0[3],
				m1[0] * v0 + m1[1] * v1 + m1[2] * v2 + m1[3],
				m2[0] * v0 + m2[1] * v1 + m2[2] * v2 + m2[3]
			);
		}

		cRT<T> operator*(const cRT<T> &that) const
		{
			cRT<T> ret;
			const T *t0 = that.m0, *t1 = that.m1, *t2 = that.m2;
			for (int i = 0; i < 3; ++i)
			{
				const T *r = (*this)[i];
				const T r0 = r[0], r1 = r[1], r2 = r[2], r3 = r[3];
				T *s = ret[i];
				s[0] = r0 * t0[0] + r1 * t1[0] + r2 * t2[0];
				s[1] = r0 * t0[1] + r1 * t1[1] + r2 * t2[1];
				s[2] = r0 * t0[2] + r1 * t1[2] + r2 * t2[2];
				s[3] = r0 * t0[3] + r1 * t1[3] + r2 * t2[3] + r3;
			}
			return ret;
		}

		cRT<T>& operator*=(const cRT<T> &that)
		{
			return *this = (*this)*that;
		}

		cRT<T>& ApplyMatOp(const eChannelType j, const cRT<T> &m)
		{
			if (j == eChannelType::I) return *this; // Identity
			if (j == eChannelType::L) return (*this) *= m; // 36 multiplies; avoid where possible
			if (j == eChannelType::LT) return Translate(m.m0[3], m.m1[3], m.m2[3]);
			if (j == eChannelType::LTX) return TranslateX(m.m0[3]);
			if (j == eChannelType::LTY) return TranslateY(m.m1[3]);
			if (j == eChannelType::LTZ) return TranslateZ(m.m2[3]);
			throw;
		}

		cRT<T>& ApplyChanOp(const eChannelType j, const T v)
		{
			if (j == eChannelType::tx) return TranslateX(v);
			if (j == eChannelType::ty) return TranslateY(v);
			if (j == eChannelType::tz) return TranslateZ(v);
			if (j == eChannelType::rx) return RotateX(v);
			if (j == eChannelType::ry) return RotateY(v);
			if (j == eChannelType::rz) return RotateZ(v);
			throw;
		}

		cVector3<T> DChanOp(const eChannelType j, const cVector3<T> &v) const
		{
			if (j == eChannelType::tx) return DTranslateX();
			if (j == eChannelType::ty) return DTranslateY();
			if (j == eChannelType::tz) return DTranslateZ();
			if (j == eChannelType::rx) return DRotateX(v);
			if (j == eChannelType::ry) return DRotateY(v);
			if (j == eChannelType::rz) return DRotateZ(v);
			throw;
		}

		cVector3<T> DecomposeR(eChannelType r1, eChannelType r2, eChannelType r3) const
		{
			cVector3<T> ret;
			const int i = r3 - eChannelType::rx, j = r2 - eChannelType::rx, k = r1 - eChannelType::rx;
			const T *row_i = (*this)[i], *row_j = (*this)[j], *row_k = (*this)[k];
			const T cj = sqrt(row_i[i] * row_i[i] + row_j[i] * row_j[i]);
			ret[1] = std::atan2<T>(-row_k[i], cj);
			if (cj > T(1e-7))
			{
				ret[2] = std::atan2<T>(row_k[j], row_k[k]);
				ret[0] = std::atan2<T>(row_j[i], row_i[i]);
			}
			else
			{
				ret[0] = std::atan2<T>(row_j[k], row_j[j]);
				ret[2] = 0.0f;
			}
			if (j == i+2 || i == j+1)
				ret = -ret;
			return ret;
		}

		Eigen::Matrix<double, 3, 4, Eigen::RowMajor> Mat() const
		{
			Eigen::Matrix<double, 3, 4, Eigen::RowMajor> M;
			M<< m0[0], m0[1], m0[2], m0[3],
				m1[0], m1[1], m1[2], m1[3],
				m2[0], m2[1], m2[2], m2[3];
			return M
		}

		T* operator[](int i) { return (i == 0 ? m0 : (i == 1 ? m1 : m2)); }
		const T* operator[](int i) const { return (i == 0 ? m0 : (i == 1 ? m1 : m2)); }

	private:
		T m0[4];
		T m1[4];
		T m2[4];
	};

	///<summary> Represents the intrinsic lens matrix [mFx 0 -mOx; 0 mFy -mOy; 0 0 -1] and distortion parameters.
	/// after multiplying, projection is achieved by dividing the vector by its z coordinate.
	/// so, this represents a camera looking down the negative z-axis. x is right and y is up.
	/// if projection happens *before* multiplying the matrix, it should divide the negative z-coordinate.
	/// the transform then becomes simply (mFx * x + mOx, mFy * y + mOy).
	/// this space, before multiplying, is a unitless space called screen space.
	/// screen space coordinates have an implicit -1 in the z-coordinate.
	/// its origin is at the lens centre. this is the space in which the lens distortion is properly applied.
	/// lens distortion uses a five-parameter model, radial: mK1_Inv, mK2_Inv, mK3_Inv and tangential: mTx_Inv, mTy_Inv.
	/// Compared to Motive, mFy = -FocalLengthY (because Motive pixels are measured from the top-left,
	/// whereas the y-axis is up in the world). Also: mTx_Inv = Tangential1*(2*ks), mTy_Inv = Tangential0*(-2*ks), 
	/// mK1_Inv = Coefficient0*(ks*ks), mK2_Inv = Coefficient1*(ks*ks*ks*ks), (where ks = mFx/ImagerPixelWidth).
	/// mFx = FocalLengthX, mOx = PrincipalX, mOy = PrincipalY are unchanged.
	/// Note that Motive's lens distortion parameters are for *undistorting* the lens.
	/// This class inverts those lens undistortion parameters to produce lens distortion parameters.
	/// The Distort method performs the conversion from screen space to pixels, and the Undistort method
	/// reverses this.
	/// </summary>
	template<class T>
	class cK
	{
		template <class T> friend class cP;
		template <class T> friend class cLensSolver;
		template <class T> friend class cCameraSolver;
		template <class T> friend class cPointSolver;
	public:
		cK(T fx = 1, T fy = -1, T ox = 0, T oy = 0, T k1 = 0, T k2 = 0, T k3 = 0, T tx = 0, T ty = 0) :
			mFx(fx), mFy(fy), mOx(ox), mOy(oy), mK1_Inv(k1), mK2_Inv(k2), mK3_Inv(k3), mTx_Inv(tx), mTy_Inv(ty)
		{
			ComputeLensInverse();
		}

		void ComputeLensInverse()
		{
			cLensSolver<T>(*this).ComputeLensInverse();
		}

		T UndistortScale() const { return std::abs<T>(mFx_Inv); }

		///<summary> Removes the effect of lens distortion on the detection, converting from pixels to unitless </summary>
		void Undistort(T &t0, T &t1) const
		{
			const T v0 = (t0 - mOx) * mFx_Inv, v1 = (t1 - mOy) * mFy_Inv, v01 = v0*v1, v00 = v0*v0, v11 = v1*v1;
			const T r2 = v00 + v11, s = T(1) + r2*(mK1_Inv + r2 * (mK2_Inv + r2 * mK3_Inv)), hr2 = T(0.5)*r2;
			t0 = v0 * s + (hr2 + v00) * mTx_Inv + v01 * mTy_Inv;
			t1 = v1 * s + (hr2 + v11) * mTy_Inv + v01 * mTx_Inv;
		}

		///<summary> Applies lens distortion to a projected point, converting to pixels </summary>
		void Distort(T &u0, T &u1) const
		{
			const T u01 = u0*u1, u00 = u0*u0, u11 = u1*u1, ur2 = u00 + u11, hur2 = T(0.5)*ur2;
			const T s = T(1) + ur2*(mK1 + ur2 * (mK2 + ur2 * mK3));
			const T v0 = u0 * s + (hur2 + u00) * mTx + u01 * mTy;
			const T v1 = u1 * s + (hur2 + u11) * mTy + u01 * mTx;
			// polish to make the result agree more exactly with the undistort
			const T v01 = v0*v1, v00 = v0*v0, v11 = v1*v1, r2 = v00 + v11, hr2 = T(0.5)*r2;
			const T r = T(1) / (T(1) + r2*(mK1_Inv + r2 * (mK2_Inv + r2 * mK3_Inv)));
			const T dx = (hr2 + v00) * mTx_Inv + v01 * mTy_Inv;
			const T dy = (hr2 + v11) * mTy_Inv + v01 * mTx_Inv;
			u0 = (u0 - dx) * r * mFx + mOx;
			u1 = (u1 - dy) * r * mFy + mOy;
		}

		///<summary> Computes a "metric ray" (px,py,-1) for the pixel coordinate </summary>
		cVector3<T> MetricRay(const cVector2<T> &v) const
		{
			cVector3<T> ret(v[0], v[1], T(-1));
			Undistort(ret[0], ret[1]);
			return ret;
		}

		///<summary> Projects a 3D point without applying lens distortion, returns unitless coordinate </summary>
		cVector2<T> ProjectNoDistort(const cVector3<T> &p) const
		{
			if (p[2] > T(-0.001)) return cVector2<T>(T(1e10), T(1e10)); // positive-z is behind the camera and not seen
			const T r = T(-1) / p[2];
			cVector2<T> ret(p[0] * r, p[1] * r);
			// TODO camera frustum clipping?
			return ret;
		}

		///<summary> Projects a 3D point and applies lens distortion, returns pixel coordinate </summary>
		cVector2<T> Project(const cVector3<T> &p) const
		{
			cVector2<T> ret(ProjectNoDistort(p));
			if (ret[0] >= T(1e10)) return ret;
			Distort(ret[0], ret[1]);
			return ret;
		}

		Eigen::Matrix<double, 3, 3, Eigen::RowMajor> Mat() const
		{
			Eigen::Matrix<double, 3, 3, Eigen::RowMajor> M;
			M<< mFx,   0, -mOx,
				  0, mFy, -mOy,
				  0,   0,   -1;
			return M;
		}

	private:
		T mFx, mFy, mOx, mOy, mK1_Inv, mK2_Inv, mK3_Inv, mTx_Inv, mTy_Inv;
		T mFx_Inv, mFy_Inv, mK1, mK2, mK3, mTx, mTy;
	};

	///<summary> represents a camera </summary>
	template<class T>
	class cP
	{
		template <class T> friend class cLensSolver;
		template <class T> friend class cCameraSolver;
		template <class T> friend class cPointSolver;
	public:
		cP() {}
		cP(const cK<T> &K, const cRT<T> &RT) : mK(K), mRT(RT) {}

		void ComputeLensInverse()
		{
			mK.ComputeLensInverse();
		}

		///<summary> given a target projection matrix, update mK and mRT to match. preserve the signs of mFx and mFy </summary>
		void UpdateP(Eigen::Matrix<double, 3, 4, Eigen::RowMajor> P)
		{
			auto R = P.block<3, 3>(0, 0); // alias
			const double det = R.determinant();
			if (det < 0) P *= -1; // deal with mirror-image solutions
			P *= 1.0 / R.row(2).norm(); // third row of R must have unit norm; so scale P
			const double ox = R.row(0).transpose().dot(R.row(2));
			const double oy = R.row(1).transpose().dot(R.row(2));
			P.row(0) -= ox * P.row(2);
			P.row(1) -= oy * P.row(2);
			const double sy = R.row(1).norm(), osy = 1.0 / sy;
			P.row(1) *= osy;
			auto r0 = R.row(1).cross(R.row(2));
			const double sx = R.row(0).transpose().dot(r0), osx = 1.0 / sx;
			ASSERT(sx > 0);
			P.row(0) *= osx;
			R.row(0) = r0; // now P contains RT and P_original = [sx 0 ox; 0 sy oy; 0 0 1] RT (ignoring skew)
			// now update K -> K*[sx 0 ox; 0 sy oy; 0 0 1], remember K = [mFx 0 -mOx; 0 mFy -mOy; 0 0 -1]
			mK.mOx -= T(mK.mFx * ox);
			mK.mOy -= T(mK.mFy * oy);
			mK.mFx *= T(sx);
			mK.mFy *= T(sy);
			mK.mFx_Inv *= T(osx);
			mK.mFy_Inv *= T(osy);
			mRT.Init(P.data());
		}

		Eigen::Matrix<double, 3, 4, Eigen::RowMajor> Mat() const
		{
			return mK.Mat() * mRT.Mat();
		}

		cVector3<T> OpticalAxis() const
		{
			return mRT.InverseRot(cVector3<T>(0.0f,0.0f,-1.0f));
		}

		cVector3<T> RayDir(const cVector2<T> &d) const
		{
			return mRT.InverseRot(mK.MetricRay(d)).Normalized();
		}

		///<summary> the position of the camera </summary>
		cVector3<T> Position() const
		{
			return mRT.Inverse(cVector3<T>(0,0,0));
		}

		cVector2<T> Project(const cVector3<T> &v) const
		{
			return mK.Project(mRT * v);
		}

		cVector2<T> ProjectNoDistort(const cVector3<T> &v) const
		{
			return mK.ProjectNoDistort(mRT * v);
		}

		void Undistort(T &vx, T &vy) const { return mK.Undistort(vx, vy); }

		T UndistortScale() const { return mK.UndistortScale(); }

	private:
		cK<T>  mK;  // camera intrinsics
		cRT<T> mRT; // camera extrinsics
	};

	typedef cRT<float> cRTf;
	typedef cK<float> cKf;
	typedef cP<float> cPf;
	typedef cRT<double> cRTd;
	typedef cK<double> cKd;
	typedef cP<double> cPd;

	/// <summary> cStats maintains a running estimate of Mean and Mean-square of a sequence of float values
	/// </summary>
	class cStats
	{
	public:
		cStats() { Reset(); }
		void Reset(float v = 0.0f)
		{
			mCount = v == 0.0f ? 0 : 1;
			mFramesNotSeen = 0;
			mMean = v;
			mMeanSq = v*v;
			mMin = v == 0.0f ? 1e10f : v;
			mMax = v == 0.0f ? 0.0f : v;
		}
		void NotSeen()
		{
			mFramesNotSeen++;
		}
		void Add(float v, int maxCount = 100)
		{
			assert(v != 0.0f);
			assert(mMin != 0.0f);
			mFramesNotSeen = 0;
			mMin = std::min(mMin, v);
			mMax = std::max(mMax, v);
			mCount++;
			// this is mathematically exact until 100 samples, then becomes 1% update
			double scale = 1.0/(mCount > maxCount ? maxCount : mCount);
			mMean += (v - mMean) * scale;
			mMeanSq += (v*v - mMeanSq) * scale;
		}
		int Count() const { return mCount; }
		double Mean() const { return mMean;  }
		double MeanSq() const { return mMeanSq; }
		double Var() const { return mMeanSq - mMean*mMean; }
		double Range() const { return mMax - mMin; }
		// short, stiff edges that have been around a long time have the lowest (best) score
		double Score() const { return mFramesNotSeen > 100  || mCount == 0 ? 1e10 : mMean * (1.0 + Range()) / (1.0 + 0.01 * mCount); } // NB when mCount == 1, Score() == mMean
		bool operator< (const cStats &that) const { return Score() < that.Score(); }

	private:
		int mCount;
		int mFramesNotSeen;
		float mMin, mMax;
		double mMean;
		double mMeanSq;
	};

	/// <summary> sEdgeStats holds statistics for an edge
	/// </summary>
	struct sEdgeStats
	{
	public:
		sEdgeStats(float m, float iv) : mean(m), invVar(iv) {}
		float GetScore(float d) const { d -= mean; return d*d*invVar; }
	private:
		float mean;
		float invVar;
	};

	/// <summary> cSparseStatsMatrix is a sparse matrix that collects statistics of distances between pairs of index markers,
	/// identified by UID. use add_item or AddFixedRow.
	/// </summary>
	typedef cUIDMatrix<sIndexDataPair<cStats> > cSparseStatsMatrix;
	typedef cUIDMatrix<sIndexDataPair<sEdgeStats> > cSparseEdgeStatsMatrix;
	typedef cMatrix<sIndexFloat> cSparseFloatMatrix;
	typedef cMatrix<sIndexDataPair<cVector2f> > cSparseDetsMatrix;

	typedef cMat<sIndexFloat> cSparseFloatMat;

	template<typename T> struct sHashEnum {};
	template<> struct sHashEnum<cVector3f> { enum { HashSize = 1 << 12, HashOffsetSize = 8 }; };
	template<> struct sHashEnum<cVector2f> { enum { HashSize = 1 << 10, HashOffsetSize = 4 }; };

	///<summary> a locality hashing function. takes a 3D point and computes a 12-bit integer. points close together
	/// get related hash values.
	///</summary>
	static int Hash(const cVector3f &v, const float s, const float o) 
	{
		return (int((v[0] - o)*s) + (int((v[1] - o)*s) << 4) + (int((v[2] - o)*s) << 8)) & 0xfff;
	}
	///<summary> a locality hashing function. takes a 2D point and computes a 10-bit integer. points close together
	/// get related hash values.
	///</summary>
	static int Hash(const cVector2f &v, const float s, const float o) 
	{
		return (int((v[0] - o)*s) + (int((v[1] - o)*s) << 5)) & 0x3ff;
	}

	static const cVec<int> HashOffsets(cVector3f)
	{
		static const int offsets3D[8] = { 0x0,0x1,0x10,0x11,0x100,0x101,0x110,0x111 };
		static std::vector<int> offsets(offsets3D, offsets3D + 8);
		return offsets;
	}

	static const cVec<int> HashOffsets(cVector2f)
	{
		static const int offsets2D[4] = { 0x0,0x1,0x20,0x21 };
		static std::vector<int> offsets(offsets2D, offsets2D + 4);
		return offsets;
	}

	///<summary> cHashCloud holds the locality hashing data for a set of source points.
	/// it can generate a sparse matrix of all the markers within some threshold distance of each point.
	/// Obviously, the worst-case scenario here is O(n m), if the matrix is dense.
	/// However, in general the threshold 't' is such that there are very few candidates to consider.
	/// The source points are hashed with an offset of t. The target points are hashed without the offset.
	/// For 3D points, using Hash(cVector3f), a source with hash 'h' matches 8 possible target hashes
	/// (modulo 0x1000): h,h+1,h+0x10,h+0x11,h+0x100,h+0x101,h+0x110,h+0x111
	/// For 2D points, using Hash(cVector2f), a source with hash 'h' matches 4 possible target hashes
	/// (modulo 0x400): h,h+1,h+0x20,h+0x21
	/// let's math this out with a simplified 1D version of the hashing function:
	/// h1 = int((x1-t)/(2*t)), the source
	/// h2 = int(x2/(2*t))    , the target
	/// so : h1 <= (x1-t)/(2*t) < h1 + 1,       NB int works by truncation, not floor, so there is a wider box at the origin :-/
	/// and: h2 <= x2/(2*t)     < h2 + 1
	/// so : h1*(2*t)+t <= x1 < h1*(2*t) + 3*t, NB multiplying by (2*t) is safe since t positive
	/// and: h2*(2*t)   <= x2 < h2*(2*t) + 2*t
	/// so : x1 - x2 > (h1-h2)*(2*t) - t,
	/// and: x2 - x1 > (h2-h1)*(2*t) - 3*t
	/// if: h2 == h1+delta, then: x1-x2 > (-2*delta - 1)*t, and: x2-x1 > (2*delta - 3)*t
	/// so : x1-x2 >= t iff: -2*delta-1 >= 1, i.e.: delta <= -1
	/// and: x2-x1 >= t iff:  2*delta-3 >= 1, i.e.: delta >= 2
	/// so the only possiblity for |x1-x2| < t is: delta in {0,1}
	/// the result follows from this and modulo arithmetic (the y and z coordinates are shifted)
	/// For efficiency, generate a table of all possible targets for each source point.
	/// All matched for a target point can then be considered by iterating through a list of points
	/// having the same hash. The actual matches are generated by measuring the actual distance.
	/// Points with x coordinate >= 1e10 are deliberately excluded.
	///</summary>
	template<class T, int S = int(sHashEnum<T>::HashSize)>
	class cHashCloud
	{
		cVec<T> mSourcePositions; // an internal copy
		const float mThreshold;
		const float mThresholdSquared;
		const float mScale;
		std::vector<int> mIndices;
		int mCounts[S + 1];

		cHashCloud(const cHashCloud&) {} // private
	public:
		template<class V>
		cHashCloud(const V &sourcePositions, float threshold) :
			mSourcePositions(sourcePositions), mThreshold(threshold), 
			mThresholdSquared(threshold*threshold), mScale(0.5f/threshold)
		{
			const int num = (int)sourcePositions.size();
			const cVec<int> &hashOffsets = HashOffsets(T());
			std::vector<int> hashesV(num);
			int *hashes = hashesV.data(), hi = 0;
			std::fill(mCounts, mCounts + S + 1, 0);
			for (const auto &p : sourcePositions)
			{
				const int h = Hash(p, mThreshold);
				hashes[hi++] = h;
				for (auto o : hashOffsets)
					mCounts[(h+o)&(S-1)]++;
			}
			// use bucket sorting for efficiency
			for (int i = 0; i != S; ++i)
				mCounts[i + 1] += mCounts[i];
			mIndices.resize(mCounts[S]);
			for (int i = num - 1; i >= 0; --i)
			{
				const int h = hashes[i];
				for (const auto &o : hashOffsets)
					mIndices[--mCounts[(h+o)&(S-1)]] = i;
			}
		}
		int Hash(const T &v) const { return Core::Hash(v, mScale, 0); }
		int Hash(const T &v, const float o) const { return Core::Hash(v, mScale, o); }
		bool Test(float &d, const T &v, const int i) const
		{
			d = v.DistanceSquared(mSourcePositions[i]);
			return (d < mThresholdSquared);
		}
		float Threshold() const { return mThreshold; }

		template<class V>
		void MakeDistanceMatrix(cSparseFloatMatrix &matrix,
			const V &targetPositions,
			const bool allowZero = true) const
		{
			float d;
			matrix.clear();
			matrix.reserve(targetPositions.size());
			for (const auto &p : targetPositions)
			{
				if (p[0] < 1e10) // skip bad points
				{
					const int h = Hash(p);
					for (int cit = mCounts[h], cend = mCounts[h+1]; cit != cend; ++cit)
					{
						const int i = mIndices[cit];
						if (Test(d, p, i) && (allowZero || d != 0))
							matrix.AddRowItem(sIndexFloat(i, std::sqrt(d)));
					}
				}
				matrix.EndRow();
			}
		}

	};
	
	struct sSlack
	{
		float s;   // the slack of the edge
		int   l;   // the label
		int   m;   // the marker
		sSlack(float slack = 0.0f, int label = 0, int marker = 0) : s(slack), l(label), m(marker) {}
		// for std::min_element
		bool operator<(const sSlack &that) const { return s < that.s; }
	};

	///<summary> Given a sparse matrix of NON-NEGATIVE costs of assigning each marker uniquely to a label, and 
	/// a threshold for unassigned markers, compute the minimum assignment cost using the Hungarian 
	/// linear-sum assignment algorithm.
	/// This implementation is based on Jonker-Volgenant, but highly optimised for this use case.
	/// Fills the vector l2m [for each label the unique marker index, or -1 for unlabelled], and returns the total cost.
	///
	/// In the original formulation, the goal is to set labelCosts and markerCosts in order to create zeros in the negative matrix:
	/// matrix - labelCosts*ones(numMarkers) - ones(numLabels)*markerCosts  [where the * is an outer product]
	/// The minimum value in a row or column can be set in the labelCost or markerCost, creating a zero.
	/// When every row and column contains a zero then the assignment is optimal.
	///
	/// Define: slack(li,mi) = matrix(li,mi) - labelCosts[li] - markerCosts[mi].
	/// In our case, matrix(li,mi) = infinity wherever the sparse matrix is not defined.
	/// Furthermore, there is a bogus 'unlabelled' possibility for each label: matrix(li,numMarkers+li) = threshold
	/// For efficiency, the code uses a single NO_MARKER label and clears it for reuse after each iteration.
	/// Then: slack(li,mi) == 0 for assigned label-marker pairs, and is positive for unassigned labels and/or markers.
	///
	/// Add labels to the assignment by finding a path that starts with an unassigned root label and ends with 
	/// an unassigned marker. This path must alternate between labels and markers and must not revisit any label or marker.
	/// Once the path reaches an unused marker, the parent label in the path can be assigned to that marker.
	/// That parent label might previously have been assigned to another marker; so that marker in turn is assigned to its 
	/// parent label in the path. This continues all the way back to the root label, thus adding a label to the assignment.
	///
	/// In order for the assignment to still be minimal, the path must be constructed in a way that it minimises the slack.
	/// This is achieved by iteratively updating a structure that holds the best-possible-edges-to-depth.
	/// Edges are added to the path in order of minimum slack = matrix[li][mi]-labelCosts[li]-markerCosts[mi]
	///
	/// slacks holds the edges: minSlacks, label and marker.
	/// m2si gives the index of the marker in the slacks vector. if m2si[mi] = ~li < 0, it indicates that li should assign to mi
	/// 
	/// optimisation: set slackOffset = edge.s-labelCosts[lj], then slack = matrix[lj][mi]-markerCosts[mi]+slackOffset
	/// optimisation: the labelCosts and markerCosts are updated outside the inner loop, by preserving the minSlacks in slacks[:depth]
	/// optimisation: the m2si vector maps from markers to indices into slacks. since slacks <= depth is reserved for minSlacks edges
	///               then the ones-complement of the parent label index is stored instead
	/// optimisation: the slacks vector holds minSlacks edges in the first depth places followed by candidate edges. each selected
	///               edge is swapped to the depth position
	///</summary>
	static void HungarianAssignment(
		std::vector<float> &workspace,
		cVec<int> &l2m,
		const cSparseFloatMat &matrix,
		const float threshold)
	{
		const int numLs = matrix.size();
		const int numMs = matrix.NumCols();
		const int NO_MARKER = -1, NO_INDEX = numLs + numMs + 3; // NO_INDEX is greater than any valid index
		workspace.resize(numLs + (numMs+1) * 6 + 3); // workspace overhead is significant
		ASSERT(sizeof(float) == sizeof(int) && sizeof(sSlack) == sizeof(int) * 3);
		// NOTE markerCosts, m2l and m2si are offset by 1 to allow an index of -1, meaning NO_MARKER
		float *labelCosts = workspace.data(), *markerCosts = labelCosts + numLs + 1, *costs_end = markerCosts + numMs;
		int *m2l = reinterpret_cast<int*>(costs_end)+1, *m2si = m2l + numMs + 1, *m2si_end = m2si + numMs;
		// NOTE slacks is offset by -1 because slacks[0] is not used
		sSlack *slacks = reinterpret_cast<sSlack*>(m2si_end)-1, *slacks_end = slacks + numMs + 3;
		ASSERT(reinterpret_cast<float*>(slacks_end) == workspace.data() + workspace.size());
		std::fill(labelCosts, costs_end, 0.0f);
		std::fill(m2l-1, m2si_end, NO_INDEX);
		// m2si contains these values: labels=[-numLs, -1], slacks indices=[1, numMs+2], unassigned = NO_INDEX
		std::fill(l2m.begin(), l2m.end(), NO_MARKER);
		sSlack *slacksHead = slacks, *slacksTail = slacks;
		int m = NO_MARKER;
		for (int li = 0; li != numLs; ++li)
		{
			float &slacksSum = labelCosts[li]; // the label cost is the sum of all the minslacks
			if (matrix.IsEmptyRow(li)) { slacksSum = threshold; continue; } // shortcut if no candidates
			++slacksTail; // slacks[0] is not used
			for (int l = li; l != NO_INDEX; l = slacksHead->l = m2l[m]) // continue until minslack picks an unlabelled marker
			{
				const float slackOffset = slacksSum - labelCosts[l], s = threshold + slackOffset;
				const int mi = NO_MARKER, si = m2si[mi]; // deal with the NO_MARKER assignment possibility
				if (si == NO_INDEX) { m2si[mi] = int(slacksTail - slacks); *slacksTail++ = sSlack(s, l, mi); }
				else if (s < slacks[si].s) { slacks[si].s = s; slacks[si].l = l; }
				for (const auto &mit : matrix[l]) // add all the edges for the label
				{
					const int mi = mit.mIndex, si = m2si[mi];
					if (si < 0) continue; // NOTE negative value means the marker is already in the path
					const float s = mit.mData - markerCosts[mi] + slackOffset;
					if (si == NO_INDEX) { m2si[mi] = int(slacksTail - slacks); *slacksTail++ = sSlack(s, l, mi); }
					else if (s < slacks[si].s) { slacks[si].s = s; slacks[si].l = l; }
				}
				const int shm((++slacksHead)->m); // slacks[0] is not used
				std::swap(*slacksHead, *std::min_element(slacksHead, slacksTail)); // swap minslack to head
				m = slacksHead->m;
				m2si[shm] = m2si[m];
				m2si[m] = ~l; // NOTE ~l is negative whereas all other m2si values are positive
				slacksSum = slacksHead->s;
			}
			while (m != NO_MARKER)
				std::swap(l2m[m2l[m] = ~m2si[m]], m); // NOTE ~~l == l
			m2l[NO_MARKER] = NO_INDEX; // permit reuse of the NO_MARKER assignment
			while (--slacksHead != slacks)
			{
				const float delta = slacksSum - slacksHead->s; // delta is the partial sum of minslacks
				labelCosts[slacksHead->l] += delta;
				markerCosts[slacksHead->m] -= delta;
			}
			while (--slacksTail != slacks)
				m2si[slacksTail->m] = NO_INDEX;
		}
		workspace.resize(numLs + numMs + 1);
	}

	static float HungarianAssignment(
		std::vector<int> &ret_l2m,
		const cSparseFloatMat &matrix,
		const float threshold)
	{
		std::vector<float> workspace; // for efficiency, don't use this version; instead reuse the workspace
		ret_l2m.resize(matrix.size());
		cVec<int> l2m(ret_l2m);
		HungarianAssignment(workspace, l2m, matrix, threshold);
		// this demonstrates how to extract the score from the workspace
		double ret = 0;
		for (const auto &v : workspace) ret += v;
		return (float)ret;
	}

	/// <summary>
	/// update a cSparseStatsMatrix with a new frame of marker IDs and positions.
	/// preserve the number of edges, and keep the highest-scoring candidates.
	/// </summary>
	static void UpdateStatsMatrix(cSparseStatsMatrix &matrix,
		const cVec<cUID> &markerIDs,
		const cVec<cVector3f> &sourcePositions,
		const float threshold,
		const int numEdges)
	{
		cUIDMatrix<sIndexFloat> newEdgeCandidates;

		std::map<cUID, cVector3f> markers;
		const int numMs = (int)markerIDs.size();
		for (int mi = 0; mi < numMs; ++mi)
		{
			const cVector3f &pos = sourcePositions[mi];
			cUID markerUID = markerIDs[mi];
			newEdgeCandidates.AddUID(markerUID);
			if (!matrix.HasUID(markerUID))
			{
				matrix.AddFixedRow(markerUID, numEdges, sIndexDataPair<cStats>(-1, cStats()));
			}
			if (pos[0] >= 1e10) continue; // ignore bad markers
			markers[markerUID] = pos;
		}

		// update existing edges
		std::map<cUID, std::set<cUID> > existingEdges;
		for (int i = 0, imax = matrix.size(); i != imax; ++i)
		{
			const cUID iuid = matrix.Index2Uid(i);
			if (!markers.count(iuid))
			{
				for (auto &s : matrix[i]) s.mData.NotSeen();
				continue;
			}
			std::set<cUID> &edges_i = existingEdges[iuid];
			auto &m = matrix[i];
			for (auto &s : matrix[i])
			{
				const int j = s.mIndex;
				if (j == -1) continue;
				const cUID juid = matrix.Index2Uid(j);
				if (!markers.count(juid))
				{
					s.mData.NotSeen();
				}
				else
				{
					const float d = markers.at(iuid).Distance(markers.at(juid));
					s.mData.Add(d);
					edges_i.insert(juid);
				}
			}
		}

		// find all edges between markers shorter than threshold
		cHashCloud<cVector3f> hashCloud(sourcePositions, threshold);
		hashCloud.MakeDistanceMatrix(newEdgeCandidates, sourcePositions, false);
		newEdgeCandidates.SortColumns();
		for (int i = 0; i != newEdgeCandidates.size(); ++i)
		{
			cUID iuid = newEdgeCandidates.Index2Uid(i);
			const int si = matrix.Uid2Index(iuid);
			std::set<cUID> &smap = existingEdges[iuid];
			auto sit = matrix[si].begin(), sit_end = matrix[si].end();
			for (const auto &git : newEdgeCandidates[i])
			{
				const cUID uid = newEdgeCandidates.Index2Uid(git.mIndex);
				if (smap.count(uid)) continue;
				const int sj = matrix.Uid2Index(uid);
				const float s = git.mData; // note, Score() == s when there is only one value
				--sit_end;
				if (s >= sit_end->mData.Score()) break;
				sit_end->mIndex = sj;
				sit_end->mData.Reset(s);
				if (sit == sit_end) break;
			}
		}
		matrix.SortColumns(); // ###CBD TODO this is relatively slow
	}

	/// <summary> extract from a matrix a set of oriented edges between pairs of marker IDs.</summary>
	template<class T>
	static void GetDrawableEdges(std::set<std::pair<cUID, cUID> > &edges, 
		const cUIDMatrix<sIndexDataPair<T> > &matrix,
		const int minFrames,
		const double maxMean,
		const double maxDev)
	{
		edges.clear();
		for (int si = 0; si != matrix.size(); ++si)
		{
			cUID u = matrix.Index2Uid(si);
			for (auto &sit : matrix[si])
			{
				int sj = sit.mIndex;
				if (sj != -1 && sit.mData.Count() >= minFrames && sit.mData.Mean() < maxMean && sit.mData.Range() < maxDev)
				{
					cUID v = matrix.Index2Uid(sj);
					if (u < v) edges.insert(std::make_pair(u, v));
					else       edges.insert(std::make_pair(v, u));
				}
			}
		}
	}

	/// <summary> Learns and represents the transform of an animating (rigid) set of points </summary>
	class cBoneMatrix
	{
		size_t mCount;
		Eigen::MatrixX3d mInit;
		Eigen::Matrix4d mL2WMatrix;
	public:
		cBoneMatrix() : mCount(0) { mL2WMatrix.setZero(); }

		size_t Count() const { return mCount; }
		/// solve: [points.T;1] = mL2WMatrix * [mInit.T + mOffset;1]
		void PushFrame(const Eigen::MatrixX3d &points)
		{
			Eigen::Vector3d mean(points.colwise().mean());
			Eigen::MatrixX3d centred(points.rowwise() - mean.transpose());
			if (mCount == 0)
			{
				mInit = centred;
			}
			Eigen::Matrix3d M(centred.transpose() * mInit);
			auto svd = M.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
			Eigen::Matrix3d u = svd.matrixU(), vt = svd.matrixV().transpose();
			Eigen::Matrix3d r = u * vt;
			// repair mirror-image solution
			if (r.determinant() < 0)
			{
				u.col(2) *= -1;
				r = u * vt;
			}
			mL2WMatrix.block<3, 3>(0, 0) = r;
			mL2WMatrix.block<1, 4>(3, 0) << 0, 0, 0, 1;
			mL2WMatrix.block<3, 1>(0, 3) << mean[0],mean[1],mean[2];
			mCount++;
			double scale = 1.0 / mCount;
			mInit += (centred * r - mInit) * scale;
		}
		const Eigen::Matrix4d& GetL2W() const { return mL2WMatrix; }

	};

	/// <summary> Learns and represents the transform of a joint connecting two animating bones </summary>
	class cJointMatrix
	{
		size_t mCount;
		Eigen::Matrix4d mLeft0, mRight0;
		Eigen::Matrix4d mSum;
		Eigen::Vector4d mJoint;
	public:
		cJointMatrix() : mCount(0) {}
		size_t Count() const { return mCount; }
		void PushFrame(const Eigen::Matrix4d &left, const Eigen::Matrix4d &right)
		{
			// given two L2W matrices...
			if (mCount == 0)
			{
				mLeft0 = left.inverse();
				mRight0 = right.inverse();
				mSum = Eigen::Matrix4d::Zero();
			}
			Eigen::Matrix4d M = (left * mLeft0) - (right * mRight0);
			mSum += (M.transpose() * M);
			mCount++;
		}
		double Solve()
		{
			auto svd = mSum.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
			mJoint = svd.matrixU().col(3);
			mJoint /= mJoint(3);
			if (svd.singularValues()(0) > 1e10)
			{
				std::cerr << "out of range";
				return 1e10;
			}
			return svd.singularValues()(3)/ svd.singularValues()(0);
		}
		Eigen::Vector3d GetJointPosition(Eigen::Matrix4d &left, Eigen::Matrix4d &right) const
		{
			Eigen::Vector3d v1 = (left * (mLeft0 * mJoint)).segment<3>(0), v2 = (right * (mRight0 * mJoint)).segment<3>(0);
			return (v1 + v2)*0.5;
		}
	};

	/// <summary> cLiveGraph holds the state of a structure that reveals links between points
	/// (even unlabelled points with no model). Sticks are upgraded to bones, then joints, then skeletons.
	/// </summary>
	class cLiveGraph
	{
	public:
		typedef std::pair<cUID, cUID> tUIDPair;
		cLiveGraph(float threshold = 1.0f, int cols = 8, int minFrames = 20, double maxMean = 1.0, double maxDev = 0.1) :
			mNumFrames(0), mThreshold(threshold), mCols(cols), mMinFrames(minFrames), mMaxMean(maxMean), mMaxDev(maxDev) {}

		void GetDisplayBones(std::vector<Core::cVector3f> &points) const
		{
			for (const auto &bone : mBoneMatrices)
			{
				Eigen::Matrix4d mat = bone.second.GetL2W();
				double s = 0.1;
				Eigen::Vector4d o = mat * Eigen::Vector4d(0, 0, 0, 1);
				Eigen::Vector4d x = mat * Eigen::Vector4d(s, 0, 0, 1);
				Eigen::Vector4d y = mat * Eigen::Vector4d(0, s, 0, 1);
				Eigen::Vector4d z = mat * Eigen::Vector4d(0, 0, s, 1);
				Core::cVector3f po(o[0], o[1], o[2]);
				Core::cVector3f px(x[0], x[1], x[2]);
				Core::cVector3f py(y[0], y[1], y[2]);
				Core::cVector3f pz(z[0], z[1], z[2]);
				points.push_back(po); points.push_back(px);
				points.push_back(po); points.push_back(py);
				points.push_back(po); points.push_back(pz);
			}
			for (const auto &pair : mJoints)
			{
				const int b1 = pair.first, b2 = pair.second;
				Eigen::Matrix4d left = mBoneMatrices.at(b1).GetL2W();
				Eigen::Matrix4d right = mBoneMatrices.at(b2).GetL2W();
				Eigen::Vector3d p1 = mBoneMatrices.at(b1).GetL2W().col(3).segment<3>(0);
				Eigen::Vector3d p2 = mBoneMatrices.at(b2).GetL2W().col(3).segment<3>(0);
				Eigen::Vector3d o = mJointMatrices.at(pair).GetJointPosition(left, right);
				points.push_back(Core::cVector3f(p1[0],p1[1],p1[2]));
				points.push_back(Core::cVector3f(o[0], o[1], o[2]));
				points.push_back(Core::cVector3f(p2[0], p2[1], p2[2]));
				points.push_back(Core::cVector3f(o[0], o[1], o[2]));
			}
		}

		const std::set<tUIDPair> & ComputeEdges()
		{
			if (mEdges.size()) return mEdges;
			if (mBoneCliques.size() == 0)
			{
				// edges are made of sticks
				Core::GetDrawableEdges(mEdges, mStats, mMinFrames, mMaxMean, mMaxDev);
			}
			else
			{
				// edges are made of bones
				for (int i = 0; i < mBoneCliques.size(); ++i)
				{
					const auto &bone = mBoneCliques[i];
					auto first = bone.begin();
					for (int j = 1; j < bone.size(); ++j)
					{
						mEdges.insert(std::make_pair(*first, bone[j]));
						mEdges.insert(std::make_pair(bone[j], bone[j+1 == bone.size() ? 1 : j+1]));
					}
				}
			}
			return mEdges;
		}

		void AddMarkers(const cVec<cUID> &markerIDs,
			const cVec<cVector3f> &markerPositions)
		{
			Core::UpdateStatsMatrix(mStats, markerIDs, markerPositions, mThreshold, mCols);
			mEdges.clear();
			mNumFrames++;
			if (mNumFrames == 5000) InferBones();
			if (mNumFrames >= 5000) UpdateBoneMatrices(markerIDs, markerPositions);
			if (mNumFrames >= 5500) UpdateJointMatrices();
		}

		void UpdateBoneMatrices(const cVec<cUID> &markerIDs,
			const cVec<cVector3f> &markerPositions)
		{
			mHasBoneMatrices.clear();
			std::map<cUID, int> uid2Index;
			const int numMs = (int)markerIDs.size();
			for (int mi = 0; mi != numMs; ++mi)
				uid2Index[markerIDs[mi]] = mi;

			const int numBones = (int)mBoneCliques.size();
			for (int bi = 0; bi != numBones; ++bi)
			{
				const cVec<Core::cUID> &clique = mBoneCliques[bi];
				size_t count = 0;
				for (auto m : clique) count += uid2Index.count(m);
				if (count < clique.size()) continue; // all points required
				Eigen::MatrixX3d points(count, 3);
				for (int pi = 0; pi != count; ++pi)
				{
					auto &p = markerPositions[uid2Index[clique[pi]]];
					points.row(pi) << p[0], p[1], p[2];
				}
				mBoneMatrices[bi].PushFrame(points);
				mHasBoneMatrices.insert(bi);
			}
		}

		void UpdateJointMatrices()
		{
			for (auto it = mJointMatrices.begin(); it != mJointMatrices.end(); ++it)
			{
				const auto &pair = it->first;
				const int b1 = pair.first, b2 = pair.second;
				auto &joint = it->second;
				if (mHasBoneMatrices.count(b1) && mHasBoneMatrices.count(b2))
				{
					joint.PushFrame(mBoneMatrices[b1].GetL2W(), mBoneMatrices[b2].GetL2W());
					if (joint.Count() > 100)
					{
						double w = joint.Solve();
						std::cerr << b1 << ", " << b2 << ", " << w << std::endl;
						if (w < 0.00001)
						{
							mJoints.insert(pair);
						}
						else
						{
							mJoints.erase(pair);
						}
					}
				}
			}
		}

		void InferBones()
		{
			Core::GetDrawableEdges(mEdges, mStats, mMinFrames, mMaxMean, mMaxDev);
			// a bone is defined by a clique of three or more points of increasing UID, all connected by edges
			std::map<cUID, std::set<cUID> > edges;
			for (const auto &eit : mEdges) edges[eit.first].insert(eit.second);
			mNumBones = 0;
			for (auto &eit : mEdges)
			{
				const cUID A = eit.first, B = eit.second;
				std::set<cUID> &edgesA = edges[A], &edgesB = edges[B];
				std::vector<cUID> inter;
				std::set_intersection(edgesA.begin(), edgesA.end(), edgesB.begin(), edgesB.end(), std::back_inserter(inter));
				if (inter.size() == 0) continue;
				// found a bone; greedily build the largest fully-connected clique; remove all edges in the clique
				// this could fail if an edge is in two bones, which could happen if the edge is the single axis of rotation between the bones (eg two points across elbow or knee)
				const cUID C = inter[0];
				std::vector<cUID> clique;
				edgesA.erase(B); edgesA.erase(C); edgesB.erase(C);
				clique.push_back(A); clique.push_back(B); clique.push_back(C);
				mBoneCliques.AddRowItem(A);
				mBoneCliques.AddRowItem(B);
				mBoneCliques.AddRowItem(C);
				for (auto it = inter.begin()+1; it != inter.end(); ++it)
				{
					const cUID D = *it;
					if (mEdges.count(tUIDPair(C, D)))
					{
						for (cUID CI : clique) edges[CI].erase(D);
						clique.push_back(D);
						mBoneCliques.AddRowItem(D);
					}
				}
				mBoneCliques.EndRow();
				int n = (int)clique.size();
				mBoneMatrices[mNumBones] = cBoneMatrix();
				for (int i = 0; i < mNumBones; ++i)
				{
					bool canMatch = true;
					if (canMatch)
					{
						auto bp = std::make_pair(i, mNumBones);
						int m = (int)mBoneCliques[i].size();
						mJointMatrices[bp] = cJointMatrix();
					}
				}
				mNumBones++;
			}
		}

	private:
		int mNumFrames;
		float mThreshold;
		int mCols;
		int mMinFrames;
		double mMaxMean;
		double mMaxDev;
		cSparseStatsMatrix mStats;
		std::set<tUIDPair> mEdges;
		int mNumBones;
		cMatrix<cUID> mBoneCliques;
		std::set<int> mHasBoneMatrices;
		std::map<int, cBoneMatrix> mBoneMatrices;
		std::map<std::pair<int,int>, cJointMatrix> mJointMatrices;
		std::set<std::pair<int, int> > mJoints;
	};

	/// <summary>given edges and a vertex ordering, construct the labelGraph with only backward links,
	/// containing mean and inverse variance.</summary>
	static void MakeLabelGraph(cSparseEdgeStatsMatrix &labelGraph,
		const std::vector<std::vector<std::pair<int, cStats> > > &edges,
		const cVec<int> &ordering,
		const cVec<cUID> &mapIdx2UID,
		const cVec<cLabel> &mapIdx2Label)
	{
		labelGraph.clear();
		labelGraph.reserve(ordering.size());
		for (auto oit = ordering.begin(), oit_end = ordering.end(); oit != oit_end; ++oit)
		{
			int oi = *oit;
			labelGraph.AddUID(mapIdx2UID[oi]);
			labelGraph.SetLabel(mapIdx2Label[oi]);
			for (auto oit2 = ordering.begin(); oit2 != oit; ++oit2)
			{
				int oj = *oit2;
				for (auto eit = edges[oi].begin(); eit != edges[oi].end(); ++eit)
				{
					if (eit->first == oj)
					{
						const cStats &s = eit->second;
						float mean = float(s.Mean()), invVar = float(1.0 / s.Var());
						labelGraph.AddRowItem(sIndexDataPair<sEdgeStats>(int(oit2 - ordering.begin()), sEdgeStats(mean, invVar)));
						break;
					}
				}
			}
			labelGraph.EndRow();
		}
	}


	///<summary>create a graph of labels with backward-linking edges and their mean distances and inverse variances
	///</summary>
	static void MakeLabelGraph(cSparseEdgeStatsMatrix &labelGraph,
		const cSparseStatsMatrix &modelMatrix,
		const int countThreshold,
		const float meanThreshold,
		const float devThreshold )
	{
		// given a modelMatrix: the statistics of the distances between pairs of markers
		// and assuming that the markers have labels...
		// choose an ordering for the labels (indices into modelMatrix)
		std::vector<std::vector<std::pair<int, cStats> > > edges(modelMatrix.size());
		std::vector<int> ordering;
		// compute the edges and the ordering using the model and various heuristics
		{
			std::vector<int> edgeCount(modelMatrix.size(), 0);
			for (int i = 0; i < modelMatrix.size(); ++i)
			{
				if (modelMatrix.Index2Label(i) == cLabel::kInvalid) continue;
				for (const auto &mj : modelMatrix[i])
				{
					const int j = mj.mIndex;
					const cStats &cs = mj.mData;
					if (modelMatrix.Index2Label(j) == cLabel::kInvalid) continue;
					if (cs.Count() > countThreshold && cs.Mean() < meanThreshold && cs.Range() < devThreshold)
					{
						edgeCount[i]++;
						edgeCount[j]++;
						edges[i].push_back(std::make_pair(j, cs));
						edges[j].push_back(std::make_pair(i, cs));
					}
				}
			}
			while (true)
			{
				auto eit = std::max_element(edgeCount.begin(), edgeCount.end());
				if (eit == edgeCount.end() || *eit == 0) break;
				const int index = int(eit - edgeCount.begin());
				*eit = 0;
				for (auto &eii : edges[index])
				{
					if (edgeCount[eii.first]) edgeCount[eii.first] += 10; // back edges count 10x
				}
				ordering.push_back(index);
			}
		}
		MakeLabelGraph(labelGraph, edges, ordering, modelMatrix.GetIdx2UID(), modelMatrix.GetIdx2Label());
	}

	///<summary>
	/// given a labelGraph (model) and a markerGraph (data), attempt to find the labels that best fit the model to the data.
	/// every edge in the model must have a match in the data. the score is sum of normalized Euclidean square-distances.
	/// edges with a missing marker score penalty; and hypotheses which are worse than the best by penalty2 are culled.
	/// maxHypotheses limits the available workspace.
	///</summary>
	static void LabelMarkers(std::vector<int> &hypotheses,
		std::vector<float> &hypothesisScores,
		const cSparseEdgeStatsMatrix &labelGraph,
		const cSparseFloatMat &markerGraph,
		const int maxHypotheses = 5000, 
		const float penalty = 20.0f, 
		const float penalty2 = 40.0f)
	{
		const int numLs = labelGraph.size();
		const int numMs = markerGraph.size();
		std::vector<float> dist(numMs*numMs, 0.0f);
		std::vector<std::vector<int> > mms1(numMs + 1);
		auto mms = mms1.begin() + 1; // so that mms[-1] yields all the markers
		std::vector<std::vector<float>::iterator> disti(numMs);
		for (int i = 0; i < numMs; ++i)
		{
			disti[i] = dist.begin() + i*numMs;
			mms[-1].push_back(i);
			for (auto &it : markerGraph[i])
			{
				const int j = it.mIndex;
				disti[i][j] = it.mData; // markerGraph contains [i][j] and [j][i]
				mms[i].push_back(j);
			}
		}
		std::vector<int> hs(maxHypotheses * numLs);
		std::vector<int> ht(maxHypotheses * numLs);
		std::vector<sIndexFloat> ss; ss.reserve(maxHypotheses);
		std::vector<sIndexFloat> st; st.reserve(maxHypotheses);
		// initial state: a single empty hypothesis
		ss.push_back(sIndexFloat(0, 0.0f));
		std::vector<unsigned char> test(numMs + 1); // whether a marker was already used in the hypothesis
		auto testi = test.begin() + 1; // so that testi[-1] = 0 can be accessed
		for (int li = 0; li < numLs; ++li) // for each label
		{
			const auto lvec = labelGraph[li];
			const float penaltyMissingMarker = penalty * (float)lvec.size();
			const auto ht0 = ht.begin(), ht1 = ht.end() - (numMs + 1)*(li + 1);
			float minSc = 1e10f;
			auto hti = ht.begin();
			for (const auto &ssi : ss) // for each hypothesis
			{
				if (ssi.mData > minSc) break;
				if (hti >= ht1) break; // out of workspace; TODO perhaps could deal with this case better
				const auto hsi = hs.begin() + ssi.mIndex;
				// missing marker hypothesis
				const float scoreMissingMarker = ssi.mData + penaltyMissingMarker;
				if (scoreMissingMarker < minSc)
				{
					if (scoreMissingMarker + penalty2 < minSc) minSc = scoreMissingMarker + penalty2;
					st.push_back(sIndexFloat(int(hti - ht0), scoreMissingMarker));
					std::copy(hsi, hsi + li, hti);
					hti += li;
					*hti++ = -1;
				}
				// don't allow a marker to be used twice
				std::fill(test.begin(), test.end(), 0);
				for (int i = 0; i < li; ++i) testi[hsi[i]] = 1;
				testi[-1] = 0;
				// only consider markers that could match the first edge
				const std::vector<int> &mm = mms[lvec.size() ? hsi[lvec[0].mIndex] : -1];
				for (const int mi : mm)
				{
					if (testi[mi]) continue; // marker already in the hypothesis
					const auto &dist_mi = disti[mi];
					// decide whether to extend the hypothesis by assigning li -> mi
					float sc = ssi.mData;
					for (auto &lit : lvec) // for each edge
					{
						const int mj = hsi[lit.mIndex]; // NB lit->mIndex < li
						if (mj == -1) { sc += penalty; continue; }
						const float d = dist_mi[mj];
						if (d == 0.0f) { sc = 1e10f; break; } // abort if edge no exist
						//if (d == 0.0f) { sc += penalty; continue; } // TODO compare performance (I expect this is worse)
						sc += lit.mData.GetScore(d);
						if (sc > minSc) break;
					}
					if (sc < minSc)
					{
						if (sc + penalty2 < minSc) minSc = sc + penalty2;
						st.push_back(sIndexFloat(int(hti - ht0), sc));
						std::copy(hsi, hsi + li, hti);
						hti += li;
						*hti++ = mi;
					}
				}
			}
			// copy the scores while culling bad hypotheses
			ss.clear();
			for (const auto &sti : st)
				if (sti.mData < minSc) ss.push_back(sti);
			std::sort(ss.begin(), ss.end()); // sort the hypotheses by score
			hs.swap(ht); // swaps pointers only
			st.clear();
		}
		for (int i = 0; i < hypothesisScores.size(); ++i)
		{
			if (i >= ss.size()) { hypothesisScores[i] = std::numeric_limits<float>::infinity(); continue; }
			hypothesisScores[i] = ss[i].mData;
			const auto hit = hs.begin() + ss[i].mIndex;
			std::copy(hit, hit + numLs, hypotheses.begin() + numLs*i);
		}
	}

	///<summary>
	/// given a labelGraph (model) and a markerGraph (data), attempt to find the labels that best fit the model to the data.
	/// every edge in the model must have a match in the data. the score is sum of normalized Euclidean square-distances.
	/// edges with a missing marker score penalty; and hypotheses which are worse than the best by penalty2 are culled.
	/// maxHypotheses limits the available workspace.
	///</summary>
	static float LabelMarkers(std::vector<cLabel> &returnLabels,
		const cSparseEdgeStatsMatrix &labelGraph,
		const cSparseFloatMat &markerGraph,
		const int maxHypotheses = 5000,
		const float penalty = 20.0f,
		const float penalty2 = 40.0f)
	{
		const int numLs = labelGraph.size();
		const int numMs = markerGraph.size();
		std::vector<int> hypothesis(numLs);
		std::vector<float> hypothesisScore(1);
		LabelMarkers(hypothesis, hypothesisScore, labelGraph, markerGraph, maxHypotheses, penalty, penalty2);
		returnLabels.resize(numMs);
		std::fill(returnLabels.begin(), returnLabels.end(), cLabel::kInvalid);
		for (int li = 0; li < numLs; ++li) // for each label
		{
			int mi = hypothesis[li];
			if (mi != -1) returnLabels[mi] = labelGraph.Index2Label(li);
		}
		return hypothesisScore[0];
	}

	struct sChannelInfo
	{
		int                      parent;
		eChannelType             type;
		cRTf                     localMatrix; // only if type < 0
		std::vector<sIndexFloat> dofs;
		sChannelInfo() {}
	};
	struct sLabelInfo
	{
		int               parent;
		cVector3f         offset;
		float             weight;
		sLabelInfo() {}
	};
	struct sLabelTarget
	{
		cVector3f position;
		int       assignment;
		cVector3f markerPosition; // only if assignment != -1
		sLabelTarget() { assignment = -1; }
	};

	///<summary> A skeleton class, used to represent the mathematics of skeleton evaluation.</summary>
	class cSkeleton
	{
	public:
		int NumChannels() const { return (int)mChannelInfo.size(); }
		int NumDofs() const { return (int)mDofLimits.size(); }
		int NumLabels() const { return (int)mLabelInfo.size(); }

		template<class V1, class V2, class V3>
		void UpdatePoseV(V1 &channelMats, V2 &labels, V3 &dofValues)
		{
			UpdatePose(cVec<cRTf>(channelMats), cVec<sLabelTarget>(labels), cVec<float>(dofValues));
		}

		///<summary> compute the ChannelMats from the DofValues (may be clipped) </summary>
		void UpdatePose(cVec<cRTf> &channelMats, 
			cVec<sLabelTarget> &labels,
			cVec<float> &dofValues) const
		{
			const int numDofs = (int)dofValues.size();
			for (int di = 0; di != numDofs; ++di)
			{
				std::pair<float, float> limits = mDofLimits[di];
				float &v = dofValues[di];
				if (v < limits.first) v = limits.first;
				if (v > limits.second) v = limits.second;
			}
			const cRTf root;
			for (int ci = 0, ci_end = NumChannels(); ci != ci_end; ++ci)
			{
				const auto &chan = mChannelInfo[ci];
				const int pi = chan.parent;
				cRTf &M = channelMats[ci];
				M = (pi == -1 ? root : channelMats[pi]);
				if (chan.type < 0) M.ApplyMatOp(chan.type, chan.localMatrix);
				else
				{
					double cv = 0.0;
					for (const auto &it : chan.dofs)
						cv += dofValues[it.mIndex] * it.mData;
					M.ApplyChanOp(chan.type, (float)cv);
				}
			}
			for (int li = 0, li_end = (int)mLabelInfo.size(); li != li_end; ++li)
			{
				const auto &label = mLabelInfo[li];
				labels[li].position = channelMats[label.parent] * label.offset;
			}
		}

		///<summary> set the target marker assignments and positions </summary>
		void SetTarget(cVec<sLabelTarget> &labels, 
			const cVec<int> &assignment, 
			const cVec<cVector3f> &markerPositions) const
		{
			ASSERT(NumLabels() == assignment.size());
			ASSERT(NumLabels() == labels.size());
			for (int li = 0, li_end = (int)mLabelInfo.size(); li != li_end; ++li)
			{
				const int mi = assignment[li];
				labels[li].assignment = mi;
				if (mi != -1) labels[li].markerPosition = markerPositions[mi];
			}
		}

		///<summary> compute the sum-of-squares residual between labels and their assigned markers</summary>
		float ResidualNorm2(const cVec<sLabelTarget> &labels) const
		{
			double s = 0.0;
			for (int li = 0, li_end = (int)mLabelInfo.size(); li != li_end; ++li)
			{
				const auto &label = mLabelInfo[li];
				const int mi = labels[li].assignment;
				const float lw = label.weight;
				if (mi == -1 || lw == 0.0f) continue; // ignore this label
				s += labels[li].markerPosition.DistanceSquared(labels[li].position) * (lw*lw);
			}
			return (float)s;
		}

		void ComputeResidual(Eigen::VectorXf &residual,
			const cVec<sLabelTarget> &labels) const
		{
			residual.setZero();
			for (int li = 0, li_end = (int)mLabelInfo.size(); li != li_end; ++li)
			{
				const auto &label = mLabelInfo[li];
				const auto &labelTarget = labels[li];
				const int mi = labelTarget.assignment, li3 = li * 3;
				const float lw = label.weight;
				if (mi == -1 || lw == 0.0f) continue; // ignore this label
				cVector3f v((labelTarget.markerPosition - labelTarget.position) * lw);
				residual.segment<3>(li3) << v[0], v[1], v[2];
			}
		}

		void Test(cVec<float> &dofValues,
			cVec<sLabelTarget> &labels) const
		{
			std::vector<cRTf> channelMatsV(NumChannels());
			cVec<cRTf> channelMats(channelMatsV);
			const int numLabels = NumLabels();
			const int numDofs = (int)dofValues.size();
			Eigen::VectorXf residual(numLabels * 3); // for numerical differentiation
			Eigen::MatrixXf JT(numDofs, numLabels * 3); // for numerical differentiation

			UpdatePose(channelMats, labels, dofValues);
			Eigen::VectorXf JTr2(numDofs);
			Eigen::MatrixXf JTJ2(numDofs, numDofs);

			ComputeJTResidualAndJTJ(JTr2, JTJ2, labels, channelMats);

			ComputeResidual(residual, labels);
			Eigen::VectorXf dresidual = Eigen::VectorXf::Zero(residual.size());
			for (int di = 0; di != numDofs; ++di)
			{
				const float delta = std::max(fabsf(dofValues[di] * 1e-3f), 1e-3f);
				std::vector<float> dofValues2(dofValues.begin(), dofValues.end());
				dofValues2[di] += delta;
				UpdatePose(channelMats, labels, cVec<float>(dofValues2));
				ComputeResidual(dresidual, labels);
				JT.row(di) = (dresidual - residual) / -delta;
			}
			Eigen::VectorXf JTr = JT * residual;
			Eigen::MatrixXf JTJ = JT * JT.transpose();

			float d3 = (JTr - JTr2).squaredNorm();
			float d4 = (JTJ - JTJ2).squaredNorm();
			ASSERT(d3 < 0.005);
			ASSERT(d4 < 0.01);
		}

		///<summary> compute the derivative of the residual with respect to channels and then dofs (Jacobian, J).
		/// returns J^T * residual and J^T * J. </summary>
		void ComputeJTResidualAndJTJ(
			Eigen::VectorXf &JTr, Eigen::MatrixXf &JTJ,
			const cVec<sLabelTarget> &labels,
			const cVec<cRTf> &channelMats) const
		{
			JTr.setZero();
			JTJ.setZero();
			std::vector<std::pair<int, cVector3f> > wds;
			for (int li = 0, li_end = (int)mLabelInfo.size(); li != li_end; ++li)
			{
				const auto &label = mLabelInfo[li];
				const auto &labelTarget = labels[li];
				const int mi = labelTarget.assignment;
				const float lw = label.weight;
				if (mi == -1 || lw == 0.0f) continue; // ignore this label
				cVector3f v((labelTarget.markerPosition - labelTarget.position) * lw);
				wds.clear();
				for (int ci = label.parent; ci != -1;)
				{
					const sChannelInfo &chan = mChannelInfo[ci];
					if (chan.type >= 0)
					{
						cVector3f d(channelMats[ci].DChanOp(chan.type, labelTarget.position));
						const float d_dot_v(d.Dot(v));
						for (const auto &it : chan.dofs)
						{
							const int di = it.mIndex;
							const float s = it.mData * lw;
							cVector3f wd(d * s);
							JTr(di) += d_dot_v * s;
							auto JTJ_row = JTJ.row(di);
							auto JTJ_col = JTJ.col(di);
							for (const auto &wit : wds)
							{
								const int dj = wit.first;
								const float d2 = wd.Dot(wit.second);
								JTJ_row(dj) += d2;
								JTJ_col(dj) += d2;
							}
							wds.push_back(std::make_pair(di, wd));
							JTJ_row(di) += wd.Dot(wd);
						}
					}
					ci = chan.parent;
				}
			}
		}

		template<class A>
		void ComputeJTResidualAndJTJDiag(
			Eigen::VectorXf &JTr, A &JTJDiag,
			const cVec<sLabelTarget> &labels,
			const cVec<cRTf> &channelMats) const
		{
			JTr.setZero();
			JTJDiag.setZero();
			std::vector<cVector3f> wds(JTJDiag.size());
			for (int li = 0, li_end = (int)mLabelInfo.size(); li != li_end; ++li)
			{
				const auto &label = mLabelInfo[li];
				const auto &labelTarget = labels[li];
				const int mi = labelTarget.assignment;
				const float lw = label.weight;
				if (mi == -1 || lw == 0.0f) continue; // ignore this label
				cVector3f v((labelTarget.markerPosition - labelTarget.position) * lw);
				for (auto &v : wds) v[0] = v[1] = v[2] = 0;
				for (int ci = label.parent; ci != -1;)
				{
					const sChannelInfo &chan = mChannelInfo[ci];
					if (chan.type >= 0)
					{
						cVector3f d(channelMats[ci].DChanOp(chan.type, labelTarget.position));
						const float d_dot_v(d.Dot(v));
						for (const auto &it : chan.dofs)
						{
							const int di = it.mIndex;
							const float s = it.mData * lw;
							cVector3f wd(d * s);
							JTr(di) += d_dot_v * s;
							float &sum = JTJDiag[di];
							const float d2 = wd.Dot(wds[di]);
							sum += 2*d2; // cross term is doubled: (a+b)^2 = a^2 + b^2 + 2 a b
							wds[di] += wd;
							sum += wd.Dot(wd);
						}
					}
					ci = chan.parent;
				}
			}
		}


		template<class V1, class V2>
		float IKSolveV(V1 &dofValues, V2 &labels, const int numFastIts, const int numSlowIts)
		{
			return IKSolve(cVec<float>(dofValues), cVec<sLabelTarget>(labels), numFastIts, numSlowIts);
		}

		///<summary> after SetTarget, iteratively update the skeleton pose to match the marker positions </summary>
		float IKSolve(cVec<float> &dofValues, 
			cVec<sLabelTarget> &labels, 
			const int numFastIts, const int numSlowIts) const
		{
			float prevRes = std::numeric_limits<float>::infinity();
			const int numDofs = NumDofs();
			const int numLabels = NumLabels();
			ASSERT(dofValues.size() == numDofs);
			// TODO this will suffer from gimbal lock; need to prep 3-axis rotations
			// eg: L ry rx rz; copy chanMats after initial rz into ry and rx; set ry=rx=rz=0; then solve;
			// then decompose difference matrix rz/L into ry rx rz
			Eigen::VectorXf JTr(numDofs), delta(numDofs);
			Eigen::MatrixXf JTJ(numDofs, numDofs);
			auto &JTJDiag = JTJ.diagonal().array();
			std::vector<cRTf> channelMatsV(NumChannels());
			cVec<cRTf> channelMats(channelMatsV);
			for (int it = 0; it < numFastIts; ++it)
			{
				UpdatePose(channelMats, labels, dofValues);
				//Test(dofValues, labels);
				// TODO JT doesn't take account of dof limits
				ComputeJTResidualAndJTJDiag(JTr, JTJDiag, labels, channelMats);
				JTJDiag += 0.05f; // damped least squares
				JTJDiag *= 1.05f; // damped least squares
				delta = Eigen::VectorXf(JTr.array() / JTJDiag * 0.1f);
				const float res = JTr.squaredNorm();
				//ASSERT(res < prevRes); // TODO monotonic convergence not guaranteed! for debugging
				prevRes = res;
				for (int di = 0; di != numDofs; ++di)
					dofValues[di] += delta(di);
			}
			for (int it = 0; it < numSlowIts; ++it)
			{
				UpdatePose(channelMats, labels, dofValues);
				//Test(dofValues, labels);
				// TODO JT doesn't take account of dof limits
				ComputeJTResidualAndJTJ(JTr, JTJ, labels, channelMats);
				JTJDiag += 0.05f; // damped least squares
				JTJDiag *= 1.05f; // damped least squares
				delta = JTJ.llt().solve(JTr);
				const float res = JTr.squaredNorm();
				//ASSERT(res < prevRes); // TODO monotonic convergence not guaranteed! for debugging
				prevRes = res;
				for (int di = 0; di != numDofs; ++di)
					dofValues[di] += delta(di);
			}
			// ensure the final pose reflects the final dof values
			UpdatePose(channelMats, labels, dofValues);
			return prevRes;
		}

		void SetLabelGraph(const cSparseEdgeStatsMatrix &labelGraph)
		{
			mLabelGraph = labelGraph;
		}

		template<class V1, class V2>
		void BootV(V1 &dofValues, V2 &labels, 
			const cVec<cVector3f> &markerPositions,
			float threshold = 0.5f,
			int maxHypotheses = 5000,
			float penalty = 20.0f,
			float penalty2 = 40.0f,
			int ikIterations = 10)
		{
			Boot(cVec<float>(dofValues), cVec<sLabelTarget>(labels), markerPositions, threshold, maxHypotheses, penalty, penalty2, ikIterations);
		}

		///<summary> generate an initial pose for the skeleton from the marker positions </summary>
		void Boot(cVec<float> &dofValues,
			cVec<sLabelTarget> &labels,
			const cVec<cVector3f> &markerPositions,
			float threshold = 0.5f,
			int maxHypotheses = 5000,
			float penalty = 20.0f,
			float penalty2 = 40.0f,
			int numFastIKIts = 3,
			int numSlowIKIts = 1) const
		{
			cSparseFloatMatrix markerGraph;
			cHashCloud<cVector3f>(markerPositions, threshold).MakeDistanceMatrix(markerGraph, markerPositions, false);
			// TODO deal with mirror-image solutions, partial labellings, etc etc
			std::vector<int> assignments(NumLabels());
			std::vector<float> score(1);
			Core::LabelMarkers(assignments, score, mLabelGraph, markerGraph, maxHypotheses, penalty, penalty2);
			// TODO from labels to initial pose
			SetTarget(labels, assignments, markerPositions);
			IKSolve(dofValues, labels, numFastIKIts, numSlowIKIts);
		}
	public:
		std::vector<sChannelInfo>             mChannelInfo; // the skeleton definition
		std::vector<std::pair<float, float> > mDofLimits; // the dof limits (must be filled)
		std::vector<sLabelInfo>               mLabelInfo; // the label definition
		cSparseEdgeStatsMatrix                mLabelGraph; // the label graph definition, used for booting
	};

	template<class T>
	class cHashCloudList2D
	{
		cHashCloudList2D(const cHashCloudList2D&) {} // private
	public:
		cHashCloudList2D(
			const cVec<cP<T> > &cameras,
			const cMat<cVector2<T> > &detections,
			const float threshold,
			const bool undistort = true) : mCameras(cameras), mUndistorted(undistort)
		{
			detections.CopyTo(mDetections);
			mHashClouds.reserve(size());
			for (int ci = 0; ci != size(); ++ci)
			{
				const cP<T> &cam = cameras[ci];
				auto &det = mDetections[ci];
				if (mUndistorted)
					for (auto &d : det) cam.Undistort(d[0], d[1]);
				mHashClouds.push_back(new cHashCloud<cVector2<T> >(det, mUndistorted ? threshold * cam.UndistortScale() : threshold));
			}
		}
		~cHashCloudList2D() { for (auto *hc : mHashClouds) delete hc;  }

		int size() const
		{
			return (int)mCameras.size();
		}

		///<summary> project labelled 3D points in all the cameras, assign the projections to the nearest detections
		/// and fill the labels of the detections. </summary>
		int ProjectAssign(
			const cVec<cVector3<T> > &points,
			const bool allowZero = true) const
		{
			const int numCameras = size();
			int count = 0;
			mAssignmentP2Ds.SetShape(numCameras, (int)points.size(), -1);
			for (int ci = 0; ci != numCameras; ++ci)
			{
				const auto &camera = mCameras[ci];
				auto &p2d = mAssignmentP2Ds[ci];
				const auto &hashCloud = *mHashClouds[ci];
				mProjections.clear();
				if (mUndistorted)
					for (const auto &pt : points)
						mProjections.push_back(camera.ProjectNoDistort(pt));
				else
					for (const auto &pt : points)
						mProjections.push_back(camera.Project(pt));
				hashCloud.MakeDistanceMatrix(mMatrix, mProjections, allowZero);
				HungarianAssignment(mWorkspace, p2d, mMatrix, hashCloud.Threshold());
				for (auto di : p2d)
					if (di != -1) count++;
			}
			return count;
		}

		///<summary> assign projected points to the nearest detections and fill the labels of the detections. </summary>
		const cMatrix<int>& Assign(
			const cMat<cVector2<T> > &points,
			const cMat<int> &detectionLabels,
			int &nextID,
			const bool allowZero = true) const
		{
			const int numCameras = size();
			mAssignmentP2Ds.SetShape(points, -1);
			for (int ci = 0; ci != numCameras; ++ci)
			{
				auto &p2d = mAssignmentP2Ds[ci];
				const auto &dls = detectionLabels[ci];
				const auto &hashCloud = *mHashClouds[ci];
				hashCloud.MakeDistanceMatrix(mMatrix, points[ci], allowZero);
				HungarianAssignment(mWorkspace, p2d, mMatrix, hashCloud.Threshold());
				for (auto &di : p2d)
					di = (di == -1 ? (nextID == -1 ? -1 : nextID++) : dls[di]);
			}
			return mAssignmentP2Ds;
		}

		void Apply(
			cMatrix<int> &detectionLabels,
			const cVec<int> &pointLabels) const
		{
			const int numCameras = size();
			detectionLabels.SetShape(mDetections, -1);
			for (int ci = 0; ci != numCameras; ++ci)
			{
				auto &dls = detectionLabels[ci];
				const auto &p2d = mAssignmentP2Ds[ci];
				for (int pi = 0; pi != p2d.size(); ++pi)
				{
					const int di = p2d[pi];
					if (di != -1)
						dls[di] = pointLabels[pi];
				}
			}
		}
	private:
		const cVec<cP<T> >                     mCameras; // a reference
		cMatrix<cVector2<T> >                  mDetections; // a copy; could be held undistorted
		bool                                   mUndistorted;
		std::vector<cHashCloud<cVector2<T> >*> mHashClouds;
		mutable cMatrix<int>                   mAssignmentP2Ds;
		mutable cSparseFloatMatrix             mMatrix;
		mutable std::vector<float>             mWorkspace;
		mutable std::vector<cVector2<T> >      mProjections;
	};

	template<class T>
	class cLensSolver
	{
		cK<T> &mK;
		double m00, m01, m02, m03, m04, m11, m12, m13, m14, m22, m23, m24, m33, m34, m44, m0, m1, m2, m3, m4;
		int mCount;
	public:
		cLensSolver(cK<T> &K) : mK(K) { Clear();  }
		cLensSolver(cP<T> &P) : mK(P.mK) { Clear(); }

		void Clear()
		{
			m00 = m01 = m02 = m03 = m04 = m11 = m12 = m13 = m14 = m22 = m23 = m24 = m33 = m34 = m44 = m0 = m1 = m2 = m3 = m4 = 0;
			mCount = 0;
		}

		int Count() const { return mCount; }

		///<summary> accumulate the equation: x0 * mK1 + x1 * mK2 + x2 * mK3 + x3 * mTx + x4 * mTy = y0 </summary>
		void Add(const double x0, const double x1, const double x2, const double x3, const double x4, const double y0)
		{
			if (y0 == 0.0) return;
			const double x00 = x0*x0, x01 = x0*x1, x02 = x0*x2, x03 = x0*x3, x04 = x0*x4;
			const double x11 = x1*x1, x12 = x1*x2, x13 = x1*x3, x14 = x1*x4, x22 = x2*x2, x23 = x2*x3, x24 = x2*x4;
			const double x33 = x3*x3, x34 = x3*x4, x44 = x4*x4;
			m00 += x00; m01 += x01; m02 += x02; m03 += x03; m04 += x04;
			m11 += x11; m12 += x12; m13 += x13; m14 += x14; m22 += x22; m23 += x23; m24 += x24;
			m33 += x33; m34 += x34; m44 += x44;
			m0 += x0 * y0; m1 += x1 * y0; m2 += x2 * y0; m3 += x3 * y0; m4 += x4 * y0;
			mCount++;
		}

		/// <summary> add equations to solve for the function to undistort from d to p </summary>
		void Add(const double d0, const double d1, const double p0, const double p1)
		{
			const double d01 = d0*d1, d00 = d0*d0, d11 = d1*d1;
			const double dr2 = d00 + d11, hdr2 = 0.5*dr2;
			const double dr4 = dr2*dr2, dr6 = dr4*dr2;
			// (d0*dr2) * mK1 + (d0*dr4) * mK2 + (d0*dr6) * mK3 + (hdr2 + d00) * mTx +        (d01) * mTy = (p0-d0)
			// (d1*dr2) * mK1 + (d1*dr4) * mK2 + (d1*dr6) * mK3 +        (d01) * mTx + (hdr2 + d11) * mTy = (p1-d1)
			Add(d0*dr2, d0*dr4, d0*dr6, hdr2 + d00, d01, p0 - d0);
			Add(d1*dr2, d1*dr4, d1*dr6, d01, hdr2 + d11, p1 - d1);
		}

		///<summary> d is a detection (pixels, distorted), p is a projection (without distortion) </summary>
		void Add(const cVector2<T> &d, const cVector2<T> &p)
		{
			if (p[0] >= 1e10) return; // bad point
			const double u0 = (d[0] - mK.mOx) * mK.mFx_Inv, u1 = (d[1] - mK.mOy) * mK.mFy_Inv;
			Add(u0, u1, p[0], p[1]);
		}

		///<summary> return a 5x5 matrix such that M x = RHS() gives the lens distortion parameters </summary>
		Eigen::Matrix<double, 5, 5, Eigen::RowMajor> Matrix() const
		{
			Eigen::Matrix<double, 5, 5, Eigen::RowMajor> ret;
			ret <<
				m00, m01, m02, m03, m04,
				m01, m11, m12, m13, m14,
				m02, m12, m22, m23, m24,
				m03, m13, m23, m33, m34,
				m04, m14, m24, m34, m44;
			return ret;
		}

		Eigen::Matrix<double, 5, 1> RHS() const
		{
			Eigen::Matrix<double, 5, 1> ret;
			ret << m0, m1, m2, m3, m4;
			return ret;
		}

		void Solve(bool inv = true)
		{
			if (mCount < 5)
			{
				if (inv)
					mK.mK1_Inv = mK.mK2_Inv = mK.mK3_Inv = mK.mTx_Inv = mK.mTy_Inv = T(0);
				else
					mK.mK1 = mK.mK2 = mK.mK3 = mK.mTx = mK.mTy = T(0);
			}
			else
			{
				Eigen::Matrix<double, 5, 1> ip = Matrix().llt().solve(RHS());
				if (inv)
				{
					mK.mK1_Inv = T(ip(0));
					mK.mK2_Inv = T(ip(1));
					mK.mK3_Inv = T(ip(2));
					mK.mTx_Inv = T(ip(3));
					mK.mTy_Inv = T(ip(4));
				}
				else
				{
					mK.mK1 = T(ip(0));
					mK.mK2 = T(ip(1));
					mK.mK3 = T(ip(2));
					mK.mTx = T(ip(3));
					mK.mTy = T(ip(4));
				}
			}
		}

		void ComputeLensInverse(const int spans = 20)
		{
			Clear();

			mK.mFx_Inv = T(1) / mK.mFx;
			mK.mFy_Inv = T(1) / mK.mFy;
			const double scalex = 2.0 * mK.mOx / (spans - 1), scaley = 2.0 * mK.mOy / (spans - 1);
			for (int y = 0; y < spans; ++y)
			{
				for (int x = 0; x < spans; ++x)
				{
					const double t0 = scalex * x, t1 = scaley * y;
					// this is just mK.Undistort written as doubles
					const double v0 = (t0 - mK.mOx) * mK.mFx_Inv, v1 = (t1 - mK.mOy) * mK.mFy_Inv;
					const double v01 = v0*v1, v00 = v0*v0, v11 = v1*v1;
					const double r2 = v00 + v11, hr2 = 0.5f*r2;
					const double s = 1.0 + r2*(mK.mK1_Inv + r2 * (mK.mK2_Inv + r2 * mK.mK3_Inv));
					const double u0 = v0 * s + (hr2 + v00) * mK.mTx_Inv + v01 * mK.mTy_Inv;
					const double u1 = v1 * s + (hr2 + v11) * mK.mTy_Inv + v01 * mK.mTx_Inv;
					Add(u0, u1, v0, v1);
				}
			}
			Solve(false);
		}

	};

	template<class T>
	class cCameraSolver
	{
		cP<T> &mP;
		double m00, m01, m11, m02, m12, m22, m03, m13, m23, m33,
			m08, m09, m19, m010, m110, m210, m011, m111, m211, m311,
			m48, m49, m59, m410, m510, m610, m411, m511, m611, m711,
			m88, m89, m99, m810, m910, m1010, m811, m911, m1011, m1111;
		int mCount;
	public:

		cCameraSolver(cP<T> &P) : mP(P) { Clear();  }

		void Clear()
		{
			m00 = m01 = m11 = m02 = m12 = m22 = m03 = m13 = m23 = m33 = 0;
			m08 = m09 = m19 = m010 = m110 = m210 = m011 = m111 = m211 = m311 = 0;
			m48 = m49 = m59 = m410 = m510 = m610 = m411 = m511 = m611 = m711 = 0;
			m88 = m89 = m99 = m810 = m910 = m1010 = m811 = m911 = m1011 = m1111 = 0;
			mCount = 0;
		}

		int Count() const { return mCount; }

		///<summary> return a matrix M such that the nontrivial solution to M x = 0 gives the camera matrix </summary>
		Eigen::Matrix<double, 12, 12, Eigen::RowMajor> Matrix() const
		{
			Eigen::Matrix<double, 12, 12, Eigen::RowMajor> mat;
			mat <<	 m00,  m01,  m02,  m03,    0,    0,    0,    0,  m08,  m09,  m010,  m011, 
					 m01,  m11,  m12,  m13,    0,    0,    0,    0,  m09,  m19,  m110,  m111, 
					 m02,  m12,  m22,  m23,    0,    0,    0,    0, m010, m110,  m210,  m211, 
					 m03,  m13,  m23,  m33,    0,    0,    0,    0, m011, m111,  m211,  m311, 
					   0,    0,    0,    0,  m00,  m01,  m02,  m03,  m48,  m49,  m410,  m411, 
					   0,    0,    0,    0,  m01,  m11,  m12,  m13,  m49,  m59,  m510,  m511, 
					   0,    0,    0,    0,  m02,  m12,  m22,  m23, m410, m510,  m610,  m611, 
					   0,    0,    0,    0,  m03,  m13,  m23,  m33, m411, m511,  m611,  m711, 
					 m08,  m09, m010, m011,  m48,  m49, m410, m411,  m88,  m89,  m810,  m811,
					 m09,  m19, m110, m111,  m49,  m59, m510, m511,  m89,  m99,  m910,  m911,
					m010, m110, m210, m211, m410, m510, m610, m611, m810, m910, m1010, m1011,
					m011, m111, m211, m311, m411, m511, m611, m711, m811, m911, m1011, m1111;
			return mat;
		}

		///<summary> coerce the smallest eigenvector into a 3x4 matrix </summary>
		bool Solve()
		{
			if (mCount < 12) return false;
			Eigen::Matrix<double, 12, 1> v;
			Eigen::Matrix<double, 12, 12, Eigen::RowMajor> M = Matrix();
			auto svd = M.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
			v = svd.matrixU().col(11);
			mP.UpdateP(Eigen::Matrix<double, 3, 4, Eigen::RowMajor>(v.data()));
			return true;
		}

		///<summary> accumulate two geometric constraints per point </summary>
		void Add(const cVector2<T> &d, const cVector3<T> &x)
		{
			Add(mP.mK.MetricRay(d), x);
		}

		///<summary> accumulate two geometric constraints per point </summary>
		void Add(const cVector3<T> &p, const cVector3<T> &x)
		{
			// given a "metric ray" p = [p0; p1; -1] and a point X = [x;y;z;1]
			// form two equations for the camera matrix
			// [M0;M1;M2] X = p  (up to scale)
			// M0.X / M2.X = p0 / -1  and M1.X / M2.X = p1 / -1
			// M0.X + p0 M2.X = 0 and M1.X + p1 M2.X = 0
			// => M_flat * (x0, x1, x2, 1, 0, 0, 0, 0, p0 * x0, p0 * x1, p0 * x2, p0) = 0
			// => M_flat * (0, 0, 0, 0, x0, x1, x2, 1, p1 * x0, p1 * x1, p1 * x2, p1) = 0
			// NOTE the units of the residual are 3D-like and so the least-squares solution will be geometric

			// The camera equations together form a matrix equation:
			// E_{nx12} x_{12} = 0_{n}.
			// multiplying on the left by E^T, we get the least-squares solution
			// (E^T E) x_{12} = (E^T 0_{n})
			// S_{12x12} x_{12} = 0_{12}
			// accumulate S as the constraints are added

			ASSERT(p[2] == -1);
			Add(x[0], x[1], x[2], p[0], p[1]);
		}

		///<summary> accumulate two constraints into the symmetric matrix </summary>
		void Add(const double x0, const double x1, const double x2, const double p0, const double p1)
		{
			// multiply all the things
			const double x00 = x0*x0, x01 = x0*x1, x02 = x0*x2, x11 = x1*x1, x12 = x1*x2, x22 = x2*x2, ps = p0*p0 + p1*p1;
			m00 += x00;
			m01 += x01;
			m11 += x11;
			m02 += x02;
			m12 += x12;
			m22 += x22;
			m03 += x0;
			m13 += x1;
			m23 += x2;
			m33 += 1;

			m08 += p0 * x00;
			m09 += p0 * x01;
			m19 += p0 * x11;
			m010 += p0 * x02;
			m110 += p0 * x12;
			m210 += p0 * x22;
			m011 += p0 * x0;
			m111 += p0 * x1;
			m211 += p0 * x2;
			m311 += p0;

			m48 += p1 * x00;
			m49 += p1 * x01;
			m59 += p1 * x11;
			m410 += p1 * x02;
			m510 += p1 * x12;
			m610 += p1 * x22;
			m411 += p1 * x0;
			m511 += p1 * x1;
			m611 += p1 * x2;
			m711 += p1;

			m88 += ps * x00;
			m89 += ps * x01;
			m99 += ps * x11;
			m810 += ps * x02;
			m910 += ps * x12;
			m1010 += ps * x22;
			m811 += ps * x0;
			m911 += ps * x1;
			m1011 += ps * x2;
			m1111 += ps;

			mCount += 2;
		}
	};

	template<class T>
	class cPointSolver
	{
		double m00, m01, m02, m11, m12, m22, m0, m1, m2;
		int mCount;
	public:
		cPointSolver() { Clear(); }

		void Clear()
		{
			// force strictly positive matrix by adding epsilon to diagonal m00-m11-m22
			m00 = m11 = m22 = 1e-8;
			m01 = m02 = m12 = m0 = m1 = m2 = 0;
			mCount = 0;
		}

		int Count() const { return mCount; }

		template<class T> cVector3<T> Solve() const 
		{
			if (mCount < 3) return cVector3<T>(1e10, 1e10, 1e10);
			const double m11m22_m12m12 = m11*m22 - m12*m12;
			const double m02m12_m01m22 = m02*m12 - m01*m22;
			const double m00m22_m02m02 = m00*m22 - m02*m02;
			const double m01m12_m02m11 = m01*m12 - m02*m11;
			const double m01m02_m00m12 = m01*m02 - m00*m12;
			const double m00m11_m01m01 = m00*m11 - m01*m01;
			const double sc = 1.0 / (m02*m01m12_m02m11 + m12*m01m02_m00m12 + m22*m00m11_m01m01);
			return cVector3<T>(
				(m0*m11m22_m12m12 + m1*m02m12_m01m22 + m2*m01m12_m02m11)*sc,
				(m0*m02m12_m01m22 + m1*m00m22_m02m02 + m2*m01m02_m00m12)*sc,
				(m0*m01m12_m02m11 + m1*m01m02_m00m12 + m2*m00m11_m01m01)*sc
				);
		}

		///<summary> accumulate two geometric constraints per ray </summary>
		cPointSolver<T>& Add(const cP<T> &camera, const cVector2<T> &d)
		{
			if (d[0] >= 1e10) return *this; // bad point
			return Add(camera, camera.mK.MetricRay(d));
		}

		///<summary> accumulate two geometric constraints per ray </summary>
		cPointSolver<T>& Add(const cP<T> &camera, const cVector3<T> &p)
		{
			// given a "metric ray" p = [p0; p1; -1]
			// form two equations for a point X = [x;y;z;1]
			// [M0;M1;M2] X = p  (up to scale)
			// M0.X / M2.X = p0 / -1  and M1.X / M2.X = p1 / -1
			// => M0.X + p0 M2.X = 0 and M1.X + p1 M2.X = 0
			// => (M0 + p0 M2).X = 0 and (M1 + p1 M2).X = 0
			// NOTE the units of the residual are 3D-like and so the least-squares solution will be geometric

			// The ray equations together form a matrix E_{nx3} x_{3} = v_{n}. 
			// Multiplying on the left by E^T, we get the least-squares equation:
			// (E^T E) x_{3} = (E^T v_{n})
			// S_{3x3} x_{3} = s_{3}
			// Accumulate S and s as each constraint is added

			ASSERT(p[2] == -1);
			const T* m0 = camera.mRT.m0;
			const T* m1 = camera.mRT.m1;
			const T* m2 = camera.mRT.m2;
			const T p0 = p[0], p1 = p[1];
			Add(m0[0] + m2[0] * p0, m0[1] + m2[1] * p0, m0[2] + m2[2] * p0, -(m0[3] + m2[3] * p0));
			Add(m1[0] + m2[0] * p1, m1[1] + m2[1] * p1, m1[2] + m2[2] * p1, -(m1[3] + m2[3] * p1));
			return *this;
		}

		/// <summary> accumulate a constraint E0 x + E1 y + E2 z = E3 </summary>
		void Add(const double E0, const double E1, const double E2, const double E3)
		{
			// |m00 m01 m02||x| = |m0|
			// |m01 m11 m12||y|   |m1|
			// |m02 m12 m22||z|   |m2|
			m00 += E0 * E0;
			m01 += E0 * E1;
			m02 += E0 * E2;
			m11 += E1 * E1;
			m12 += E1 * E2;
			m22 += E2 * E2;
			m0 += E0 * E3;
			m1 += E1 * E3;
			m2 += E2 * E3;
			mCount++;
		}
	};

	///<summary> given labelled detections and cameras, compute all the 3D points by solving the ray equations </summary>
	template <class T>
	static void IntersectRays(
		std::vector<int> &pointLabels,
		std::vector<cVector3<T> > &points,
		const cMat<int> &detectionLabels,
		const cMat<cVector2<T> > &detections,
		const cVec<cP<T> > &cameras,
		const int minRays)
	{
		std::map<int, cPointSolver<T> > rays;
		const int numCameras = (int)cameras.size();
		for (int ci = 0; ci != numCameras; ++ci)
		{
			const cP<T> &camera = cameras[ci];
			const auto &labels = detectionLabels[ci];
			const auto &dets = detections[ci];
			const int numDets = (int)labels.size();
			for (int di = 0; di != numDets; ++di)
			{
				const int li = labels[di];
				if (li != -1)
					rays[li].Add(camera, dets[di]);
			}
		}
		points.reserve(rays.size());
		for (auto &rit : rays)
		{
			const int li = rit.first;
			const cPointSolver<T> &r = rit.second;
			if (r.Count() >= minRays * 2)
			{
				pointLabels.push_back(li);
				points.push_back(r.Solve<T>());
			}
		}
	}

	struct sRay
	{
		float order;
		int ri;

		sRay(int _ri) : ri(_ri) {}
		bool operator<(const sRay &that) const { return order < that.order; }
	};

	template<class T>
	static void SortRays(
		cVec<sRay> &cray,
		const cVec<int> &dls,
		const cVec<cVector3<T> > &drs, 
		const cVector3<T> &axis, 
		const cVector3<T> &vec)
	{
		for (auto ray = cray.begin(); ray != cray.end();) // NOTE cray.end() may change during the iteration
		{
			if (dls[ray->ri] == -1)
			{
				ray->order = asin(drs[ray->ri].Cross(axis).Normalized().Dot(vec));
				++ray;
			}
			else
			{
				*ray = cray.back();
				cray.pop_back(); // NOTE this shortens the vector
			}
		}
		std::sort(cray.begin(), cray.end());
	}

	///<summary> given detections and cameras, pick up some 3D points to track. detectionLabels is
	/// allowed to be filled on entry, for example to pick up more points; then pointLabels and points
	/// should also be filled, or else the original detectionLabels will be lost in the cleanup. </summary>
	template<class T>
	static int BootPoints(
		std::vector<int> &pointLabels,
		std::vector<cVector3<T> > &points,
		cMatrix<int> &detectionLabels,
		const cMat<cVector2<T> > &detections,
		const cVec<cP<T> > &cameras,
		const int firstId = 0,
		const int minRays = 2,
		const T threshold2D = T(7.0),
		const T angleThreshold2D = T(1.0),
		const T floorClip = T(-0.1))
	{
		int nextId = firstId;
		const bool initDetectionLabels = !detectionLabels.size();
		if (initDetectionLabels) detectionLabels.SetShape(detections, -1);
		const cVec<cVector2<T> > &dds = detections.Data();
		const cVec<int> &dls = detectionLabels.Data();
		const int numCameras = (int)cameras.size();
		const int numRays = (int)detections.Data().size();
		cMatrix<sRay> rays; rays.reserve(numCameras);
		std::vector<cVector3<T> > drsV(numRays);
		cVec<cVector3<T> > drs(drsV);
		const cHashCloudList2D<T> hashClouds(cameras, detections, threshold2D);
		for (int ci = 0, ri = 0; ci != numCameras; ++ci)
		{
			const cP<T> &camera = cameras[ci];
			for (const auto &det : detections[ci])
			{
				if (dls[ri] == -1) rays.AddRowItem(sRay(ri));
				drs[ri] = camera.RayDir(det);
				ri++;
			}
			rays.EndRow();
		}
		if (initDetectionLabels && points.size())
		{
			hashClouds.ProjectAssign(points);
			hashClouds.Apply(detectionLabels, pointLabels);
		}
		std::vector<cVec<sRay> > camRays; camRays.reserve(numCameras); // make a copy of the bounds
		for (int ci = 0; ci != numCameras; ++ci)
			camRays.push_back(rays[ci]);
		for (int c1 = 0; c1 != numCameras; ++c1)
		{
			auto &c1Rays = camRays[c1];
			if (c1Rays.size() == 0) continue;
			const cP<T> &cam1 = cameras[c1];
			const cVector3<T> c1pos = cam1.Position(), c1opt = cam1.OpticalAxis();
			const T angleThreshold = angleThreshold2D * cam1.UndistortScale();
			for (int c2 = 0; c2 != c1; ++c2)
			{
				auto &c2Rays = camRays[c2];
				if (c2Rays.size() == 0) continue;
				const cP<T> &cam2 = cameras[c2];
				const cVector3<T> axis((c1pos - cam2.Position()).Normalized());
				const cVector3<T> vec(axis.Cross(c1opt.Cross(axis)).Normalized());
				SortRays(c1Rays, dls, drs, axis, vec);
				SortRays(c2Rays, dls, drs, axis, vec);
				float badOrder = -2.0f; // badOrder < -pi/2
				for (auto r1(c1Rays.begin()), r2(c2Rays.begin()), r1e(c1Rays.end()), r2e(c2Rays.end()); r1 != r1e && r2 != r2e; )
				{
					const float r1o = r1->order, r2o = r2->order;
					if (fabsf(r1o - r2o) < angleThreshold)
					{
						const float hi = std::max(r1o, r2o) + angleThreshold;
						if ((r1 + 1 != r1e && r1[1].order < hi) || (r2 + 1 != r2e && r2[1].order < hi))
							badOrder = hi; // duplicate matches = bad

						if (r1o > badOrder && r2o > badOrder) // found a unique match
						{
							cVector3<T> pt(cPointSolver<T>().Add(cam1, dds[r1->ri]).Add(cam2, dds[r2->ri]).Solve<T>());
							if (pt[1] >= floorClip)
							{
								std::vector<cVector3<T> > testPoint(1, pt);
								const int count = hashClouds.ProjectAssign(testPoint);
								if (count >= minRays)
								{
									std::vector<int> testLabel(1, nextId);
									hashClouds.Apply(detectionLabels, testLabel);
									pointLabels.push_back(nextId++);
									points.push_back(pt);
								}
							}
						}
					}
					if (r1o < r2o) ++r1; else ++r2;
				}
			}
		}
		// cleanup
		detectionLabels.clear();
		hashClouds.ProjectAssign(points);
		hashClouds.Apply(detectionLabels, pointLabels);
		pointLabels.clear();
		points.clear();
		IntersectRays(pointLabels, points, detectionLabels, detections, cameras, minRays);
		detectionLabels.clear();
		hashClouds.ProjectAssign(points);
		hashClouds.Apply(detectionLabels, pointLabels);
		return nextId;
	}

	class cTrack2D
	{
		std::vector<cPf>                       mCameras;
		cMatrix<int>                           mDetectionLabels;
		cMatrix<cVector2f>                     mPredictions;
		std::map<int, std::vector<cVector2f> > mTracks;
	public:
		float                                  mThreshold2D = 8.0f;
		int                                    mNextID = 0;

		cTrack2D(const std::vector<cPf> &cameras) : mCameras(cameras) {}

		void AddFrame(const cMat<cVector2f> &detections)
		{
			cHashCloudList2D<float> hashCloud(mCameras, mPredictions, mThreshold2D, false);
			mDetectionLabels = hashCloud.Assign(detections, mDetectionLabels, mNextID, true);
			mPredictions.clear();
			const int numCams = (int)mCameras.size();
			for (int ci = 0; ci != numCams; ++ci)
			{
				const auto &p2d = mDetectionLabels[ci];
				const auto &cdets = detections[ci];
				for (int pi = 0, pend = (int)p2d.size(); pi != pend; ++pi)
				{
					std::vector<cVector2f> &track = mTracks[p2d[pi]];
					const cVector2f &det = cdets[pi];
					mPredictions.AddRowItem(track.size() == 0 ? det : det * 1.8f - track.back() * 0.8f);
					track.push_back(det);
				}
				mPredictions.EndRow();
			}
		}
	};

	class cTrack3D
	{
		std::vector<cPf>             mCameras;
		std::vector<cVector3f>       mPoints;
		std::vector<cVector3f>       mVels;
		std::vector<int>             mPointLabels;
		std::map<int, std::vector<cVector3f> > mTracks;
		int                          mNextPointID = 0;

	public:
		float                        mThreshold2D = 8.0f; // pixels
		float                        mThreshold3D = 0.05f; // meters
		float                        mAngleThreshold = 1.0f; // pixels
		float                        mFloorClip = -0.1f; // meters
		int                          mMinRays = 4;

		cTrack3D(const std::vector<cPf> &cameras) : mCameras(cameras) {}

		const std::vector<cVector3f>& GetPoints() const { return mPoints; }
		const std::vector<int>& GetPointLabels() const { return mPointLabels; }

		void AddFrame(const cMat<cVector2f> &detections)
		{
			cMatrix<int> detectionLabels;
			std::vector<int> mPointLabelsPrev(mPointLabels);
			std::vector<cVector3f> mPointsPrev(mPoints);
			for (int mi = 0, mi_end = (int)mPoints.size(); mi != mi_end; ++mi)
				mPoints[mi] += mVels[mi] * 0.8f;
			Core::BootPoints<float>(mPointLabels, mPoints, detectionLabels, detections, mCameras,
				mNextPointID, mMinRays, mThreshold2D, mAngleThreshold, mFloorClip);
			cHashCloud<cVector3f> hashcloud(mPointsPrev, mThreshold3D);
			cSparseFloatMatrix matrix;
			hashcloud.MakeDistanceMatrix(matrix, mPoints, true);
			std::vector<int> m2pm;
			HungarianAssignment(m2pm, matrix, mThreshold3D);
			// TODO preserve flickering points?
			mVels.resize(mPoints.size());
			for (int mi = 0, mi_end = (int)m2pm.size(); mi != mi_end; ++mi)
			{
				const int pmi = m2pm[mi];
				const int label = (pmi == -1 ? mNextPointID++ : mPointLabelsPrev[pmi]);
				mPointLabels[mi] = label;
				if (pmi != -1) mVels[mi] = mPoints[mi] - mPointsPrev[pmi];
				else
					mVels[mi] = cVector3f(); // TODO initialise velocity
				mTracks[label].push_back(mPoints[mi]);
			}
		}
	};

	class cTrackModel
	{
		std::vector<cPf>             mCameras;
		std::vector<cSkeleton>       mSkeletons;
		cMatrix<float>               mDofs;
		cMatrix<float>               mPrevDofs;
		cMatrix<cRTf>                mChannelMats;
		cMatrix<sLabelTarget>        mLabelTargets;
		std::vector<cVector3f>       mLabelPositions;
		std::vector<int>             mLabelIDs;
		int                          mNextPointID = 0;

	public:
		float                        mThreshold2D = 8.0f; // pixels
		float                        mThreshold3D = 0.05f; // meters
		float                        mAngleThreshold = 1.0f; // pixels
		float                        mFloorClip = -0.1f; // meters
		int                          mMinRays = 4;
		int                          mNumFastIKIts = 3;
		int                          mNumSlowIKIts = 1;
		int                          mMaxHypotheses = 5000;
		float                        mPenalty = 20.0f;
		float                        mPenalty2 = 40.0f;

		cTrackModel(const std::vector<cPf> &cameras, const std::vector<cSkeleton> &skeletons, std::vector<float> &dofValues) : 
			mCameras(cameras), mSkeletons(skeletons)
		{
			for (auto &skel : mSkeletons)
			{
				for (int i = skel.NumDofs(); i; --i) mDofs.AddRowItem(0.0f);
				mDofs.EndRow();
				for (int i = skel.NumChannels(); i; --i) mChannelMats.AddRowItem(cRTf());
				mChannelMats.EndRow();
				for (int i = skel.NumLabels(); i; --i)
				{
					mLabelTargets.AddRowItem(sLabelTarget());
					mLabelIDs.push_back((int)mLabelIDs.size());
					mLabelPositions.push_back(cVector3f());
				}
				mLabelTargets.EndRow();
			}
			mDofs.Data() = dofValues;
			mDofs.CopyTo(mPrevDofs);
			mNextPointID = (int)mLabelIDs.size();
		}

		const std::vector<cVector3f>& GetPoints() const { return mLabelPositions; }
		const std::vector<float>& GetDofs() const { return mDofs.Data(); }

		void SetDofs(std::vector<float> &dofs)
		{
			mDofs.Data() = dofs;
			mDofs.CopyTo(mPrevDofs);
		}

		std::vector<cVector3f> GetEdges()
		{
			std::vector<cVector3f> ret;
			const int numSkeletons = (int)mSkeletons.size();
			for (int si = 0; si != numSkeletons; ++si)
			{
				auto &skel = mSkeletons[si];
				auto &dofs = mDofs[si];
				auto &chanMats = mChannelMats[si];
				auto &labels = mLabelTargets[si];
				skel.UpdatePose(chanMats, labels, dofs);
				for (int ci = 0, ci_end = chanMats.size(); ci != ci_end; ++ci)
				{
					int pi = skel.mChannelInfo[ci].parent;
					if (pi == -1) continue;
					ret.push_back(chanMats[pi].Origin());
					ret.push_back(chanMats[ci].Origin());
				}
			}
			return ret;
		}

		void Boot(const cMat<cVector2f> &detections)
		{
			cMatrix<int> detectionLabels;
			std::vector<int> mPointLabelsTmp(mLabelIDs);
			std::vector<cVector3f> mPointsTmp(mLabelPositions);
			Core::BootPoints<float>(mPointLabelsTmp, mPointsTmp, detectionLabels, detections, mCameras,
				mNextPointID, mMinRays, mThreshold2D, mAngleThreshold, mFloorClip);
			const int numSkeletons = (int)mSkeletons.size();
			for (int si = 0, pi = 0; si != numSkeletons; ++si)
			{
				auto &skel = mSkeletons[si];
				auto &dofs = mDofs[si];
				auto &labelTargets = mLabelTargets[si];
				auto &chanMats = mChannelMats[si];
				skel.Boot(dofs, labelTargets, mPointsTmp, mThreshold3D, mMaxHypotheses, mPenalty, mPenalty2, mNumFastIKIts, mNumSlowIKIts);
				auto &prevDofs = mPrevDofs[si];
				skel.UpdatePose(chanMats, labelTargets, dofs);
				for (auto &l : labelTargets) mLabelPositions[pi++] = l.position;
			}
			mDofs.CopyTo(mPrevDofs);
		}

		void AddFrame(const cMat<cVector2f> &detections)
		{
			// assuming that the skeletons are correctly posed:
			// A) predict the new skeleton pose
			// B) assign rays, solve label positions
			// C) IK solve
			// D) assign rays
			// E) track unlabelled points
			// F) boot unassigned labels
			const int numSkeletons = (int)mSkeletons.size();
			for (int si = 0, pi = 0; si != numSkeletons; ++si)
			{
				auto &skel = mSkeletons[si];
				auto &dofs = mDofs[si];
				auto &prevDofs = mPrevDofs[si];
				auto &chanMats = mChannelMats[si];
				auto &labelTargets = mLabelTargets[si];
				const int numDofs = (int)dofs.size();
				// A: predict
				for (int di = 0; di != numDofs; ++di)
				{
					float old = dofs[di];
					dofs[di] = 1.8f * dofs[di] - 0.8f * prevDofs[di];
					prevDofs[di] = old;
				}
				// B: solve labels
				skel.UpdatePose(chanMats, labelTargets, dofs);
				for (auto &l : labelTargets) mLabelPositions[pi++] = l.position;
			}
			cMatrix<int> detectionLabels;
			std::vector<int> markerLabelIDs(mLabelIDs);
			std::vector<cVector3f> markers(mLabelPositions);
			std::vector<int> l2m;
			{
				BootPoints<float>(markerLabelIDs, markers, detectionLabels, detections, mCameras,
					mNextPointID, mMinRays, mThreshold2D, mAngleThreshold, mFloorClip);
				cHashCloud<cVector3f> hashcloud(markers, mThreshold3D);
				cSparseFloatMatrix matrix;
				hashcloud.MakeDistanceMatrix(matrix, mLabelPositions, true);
				HungarianAssignment(l2m, matrix, mThreshold3D);
			}
			// TODO preserve flickering points?
			auto &targets = mLabelTargets.Data();
			ASSERT(l2m.size() == mLabelPositions.size());
			for (int li = 0, li_end = (int)l2m.size(); li != li_end; ++li)
			{
				const int mi = l2m[li];
				targets[li].assignment = -1;
				if (mi != -1)
				{
					targets[li].assignment = li;
					targets[li].markerPosition = markers[mi];
					mLabelPositions[li] = markers[mi];
				}
			}
			// C: IK
			for (int si = 0, pi = 0; si != numSkeletons; ++si)
			{
				auto &skel = mSkeletons[si];
				auto &chanMats = mChannelMats[si];
				auto &dofs = mDofs[si];
				auto &labels = mLabelTargets[si];
				skel.IKSolve(dofs, labels, mNumFastIKIts, mNumSlowIKIts);
				skel.UpdatePose(chanMats, labels, dofs);
				for (auto &l : labels) mLabelPositions[pi++] = l.position;
			}
		}
	};

}

