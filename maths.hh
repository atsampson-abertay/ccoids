/*
 *  maths.hh - maths utilities for ccoids
 *  Adam Sampson
 *
 *  Copyright (C) 2011, Adam Sampson
 *  All rights reserved.
 *  
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions
 *  are met:
 *  
 *   1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in
 *      the documentation and/or other materials provided with the
 *      distribution.
 *   3. Neither the name of the CoSMoS Project nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *  
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef MATHS_HH
#define MATHS_HH

template <typename ELEMENT> class Vector {
public:
	Vector(ELEMENT x, ELEMENT y) : x_(x), y_(y) {
	}
	// FIXME Is this a bad idea?
	template <typename OTHER>
	Vector(Vector<OTHER> v) : x_(v.x_), y_(v.y_) {
	}

	Vector<ELEMENT> operator+(const Vector<ELEMENT>& b) const {
		return Vector<ELEMENT>(x_ + b.x_, y_ + b.y_);
	}
	Vector<ELEMENT>& operator+=(const Vector<ELEMENT> &b) {
		*this = *this + b;
		return *this;
	}
	Vector<ELEMENT> operator-(const Vector<ELEMENT>& b) const {
		return Vector<ELEMENT>(x_ - b.x_, y_ - b.y_);
	}
	Vector<ELEMENT>& operator-=(const Vector<ELEMENT> &b) {
		*this = *this - b;
		return *this;
	}

	Vector<ELEMENT> operator*(ELEMENT v) const {
		return Vector<ELEMENT>(x_ * v, y_ * v);
	}
	Vector<ELEMENT>& operator*=(ELEMENT v) {
		*this = *this * v;
		return *this;
	}
	Vector<ELEMENT> operator/(ELEMENT v) const {
		return Vector<ELEMENT>(x_ / v, y_ / v);
	}
	Vector<ELEMENT>& operator/=(ELEMENT v) {
		*this = *this / v;
		return *this;
	}

	bool operator==(const Vector<ELEMENT>& b) const {
		return (x_ == b.x_) && (y_ == b.y_);
	}

	ELEMENT mag2() const {
		return (x_ * x_) + (y_ * y_);
	}

	ELEMENT x_, y_;
};

float rand_float();
int oversign(float v);
float angle_diff(float a, float b);

#endif
