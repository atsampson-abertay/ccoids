/*
 *  controls.hh - tweakable controls for ccoids
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

#ifndef CONTROLS_HH
#define CONTROLS_HH

#include <vector>
#ifdef HAVE_LIBPORTMIDI
#include <portmidi.h>
#endif

class Control {
public:
	Control(float& value, float min, float initial, float max)
		: value_(value), min_(min), initial_(initial), max_(max) {
		reset();
	}

	void reset() {
		value_ = initial_;
	}

	float get() {
		return (value_ - min_) / (max_ - min_);
	}

	void set(float frac) {
		value_ = min_ + ((max_ - min_) * frac);
	}

private:
	float& value_;
	float min_, initial_, max_;
};

class Controls {
public:
	Controls();
	~Controls();

	void add_control(Control *control) {
		controls_.push_back(control);
	}
	void poll_controls();
	void reset_controls();
	void send_controls();

private:
	void change_control(int num, float value);

	typedef std::vector<Control *> ControlVector;
	ControlVector controls_;
#ifdef HAVE_LIBPORTMIDI
	PortMidiStream* in_stream_;
	PortMidiStream* out_stream_;
#endif

	static const int MAX_EVENTS = 1000;
};

#endif
