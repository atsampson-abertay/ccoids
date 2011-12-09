/*
 *  controls.cc - CoSMoS Demos
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

#include "controls.hh"

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <boost/foreach.hpp>

using namespace std;

/*{{{  adjusters */
class SingleAdjuster : public Adjuster {
public:
	SingleAdjuster(int control)
		: control_(control), count_(0) {
	}

	void operator()(float& value, float min,
	                float initial, float max) {
		if (count_ == control_) {
			adjust(value, min, initial, max);
		}
		++count_;
	}

	virtual void adjust(float& value, float min,
	                    float initial, float max) = 0;

private:
	int control_, count_;
};

class GetAdjuster : public SingleAdjuster {
public:
	GetAdjuster(int control)
		: SingleAdjuster(control), value_(0.0) {
	}

	void adjust(float& value, float min, float initial, float max) {
		value_ = (value - min) / (max - min);
	}

	float value() {
		return value_;
	}

private:
	float value_;
};

class SetAdjuster : public SingleAdjuster {
public:
	SetAdjuster(int control, float value)
		: SingleAdjuster(control), value_(value) {
	}

	void adjust(float& value, float min, float initial, float max) {
		value = (value_ * (max - min)) + min;
	}

private:
	float value_;
};

class ResetAdjuster : public Adjuster {
	void operator()(float& value, float min,
	                float initial, float max) {
		value = initial;
	}
};
/*}}}*/

Controls::Controls()
	: selected_(-1) {
#ifdef HAVE_LIBPORTMIDI
	if (Pm_Initialize() != pmNoError) {
		cerr << "Pm_Initialize failed" << endl;
		exit(1);
	}

	PmDeviceID in_device = Pm_GetDefaultInputDeviceID();
	PmDeviceID out_device = Pm_GetDefaultOutputDeviceID();
	PmDeviceID max = Pm_CountDevices();
	for (PmDeviceID device = 0; device < max; ++device) {
		const PmDeviceInfo *devinfo = Pm_GetDeviceInfo(device);

		cout << "Device " << device << ": " << devinfo->interf << ", " << devinfo->name << endl;
		if (strstr(devinfo->name, "BCF2000 MIDI 1") != NULL
		    || strstr(devinfo->name, "nanoKONTROL") != NULL) {
			if (devinfo->input) {
				in_device = device;
			}
			if (devinfo->output) {
				out_device = device;
			}
		}
	}

	if (in_device == pmNoDevice) {
		cerr << "Didn't find a PortMidi input device" << endl;
		exit(1);
	}
	if (out_device == pmNoDevice) {
		cerr << "Didn't find a PortMidi output device" << endl;
		exit(1);
	}
	cout << "Using devices " << in_device << ", " << out_device << endl;

	if (Pm_OpenInput(&in_stream_, in_device, NULL, MAX_EVENTS, NULL, NULL) != pmNoError) {
		cerr << "PmOpenInput failed" << endl;
		exit(1);
	}
	if (Pm_OpenOutput(&out_stream_, out_device, NULL, MAX_EVENTS, NULL, NULL, 0) != pmNoError) {
		cerr << "PmOpenOutput failed" << endl;
		exit(1);
	}
#endif
}

void Controls::add_and_init(AdjustablePtr adj) {
	adjustables_.push_back(adj);

	ResetAdjuster adjuster;
	adj->adjust_with(adjuster);
}

void Controls::poll() {
	// Ensure we've got something selected, if possible.
	if (!selected_valid()) handle_select(0);

#ifdef HAVE_LIBPORTMIDI
	while (Pm_Poll(in_stream_) == TRUE) {
		PmEvent events[MAX_EVENTS];
		int count = Pm_Read(in_stream_, events, MAX_EVENTS);

		for (int i = 0; i < count; ++i) {
			PmMessage& msg(events[i].message);
			int status = Pm_MessageStatus(msg);
			int data1 = Pm_MessageData1(msg);
			int data2 = Pm_MessageData2(msg);

			if (status == 176 && data1 >= 81 && data1 <= 88) {
				// Fader change on the BCF2000
				handle_set(data1 - 81, data2 / 127.0);
			} else if (status == 176 && data1 >= 0 && data1 <= 7) {
				// Fader change on the nanoKONTROL
				handle_set(data1, data2 / 127.0);
			} else if (status == 176 && data1 == 89) {
				// Reset button on the BCF2000
				handle_reset();
			} else {
				cout << "Unhandled MIDI: " << status << ", " << data1 << ", " << data2 << endl;
			}
		}
	}
#endif
}

void Controls::adjust_selected_with(Adjuster& adjuster) {
	if (selected_valid()) {
		adjustables_[selected_]->adjust_with(adjuster);
	}
}

void Controls::handle_set(int control, float value) {
	SetAdjuster adjuster(control, value);
	adjust_selected_with(adjuster);
}

void Controls::handle_reset() {
	ResetAdjuster adjuster;
	adjust_selected_with(adjuster);

	handle_select(selected_);
}

void Controls::handle_select(int num) {
	selected_ = num;

	if (!selected_valid()) return;
	Adjustable& adj(*adjustables_[selected_]);

	// FIXME: set lights to indicate selected

	for (int i = 0; i < MAX_CONTROLS; ++i) {
		GetAdjuster adjuster(i);
		adjust_selected_with(adjuster);
		const float value = adjuster.value();

#ifdef HAVE_LIBPORTMIDI
		PmMessage msg = Pm_Message(176, 81 + i, int(value * 127));
		Pm_WriteShort(out_stream_, 0, msg);
#endif
	}
}
