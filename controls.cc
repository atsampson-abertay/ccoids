/*
 *  context.cc - CoSMoS Demos
 *  Adam Sampson
 *
 *  Copyright (C) 2009, Adam Sampson
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

using namespace std;

Controls::Controls()
	: changed_(false) {
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

Controls::~Controls() {
	// FIXME: use auto_ptr
	for (ControlVector::iterator it = controls_.begin(); it != controls_.end(); ++it) {
		delete *it;
	}
}

void Controls::poll_controls() {
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
				change_control(data1 - 81, data2 / 127.0);
			} else if (status == 176 && data1 >= 0 && data1 <= 7) {
				// Fader change on the nanoKONTROL
				change_control(data1, data2 / 127.0);
			} else if (status == 176 && data1 == 89) {
				// Reset button on the BCF2000
				reset_controls();
				send_controls();
			} else {
				cout << "Unhandled MIDI: " << status << ", " << data1 << ", " << data2 << endl;
			}
		}
	}
#endif
}

void Controls::reset_controls() {
	for (int i = 0; i < controls_.size(); ++i) {
		controls_[i]->reset();
	}
}

void Controls::send_controls() {
#ifdef HAVE_LIBPORTMIDI
	for (int i = 0; i < controls_.size(); ++i) {
		float value = controls_[i]->get();
		PmMessage msg = Pm_Message(176, 81 + i, int(value * 127));
		Pm_WriteShort(out_stream_, 0, msg);
	}
#endif
}

Controls::StateVector Controls::states() {
	Controls::StateVector vec;

	for (int i = 0; i < controls_.size(); ++i) {
		Control *c = controls_[i];
		vec.push_back(State(c->initial(), c->get()));
	}

	return vec;
}

void Controls::change_control(int num, float value) {
	if (num < 0 || num > controls_.size()) {
		return;
	}

	cout << "control " << num << " to " << value << endl;
	controls_[num]->set(value);
	changed_ = true;
}
