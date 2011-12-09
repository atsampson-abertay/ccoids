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
#ifdef HAVE_LIBPORTMIDI
#include <portmidi.h>
#endif

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

/*{{{  control devices */
#ifdef HAVE_LIBPORTMIDI
class MIDIDevice : public ControlDevice {
public:
	MIDIDevice()
		: stream_(NULL) {
	}

	// FIXME: destructor

	virtual void open(PmDeviceID id) = 0;

protected:
	static const int MAX_EVENTS = 1000;

	PortMidiStream* stream_;
};

class MIDIInputDevice : public MIDIDevice {
public:
	void open(PmDeviceID id) {
		if (Pm_OpenInput(&stream_, id, NULL, MAX_EVENTS, NULL, NULL)
		    != pmNoError) {
			cerr << "PmOpenInput failed" << endl;
			exit(1);
		}
	}

	void poll(Controls& controls) {
		while (Pm_Poll(stream_) == TRUE) {
			PmEvent events[MAX_EVENTS];
			int count = Pm_Read(stream_, events, MAX_EVENTS);

			for (int i = 0; i < count; ++i) {
				PmMessage& msg(events[i].message);
				int status = Pm_MessageStatus(msg);
				int data1 = Pm_MessageData1(msg);
				int data2 = Pm_MessageData2(msg);

				if ((status & 0xF0) == 0xB0) {
					handle_cc(controls,
					          status & 0x0F, data1, data2);
				} else {
					cout << "Unhandled MIDI: " << status << ", " << data1 << ", " << data2 << endl;
				}
			}
		}
	}

protected:
	virtual void handle_cc(Controls& controls,
	                       int channel, int controller, int value) = 0;
};

class MIDIOutputDevice : public MIDIDevice {
protected:
	void open(PmDeviceID id) {
		if (Pm_OpenOutput(&stream_, id, NULL, MAX_EVENTS, NULL, NULL, 0)
		    != pmNoError) {
			cerr << "PmOpenOutput failed" << endl;
			exit(1);
		}
	}

	void send_cc(int channel, int controller, int value) {
		PmMessage msg = Pm_Message(0xB0 | channel, controller, value);
		Pm_WriteShort(stream_, 0, msg);
	}
};

class BCF2000InputDevice : public MIDIInputDevice {
protected:
	void handle_cc(Controls& controls,
	               int channel, int controller, int value) {
		float scaled = value / 127.0;
		if (controller >= 81 && controller <= 88) {
			// Fader
			controls.handle_set(controller - 81, scaled);
		} else if (controller >= 1 && controller <= 8) {
			// Knob
			controls.handle_set(controller + 7, scaled);
		} else if (controller == 89) {
			// First of the bottom right buttons
			controls.handle_reset();
		}
	}
};

class BCF2000OutputDevice : public MIDIOutputDevice {
public:
	void send_control(int control, float value) {
		int scaled = value * 127;
		if (control >= 0 && control <= 7) {
			// Fader
			send_cc(0, 81 + control, scaled);
		} else if (control >= 8 && control <= 15) {
			// Knob
			send_cc(0, control - 7, scaled);
		}
	}
};

class NanoKontrolInputDevice : public MIDIInputDevice {
protected:
	void handle_cc(Controls& controls,
	               int channel, int controller, int value) {
		float scaled = value / 127.0;
		if (controller >= 0 && controller <= 7) {
			// Fader
			controls.handle_set(controller, scaled);
		} else if (controller >= 16 && controller <= 23) {
			// Knob
			controls.handle_set(controller - 8, scaled);
		}
	}
};
#endif
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

		boost::shared_ptr<MIDIDevice> cd;

		cout << "Device " << device << ": " << devinfo->interf << ", " << devinfo->name << endl;
		if (strstr(devinfo->name, "BCF2000 MIDI 1") != NULL) {
			if (devinfo->input) {
				cd.reset(new BCF2000InputDevice());
			} else if (devinfo->output) {
				cd.reset(new BCF2000OutputDevice());
			}
		}
		if (strstr(devinfo->name, "nanoKONTROL") != NULL) {
			if (devinfo->input) {
				cd.reset(new NanoKontrolInputDevice());
			}
		}

		if (cd) {
			cd->open(device);
			devices_.push_back(cd);
		}
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

	BOOST_FOREACH(DevicePtr& cd, devices_) {
		cd->poll(*this);
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

		BOOST_FOREACH(DevicePtr& cd, devices_) {
			cd->send_control(i, value);
		}
	}
}

void Controls::adjust_selected_with(Adjuster& adjuster) {
	if (selected_valid()) {
		adjustables_[selected_]->adjust_with(adjuster);
	}
}
