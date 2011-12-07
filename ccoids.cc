/*
 *  ccoids.cc - CoSMoS Demos
 *  Adam Sampson
 *
 *  Copyright (C) 2009, 2011, Adam Sampson
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

// Boids in C++ using CCSP for concurrency.
// Phase structure:
// Phase 1  Viewers update
// Phase 2  Agents look and compute
// Phase 3  Agents send updates

#include "barrier.hh"
#include "context.hh"
#include "shared.hh"
#include "timer.hh"

#include <iostream>
#include <vector>
#include <map>
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <stdint.h>
#include <SDL.h>
#include <SDL_gfxPrimitives.h>
#include <gtkmm/drawingarea.h>
#include <gtkmm/main.h>
#include <gtkmm/window.h>
#ifdef HAVE_LIBPORTMIDI
#include <portmidi.h>
#endif

using namespace std;

const int BIRDS = 500;
const int WIDTH_LOCATIONS = 8;
const int HEIGHT_LOCATIONS = 5;
const int DISPLAY_HEIGHT = 850;
const int DISPLAY_PERIOD = 1000000 / 25;
const float MAX_INITIAL_SPEED = 0.1;

struct Settings {
	float vision_radius;
	float vision_angle;
	float mean_velocity_fraction;
	float centre_of_mass_fraction;
	float repulsion_distance;
	float repulsion_fraction;
	float smooth_acceleration;
	float speed_limit;
};

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

float rand_float() {
	return rand() / (1.0 * RAND_MAX);
}

int oversign(float v) {
	if (v < 0) {
		return -1;
	} else if (v > 1) {
		return 1;
	} else {
		return 0;
	}
}

static float angle_diff(float a, float b) {
	float r = a - b;
	if (r < -M_PI) {
		r += 2.0 * M_PI;
	} else if (r > M_PI) {
		r -= 2.0 * M_PI;
	}
	return fabs(r);
}

const Vector<int> DIRECTIONS[] = {
	Vector<int>(-1, -1), Vector<int>(0, -1), Vector<int>(1, -1),
	Vector<int>(-1,  0),                     Vector<int>(1,  0),
	Vector<int>(-1,  1), Vector<int>(0,  1), Vector<int>(1,  1)
};
const int NUM_DIRECTIONS = 8;

class AgentInfo {
public:
	AgentInfo()
		: id_(-1), local_id_(-1), pos_(0.0, 0.0), vel_(0.0, 0.0) {
	}

	int id_;
	int local_id_;
	Vector<float> pos_, vel_;
};

class Location;
class Viewer;
typedef vector<AgentInfo> AIVector;
typedef map<int, AgentInfo> AIMap;
typedef map<int, Shared<Location> *> LocMap;
typedef map<int, Vector<float> > VecMap;
typedef map<int, Shared<Viewer> *> ViewerMap;

class Location {
public:
	Location(int id, Shared<Viewer> *viewer)
		: id_(id), id_counter_(0), viewer_(viewer) {
	}

	int id() const {
		return id_;
	}

	void neighbour(int dir, Shared<Location> *loc) {
		neighbours_[dir] = loc;
	}

	Shared<Location> *enter(AgentInfo& info, Shared<Viewer> *& viewer) {
		info.local_id_ = id_counter_++;
		viewer = viewer_;
		return update(info);
	}

	Shared<Location> *update(AgentInfo& info) {
		infos_[info.local_id_] = info;

		Vector<int> signs(oversign(info.pos_.x_), oversign(info.pos_.y_));
		if (signs.x_ == 0 && signs.y_ == 0) {
			// Stay here.
			return NULL;
		} else {
			// Go to a neighbour.
			int dir = -1;
			for (int i = 0; i < NUM_DIRECTIONS; ++i) {
				if (DIRECTIONS[i] == signs) {
					dir = i;
					break;
				}
			}
			info.pos_ -= DIRECTIONS[dir];
			infos_.erase(info.local_id_);
			return neighbours_[dir];
		}
	}

	void look(AIMap::const_iterator& begin,
	          AIMap::const_iterator& end) {
		// FIXME a bit ugly; could use an explicit Mobile
		begin = infos_.begin();
		end = infos_.end();
	}

private:
	int id_;
	int id_counter_;
	Shared<Viewer> *viewer_;
	AIMap infos_;
	LocMap neighbours_;
};

// FIXME Have a data type for this instead?
int loc_id(int x, int y) {
	return (y * WIDTH_LOCATIONS) + x;
}

class World {
public:
	World() {
	}

	~World() {
		for (LocMap::iterator it = locations_.begin(); it != locations_.end(); ++it) {
			delete it->second;
		}
	}

	// The World owns the location after this.
	void add(Context& ctx, Shared<Location> *loc) {
		Claim<Location> c(ctx, *loc);
		locations_[c->id()] = loc;
	}

	Shared<Location> *get(int id) {
		return locations_[id];
	}

private:
	LocMap locations_;
};

class Viewer {
public:
	Viewer() {
	}

	~Viewer() {
	}

	void location(Vector<float> offset, Shared<Location> *loc) {
		offsets_.push_back(offset);
		locations_.push_back(loc);
	}

	void update(Context& ctx) {
		infos_.clear();
		for (int i = 0; i < offsets_.size(); ++i) {
			Claim<Location> c(ctx, *locations_[i]);
			AIMap::const_iterator it, end;
			c->look(it, end);

			for (; it != end; ++it) {
				AgentInfo info = it->second;
				info.pos_ += offsets_[i];
				infos_.push_back(info);
			}
		}
	}

	void look(AIVector::const_iterator& begin,
	          AIVector::const_iterator& end) {
		begin = infos_.begin();
		end = infos_.end();
	}

private:
	vector<Vector<float> > offsets_;
	vector<Shared<Location> *> locations_;
	AIVector infos_;
};

// FIXME Generalise into PhaseAdapter
class ViewerUpdater : public Activity {
public:
	ViewerUpdater(Shared<Viewer> *viewer, Barrier& bar)
		: viewer_(viewer), bar_(bar) {
	}

	void run(Context& ctx) {
		while (true) {
			bar_.sync(ctx); // Phase 1

			{
				Claim<Viewer> c(ctx, *viewer_);
				c->update(ctx);
			}

			bar_.sync(ctx); // Phase 2
			bar_.sync(ctx); // Phase 3
		}
	}

private:
	Barrier& bar_;
	Shared<Viewer> *viewer_;
};

class Boid : public Activity {
public:
	Boid(AgentInfo info, Shared<Location> *loc, Barrier& bar, Settings& settings)
		: info_(info), loc_(loc), bar_(bar), settings_(settings) {
	}

	void do_update(Context &ctx, bool enter) {
		while (true) {
			Claim<Location> c(ctx, *loc_);

			Shared<Location> *new_loc;
			if (enter) {
				new_loc = c->enter(info_, viewer_);
			} else {
				new_loc = c->update(info_);
			}

			if (new_loc == NULL) {
				return;
			} else {
				loc_ = new_loc;
				enter = true;
			}
		}
	}

	void run(Context& ctx) {
		do_update(ctx, true);

		while (true) {
			bar_.sync(ctx); // Phase 1
			bar_.sync(ctx); // Phase 2

			typedef vector<AgentInfo> AIVector;
			AIVector view;

			{
				Claim<Viewer> c(ctx, *viewer_);
				AIVector::const_iterator it, end;
				c->look(it, end);

				float my_angle = atan2f(info_.vel_.x_, info_.vel_.y_);
				const float max_r2 = settings_.vision_radius * settings_.vision_radius;
				const float max_diff = ((settings_.vision_angle / 2.0) * M_PI) / 180.0;

				for (; it != end; ++it) {
					AgentInfo that = *it;

					if (that.id_ == info_.id_) {
						// This bird -- ignore
						continue;
					}

					// Compute relative position
					that.pos_ -= info_.pos_;

					if (that.pos_.mag2() > max_r2) {
						// Too far away -- ignore
						continue;
					}

					float angle = atan2(that.pos_.x_, that.pos_.y_);
					if (angle_diff(angle, my_angle) > max_diff) {
						// Out of field of view -- ignore
						continue;
					}

					view.push_back(that);
				}
			}

			int seen = view.size();

			Vector<float> accel(0.0, 0.0);

			// Move towards centroid of visible flock
			{
				Vector<float> com(0.0, 0.0);
				for (AIVector::iterator it = view.begin(); it != view.end(); ++it) {
					com += it->pos_;
				}
				if (seen > 0) {
					com /= (float) seen;
				}
				accel += com / settings_.centre_of_mass_fraction;
			}

			// Move away from birds that are too close
			{
				Vector<float> push(0.0, 0.0);
				for (AIVector::iterator it = view.begin(); it != view.end(); ++it) {
					if (it->pos_.mag2() < (settings_.repulsion_distance * settings_.repulsion_distance)) {
						push -= it->pos_;
					}
				}
				accel += push / settings_.repulsion_fraction;
			}

			// Match velocity
			{
				Vector<float> perceived(0.0, 0.0);
				for (AIVector::iterator it = view.begin(); it != view.end(); ++it) {
					perceived += it->vel_;
				}
				if (seen > 0) {
					perceived /= (float) seen;
				}
				perceived -= info_.vel_;
				accel += perceived / settings_.mean_velocity_fraction;
			}

			info_.vel_ += accel / settings_.smooth_acceleration;

			// Apply speed limit
			float mag = info_.vel_.mag2();
			const float speed_limit2 = settings_.speed_limit * settings_.speed_limit;
			if (mag > speed_limit2) {
				info_.vel_ /= mag / speed_limit2;
			}

			info_.pos_ += info_.vel_;

			bar_.sync(ctx); // Phase 3

			do_update(ctx, false);
		}
	}

private:
	AgentInfo info_;
	Shared<Location> *loc_;
	Shared<Viewer> *viewer_;
	Barrier& bar_;
	Settings& settings_;
};

class Display : public Activity {
public:
	Display(Shared<World>& world, Barrier& bar)
		: world_(world), bar_(bar) {
	}

	void run(Context& ctx) {
		init_display(ctx);

		Timer tim;
		TimeVal next = tim.read(ctx);
		while (true) {
			bar_.sync(ctx); // Phase 1
			bar_.sync(ctx); // Phase 2

			if (after(tim.read(ctx), next)) {
				fetch_blobs(ctx);
				draw_display(ctx);

				next += DISPLAY_PERIOD;
			}

			bar_.sync(ctx); // Phase 3
		}
	}

protected:
	virtual void init_display(Context& ctx) = 0;
	virtual void draw_display(Context& ctx) = 0;

	void fetch_blobs(Context& ctx) {
		blobs_.clear();
		for (int x = 0; x < WIDTH_LOCATIONS; ++x) {
			for (int y = 0; y < HEIGHT_LOCATIONS; ++y) {
				Shared<Location> *loc;
				{
					Claim<World> c(ctx, world_);
					loc = c->get(loc_id(x, y));
				}
				Claim<Location> c(ctx, *loc);

				AIMap::const_iterator it, end;
				c->look(it, end);

				for (; it != end; ++it) {
					const AgentInfo& info = it->second;

					Vector<float> offset(x, y);
					Vector<float> pos(offset + info.pos_);
					Vector<float> tail(pos + (info.vel_ * -4.0));
					float angle = atan2f(info.vel_.y_, info.vel_.x_);
					blobs_.push_back(Blob(pos, tail, sqrtf(info.vel_.mag2()), angle));
				}
			}
		}
	}

	struct Blob {
		Blob(Vector<float> pos, Vector<float> tail, float speed, float angle)
			: pos_(pos), tail_(tail), speed_(speed), angle_(angle) {
		}
		Vector<float> pos_, tail_;
		float speed_, angle_;
	};
	typedef vector<Blob> BlobVector;
	BlobVector blobs_;

	Shared<World>& world_;
	Barrier& bar_;
};

// We must have something that handles SDL_QUIT events, else our program won't
// exit on SIGINT...
class SDLEventProcessor : public Activity {
public:
	void run(Context& ctx) {
		Timer tim;

		while (true) {
			SDL_Event event;
			while (SDL_PollEvent(&event)) {
				if (event.type == SDL_QUIT) {
					exit(0);
				}
			}

			tim.delay(ctx, 100000);
		}
	}
};

class SDLDisplay : public Display {
public:
	SDLDisplay(Shared<World>& world, Barrier& bar)
		: Display(world, bar) {
	}

protected:
	virtual void init_display(Context& ctx) {
		if (SDL_Init(SDL_INIT_VIDEO) < 0) {
			exit(1);
		}
		atexit(SDL_Quit);

		ctx.spawn(new SDLEventProcessor);

		SDL_WM_SetCaption("ccoids", "ccoids");
		surface_ = SDL_SetVideoMode(WIDTH_LOCATIONS * SCALE,
		                            HEIGHT_LOCATIONS * SCALE,
		                            32, SDL_DOUBLEBUF);

	}

	virtual void draw_display(Context& ctx) {
		boxColor(surface_, 0, 0, WIDTH_LOCATIONS * SCALE, HEIGHT_LOCATIONS * SCALE, BACKGROUND_COLOUR);

#ifdef SHOW_GRID
		for (int x = 0; x < WIDTH_LOCATIONS; ++x) {
			vlineColor(surface_, x * SCALE, 0, HEIGHT_LOCATIONS * SCALE, GRID_COLOUR);
		}
		for (int y = 0; y < HEIGHT_LOCATIONS; ++y) {
			hlineColor(surface_, 0, WIDTH_LOCATIONS * SCALE, y * SCALE, GRID_COLOUR);
		}
#endif

#ifdef SHOW_LOLLIPOPS
		// Draw all the tails.
		for (BlobVector::iterator it = blobs_.begin(); it != blobs_.end(); ++it) {
			Vector<int> pos(it->pos_ * SCALE);
			Vector<int> tail(it->tail_ * SCALE);
			lineColor(surface_, pos.x_, pos.y_, tail.x_, tail.y_, TAIL_COLOUR);
		}

		// Draw all the blobs.
		for (BlobVector::iterator it = blobs_.begin(); it != blobs_.end(); ++it) {
			Vector<int> pos(it->pos_ * SCALE);
			filledCircleColor(surface_, pos.x_, pos.y_, BLOB_SIZE, AGENT_COLOUR);
		}
#endif

		// Draw all the blobs.
		for (BlobVector::iterator it = blobs_.begin(); it != blobs_.end(); ++it) {
			Vector<int> pos(it->pos_ * SCALE);
			int degrees = (it->angle_ * (180 / M_PI)) + 180;
			int width = 45 - (it->speed_ * 100.0);
			int length = (1.0 + it->speed_ * 10.0) * (SCALE / 20);
			filledPieColor(surface_, pos.x_, pos.y_, length, int(degrees) - width, int(degrees) + width, AGENT_COLOUR);
		}

		SDL_UpdateRect(surface_, 0, 0, 0, 0);
		SDL_Flip(surface_);
	}

private:
	static const Uint32 BACKGROUND_COLOUR = 0x000000FF;
	static const Uint32 GRID_COLOUR = 0x447744FF;
	static const Uint32 AGENT_COLOUR = 0xFFFFF0A0;
	static const Uint32 TAIL_COLOUR = 0x604030FF;

	static const int SCALE = DISPLAY_HEIGHT / HEIGHT_LOCATIONS;
	static const int BLOB_SIZE = SCALE / 50;

	SDL_Surface *surface_;
};

class GtkEventProcessor : public Activity {
public:
	void run(Context& ctx) {
		Timer tim;

		while (true) {
			while (Gtk::Main::events_pending()) {
				Gtk::Main::iteration();
			}

			// FIXME: Could just yield here?
			tim.delay(ctx, 10000);
		}
	}
};

class GtkDisplay : public Display {
public:
	GtkDisplay(Shared<World>& world, Barrier& bar)
		: Display(world, bar), window_(blobs_) {
	}

protected:
	void init_display(Context& ctx) {
		window_.show();

		ctx.spawn(new GtkEventProcessor);
	}

	void draw_display(Context& ctx) {
		// FIXME: nothing to do; should send signal to invalidate
	}

private:
	class DisplayArea : public Gtk::DrawingArea {
	public:
		DisplayArea(const BlobVector& blobs)
			: blobs_(blobs) {
			Glib::signal_timeout().connect(sigc::mem_fun(*this, &DisplayArea::on_timeout), 500);
		}

	protected:
		virtual bool on_timeout() {
			// Invalidate the whole window, forcing it to be
			// redrawn.

			Glib::RefPtr<Gdk::Window> window = get_window();
			if (!window) {
				return true;
			}

			Gtk::Allocation alloc = get_allocation();
			Gdk::Rectangle rect(0, 0,
			                    alloc.get_width(),
			                    alloc.get_height());
			window->invalidate_rect(rect, false);

			return true;
		}

		virtual bool on_expose_event(GdkEventExpose *event) {
			Glib::RefPtr<Gdk::Window> window = get_window();
			if (!window) {
				return true;
			}

			Cairo::RefPtr<Cairo::Context> cr = window->create_cairo_context();

			Gtk::Allocation alloc = get_allocation();
			const float display_scale = alloc.get_height();
			cr->scale(display_scale, display_scale);

			const float world_scale = 1.0 / HEIGHT_LOCATIONS;

			// Fill background in black
			cr->save();
			cr->set_source_rgb(0.0, 0.0, 0.0);
			cr->paint();
			cr->restore();

			// Draw the blobs
			cr->save();
			cr->set_line_width(0.02);
			cr->set_source_rgba(1.0, 1.0, 1.0, 0.5);
			cr->set_line_cap(Cairo::LINE_CAP_ROUND);
			// FIXME: should lock blobs
			for (BlobVector::const_iterator it = blobs_.begin(); it != blobs_.end(); ++it) {
				const Vector<float>& pos(it->pos_);
				cr->move_to(pos.x_ * world_scale, pos.y_ * world_scale);
				const Vector<float>& tail(it->tail_);
				cr->line_to(tail.x_ * world_scale, tail.y_ * world_scale);
				cr->stroke();
			}
			cr->restore();

			return true;
		}

	private:
		const BlobVector& blobs_;
	};

	class DisplayWindow : public Gtk::Window {
	public:
		DisplayWindow(const BlobVector& blobs)
			: area_(blobs) {
			set_title("ccoids");

			add(area_);

			show_all();
		}

	protected:
		virtual bool on_delete_event(GdkEventAny *event) {
			// A bit brute-force, but it works...
			exit(0);

			return false;
		}

		DisplayArea area_;
	};

	DisplayWindow window_;
};

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

class Controls : public Activity {
public:
	Controls(Barrier& bar, Settings& settings)
		: bar_(bar), settings_(settings) {
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

		controls_.push_back(new Control(settings.vision_radius, 0.0f, 0.25f, 1.0f));
		controls_.push_back(new Control(settings.vision_angle, 0.0f, 200.0f, 360.0f));
		controls_.push_back(new Control(settings.mean_velocity_fraction, 1.0f, 8.0f, 20.0f));
		controls_.push_back(new Control(settings.centre_of_mass_fraction, 1.0f, 45.0f, 90.0f));
		controls_.push_back(new Control(settings.repulsion_distance, 0.0f, 0.05f, 0.5f));
		controls_.push_back(new Control(settings.repulsion_fraction, 1.0f, 4.0f, 8.0f));
		controls_.push_back(new Control(settings.smooth_acceleration, 1.0f, 5.0f, 20.0f));
		controls_.push_back(new Control(settings.speed_limit, 0.0f, 0.03f, 0.2f));

		send_controls();
	}

	~Controls() {
		// FIXME: use auto_ptr
		for (ControlVector::iterator it = controls_.begin(); it != controls_.end(); ++it) {
			delete *it;
		}
	}

	void run(Context& ctx) {
		while (true) {
			bar_.sync(ctx); // Phase 1
			bar_.sync(ctx); // Phase 2

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

			bar_.sync(ctx); // Phase 3
		}
	}

private:
	void change_control(int num, float value) {
		if (num < 0 || num > controls_.size()) {
			return;
		}

		cout << "control " << num << " to " << value << endl;
		controls_[num]->set(value);
	}

	void reset_controls() {
		for (int i = 0; i < controls_.size(); ++i) {
			controls_[i]->reset();
		}
	}

	void send_controls() {
#ifdef HAVE_LIBPORTMIDI
		for (int i = 0; i < controls_.size(); ++i) {
			float value = controls_[i]->get();
			PmMessage msg = Pm_Message(176, 81 + i, int(value * 127));
			Pm_WriteShort(out_stream_, 0, msg);
		}
#endif
	}

	typedef vector<Control *> ControlVector;
	ControlVector controls_;
#ifdef HAVE_LIBPORTMIDI
	PortMidiStream* in_stream_;
	PortMidiStream* out_stream_;
#endif
	Barrier& bar_;
	Settings& settings_;

	static const int MAX_EVENTS = 1000;
};

class Ccoids : public Activity {
public:
	void run(Context& ctx) {
		cout << "ccoids starting" << endl;

		Barrier bar(ctx, 1);
		Settings settings;

		ViewerMap viewers;
		// FIXME viewers owned by this map

		// Set up the world.
		// We can do this privately before sharing it.
		World *w = new World;
		for (int x = 0; x < WIDTH_LOCATIONS; ++x) {
			for (int y = 0; y < HEIGHT_LOCATIONS; ++y) {
				const int id = loc_id(x, y);
				Shared<Viewer> *viewer = new Shared<Viewer>(ctx, new Viewer);
				w->add(ctx, new Shared<Location>(ctx, new Location(id, viewer)));
				viewers[id] = viewer;
			}
		}
		// This is slightly more complicated than in occam, because we
		// can't create all the interfaces (for the neighbours) before
		// spawning the servers -- so we need this second step.
		for (int x = 0; x < WIDTH_LOCATIONS; ++x) {
			for (int y = 0; y < HEIGHT_LOCATIONS; ++y) {
				int id = loc_id(x, y);
				Shared<Location> *loc = w->get(id);
				Claim<Location> c(ctx, *loc);
				Claim<Viewer> v(ctx, *viewers[id]);
				v->location(Vector<float>(0.0, 0.0), loc);
				for (int i = 0; i < NUM_DIRECTIONS; ++i) {
					Vector<int> dir = DIRECTIONS[i];
					int nx = (dir.x_ + x + WIDTH_LOCATIONS) % WIDTH_LOCATIONS;
					int ny = (dir.y_ + y + HEIGHT_LOCATIONS) % HEIGHT_LOCATIONS;
					Shared<Location> *n = w->get(loc_id(nx, ny));
					c->neighbour(i, n);
					v->location(Vector<float>(dir), n);
				}
			}
		}
		Shared<World> world(ctx, w);

		{
			Context f(ctx);

			// Creating the Controls initialises the settings.
			f.spawn(new Controls(bar.enroll(), settings));

			for (ViewerMap::iterator it = viewers.begin();
			     it != viewers.end();
			     ++it) {
				f.spawn(new ViewerUpdater(it->second, bar.enroll()));
			}

			for (int id = 0; id < BIRDS; id++) {
				Vector<float> pos(rand_float() * WIDTH_LOCATIONS,
				                  rand_float() * HEIGHT_LOCATIONS);
				Vector<int> pos_loc(pos);

				Shared<Location> *loc;
				{
					Claim<World> c(ctx, world);
					loc = c->get(loc_id(pos_loc.x_, pos_loc.y_));
				}

				AgentInfo info;
				info.id_ = id;
				info.pos_ = Vector<float>(pos.x_ - pos_loc.x_,
				                          pos.y_ - pos_loc.y_);

				float speed = rand_float() * MAX_INITIAL_SPEED;
				float dir = rand_float() * 4.0 * M_PI;
				info.vel_ = Vector<float>(speed * cos(dir),
				                          speed * sin(dir));

				f.spawn(new Boid(info, loc, bar.enroll(), settings));
			}

#ifndef USE_GTK_DISPLAY
			f.spawn(new SDLDisplay(world, bar));
#else
			f.spawn(new GtkDisplay(world, bar));
#endif
		}

		cout << "ccoids finished" << endl;
	}
};

int main(int argc, char *argv[]) {
#ifdef USE_GTK_DISPLAY
	Gtk::Main gtkmain(argc, argv);
#endif
	return initial_activity(argc, argv, new Ccoids);
}
