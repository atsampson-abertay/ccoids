// Boids in C++ using CCSP for concurrency.

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

using namespace std;

const int BIRDS = 1000;
const int WIDTH_LOCATIONS = 6;
const int HEIGHT_LOCATIONS = 4;
const float VIEW_RADIUS = 0.5;
const float ATTRACT_FACTOR = 0.02;
const float AVOID_RADIUS = 0.01;
const float AVOID_FACTOR = 0.25;
const float VELOCITY_FACTOR = 0.125;
const float WIND_FACTOR = 0.001;

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
typedef map<int, AgentInfo> AIMap;
typedef map<int, Shared<Location> *> LocMap;

class Location {
public:
	Location(int id) : id_(id), id_counter_(0) {
	}

	int id() const {
		return id_;
	}

	void neighbour(int dir, Shared<Location> *loc) {
		neighbours_[dir] = loc;
	}

	Shared<Location> *enter(AgentInfo& info) {
		info.local_id_ = id_counter_++;
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

class Boid : public Activity {
public:
	Boid(AgentInfo info, Shared<Location> *loc, Barrier& bar)
		: info_(info), loc_(loc), bar_(bar) {
	}

	void do_update(Context &ctx, bool enter) {
		while (true) {
			Claim<Location> c(ctx, *loc_);

			Shared<Location> *new_loc;
			if (enter) {
				new_loc = c->enter(info_);
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

		float angle = 0.0;

		while (true) {
			bar_.sync(ctx); // Phase 1

			angle += 0.001;

			typedef vector<AgentInfo> AIVector;
			AIVector view;

			{
				Claim<Location> c(ctx, *loc_);
				AIMap::const_iterator it, end;
				c->look(it, end);

				for (; it != end; ++it) {
					AgentInfo that = it->second;

					if (that.id_ == info_.id_) {
						// This bird -- ignore
						continue;
					}

					// Compute relative position
					that.pos_ -= info_.pos_;

					if (that.pos_.mag2() > (VIEW_RADIUS * VIEW_RADIUS)) {
						// Too far away -- ignore
						continue;
					}

					view.push_back(that);
				}
			}

			bar_.sync(ctx); // Phase 2

			int seen = view.size();

			info_.vel_ = Vector<float>(sin(angle) * WIND_FACTOR,
			                           cos(angle) * WIND_FACTOR);

			// Move towards centroid of visible flock
			{
				Vector<float> accel(0.0, 0.0);
				for (AIVector::iterator it = view.begin(); it != view.end(); ++it) {
					accel += it->pos_;
				}
				if (seen > 0) {
					accel /= (float) view.size();
				}
				info_.vel_ += accel * ATTRACT_FACTOR;
			}

			// Move away from birds that are too close
			{
				Vector<float> accel(0.0, 0.0);
				for (AIVector::iterator it = view.begin(); it != view.end(); ++it) {
					if (it->pos_.mag2() < (AVOID_RADIUS * AVOID_RADIUS)) {
						accel -= it->pos_;
					}
				}
				info_.vel_ += accel * AVOID_FACTOR;
			}

			// Match velocity
			{
				Vector<float> accel(0.0, 0.0);
				for (AIVector::iterator it = view.begin(); it != view.end(); ++it) {
					accel -= it->vel_;
				}
				if (seen > 0) {
					accel /= (float) view.size();
				}
				info_.vel_ += accel * VELOCITY_FACTOR;
			}

			info_.pos_ += info_.vel_;

			do_update(ctx, false);
		}
	}

private:
	AgentInfo info_;
	Shared<Location> *loc_;
	Barrier& bar_;
};

// We must have something that handles SDL_QUIT events, else our program won't
// exit on SIGINT...
class EventHandler : public Activity {
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

class Display : public Activity {
public:
	Display(Shared<World>& world, Barrier& bar)
		: world_(world), bar_(bar) {
	}

	void run(Context& ctx) {
		if (SDL_Init(SDL_INIT_VIDEO) < 0) {
			exit(1);
		}
		atexit(SDL_Quit);

		ctx.spawn(new EventHandler);

		int scale = 500 / HEIGHT_LOCATIONS;

		SDL_WM_SetCaption("ccoids", "ccoids");
		SDL_Surface *display = SDL_SetVideoMode(WIDTH_LOCATIONS * scale,
		                                        HEIGHT_LOCATIONS * scale,
		                                        32, SDL_DOUBLEBUF);

		while (true) {
			bar_.sync(ctx); // Phase 1

			SDL_FillRect(display, NULL, 0);
			uint32_t *pixels = (uint32_t *) display->pixels;

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

						Vector<int> p(info.pos_ * (float) scale);
						p += Vector<int>(x, y) * scale;
						pixels[(p.y_ * scale * WIDTH_LOCATIONS) + p.x_] = 0xFFFFFF;
					}
				}
			}

			bar_.sync(ctx); // Phase 2

			SDL_UpdateRect(display, 0, 0, 0, 0);
			SDL_Flip(display);
		}
	}

private:
	Shared<World>& world_;
	Barrier& bar_;
};

class Ccoids : public Activity {
	void run(Context& ctx) {
		cout << "ccoids starting" << endl;

		Barrier bar(ctx, 1);

		// Set up the world.
		// We can do this privately before sharing it.
		World *w = new World;
		for (int x = 0; x < WIDTH_LOCATIONS; ++x) {
			for (int y = 0; y < HEIGHT_LOCATIONS; ++y) {
				w->add(ctx, new Shared<Location>(ctx, new Location(loc_id(x, y))));
			}
		}
		// This is slightly more complicated than in occam, because we
		// can't create all the interfaces (for the neighbours) before
		// spawning the servers -- so we need this second step.
		for (int x = 0; x < WIDTH_LOCATIONS; ++x) {
			for (int y = 0; y < HEIGHT_LOCATIONS; ++y) {
				Shared<Location> *loc = w->get(loc_id(x, y));
				Claim<Location> c(ctx, *loc);
				for (int i = 0; i < NUM_DIRECTIONS; ++i) {
					Vector<int> dir = DIRECTIONS[i];
					int nx = (dir.x_ + x + WIDTH_LOCATIONS) % WIDTH_LOCATIONS;
					int ny = (dir.y_ + y + HEIGHT_LOCATIONS) % HEIGHT_LOCATIONS;
					Shared<Location> *n = w->get(loc_id(nx, ny));
					c->neighbour(i, n);
				}
			}
		}
		Shared<World> world(ctx, w);

		{
			Context f(ctx);

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

				f.spawn(new Boid(info, loc, bar.enroll()));
			}

			f.spawn(new Display(world, bar));
		}

		cout << "ccoids finished" << endl;
	}
};

int main(int argc, char *argv[]) {
	return initial_activity(argc, argv, new Ccoids);
}
