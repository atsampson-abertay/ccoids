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

// FIXME no wrapping at the moment!

const int BIRDS = 300;
const float VIEW_RADIUS = 0.2;
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

	ELEMENT mag2() const {
		return (x_ * x_) + (y_ * y_);
	}

	ELEMENT x_, y_;
};

class AgentInfo {
public:
	AgentInfo()
		: id_(-1), local_id_(-1), pos_(0.0, 0.0), vel_(0.0, 0.0) {
	}

	int id_;
	int local_id_;
	Vector<float> pos_, vel_;
};

typedef map<int, AgentInfo> AIMap;
class World {
public:
	World() : id_counter_(0) {
	}

	void enter(AgentInfo& info) {
		info.local_id_ = id_counter_++;
		infos_[info.local_id_] = info;
	}

	void update(AgentInfo& info) {
		infos_[info.local_id_] = info;
	}

	void look(AIMap::const_iterator& begin,
	          AIMap::const_iterator& end) {
		// FIXME a bit ugly; could use an explicit Mobile
		begin = infos_.begin();
		end = infos_.end();
	}

private:
	int id_counter_;
	AIMap infos_;
};

class Boid : public Activity {
public:
	Boid(AgentInfo info, Shared<World>& world, Barrier& bar)
		: info_(info), world_(world), bar_(bar) {
	}

	void run(Context& ctx) {
		{
			Claim<World> c(ctx, world_);
			c->enter(info_);
		}

		float angle = 0.0;

		while (true) {
			bar_.sync(ctx); // Phase 1

			angle += 0.001;

			typedef vector<AgentInfo> AIVector;
			AIVector view;

			{
				Claim<World> c(ctx, world_);
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
			while (info_.pos_.x_ < -0.0) {
				info_.pos_.x_ += 1.0;
			}
			while (info_.pos_.x_ > 1.0) {
				info_.pos_.x_ -= 1.0;
			}
			while (info_.pos_.y_ < -0.0) {
				info_.pos_.y_ += 1.0;
			}
			while (info_.pos_.y_ > 1.0) {
				info_.pos_.y_ -= 1.0;
			}

			{
				Claim<World> c(ctx, world_);
				c->update(info_);
			}
		}
	}

private:
	AgentInfo info_;
	Shared<World>& world_;
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

		int size = 400;

		SDL_WM_SetCaption("ccoids", "ccoids");
		SDL_Surface *display = SDL_SetVideoMode(size, size, 32, SDL_DOUBLEBUF);

		while (true) {
			bar_.sync(ctx); // Phase 1

			SDL_FillRect(display, NULL, 0);
			uint32_t *pixels = (uint32_t *) display->pixels;

			{
				Claim<World> c(ctx, world_);

				AIMap::const_iterator it, end;
				c->look(it, end);

				for (; it != end; ++it) {
					const AgentInfo& info = it->second;

					Vector<int> p(info.pos_ * (float) size);
					pixels[(p.y_ * size) + p.x_] = 0xFFFFFF;
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
		Shared<World> world(ctx, new World);
		{
			Context f(ctx);

			for (int id = 0; id < BIRDS; id++) {
				AgentInfo info;
				info.id_ = id;
				info.pos_ = Vector<float>(rand() / (1.0 * RAND_MAX),
				                          rand() / (1.0 * RAND_MAX));

				f.spawn(new Boid(info, world, bar.enroll()));
			}

			f.spawn(new Display(world, bar));
		}

		cout << "ccoids finished" << endl;
	}
};

int main(int argc, char *argv[]) {
	return initial_activity(argc, argv, new Ccoids);
}
