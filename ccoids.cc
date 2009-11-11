// Boids in C++ using CCSP for concurrency.

#include "shared.hh"
#include "process.hh"
#include "barrier.hh"

#include <iostream>
#include <vector>
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

class AgentInfo {
public:
	AgentInfo(int id, float x, float y)
		: id_(id), local_id_(0), x_(x), y_(y), dx_(0.0), dy_(0.0) {
	}

	int id_;
	int local_id_;
	float x_, y_;
	float dx_, dy_;
};

class World {
public:
	World() : id_counter_(0) {
	}

	void enter(AgentInfo& info) {
		info.local_id_ = id_counter_++;
		infos_.push_back(info);
	}

	void update(AgentInfo& info) {
		// FIXME use a map!
		for (int i = 0; i < infos_.size(); i++) {
			if (infos_[i].local_id_ == info.local_id_) {
				infos_[i] = info;
				break;
			}
		}
	}

	void look(vector<AgentInfo> **infos) {
		// FIXME a bit ugly; could use an explicit Mobile
		*infos = &infos_;
	}

private:
	int id_counter_;
	vector<AgentInfo> infos_;
};

class Boid : public Activity {
public:
	Boid(AgentInfo info, Shared<World>& world, Barrier& bar)
		: info_(info), world_(world), bar_(bar) {
	}

protected:
	void run() {
		{
			Claim<World> c(*ctx_, world_);
			c->enter(info_);
		}

		float angle = 0.0;

		while (true) {
			bar_.sync(*ctx_); // Phase 1

			angle += 0.001;

			vector<AgentInfo> view;

			{
				Claim<World> c(*ctx_, world_);
				vector<AgentInfo> *infos;
				c->look(&infos);

				for (vector<AgentInfo>::iterator it = infos->begin(); it != infos->end(); ++it) {
					AgentInfo that = *it;

					if (that.id_ == info_.id_) {
						// This bird -- ignore
						continue;
					}

					// Compute relative position
					that.x_ -= info_.x_;
					that.y_ -= info_.y_;

					if ((that.x_ * that.x_) + (that.y_ * that.y_) > (VIEW_RADIUS * VIEW_RADIUS)) {
						// Too far away -- ignore
						continue;
					}

					view.push_back(that);
				}
			}

			bar_.sync(*ctx_); // Phase 2

			int seen = view.size();

			info_.dx_ = sin(angle) * WIND_FACTOR;
			info_.dy_ = cos(angle) * WIND_FACTOR;

			// Move towards centroid of visible flock
			float x = 0.0, y = 0.0;
			for (vector<AgentInfo>::iterator it = view.begin(); it != view.end(); ++it) {
				x += it->x_;
				y += it->y_;
			}
			if (seen > 0) {
				x /= view.size();
				y /= view.size();
			}
			info_.dx_ += x * ATTRACT_FACTOR;
			info_.dy_ += y * ATTRACT_FACTOR;

			// Move away from birds that are too close
			x = 0.0;
			y = 0.0;
			for (vector<AgentInfo>::iterator it = view.begin(); it != view.end(); ++it) {
				if ((it->x_ * it->x_) + (it->y_ * it->y_) < (AVOID_RADIUS * AVOID_RADIUS)) {
					x -= it->x_;
					y -= it->y_;
				}
			}
			info_.dx_ += x * AVOID_FACTOR;
			info_.dy_ += y * AVOID_FACTOR;

			// Match velocity
			x = 0.0;
			y = 0.0;
			for (vector<AgentInfo>::iterator it = view.begin(); it != view.end(); ++it) {
				x -= it->dx_;
				y -= it->dy_;
			}
			if (seen > 0) {
				x /= view.size();
				y /= view.size();
			}
			info_.dx_ += x * VELOCITY_FACTOR;
			info_.dy_ += y * VELOCITY_FACTOR;

			info_.x_ += info_.dx_;
			while (info_.x_ < -0.0) {
				info_.x_ += 1.0;
			}
			while (info_.x_ > 1.0) {
				info_.x_ -= 1.0;
			}
			info_.y_ += info_.dy_;
			while (info_.y_ < -0.0) {
				info_.y_ += 1.0;
			}
			while (info_.y_ > 1.0) {
				info_.y_ -= 1.0;
			}

			{
				Claim<World> c(*ctx_, world_);
				c->update(info_);
			}
		}
	}

private:
	AgentInfo info_;
	Shared<World>& world_;
	Barrier& bar_;
};

class Display : public Activity {
public:
	Display(Shared<World>& world, Barrier& bar)
		: world_(world), bar_(bar) {
	}

protected:
	void run() {
		if (SDL_Init(SDL_INIT_VIDEO) < 0) {
			exit(1);
		}
		atexit(SDL_Quit);

		int size = 400;

		SDL_WM_SetCaption("ccoids", "ccoids");
		SDL_Surface *display = SDL_SetVideoMode(size, size, 32, SDL_DOUBLEBUF);

		while (true) {
			bar_.sync(*ctx_); // Phase 1

			SDL_FillRect(display, NULL, 0);
			uint32_t *pixels = (uint32_t *) display->pixels;

			{
				Claim<World> c(*ctx_, world_);

				vector<AgentInfo> *infos;
				c->look(&infos);

				for (vector<AgentInfo>::iterator it = infos->begin(); it != infos->end(); ++it) {

					int x = it->x_ * size;
					int y = it->y_ * size;
					pixels[(y * size) + x] = 0xFFFFFF;
				}
			}

			bar_.sync(*ctx_); // Phase 2

			SDL_UpdateRect(display, 0, 0, 0, 0);
			SDL_Flip(display);
		}
	}

private:
	Shared<World>& world_;
	Barrier& bar_;
};

class Ccoids : public InitialActivity {
	void run() {
		cout << "ccoids starting" << endl;

		Barrier bar(*ctx_, 1);
		Shared<World> world(*ctx_, new World);
		{
			Forking f(*this);

			for (int id = 0; id < BIRDS; id++) {
				float x, y;
				x = rand() / (1.0 * RAND_MAX);
				y = rand() / (1.0 * RAND_MAX);

				AgentInfo info(id, x, y);
				f.fork(new Boid(info, world, bar.enroll()));
			}

			f.fork(new Display(world, bar));
		}

		cout << "ccoids finished" << endl;
	}
};

int main(int argc, char *argv[]) {
	return Ccoids().main(argc, argv);
}
