/*
 *  ccoids.cc - CoSMoS Demos
 *  Adam Sampson
 *
 *  Copyright (C) 2009, 2011, 2012, 2013 Adam Sampson
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

#include "ccoids.hh"
#include "colour.hh"
#include "controls.hh"
#include "maths.hh"
#include "shared.hh"

#include <iostream>
#include <vector>
#include <map>
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <stdint.h>
#include <SDL.h>
#include <SDL_gfxPrimitives.h>
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#include <tbb/parallel_for_each.h>
#include <tbb/tick_count.h>

using namespace std;

class Activity {
public:
    // Phase 1  Viewers update
    virtual void phase1() {
    }

    // Phase 2  Agents look and compute
    virtual void phase2() {
    }

    // Phase 3  Agents send updates
    virtual void phase3() {
    }
};

typedef boost::shared_ptr<Activity> ActivityPtr;

void run_phase1(ActivityPtr& ap) {
    ap->phase1();
}

void run_phase2(ActivityPtr& ap) {
    ap->phase2();
}

void run_phase3(ActivityPtr& ap) {
    ap->phase3();
}

// A region of space, holding the agents that are currently in that region.
// The width of the region is the maximum view radius in the simulation.
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

// Compute the ID of a location from its position in the world.
// FIXME Have a data type for this instead?
// FIXME Move into Config
int loc_id(int x, int y, const Config& config) {
    return (y * config.width_locations) + x;
}

// The world: a collection of regions of space.
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
    void add(Shared<Location> *loc) {
        Claim<Location> c(*loc);
        locations_[c->id()] = loc;
    }

    Shared<Location> *get(int id) {
        return locations_[id];
    }

private:
    LocMap locations_;
};

// A cache for the view from a single location.
// This keeps a copy of all the agent information for birds in this location
// and the surrounding location.
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

    void update() {
        infos_.clear();
        for (int i = 0; i < offsets_.size(); ++i) {
            Claim<Location> c(*locations_[i]);
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
    ViewerUpdater(Shared<Viewer> *viewer)
        : viewer_(viewer) {
    }

    void phase1() {
        Claim<Viewer> c(*viewer_);
        c->update();
    }

private:
    Shared<Viewer> *viewer_;
};

// A boid.
class Boid : public Activity {
public:
    Boid(AgentInfo info, Shared<Location> *loc, Params& params)
        : info_(info), loc_(loc), params_(params) {
        do_update(true);
    }

    void do_update(bool enter) {
        while (true) {
            Claim<Location> c(*loc_);

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

    void phase2() {
        typedef vector<AgentInfo> AIVector;
        AIVector view;

        {
            Claim<Viewer> c(*viewer_);
            AIVector::const_iterator it, end;
            c->look(it, end);

            float my_angle = atan2f(info_.vel_.x_, info_.vel_.y_);
            const float max_r2 = params_.vision_radius * params_.vision_radius;
            const float max_diff = ((params_.vision_angle / 2.0) * M_PI) / 180.0;

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
            BOOST_FOREACH(AgentInfo &info, view) {
                com += info.pos_;
            }
            if (seen > 0) {
                com /= (float) seen;
            }
            accel += com / params_.centre_of_mass_fraction;
        }

        // Move away from birds that are too close
        {
            Vector<float> push(0.0, 0.0);
            BOOST_FOREACH(AgentInfo &info, view) {
                if (info.pos_.mag2() < (params_.repulsion_distance * params_.repulsion_distance)) {
                    push -= info.pos_;
                }
            }
            accel += push / params_.repulsion_fraction;
        }

        // Match velocity
        {
            Vector<float> perceived(0.0, 0.0);
            BOOST_FOREACH(AgentInfo &info, view) {
                perceived += info.vel_;
            }
            if (seen > 0) {
                perceived /= (float) seen;
            }
            perceived -= info_.vel_;
            accel += perceived / params_.mean_velocity_fraction;
        }

        info_.vel_ += accel / params_.smooth_acceleration;

        // Apply speed limit
        float mag = info_.vel_.mag2();
        const float speed_limit2 = params_.speed_limit * params_.speed_limit;
        if (mag > speed_limit2) {
            info_.vel_ /= mag / speed_limit2;
        }

        info_.pos_ += info_.vel_;

        info_.plumage_ = params_.plumage;
    }

    void phase3() {
        do_update(false);
    }

private:
    AgentInfo info_;
    Shared<Location> *loc_;
    Shared<Viewer> *viewer_;
    Params& params_;
};

class Display : public Activity {
public:
    Display(Shared<World>& world, Config& config)
        : world_(world), config_(config), last_(tbb::tick_count::now()), period_(0.0) {
    }

    void phase2() {
        tbb::tick_count now = tbb::tick_count::now();

        if ((now - last_).seconds() >= period_) {
            fetch_agents();
            draw_display();

            last_ = now;
            period_ = 1.0 / config_.display_fps;
        }
    }

protected:
    virtual void draw_display() = 0;

    void fetch_agents() {
        agents_.clear();
        for (int x = 0; x < config_.width_locations; ++x) {
            for (int y = 0; y < config_.height_locations; ++y) {
                Vector<float> offset(x, y);

                Shared<Location> *loc;
                {
                    Claim<World> c(world_);
                    loc = c->get(loc_id(x, y, config_));
                }
                Claim<Location> c(*loc);

                AIMap::const_iterator it, end;
                c->look(it, end);

                for (; it != end; ++it) {
                    agents_.push_back(it->second);
                    AgentInfo& info(agents_.back());

                    info.pos_ += offset;
                }
            }
        }
    }

    AIVector agents_;

    Shared<World>& world_;
    Config& config_;
    tbb::tick_count last_;
    double period_;
};

class SDLDisplay : public Display {
public:
    SDLDisplay(Shared<World>& world, Config& config,
               Controls& controls)
        : Display(world, config), controls_(controls), fullscreen_(false) {
        if (SDL_Init(SDL_INIT_VIDEO) < 0) {
            exit(1);
        }
        atexit(SDL_Quit);

        scale_ = config_.display_height / config_.height_locations;

        SDL_WM_SetCaption("ccoids", "ccoids");
        screen_ = SDL_SetVideoMode(config_.width_locations * scale_,
                                   config_.height_locations * scale_,
                                   32, SDL_DOUBLEBUF);
    }

protected:
    virtual void draw_display() {
        // We must handle at least SDL_QUIT events, else our program won't
        // exit on SIGINT...
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            handle_event(event);
        }

        boxColor(screen_,
                 0, 0,
                 config_.width_locations * scale_,
                 config_.height_locations * scale_,
                 BACKGROUND_COLOUR);

        // Draw transparent shadows behind the blobs.
        const int blob_size = 0.05 * scale_;
        BOOST_FOREACH(AgentInfo& info, agents_) {
            Colour colour = hsv(info.plumage_, 0.7, 1.0);
            colour.a = 0.1;

            Vector<int> pos(info.pos_ * scale_);
            filledCircleColor(screen_, pos.x_, pos.y_, blob_size,
                              colour.to_uint32());
        }

        // Draw all the tails.
        BOOST_FOREACH(AgentInfo& info, agents_) {
            Colour colour = hsv(info.plumage_, 1.0, 1.0);
            colour.a = 0.4;

            Vector<int> pos(info.pos_ * scale_);
            Vector<int> tail((info.pos_ - info.vel_ * 4.0) * scale_);
            lineColor(screen_, pos.x_, pos.y_, tail.x_, tail.y_,
                      colour.to_uint32());
        }

        // Draw all the blobs.
        const int boid_size = 0.02 * scale_;
        BOOST_FOREACH(AgentInfo& info, agents_) {
            Colour colour = hsv(info.plumage_, 0.5, 1.0);
            colour.a = 0.8;

            Vector<int> pos(info.pos_ * scale_);
            filledCircleColor(screen_, pos.x_, pos.y_, boid_size,
                              colour.to_uint32());
        }

        // FIXME: reimplement this
#if 0
        const int cc_max = 50;
        if (controls_.changed()) {
            controls_counter_ = cc_max;
        }

        if (controls_counter_ > 0) {
            --controls_counter_;
            const int alpha = (0xFF * controls_counter_) / cc_max;
            const Uint32 bg = 0x4040A000 | alpha;
            const Uint32 marker = 0xA0A0FF00 | alpha;
            const Uint32 cursor = 0xFFFFFF00 | alpha;

            Controls::StateVector states = controls_.states();
            for (int i = 0; i < states.size(); ++i) {
                const int l = (i * 30) + 20;
                const int r = l + 25;
                const int t = 20;
                const int h = 100;
                boxColor(screen_, l, t, r, t + h, bg);
                hlineColor(screen_, l, r, t + (h * (1.0 - states[i].initial_)), marker);
                hlineColor(screen_, l, r, t + (h * (1.0 - states[i].value_)), cursor);
            }
        }
#endif

        SDL_UpdateRect(screen_, 0, 0, 0, 0);
        SDL_Flip(screen_);
    }

private:
    void handle_event(SDL_Event& event) {
        if (event.type == SDL_QUIT) {
            quit();
        }
        if (event.type == SDL_KEYDOWN) {
            SDL_keysym &sym(event.key.keysym);
            bool alt_down = (sym.mod & KMOD_ALT) != 0;

            if (sym.sym == SDLK_ESCAPE) {
                quit();
            }
            if (sym.sym == SDLK_RETURN && alt_down) {
                toggle_fullscreen();
            }
        }
    }

    void toggle_fullscreen() {
        SDL_WM_ToggleFullScreen(screen_);
        fullscreen_ = !fullscreen_;
    }

    void quit() {
        if (fullscreen_) {
            toggle_fullscreen();
        }
        exit(0);
    }

    static const Uint32 BACKGROUND_COLOUR = 0x000000FF;

    // The width of a world unit, in pixels
    int scale_;

    SDL_Surface *screen_;
    Controls& controls_;
    int controls_counter_;
    bool fullscreen_;
};

// FIXME: I'm not wild about using MI here
// FIXME: this is also a phase adapter
class BoidControls : public Controls, public Activity {
public:
    void phase2() {
        poll();
    }
};

// The simulation.
class Ccoids {
public:
    Ccoids(const Config& config)
        : config_(config) {
    }

    int run() {
        cout << "ccoids starting" << endl;

        ViewerMap viewers;
        // FIXME viewers owned by this map

        // Set up the world.
        // We can do this privately before sharing it.
        World *w = new World;
        for (int x = 0; x < config_.width_locations; ++x) {
            for (int y = 0; y < config_.height_locations; ++y) {
                const int id = loc_id(x, y, config_);
                Shared<Viewer> *viewer = new Shared<Viewer>(new Viewer);
                w->add(new Shared<Location>(new Location(id, viewer)));
                viewers[id] = viewer;
            }
        }
        // This is slightly more complicated than in occam, because we
        // can't create all the interfaces (for the neighbours) before
        // spawning the servers -- so we need this second step.
        for (int x = 0; x < config_.width_locations; ++x) {
            for (int y = 0; y < config_.height_locations; ++y) {
                int id = loc_id(x, y, config_);
                Shared<Location> *loc = w->get(id);
                Claim<Location> c(*loc);
                Claim<Viewer> v(*viewers[id]);
                v->location(Vector<float>(0.0, 0.0), loc);
                for (int i = 0; i < NUM_DIRECTIONS; ++i) {
                    Vector<int> dir = DIRECTIONS[i];
                    int nx = (dir.x_ + x + config_.width_locations) % config_.width_locations;
                    int ny = (dir.y_ + y + config_.height_locations) % config_.height_locations;
                    Shared<Location> *n = w->get(loc_id(nx, ny, config_));
                    c->neighbour(i, n);
                    v->location(Vector<float>(dir), n);
                }
            }
        }
        Shared<World> world(w);

        BoidControls *controls = new BoidControls();

        std::vector<ParamsPtr> params;
        for (int i = 0; i < config_.initial_populations; ++i) {
            ParamsPtr p(new Params);
            params.push_back(p);
            controls->add_and_init(p);
        }

        {
            ActivityPtr ap(controls);
            activities_.push_back(ap);
        }

        for (ViewerMap::iterator it = viewers.begin();
             it != viewers.end();
             ++it) {
            ActivityPtr ap(new ViewerUpdater(it->second));
            activities_.push_back(ap);
        }

        const int count = config_.initial_birds / params.size();
        BOOST_FOREACH(ParamsPtr& p, params) {
            add_boids(world, count, *p);
        }

        {
            ActivityPtr ap(new SDLDisplay(world, config_, *controls));
            activities_.push_back(ap);
        }

        // Run the activities.
        while (true) {
            tbb::parallel_for_each(activities_.begin(), activities_.end(), run_phase1);
            tbb::parallel_for_each(activities_.begin(), activities_.end(), run_phase2);
            tbb::parallel_for_each(activities_.begin(), activities_.end(), run_phase3);
        }

        cout << "ccoids finished" << endl;
        return 0;
    }

private:
    typedef boost::shared_ptr<Params> ParamsPtr;

    void add_boids(Shared<World>& world, int count, Params& params) {
        for (int id = 0; id < count; id++) {
            Vector<float> pos(rand_float() * config_.width_locations,
                      rand_float() * config_.height_locations);
            Vector<int> pos_loc(pos);

            Shared<Location> *loc;
            {
                Claim<World> c(world);
                loc = c->get(loc_id(pos_loc.x_, pos_loc.y_, config_));
            }

            AgentInfo info;
            info.id_ = id;
            info.pos_ = Vector<float>(pos.x_ - pos_loc.x_,
                          pos.y_ - pos_loc.y_);

            float speed = rand_float() * config_.max_initial_speed;
            float dir = rand_float() * 4.0 * M_PI;
            info.vel_ = Vector<float>(speed * cos(dir),
                          speed * sin(dir));

            ActivityPtr ap(new Boid(info, loc, params));
            activities_.push_back(ap);
        }
    }

    Config config_;
    std::vector<ActivityPtr> activities_;
};

void parse_options(int argc, char *argv[], Config& config) {
    namespace opts = boost::program_options;

    opts::options_description desc("Options");
#define simple(Type, Name, Default) opts::value<Type>(&config.Name)->default_value(Default)
    desc.add_options()
        ("help", "display this help and exit")
        ("birds", simple(int, initial_birds, 500),
         "initial number of birds")
        ("populations", simple(int, initial_populations, 1),
         "initial number of populations")
        ("max-initial-speed", simple(float, max_initial_speed, 0.1),
         "initial speed of birds")
        ("world-width", simple(int, width_locations, 8),
         "width of world in units")
        ("world-height", simple(int, height_locations, 5),
         "height of world in units")
        ("display-height", simple(int, display_height, 550),
         "height of display window")
        ("display-fps", simple(int, display_fps, 25),
         "frames per second to display")
        ;
#undef simple

    opts::variables_map vars;
    opts::store(opts::parse_command_line(argc, argv, desc), vars);
    opts::notify(vars);

    if (vars.count("help")) {
        /*{{{  show help */
        std::cout << desc << std::endl;
        exit(0);
        /*}}}*/
    }
}

int main(int argc, char *argv[]) {
    Config config;
    parse_options(argc, argv, config);

    Ccoids ccoids(config);
    return ccoids.run();
}
