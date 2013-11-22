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

#include "boid.hh"
#include "ccoids.hh"
#include "controls.hh"
#include "display.hh"
#include "maths.hh"
#include "shared.hh"
#include "world.hh"

#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#include <cstdlib>
#include <iostream>
#include <map>
#include <tbb/parallel_for_each.h>
#include <tbb/tick_count.h>
#include <vector>

void run_phase1(ActivityPtr& ap) {
    ap->phase1();
}

void run_phase2(ActivityPtr& ap) {
    ap->phase2();
}

void run_phase3(ActivityPtr& ap) {
    ap->phase3();
}

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
        std::cout << "ccoids starting" << std::endl;

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

        SDLDisplay display(world, config_, *controls);

        tbb::tick_count last_display;
        const double display_period = 1.0 / config_.display_fps;

        while (true) {
            // Does the display need updating?
            tbb::tick_count now = tbb::tick_count::now();
            if ((now - last_display).seconds() >= display_period) {
                display.update();
                last_display = now;
            }

            // Run the activities.
            tbb::parallel_for_each(activities_.begin(), activities_.end(), run_phase1);
            tbb::parallel_for_each(activities_.begin(), activities_.end(), run_phase2);
            tbb::parallel_for_each(activities_.begin(), activities_.end(), run_phase3);
        }

        std::cout << "ccoids finished" << std::endl;
        return 0;
    }

private:
    typedef std::map<int, Shared<Viewer> *> ViewerMap;
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
