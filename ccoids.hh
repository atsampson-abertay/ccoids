/*
 *  ccoids.hh - simulation definitions for ccoids
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

#ifndef CCOIDS_HH
#define CCOIDS_HH

#include "maths.hh"
#include "controls.hh"

#include <boost/shared_ptr.hpp>

// Settings for the whole simulation.
// These can't be changed after startup.
struct Config {
    int initial_birds;
    int initial_populations;
    float max_initial_speed;
    int width_locations;
    int height_locations;
    int display_height;
    int display_fps;
};

// Parameters for a boid's behaviour.
class Params : public Adjustable {
public:
    float vision_radius;
    float vision_angle;
    float mean_velocity_fraction;
    float centre_of_mass_fraction;
    float repulsion_distance;
    float repulsion_fraction;
    float smooth_acceleration;
    float speed_limit;

    float plumage;

    void adjust_with(Adjuster& adjust) {
        adjust(vision_radius, 0.0f, 0.25f, 1.0f);
        adjust(vision_angle, 0.0f, 200.0f, 360.0f);
        adjust(mean_velocity_fraction, 1.0f, 8.0f, 20.0f);
        adjust(centre_of_mass_fraction, 1.0f, 45.0f, 90.0f);
        adjust(repulsion_distance, 0.0f, 0.05f, 0.5f);
        adjust(repulsion_fraction, 1.0f, 4.0f, 8.0f);
        adjust(smooth_acceleration, 1.0f, 5.0f, 20.0f);
        adjust(speed_limit, 0.0f, 0.03f, 0.2f);

        adjust(plumage, 0.0f, 0.5f, 1.0f);
    }
};

const Vector<int> DIRECTIONS[] = {
    Vector<int>(-1, -1), Vector<int>(0, -1), Vector<int>(1, -1),
    Vector<int>(-1,  0),                     Vector<int>(1,  0),
    Vector<int>(-1,  1), Vector<int>(0,  1), Vector<int>(1,  1)
};
const int NUM_DIRECTIONS = 8;

class AgentInfo {
public:
    AgentInfo()
        : id_(-1), local_id_(-1), pos_(0.0, 0.0), vel_(0.0, 0.0),
          plumage_(0.0) {
    }

    int id_;
    int local_id_;
    Vector<float> pos_, vel_;
    float plumage_;
};

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

#endif
