/*
 *  boid.cc - CoSMoS Demos
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
#include "world.hh"

#include <boost/foreach.hpp>
#include <cmath>
#include <vector>

Boid::Boid(AgentInfo info, Shared<Location> *loc, Params& params)
    : info_(info), loc_(loc), params_(params) {
    do_update(true);
}

void Boid::phase2() {
    typedef std::vector<AgentInfo> AIVector;
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

void Boid::phase3() {
    do_update(false);
}

void Boid::do_update(bool enter) {
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
