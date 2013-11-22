/*
 *  world.cc - CoSMoS Demos
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

#include "world.hh"

Location::Location(int id, Shared<Viewer> *viewer)
    : id_(id), id_counter_(0), viewer_(viewer) {
}

void Location::neighbour(int dir, Shared<Location> *loc) {
    neighbours_[dir] = loc;
}

Shared<Location> *Location::enter(AgentInfo& info, Shared<Viewer> *& viewer) {
    info.local_id_ = id_counter_++;
    viewer = viewer_;
    return update(info);
}

Shared<Location> *Location::update(AgentInfo& info) {
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

void Location::look(AIMap::const_iterator& begin, AIMap::const_iterator& end) {
    // FIXME a bit ugly; could use an explicit Mobile
    begin = infos_.begin();
    end = infos_.end();
}

// FIXME Have a data type for this instead?
// FIXME Move into Config
int loc_id(int x, int y, const Config& config) {
    return (y * config.width_locations) + x;
}

World::~World() {
    for (LocMap::iterator it = locations_.begin(); it != locations_.end(); ++it) {
        delete it->second;
    }
}

void World::add(Shared<Location> *loc) {
    Claim<Location> c(*loc);
    locations_[c->id()] = loc;
}

void Viewer::location(Vector<float> offset, Shared<Location> *loc) {
    offsets_.push_back(offset);
    locations_.push_back(loc);
}

void Viewer::update() {
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

void Viewer::look(AIVector::const_iterator& begin, AIVector::const_iterator& end) {
    begin = infos_.begin();
    end = infos_.end();
}
