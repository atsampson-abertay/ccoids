/*
 *  world.hh - CoSMoS Demos
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

#ifndef WORLD_HH
#define WORLD_HH

#include "ccoids.hh"
#include "shared.hh"

#include <map>
#include <vector>

class Location;
class Viewer;

typedef std::vector<AgentInfo> AIVector;
typedef std::map<int, AgentInfo> AIMap;
typedef std::map<int, Shared<Location> *> LocMap;

// A region of space, holding the agents that are currently in that region.
// The width of the region is the maximum view radius in the simulation.
class Location {
public:
    Location(int id, Shared<Viewer> *viewer);

    int id() const {
        return id_;
    }

    void neighbour(int dir, Shared<Location> *loc);
    Shared<Location> *enter(AgentInfo& info, Shared<Viewer> *& viewer);
    Shared<Location> *update(AgentInfo& info);
    void look(AIMap::const_iterator& begin, AIMap::const_iterator& end);

private:
    int id_;
    int id_counter_;
    Shared<Viewer> *viewer_;
    AIMap infos_;
    LocMap neighbours_;
};

// Compute the ID of a location from its position in the world.
int loc_id(int x, int y, const Config& config);

// The world: a collection of regions of space.
class World {
public:
    ~World();

    Shared<Location> *get(int id) {
        return locations_[id];
    }

    // The World owns the location after this.
    void add(Shared<Location> *loc);

private:
    LocMap locations_;
};

// A cache for the view from a single location.
// This keeps a copy of all the agent information for birds in this location
// and the surrounding location.
class Viewer {
public:
    void location(Vector<float> offset, Shared<Location> *loc);
    void update();
    void look(AIVector::const_iterator& begin, AIVector::const_iterator& end);

private:
    std::vector<Vector<float> > offsets_;
    std::vector<Shared<Location> *> locations_;
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

#endif
