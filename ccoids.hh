/*
 *  ccoids.cc - simulation definitions for ccoids
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

#ifndef CCOIDS_HH
#define CCOIDS_HH

#include "maths.hh"
#include "shared.hh"

#include <vector>
#include <map>

struct Settings {
	/*{{{  settings that can't change after startup */
	int initial_birds;
	float max_initial_speed;
	int width_locations;
	int height_locations;
	int display_height;
	int display_fps;
	/*}}}*/
	/*{{{ runtime-tweakable settings  */
	float vision_radius;
	float vision_angle;
	float mean_velocity_fraction;
	float centre_of_mass_fraction;
	float repulsion_distance;
	float repulsion_fraction;
	float smooth_acceleration;
	float speed_limit;
	/*}}}*/
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
		: id_(-1), local_id_(-1), pos_(0.0, 0.0), vel_(0.0, 0.0) {
	}

	int id_;
	int local_id_;
	Vector<float> pos_, vel_;
};

class Location;
class Viewer;
typedef std::vector<AgentInfo> AIVector;
typedef std::map<int, AgentInfo> AIMap;
typedef std::map<int, Shared<Location> *> LocMap;
typedef std::map<int, Vector<float> > VecMap;
typedef std::map<int, Shared<Viewer> *> ViewerMap;

#endif
