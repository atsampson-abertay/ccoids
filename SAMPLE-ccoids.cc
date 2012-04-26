/*
 *  SAMPLE-ccoids.cc - CoSMoS Demos
 *  Adam Sampson
 *
 *  Copyright (C) 2009, Adam Sampson
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

#include "shared.hh"
#include "process.hh"

#include <iostream>

typedef enum {
    AT_BOID,
    AT_CYLINDER
} AgentType;

class Vector2 {
    float x, y;
};

class Position {
    Vector2 pos;
};

class AgentInfo {
    int id, local_id;
    AgentType type;
    Position position, update;
    Vector2 velocity;
    float radius;
    int colour;
};

class AgentUpdateResp {
    enum {
        AU_StayHere,
        AU_GoThere
    } type;
    union {
        Shared<Viewer> *viewer;
        Shared<Location> *location;
    } u;
};

class Location {
public:
    Location(int id) : id_(id), id_counter_(10000 * id) {
    }

    void enter(AgentInfo& info, AgentUpdateResp& resp) {
        info.local_id = id_counter_++;
        infos_.push_back(info);
        handle_agent_update(infos_.size() - 1, resp);
    }

private:
    void handle_agent_update(int n, AgentUpdateResp& resp) {
        int neighbour;
        compute_direction(id_, infos_[n].position, neighbour);
        if (neighbour == -1) {
            resp.type = AU_StayHere;
            resp.u.viewer = viewer;
        } else {
            resp.type = AU_GoThere;
            resp.u.location = neighbours_[neighbour];
        }
    }

    int id_;
    int id_counter_;
    vector<AgentInfo> infos_;
};

#if 0
class Test {
public:
    Test() : n_(0) {
        std::cout << "ctr" << std::endl;
    }

    ~Test() {
        std::cout << "del" << std::endl;
    }

    void foo() {
        std::cout << "foo! " << n_++ << std::endl;
    }

private:
    int n_;
};

class HelloActivity : public Activity {
public:
    HelloActivity(Shared<Test>& s) : s_(s) {
    }

    void run() {
        std::cout << "hello, world" << std::endl;

        {
            Claim<Test> c(*ctx_, s_);

            c->foo();
        }
    }

private:
    Shared<Test>& s_;
};
#endif

class Ccoids : public InitialActivity {
    void run() {
        std::cout << "I'm a CCSP process" << std::endl;

        Shared<Test> s(*ctx_, new Test);
        {
            Forking f(*this);
            for (int i = 0; i < 20; i++) {
                f.fork(new HelloActivity(s));
            }
        }

        std::cout << "fork finished" << std::endl;
    }
};

int main(int argc, char *argv[]) {
    return Ccoids().main(argc, argv);
}
