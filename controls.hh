/*
 *  controls.hh - tweakable controls for ccoids
 *  Adam Sampson
 *
 *  Copyright (C) 2011, Adam Sampson
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

#ifndef CONTROLS_HH
#define CONTROLS_HH

#include <vector>
#include <boost/shared_ptr.hpp>

class Adjustable;
class Adjuster;
class ControlDevice;
class Controls;

// Visitor interface for adjustable parameters.
class Adjuster {
public:
    virtual void operator()(float& value, float min,
                            float initial, float max) = 0;
};

// Interface for classes that have a set of adjustable parameters;
// the adjust_with method should call adjuster.adjust on each of them.
class Adjustable {
public:
    virtual void adjust_with(Adjuster& adjust) = 0;
};

class ControlDevice {
public:
    virtual void poll(Controls& controls) {
    }
    virtual void send_control(int control, float value) {
    }
};

class Controls {
public:
    typedef boost::shared_ptr<Adjustable> AdjustablePtr;
    typedef boost::shared_ptr<ControlDevice> DevicePtr;

    Controls();

    void add_and_init(AdjustablePtr adj);
    void poll();

    // FIXME: should these be private and friends-ified?
    void handle_set(int control, float value);
    void handle_reset();
    void handle_select(int num);

private:
    void adjust_selected_with(Adjuster& adjuster);

    typedef std::vector<AdjustablePtr> AdjustableVector;
    AdjustableVector adjustables_;

    bool selected_valid() {
        return (selected_ >= 0) && (selected_ < adjustables_.size());
    }
    int selected_;

    typedef std::vector<DevicePtr> DeviceVector;
    DeviceVector devices_;

    static const int MAX_CONTROLS = 16;
};

#endif
