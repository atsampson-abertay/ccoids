/*
 *  display.hh - CoSMoS Demos
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

#ifndef DISPLAY_HH
#define DISPLAY_HH

#include "ccoids.hh"
#include "shared.hh"
#include "world.hh"

#include <SDL.h>

class Display {
public:
    Display(Shared<World>& world, Config& config);

    void update();

protected:
    virtual void draw_display() = 0;

    void fetch_agents();

    AIVector agents_;

    Shared<World>& world_;
    Config& config_;
};

class SDLDisplay : public Display {
public:
    SDLDisplay(Shared<World>& world, Config& config, Controls& controls);

protected:
    virtual void draw_display();

private:
    void handle_event(SDL_Event& event);
    void toggle_fullscreen();
    void quit();

    static const Uint32 BACKGROUND_COLOUR = 0x000000FF;

    // The width of a world unit, in pixels
    int scale_;

    SDL_Surface *screen_;
    Controls& controls_;
    int controls_counter_;
    bool fullscreen_;
};

#endif
