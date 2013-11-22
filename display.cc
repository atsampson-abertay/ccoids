/*
 *  display.cc - CoSMoS Demos
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

#include "colour.hh"
#include "display.hh"

#include <SDL_gfxPrimitives.h>
#include <boost/foreach.hpp>

Display::Display(Shared<World>& world, Config& config)
    : world_(world), config_(config) {
}

void Display::update() {
    fetch_agents();
    draw_display();
}

void Display::fetch_agents() {
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

SDLDisplay::SDLDisplay(Shared<World>& world, Config& config, Controls& controls)
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

void SDLDisplay::draw_display() {
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

void SDLDisplay::handle_event(SDL_Event& event) {
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

void SDLDisplay::toggle_fullscreen() {
    SDL_WM_ToggleFullScreen(screen_);
    fullscreen_ = !fullscreen_;
}

void SDLDisplay::quit() {
    if (fullscreen_) {
        toggle_fullscreen();
    }
    exit(0);
}
