/** Colour definitions. */

/*
 *  Copyright 2010 Adam Sampson
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

#ifndef COLOUR_HH
#define COLOUR_HH

#include <inttypes.h>

static uint8_t component_to_uint8(float f) {
    return f * 0xFF;
}

/*{{{  class Colour */
class Colour {
public:
    Colour() : r(0.0), g(0.0), b(0.0), a(0.0) {}
    Colour(float r, float g, float b) : r(r), g(g), b(b), a(1.0) {}
    Colour(float r, float g, float b, float a) : r(r), g(g), b(b), a(a) {}

    typedef float triple[3];
    void to_triple(triple& t) {
        t[0] = r;
        t[1] = g;
        t[2] = b;
    }

    typedef float quad[4];
    void to_quad(quad& q) {
        q[0] = r;
        q[1] = g;
        q[2] = b;
        q[3] = a;
    }

    uint32_t to_uint32() {
        return (component_to_uint8(r) << 24)
               | (component_to_uint8(g) << 16)
               | (component_to_uint8(b) << 8)
               | component_to_uint8(a);
    }
    uint32_t to_rgba() {
        union {
            uint32_t v;
            uint8_t c[4];
        } u;
        u.c[0] = component_to_uint8(r);
        u.c[1] = component_to_uint8(g);
        u.c[2] = component_to_uint8(b);
        u.c[3] = component_to_uint8(a);
        return u.v;
    }

    float r, g, b, a;
};
/*}}}*/

/** Convert an HSV colour triple to RGB.
    Based on: http://www.cs.rit.edu/~ncs/color/t_convert.html */
Colour hsv(float h, float s, float v);

#endif
