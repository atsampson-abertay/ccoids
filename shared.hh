/*
 *  shared.hh - CoSMoS Demos
 *  Adam Sampson
 *
 *  Copyright (C) 2009, 2012, 2013 Adam Sampson
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

#ifndef SHARED_HH
#define SHARED_HH

#include <tbb/queuing_mutex.h>

typedef tbb::queuing_mutex SharedMutex;

template <typename INNER> class Claim;

// A shared reference to an object.
template <typename INNER> class Shared {
public:
    Shared(INNER *inner) : inner_(inner) {
    }

    ~Shared() {
        delete inner_;
    }

private:
    INNER *inner_;
    SharedMutex mutex_;

    friend class Claim<INNER>;
};

// Claim a shared reference. The shared resource is claimed for as long as this
// is in scope.
template <typename INNER> class Claim {
public:
    Claim(Shared<INNER>& shared)
        : shared_(shared), lock_(shared.mutex_) {
    }

    INNER *operator->() {
        return shared_.inner_;
    }

private:
    Shared<INNER>& shared_;
    SharedMutex::scoped_lock lock_;
};

#endif
