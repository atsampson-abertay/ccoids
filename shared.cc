#include "shared.hh"
#include "runtime.hh"

#include <iostream>

Mutex::Mutex(Context& ctx) : ctx_(ctx) {
	word type = MT_SIMPLE | MT_MAKE_TYPE(MT_CB) | MT_CB_SHARED;
	data_ = MTAlloc(ctx.wptr(), type, 0);
}

Mutex::~Mutex() {
	MTRelease(ctx_.wptr(), data_);
}

void Mutex::claim(Context& ctx) {
	MTLock(ctx.wptr(), data_, MT_CB_CLIENT);
}

void Mutex::release(Context& ctx) {
	MTUnlock(ctx.wptr(), data_, MT_CB_CLIENT);
}
