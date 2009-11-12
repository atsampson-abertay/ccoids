#include "shared.hh"
#include "contextimpl.hh"

Mutex::Mutex(Context& ctx) : ctx_(ctx) {
	word type = MT_SIMPLE | MT_MAKE_TYPE(MT_CB) | MT_CB_SHARED;
	data_ = MTAlloc(wptr(ctx), type, 0);
}

Mutex::~Mutex() {
	MTRelease(wptr(ctx_), data_);
}

void Mutex::claim(Context& ctx) {
	MTLock(wptr(ctx), data_, MT_CB_CLIENT);
}

void Mutex::release(Context& ctx) {
	MTUnlock(wptr(ctx), data_, MT_CB_CLIENT);
}
