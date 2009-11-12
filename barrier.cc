#include "barrier.hh"
#include "contextimpl.hh"

Barrier::Barrier(Context& ctx, int count) : ctx_(ctx) {
	data_ = MTAlloc(wptr(ctx), MT_MAKE_BARRIER(MT_BARRIER_FULL), count);
}

Barrier::~Barrier() {
	MTRelease(wptr(ctx_), data_);
}

Barrier& Barrier::enroll() {
	MTEnroll(wptr(ctx_), data_, 1);
	return *this;
}

void Barrier::resign(Context& ctx) {
	MTResign(wptr(ctx_), data_, 1);
}

void Barrier::sync(Context& ctx) {
	MTSync(wptr(ctx), data_);
}
