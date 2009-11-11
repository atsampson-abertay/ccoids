#include "runtime.hh"
#include "barrier.hh"

Barrier::Barrier(Context& ctx, int count) : ctx_(ctx) {
	data_ = MTAlloc(ctx.wptr(), MT_MAKE_BARRIER(MT_BARRIER_FULL), count);
}

Barrier::~Barrier() {
	MTRelease(ctx_.wptr(), data_);
}

Barrier& Barrier::enroll() {
	MTEnroll(ctx_.wptr(), data_, 1);
	return *this;
}

void Barrier::resign(Context& ctx) {
	MTResign(ctx_.wptr(), data_, 1);
}

void Barrier::sync(Context& ctx) {
	MTSync(ctx.wptr(), data_);
}
