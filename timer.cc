#include "timer.hh"
#include "contextimpl.hh"

Timer::Timer() : last_(0) {
}

Timer::~Timer() {
}

TimeVal Timer::read(Context &ctx) {
	return TimerRead(wptr(ctx));
}

void Timer::wait(Context &ctx, TimeVal until) {
	TimerWait(wptr(ctx), until);
}

void Timer::reset() {
	last_ = 0;
}

void Timer::delay(Context &ctx, TimeVal period) {
	if (last_ == 0) {
		last_ = read(ctx);
	}
	last_ = Time_PLUS(last_, period);
	wait(ctx, last_);
}

bool after(TimeVal a, TimeVal b) {
	return (a - b) > 0;
}
