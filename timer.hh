#ifndef TIMER_HH
#define TIMER_HH

#include "context.hh"

typedef int TimeVal;

class Timer {
public:
	Timer();
	virtual ~Timer();
	TimeVal read(Context& ctx);
	void wait(Context& ctx, TimeVal until);
	void reset();
	void delay(Context& ctx, TimeVal period);

private:
	TimeVal last_;
};

bool after(TimeVal a, TimeVal b);

#endif
