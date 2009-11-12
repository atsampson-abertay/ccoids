#ifndef BARRIER_HH
#define BARRIER_HH

#include "context.hh"

class Barrier {
public:
	Barrier(Context& ctx, int count);
	~Barrier();
	Barrier& enroll();
	void resign(Context& ctx);
	void sync(Context& ctx);

private:
	Context& ctx_;
	void *data_;
};

#endif
