#ifndef CONTEXTIMPL_HH
#define CONTEXTIMPL_HH

#include "context.hh"

extern "C" {
#include <ccsp_cif.h>
}

class ContextImpl {
public:
	ContextImpl(Workspace wptr);
	virtual ~ContextImpl();
	void spawn(Activity *child);

private:
	Workspace wptr_;
	mt_barrier_t *bar_;

	friend Workspace wptr(Context& ctx);
};

inline ContextImpl *context_impl(Context& ctx) {
	return ctx.impl_;
}

inline Workspace wptr(Context& ctx) {
	return context_impl(ctx)->wptr_;
}

#endif
