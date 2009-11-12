#include "context.hh"
#include "contextimpl.hh"

Activity::Activity() {
}

Activity::~Activity() {
}

int Activity::stack_size() {
	return 65536;
}

ContextImpl::ContextImpl(Workspace wptr) : wptr_(wptr) {
	bar_ = (mt_barrier_t *) MTAlloc(wptr, MT_MAKE_BARRIER(MT_BARRIER_FORKING), 0);
}

ContextImpl::~ContextImpl() {
	MTSync(wptr_, bar_);
}

extern "C" {
	static void run_activity(Workspace wptr) {
		Activity *a = ProcGetParam(wptr, 0, Activity *);
		mt_barrier_t *bar = ProcGetParam(wptr, 1, mt_barrier_t *);

		{
			Context ctx(new ContextImpl(wptr));
			a->run(ctx);
		}
		delete a;

		if (bar == NULL) {
			Shutdown(wptr);
		} else {
			MTRelease(wptr, bar);
		}
	}
}

void ContextImpl::spawn(Activity *child) {
	Workspace child_wptr = ProcAlloc(wptr_, 2, child->stack_size());
	ProcParam(wptr_, child_wptr, 0, child);
	mt_barrier_t *bar = (mt_barrier_t *) MTClone(wptr_, bar_);
	ProcParam(wptr_, child_wptr, 1, bar);
	ProcStart(wptr_, child_wptr, run_activity);
}

Context::Context(ContextImpl *impl) : impl_(impl) {
}

Context::Context(Context& parent)
	: impl_(new ContextImpl(wptr(parent))) {
}

Context::~Context() {
	delete impl_;
}

void Context::spawn(Activity *act) {
	impl_->spawn(act);
}

int initial_activity(int argc, char *argv[], Activity *child) {
	if (!ccsp_init()) {
		return 1;
	}

	Workspace wptr = ProcAllocInitial(2, child->stack_size());
	ProcParam(wptr, wptr, 0, child);
	ProcParam(wptr, wptr, 1, NULL);
	ProcStartInitial(wptr, run_activity);

	// NOTREACHED
	return 0;
}
