#include "process.hh"
#include "runtime.hh"

void run_activity(Activity *a) {
	a->run();
	delete a;
}

extern "C" {
	static void run_activity(Workspace wptr) {
		Activity *a = ProcGetParam(wptr, 0, Activity *);
		run_activity(a);
	}
}

Activity::Activity() {
	ctx_ = new Context;
}

Activity::~Activity() {
	delete ctx_;
}

void Activity::spawn(Activity *child) {
	Workspace wptr = ctx_->wptr();
	Workspace child_wptr = ProcAlloc(wptr, 1, 65536);
	child->ctx_->wptr(child_wptr);
	ProcParam(wptr, child_wptr, 0, child);
	ProcStart(wptr, child_wptr, run_activity);
}

InitialActivity::InitialActivity() : Activity() {
}

InitialActivity::~InitialActivity() {
	Shutdown(ctx_->wptr());
}

int InitialActivity::main(int argc, char *argv[]) {
	if (!ccsp_init()) {
		return 1;
	}

	ctx_->wptr(ProcAllocInitial(1, 65536));
	ProcParam(ctx_->wptr(), ctx_->wptr(), 0, this);
	ProcStartInitial(ctx_->wptr(), run_activity);

	// NOTREACHED
	return 0;
}

class ForkedActivity : public Activity {
public:
	ForkedActivity(Activity *child, mt_barrier_t *bar)
		: child_(child), bar_(bar) {
	}

protected:
	void run() {
		// FIXME This is very ugly
		child_->ctx_->wptr(ctx_->wptr());
		child_->run();
		delete child_;

		MTRelease(ctx_->wptr(), bar_);
	}

private:
	Activity *child_;
	mt_barrier_t *bar_;
};

Forking::Forking(Activity& act) : act_(act) {
	data_ = MTAlloc(act_.ctx_->wptr(),
	                MT_MAKE_BARRIER(MT_BARRIER_FORKING), 0);
}

Forking::~Forking() {
	MTSync(act_.ctx_->wptr(), data_);
}

void Forking::fork(Activity *child) {
	mt_barrier_t *bar = (mt_barrier_t *) MTClone(act_.ctx_->wptr(), data_);
	act_.spawn(new ForkedActivity(child, bar));
}
