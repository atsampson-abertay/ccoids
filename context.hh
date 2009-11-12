#ifndef CONTEXT_HH
#define CONTEXT_HH

class Activity;
class Context;
class ContextImpl;

class Activity {
public:
	Activity();
	virtual ~Activity();
	virtual void run(Context& ctx) = 0;
	virtual int stack_size();
};

class Context {
public:
	Context(ContextImpl *impl);
	Context(Context& parent);
	virtual ~Context();
	void spawn(Activity *child);

	ContextImpl *impl_;

	// FIXME Can we get rid of this somehow?
	friend ContextImpl *context_impl(Context&);
};

int initial_activity(int argc, char *argv[], Activity *child);

#endif
