#ifndef RUNTIME_HH
#define RUNTIME_HH

extern "C" {
#include <ccsp_cif.h>
}

class Context {
public:
	Context() : wptr_(0) {}

	inline Workspace wptr() {
		return wptr_;
	}

	inline void wptr(Workspace v) {
		wptr_ = v;
	}

private:
	Workspace wptr_;
};

#endif
