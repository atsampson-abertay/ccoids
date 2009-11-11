#ifndef SHARED_HH
#define SHARED_HH

class Context;

class Mutex {
public:
	Mutex(Context& ctx);
	~Mutex();
	void claim(Context& ctx);
	void release(Context& ctx);

private:
	Context& ctx_;
	void *data_;
};

template <typename INNER> class Claim;

// A shared reference to an object.
template <typename INNER> class Shared {
public:
	Shared(Context& ctx, INNER *inner) : inner_(inner), mutex_(ctx) {
	}

	~Shared() {
		delete inner_;
	}

private:
	INNER *inner_;
	Mutex mutex_;

	friend class Claim<INNER>;
};

// Claim a shared reference. The shared resource is claimed for as long as this
// is in scope.
template <typename INNER> class Claim {
public:
	Claim(Context& ctx, Shared<INNER>& shared)
		: shared_(shared), ctx_(ctx) {
		shared_.mutex_.claim(ctx_);
	}

	~Claim() {
		shared_.mutex_.release(ctx_);
	}

	INNER *operator->() {
		return shared_.inner_;
	}

private:
	Shared<INNER>& shared_;
	Context& ctx_;
};

#endif
