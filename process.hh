#ifndef PROCESS_HH
#define PROCESS_HH

class Context;

class Activity {
public:
	Activity();
	virtual ~Activity();
	void spawn(Activity *child);

protected:
	virtual void run() = 0;

	Context *ctx_;

	friend void run_activity(Activity *a);
	friend class Forking; // FIXME
	friend class ForkedActivity; // FIXME
};

class InitialActivity : public Activity {
public:
	InitialActivity();
	virtual ~InitialActivity();
	int main(int argc, char *argv[]);
};

class Forking {
public:
	Forking(Activity& act);
	~Forking();

	void fork(Activity *child);

private:
	Activity& act_;
	void *data_;
};

#endif
