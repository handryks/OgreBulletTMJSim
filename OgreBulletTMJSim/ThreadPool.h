#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>

class ThreadPool;

// our worker thread objects
class Worker {
public:
	Worker(ThreadPool &s) : pool(s) { }
	void operator()();
private:
	ThreadPool &pool;
};

// the actual thread pool
class ThreadPool {
public:
	ThreadPool(size_t);
	template<class F>
	void enqueue(F f);
	~ThreadPool();
private:
	// need to keep track of threads so we can join them
	std::vector< std::unique_ptr<boost::thread> > workers;

	// the io_service we are wrapping
	boost::asio::io_service service;
	boost::asio::io_service::work working;
	friend class Worker;
};

// all the workers do is execute the io_service
void Worker::operator()() { pool.service.run(); }

// the constructor just launches some amount of workers
ThreadPool::ThreadPool(size_t threads) : working(service)
{
	for (size_t i = 0; i<threads; ++i)
		workers.push_back(
		std::unique_ptr<boost::thread>(
		new boost::thread(Worker(*this))
		)
		);
}

// add new work item to the pool
template<class F>
void ThreadPool::enqueue(F f)
{
	service.post(f);
}

// the destructor joins all threads
ThreadPool::~ThreadPool()
{
	service.stop();
	for (size_t i = 0; i<workers.size(); ++i)
		workers[i]->join();
}