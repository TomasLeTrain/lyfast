#pragma once

#include "blazing/motions/motion.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "units/units.hpp"
#include <cstddef>
#include <iostream>
#include <list>
#include <memory>
#include <mutex>
#include <numbers>
#include <optional>
#include <ostream>
#include <queue>
#include <type_traits>

namespace blazing {
class Executor {
  public:
    virtual void addMotion(std::unique_ptr<MotionBase> motion) = 0;
    virtual ~Executor() = default;
};

// TODO: figure out a way to keep the motions from running at the same (maybe
// passing a motion mutex)

template<typename M>
constexpr void operator|(M&& motion, Executor& executor) {
    // creates a copy of the temporary motion object and creates one owned by
    // the executor
    executor.addMotion(
      std::move(std::make_unique<std::decay_t<M>>(std::forward<M>(motion))));
}

class RunExecutor : public Executor {
  public:
    RunExecutor() {}

    // executes as soon as motion gets added
    void addMotion(std::unique_ptr<MotionBase> motion) override;
};

// virtual class that implements some async-specific methods
class AsyncExecutorBase : public Executor {
  protected:
    bool start_of_motion = true;

    size_t finished_index = 0;
    size_t latest_motion_index = 0;
    pros::RecursiveMutex m_mutex;
    std::uint8_t m_currentCompStatus;

  public:
    // main update logic
    virtual void update() = 0;

    // exit current motion, moves onto next motion immediately
    virtual void exitCurrent() = 0;

    // returns number of queued motions
    virtual size_t numQueuedMotions() = 0;

    // returns true if there are motions to execute
    virtual bool hasMotions();

    // start the async task
    virtual void init();

    // blocks until all the motions in the queue have finished
    virtual void wait();

    // exits all motions that were gonna be executed from queue
    virtual void exitAll();

    // blocks until the function returns true. Also exists if there are no
    // motions queued.
    virtual void waitUntil(std::function<bool()> condition);

    // blocks until the function returns true, after which it exists all queued.
    // Also exist if no motions are queued. motions
    virtual void stopIf(std::function<bool()> condition);

    // gets index of latest added motion
    virtual size_t getCurrentIndex();

    // gets index of last motion that was finished
    virtual size_t getFinishedIndex();

    // waits until the finished index matches the given index
    virtual void waitUntilIndex(size_t index);

	virtual void checkCompStatus();
};

class AsyncExecutor : public AsyncExecutorBase {
  private:
    std::queue<std::unique_ptr<MotionBase>> motions;

  public:
    AsyncExecutor() {}

    // executes as soon as motion gets added
    void addMotion(std::unique_ptr<MotionBase> motion) override;

    // main update logic
    void update() override;

    // exit current motion, moves onto next motion immediately
    void exitCurrent() override;

    // returns number of queued motions
    size_t numQueuedMotions() override;
};

struct ChainOptions {
    std::optional<Time> fuse_start_time = std::nullopt;
};

// similar to the async executor, however instead of immediately going from one
// motion to another, it gradually takes the input from two motions and blends
// them to have one smooth motion
class ChainedExecutor : public AsyncExecutorBase {
  private:
    std::list<std::unique_ptr<MotionBase>> motions;
    std::optional<Time> fuse_start_time = std::nullopt;
    Time default_fusing_duration;

    std::function<Voltage(Voltage, Voltage, double)> chain_interpolation =
      [](Voltage a, Voltage b, double t) {
          return (1 - t) * a + t * b;
      };

  public:
    ChainedExecutor(Time fusing_time);

    ChainedExecutor(Time default_fuse_duration,
                    std::function<Voltage(Voltage, Voltage, double)>
                      custom_chain_interpolation);

    // executes as soon as motion gets added
    void addMotion(std::unique_ptr<MotionBase> motion) override;

    // main update logic
    void update() override;

    // exit current motion, moves onto next motion immediately
    void exitCurrent() override;

    // returns number of queued motions
    size_t numQueuedMotions() override;
};
} // namespace blazing
