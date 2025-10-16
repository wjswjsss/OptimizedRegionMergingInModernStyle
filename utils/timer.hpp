/**
 * @file timer.hpp
 * @brief Simple high-resolution timer for performance measurement
 * 
 * @author Claude
 * @date 2025
 */

#pragma once

#include <chrono>

namespace region_merging {

/**
 * @class Timer
 * @brief High-resolution timer using std::chrono
 * 
 * Usage:
 *   Timer t;
 *   t.start();
 *   // ... do work ...
 *   double seconds = t.elapsed();
 */
class Timer {
public:
    using Clock = std::chrono::high_resolution_clock;
    using TimePoint = std::chrono::time_point<Clock>;
    
    /**
     * @brief Constructor (automatically starts timer)
     */
    Timer() {
        start();
    }
    
    /**
     * @brief Start/restart the timer
     */
    void start() {
        start_time = Clock::now();
    }
    
    /**
     * @brief Get elapsed time in seconds
     * @return Elapsed time since start()
     */
    double elapsed() const {
        auto end_time = Clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
            end_time - start_time
        );
        return duration.count() / 1000000.0;  // Convert to seconds
    }
    
    /**
     * @brief Get elapsed time in milliseconds
     * @return Elapsed time in ms
     */
    double elapsed_ms() const {
        auto end_time = Clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            end_time - start_time
        );
        return static_cast<double>(duration.count());
    }
    
    /**
     * @brief Restart timer and return elapsed time
     * @return Elapsed time since previous start()
     */
    double restart() {
        double e = elapsed();
        start();
        return e;
    }
    
private:
    TimePoint start_time;
};

} // namespace region_merging
