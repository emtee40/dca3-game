#ifndef VMU_H_
#define VMU_H_

#include "common.h"
#include "sampman.h"

#include <thread>
#include <mutex>
#include <shared_mutex>
#include <chrono>
#include <numeric>
#include <array>
#include <functional>
#include <atomic>
#include <memory>
#include <string_view>

#ifdef RW_DC
#   include <dc/maple.h>
#endif

#ifndef VMU_DEFALT_PATH
#   define VMU_DEFAULT_PATH "a1"
#endif

#if (!defined(RW_DC) || !defined(DC_TEXCONV))

constexpr float operator "" _MB(unsigned long long value) {
    return 1024.0f * 1024.0f * value;
}

#   define RAIIVmuBeep(...) RAIIVmuBeeper beep(__VA_ARGS__)

class RAIIVmuBeeper {
private:
    maple_device_t* device_;
public:
    RAIIVmuBeeper(maple_device_t *dev, float period, float duty=0.5f);
    RAIIVmuBeeper(std::string_view address, float period, float duty=0.5f);
    ~RAIIVmuBeeper();
};

class VmuProfiler: public std::thread {
private:
    constexpr static auto         updateRate_  = std::chrono::milliseconds(100);
    constexpr static size_t       fpsSamples   = 20;

    static inline 
    std::unique_ptr<VmuProfiler> instance_     = {};
    
    mutable std::shared_mutex     mtx_         = {};
    bool                          updated_     = false;
    std::array<float, fpsSamples> fps_         = { 0.0f };
    size_t                        fpsFrame_    = 0;
    float                         vertBuffUse_ = 0.0f;
    std::atomic<bool>             stopRequest_ = false;

protected:
    // Default constructor, spawns monitor thread
    VmuProfiler();

    // Main entry point and loop for the monitor thread
    virtual void run();

public:

    // Returns total % of heap space currently in-use
    static float heapUtilization();
    
    // Returns the % of the PVR vertex buffer which is utilized
    static float vertexBufferUtilization();

    // Returns pointer to singleton, creating it + spawning monitor thread upon first call
    static VmuProfiler *getInstance();
    static void destroyInstance();

    // Automatically signals thread to exit
    ~VmuProfiler();

    // To be called every frame, so we can update FPS stats too!
    void updateVertexBufferUsage();
};

#else
#   define RAIIVmuBeep(...)
#endif

#endif
