// UUIDGenerator.cpp
#include "UUIDGenerator.h"
#include <random>
#include <chrono>

uint64_t GenerateUUID()
{
    static std::mt19937_64 rng(
        std::chrono::steady_clock::now().time_since_epoch().count()
    );
    return rng();
}