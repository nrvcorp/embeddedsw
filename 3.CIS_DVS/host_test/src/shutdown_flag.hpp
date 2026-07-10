/* shutdown_flag.hpp  */
#pragma once
#include <atomic>
#include <memory>

using ShutdownFlag = std::shared_ptr<std::atomic<bool>>;

