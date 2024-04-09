#ifndef GLOBAL_VALUE
#define GLOBAL_VALUE

#include <memory>
#include <mutex>

#include "configuration_manager.h"

extern std::shared_ptr<mviz::replay::MvizReplayConfig> g_mrcfg;

#endif