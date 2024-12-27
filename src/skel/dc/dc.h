#pragma once
#include <string>

std::string getBuildId();
const char* getSourceId();
const char* getCIJobId();
const char* getExecutableTag();