// This is an experimental integration of cartographer into RDK.
#ifndef CONFIG_H_
#define CONFIG_H_

#include <atomic>
#include <string>

#include "slam_service.h"

namespace viam {
namespace config {

// Parses and validates the command line arguments. Sets the log level. Throws
// an exception if the arguments are malformed.
void ParseAndValidateConfigParams(int argc, char** argv,
                                  SLAMServiceImpl* slamService);

// Parse a config parameter map for a specific variable name and return the
// value as a string. Returns empty if the variable is not found within the map.
std::string ConfigParamParser(std::string map, std::string varName);

// Overwrites cartographer specific config parameters
void OverwriteCartoConfigParam(SLAMServiceImpl* slamService,
                               const std::string& parameter);

// Resets command line flags to their default values.
void ResetFlagsForTesting();

}  // namespace config
}  // namespace viam

#endif  // CONFIG_H_
