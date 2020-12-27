
#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <unistd.h>

#include <string>

#include <nlohmann/json.hpp>

class Config : public nlohmann::json {
 public:
  explicit Config(const std::string &config_path = "");
};

#endif //__CONFIG_H__
