//
// Created by slovak on 11/22/19.
//

#include "Config.h"

#include <climits>
#include <stdexcept>

#include <iostream>
#include <fstream>


Config::Config(const std::string &config_path) {

  const std::string dump_base_path = std::string(PROJECT_SOURCE_DIR) + "/build/";

  std::string m_config_path = config_path;

  if (m_config_path.empty()) {
    m_config_path = std::string(PROJECT_SOURCE_DIR) + "/config/conf.yaml";
  }

  const std::string tmp_json_file_path = dump_base_path + "_conf.json";

  std::string command = std::string("python3 -c '") +
      std::string("config_file_yaml_path = \"") + m_config_path + std::string("\"\n") +
      "import yaml\n"
      "import json\n"
      "with open(config_file_yaml_path, \"r\") as yaml_in, open(\"" + tmp_json_file_path +
      "\", \"w\") as json_out:\n"
      "    yaml_object = yaml.safe_load(yaml_in)\n"
      "    json.dump(yaml_object, json_out)\n"
      "'";

  std::cout << std::endl << command << std::endl;

  auto res = system(command.c_str());

  if (res) {
    throw std::runtime_error("Conversion yaml to json has failed with exit code: " + std::to_string(res));
  }

  std::ifstream i(tmp_json_file_path);
  i >> *this;

  (*this)["dump_base_path"] = std::string(dump_base_path);

  // Cmake defines
  (*this)["PROJECT_SOURCE_DIR"] = std::string(PROJECT_SOURCE_DIR);

  std::ofstream o(tmp_json_file_path);
  o << (*this).dump(1, '\t');
}
