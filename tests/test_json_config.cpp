#include <cassert>
#include <stdexcept>
#include <string>

#include "acs/config/JsonConfig.h"

int main() {
  using acs::config::JsonParser;

  const auto root = JsonParser::parse(R"({"name":"sim\r\n","values":[1,2,3],"enabled":true})");
  assert(root.has("name"));
  assert(root.get("name").as_string() == std::string("sim\r\n"));
  assert(root.get("values").as_array().size() == 3);
  assert(root.get("enabled").as_bool());

  bool rejected_trailing_content = false;
  try {
    (void)JsonParser::parse(R"({"dt_s":0.01} garbage)");
  } catch (const std::runtime_error&) {
    rejected_trailing_content = true;
  }
  assert(rejected_trailing_content);

  return 0;
}
