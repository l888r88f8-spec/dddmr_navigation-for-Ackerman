#include <fstream>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

namespace
{
std::string readFile(const std::string & path)
{
  std::ifstream file(path);
  std::ostringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}
}  // namespace

TEST(P2PMoveBaseConfigTest, LocalizationConfigDoesNotExposeLocalAvoidanceControl)
{
  const std::string config = readFile(
    std::string(P2P_MOVE_BASE_SOURCE_DIR) + "/config/p2p_move_base_localization.yaml");

  ASSERT_FALSE(config.empty());
  EXPECT_EQ(config.find("\n    local_avoidance:"), std::string::npos);
  EXPECT_NE(config.find("\n    local_obstacle_layer:"), std::string::npos);
  EXPECT_NE(config.find("collision_confirm_cycles"), std::string::npos);
  EXPECT_NE(config.find("blocked_confirm_time"), std::string::npos);
}
