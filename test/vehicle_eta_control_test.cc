#include "gtest/gtest.h"

#include <filesystem>
#include <fstream>

#include "motis/rt/vehicle_eta_control.h"

namespace motis {
namespace {

namespace fs = std::filesystem;

void write(fs::path const& path, std::string_view const content) {
  fs::create_directories(path.parent_path());
  auto out = std::ofstream{path};
  out << content;
}

TEST(vehicle_eta_control, applies_precedence_and_retains_last_good) {
  auto const dir = fs::temp_directory_path() / "motis-vehicle-eta-control-test";
  fs::remove_all(dir);
  auto const path = dir / "control.json";
  auto control = vehicle_eta_runtime_control{path};
  using mode = config::timetable::vehicle_eta::mode;

  EXPECT_EQ(control.resolve("A", nigiri::clasz::kBus, mode::shadow),
            mode::shadow);
  write(path, R"({"version":1,"forceOff":false,"feeds":[
    {"feedId":"A","state":"shadow"},
    {"feedId":"A","modes":["BUS"],"state":"effective"}
  ]})");
  EXPECT_TRUE(control.reload());
  EXPECT_EQ(control.resolve("A", nigiri::clasz::kBus, mode::off),
            mode::effective);
  EXPECT_EQ(control.resolve("A", nigiri::clasz::kTram, mode::off),
            mode::shadow);
  auto config = motis::config{
      .timetable_ = motis::config::timetable{
          .datasets_ = {{"A", {}}},
          .vehicle_eta_ =
              motis::config::timetable::vehicle_eta{.mode_ = mode::off}}};
  EXPECT_TRUE(control.enabled(config));
  EXPECT_EQ(control.resolve(config, "A", nigiri::clasz::kBus), mode::effective);

  write(path, R"({"version":1,"forceOff":false,"feeds":[
    {"feedId":"A","modes":["AIRPLANE","HIGHSPEED_RAIL","FERRY","AERIAL_LIFT"],"state":"effective"}
  ]})");
  EXPECT_TRUE(control.reload());
  EXPECT_EQ(control.resolve("A", nigiri::clasz::kAir, mode::off),
            mode::effective);
  EXPECT_EQ(control.resolve("A", nigiri::clasz::kHighSpeed, mode::off),
            mode::effective);
  EXPECT_EQ(control.resolve("A", nigiri::clasz::kShip, mode::off),
            mode::effective);
  EXPECT_EQ(control.resolve("A", nigiri::clasz::kAerialLift, mode::off),
            mode::effective);

  write(path, "{invalid");
  EXPECT_FALSE(control.reload());
  EXPECT_EQ(control.resolve("A", nigiri::clasz::kBus, mode::off),
            mode::effective);

  write(path, R"({"version":1,"forceOff":true,"feeds":[]})");
  EXPECT_TRUE(control.reload());
  EXPECT_EQ(control.resolve("A", nigiri::clasz::kBus, mode::effective),
            mode::off);
  fs::remove_all(dir);
}

}  // namespace
}  // namespace motis
