#include "render.h"
#include "route_model.h"
#include "route_planner.h"
#include <fstream>
#include <io2d.h>
#include <iostream>
#include <limits.h>
#include <optional>
#include <string>
#include <vector>

using namespace std::experimental;

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path) {
  std::ifstream is{path, std::ios::binary | std::ios::ate};
  if (!is)
    return std::nullopt;

  auto size = is.tellg();
  std::vector<std::byte> contents(size);

  is.seekg(0);
  is.read((char *)contents.data(), size);

  if (contents.empty())
    return std::nullopt;
  return std::move(contents);
}

float start_x;
float start_y;
float end_x;
float end_y;

// Input Validation.
void user_input(float &val, std::string prompt) {
  std::cout << prompt;
  std::cin >> std::setw(1) >> val;

  // Error check.
  while (!std::cin.good() || val > 100 || val < 0) {
    // Clear stream.
    std::cout << "Error: please enter a float [0,100]!\n";
    std::cin.clear();
    std::cin.ignore(INT_MAX, '\n');
    // Get input again.
    std::cout << prompt;
    std::cin >> std::setw(1) >> val;
  }
}

int main(int argc, const char **argv) {
  std::string osm_data_file = "";
  if (argc > 1) {
    for (int i = 1; i < argc; ++i)
      if (std::string_view{argv[i]} == "-f" && ++i < argc)
        osm_data_file = argv[i];
  } else {
    std::cout << "To specify a map file use the following format: "
              << std::endl;
    std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;
    osm_data_file = "../map.osm";
  }

  std::vector<std::byte> osm_data;

  if (osm_data.empty() && !osm_data_file.empty()) {
    std::cout << "Reading OpenStreetMap data from the following file: "
              << osm_data_file << std::endl;
    auto data = ReadFile(osm_data_file);
    if (!data)
      std::cout << "Failed to read." << std::endl;
    else
      osm_data = std::move(*data);
  }

  // Input the start_node and the end_node.
  user_input(start_x, "input start_x: ");
  user_input(start_y, "input start_y: ");
  user_input(end_x, "input end_x: ");
  user_input(end_y, "input end_y: ");

  // Build Model.
  RouteModel model{osm_data};

  // Create RoutePlanner object and perform A* search.
  RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};
  route_planner.AStarSearch();

  std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

  // Render results of search.
  Render render{model};

  auto display = io2d::output_surface{400,
                                      400,
                                      io2d::format::argb32,
                                      io2d::scaling::none,
                                      io2d::refresh_style::fixed,
                                      30};
  display.size_change_callback([](io2d::output_surface &surface) {
    surface.dimensions(surface.display_dimensions());
  });
  display.draw_callback(
      [&](io2d::output_surface &surface) { render.Display(surface); });
  display.begin_show();
}
