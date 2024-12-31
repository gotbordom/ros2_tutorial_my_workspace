// std headers
#include <cstdio>
#include <memory> // smart pointers

// thrid party headers
#include <pluginlib/class_loader.hpp>

// project headers
#include <my_polygon_base/regular_polygon.hpp>

auto main(int argc, char ** argv) -> int
{
  // This is only to clear warnings.
  (void) argc;
  (void) argv;

  pluginlib::ClassLoader<my_polygon_base::RegularPolygon> poly_loader(
    "my_polygon_base",
    "my_polygon_base::RegularPolygon"
  );

  try
  {
    std::shared_ptr<my_polygon_base::RegularPolygon> triangle = 
      poly_loader.createSharedInstance("my_polygon_plugins::Triangle");
    triangle->initialize(10.0);

    std::shared_ptr<my_polygon_base::RegularPolygon> square = 
      poly_loader.createSharedInstance("my_polygon_plugins::Square");
    square->initialize(10.0);

    printf("Triangle area: %.2f\n", triangle->area());
    printf("Square area: %.2f\n", square->area());
  }
  catch(pluginlib::PluginlibException& ex)
  {
    printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
  }
  return 0;
}
