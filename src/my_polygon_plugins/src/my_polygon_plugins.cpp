// std headers
#include <cmath>    // std::sqrt

// third party headers

// project headers
#include <my_polygon_base/regular_polygon.hpp>

namespace my_polygon_plugins
{
  class Square : public my_polygon_base::RegularPolygon
  {
  public:
    auto initialize(double side_length) -> void override
    {
      side_length_ = side_length;
    }

    auto area() -> double override
    {
      return side_length_ * side_length_;
    }
  protected:
    double side_length_;
  };

  class Triangle : public my_polygon_base::RegularPolygon
  {
  public: 
    auto initialize(double side_length) -> void override
    {
      side_length_ = side_length;
    }

    auto area() -> double override
    {
      return 0.5 * side_length_ * get_height(); 
    }

    auto get_height() -> double 
    {
        //  What we know:
        //  a^2 + b^2 = c^2
        //  a = 0.5 * side_length_
        //  b = height ( what we are solving for )
        //  c = side_length_
        // So b = sqrt( c^2 - a^2 )
        auto a = 0.5 * side_length_;
        auto c = side_length_;
        return std::sqrt(c * c - a * a);
    }
  protected:
    double side_length_;
  };
} // namespace my_polygon_plugins

/*

The implementation of the Square and Triangle classes is fairly straightforward: save the side length, and use it to calculate the area. The only piece that is pluginlib specific is the last three lines, which invokes some magical macros that register the classes as actual plugins. Let’s go through the arguments to the PLUGINLIB_EXPORT_CLASS macro:
Param 1: The fully-qualified type of the plugin class, in this case, polygon_plugins::Square.
Param 2: The fully-qualified type of the base class, in this case, polygon_base::RegularPolygon.

*/
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(my_polygon_plugins::Square,   my_polygon_base::RegularPolygon)
PLUGINLIB_EXPORT_CLASS(my_polygon_plugins::Triangle, my_polygon_base::RegularPolygon)