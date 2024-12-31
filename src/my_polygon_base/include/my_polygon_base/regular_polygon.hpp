#pragma once // Instead of a header guard

namespace my_polygon_base
{
  class RegularPolygon
  {
  public: 
    virtual auto initialize(double side_length) -> void = 0;
    virtual auto area() -> double = 0;
    virtual ~RegularPolygon(){}
  protected:
    RegularPolygon(){}
  };
} // namespace my_polygon_base