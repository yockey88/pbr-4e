/**
 * \file math/bounds.hpp
 **/
#ifndef PBR_BOUNDS_HPP
#define PBR_BOUNDS_HPP

#include "math/vecmath.hpp"

namespace pbr {

  class Bounds2 {
    public:
      Bounds2();

      explicit Bounds2(const Point2& p)
        : p_min(p) , p_max(p) {}

      Bounds2(const Point2& p1 , const Point2& p2)
        : p_min(Min(p1 , p2)) , p_max(Max(p1 , p2)) {}

      bool Overlaps(const Bounds2& other) const;
      bool Inside(const Point2& p) const;
      bool InsideExclusive(const Point2& p) const;
      bool IsEmpty() const;
      bool IsDegenerate() const;

      float Distance(const Point2& p) const;
      float DistanceSquared(const Point2& p) const;

      float Area() const;

      uint32_t MaxDimension() const;

      Point2 Lerp(const Point2& t) const;

      glm::vec2 Offset(const Point2& p) const;

      glm::vec2 Diagonal() const;

      void BoundCircle(Point2& center , float& radius) const;

      static Bounds2 Union(const Bounds2& b , const Point2& p);

      static Bounds2 Union(const Bounds2& b1 , const Bounds2& b2);
      static Bounds2 Intersect(const Bounds2& b1 , const Bounds2& b2);
      
      template <typename T>
      static Bounds2 Expand(const Bounds2& b , T delta) {
        Bounds2 ret;
        ret.p_min = b.p_min - glm::vec2(delta);
        ret.p_max = b.p_max + glm::vec2(delta);
        return ret;
      }

    private:
      Point2 p_min;
      Point2 p_max;
  };

  class Bounds3 {
    public:
      Bounds3();

      explicit Bounds3(const Point3& p)
        : p_min(p) , p_max(p) {}

      Bounds3(const Point3& p1 , const Point3& p2)
        : p_min(Min(p1 , p2)) , p_max(Max(p1 , p2)) {}

      Point3 operator[](int32_t p) const;
      Point3& operator[](int32_t p);

      bool Overlaps(const Bounds3& other) const;
      bool Inside(const Point3& p) const;
      bool InsideExclusive(const Point3& p) const;
      bool IsEmpty() const;
      bool IsDegenerate() const;

      float Distance(const Point3& p) const;
      float DistanceSquared(const Point3& p) const;

      float SurfaceArea() const;
      float Volume() const;

      uint32_t MaxDimension() const;

      Point3 Lerp(const Point3& t) const;

      glm::vec3 Offset(const Point3& p) const;

      glm::vec3 Diagonal() const;

      glm::vec3 Corner(uint32_t c) const;

      void BoundSphere(Point3& center , float& radius) const;

      static Bounds3 Union(const Bounds3& b , const Point3& p);

      static Bounds3 Union(const Bounds3& b1 , const Bounds3& b2);
      static Bounds3 Intersect(const Bounds3& b1 , const Bounds3& b2);
      
      template <typename T>
      static Bounds3 Expand(const Bounds3& b , T delta) {
        Bounds3 ret;
        ret.p_min = b.p_min - glm::vec3(delta);
        ret.p_max = b.p_max + glm::vec3(delta);
        return ret;
      }

    private:
      Point3 p_min;
      Point3 p_max;
  };

} // namespace pbr

#endif // !PBR_BOUNDS_HPP
