/**
 * \file math/bounds.cpp
 **/
#include "math/bounds.hpp"

namespace pbr {

  Bounds2::Bounds2() {
    float min_num = std::numeric_limits<float>::lowest();
    float max_num = std::numeric_limits<float>::max();
    p_min = Point2{ max_num , max_num };
    p_max = Point2{ min_num , min_num };
  }
      
  bool Bounds2::Overlaps(const Bounds2& other) const {
    bool x = (p_max.x >= other.p_min.x) && (p_min.x <= other.p_min.x);
    bool y = (p_max.y >= other.p_min.y) && (p_min.y <= other.p_min.y);
    return x && y;
  }
      
  bool Bounds2::Inside(const Point2& p) const {
    return (p.x >= p_min.x && p.x <= p_max.x &&
            p.y >= p_min.y && p.y <= p_max.y); 
  }
      
  bool Bounds2::InsideExclusive(const Point2& p) const {
    return (p.x >= p_min.x && p.x < p_max.x &&
            p.y >= p_min.y && p.y < p_max.y); 
  }

  bool Bounds2::IsEmpty() const {
    return p_min.x >= p_max.x ||
           p_min.y >= p_max.y;
  }

  bool Bounds2::IsDegenerate() const {
    return p_min.x > p_max.x ||
           p_min.y > p_max.y;
  }
    
  float Bounds2::Distance(const Point2& p) const {
    auto dist_2 = DistanceSquared(p);
    return glm::sqrt(dist_2);
  }
      
  float Bounds2::DistanceSquared(const Point2& p) const {
    auto dx = std::max<float>({ 0 , p_min.x - p.x , p.x - p_max.x });
    auto dy = std::max<float>({ 0 , p_min.y - p.y , p.y - p_max.y });
    return glm::pow(dx , 2) + glm::pow(dy , 2);
  }

  float Bounds2::Area() const {
    auto d = Diagonal();
    return d.x * d.y;
  }
      
  uint32_t Bounds2::MaxDimension() const {
    auto d = Diagonal();
    if (d.x > d.y) {
      return 0;
    } else {
      return 1;
    }
  }
      
  Point2 Bounds2::Lerp(const Point2& t) const {
    return Point2{
      pbr::Lerp(t.x , p_min.x , p_max.x) ,
      pbr::Lerp(t.y , p_min.y , p_max.y) ,
    };
  }
      
  glm::vec2 Bounds2::Offset(const Point2& p) const {
    glm::vec2 o = p - p_min;
    if (p_max.x > p_min.x) {
      o.x /= p_max.x - p_min.x;
    }
    if (p_max.y > p_min.y) {
      o.y /= p_max.y - p_min.y;
    }
    return o;
  }
      
  glm::vec2 Bounds2::Diagonal() const {
    return p_max - p_min;
  }
      
  void Bounds2::BoundCircle(Point2& center , float& radius) const {
    center = (p_min + p_max) / 2;
    radius = Inside(center) ?
      Distance(center) : 0;
  }
      
  Bounds2 Bounds2::Union(const Bounds2& b , const Point2& p) {
    Bounds2 ret;
    ret.p_min = Min(b.p_min , p);
    ret.p_max = Max(b.p_max , p);
    return ret;
  }
      
  Bounds2 Bounds2::Union(const Bounds2& b1 , const Bounds2& b2) {
    Bounds2 ret;
    ret.p_min = Min(b1.p_min , b2.p_min);
    ret.p_max = Max(b1.p_max , b2.p_max);
    return ret;
  }
      
  Bounds2 Bounds2::Intersect(const Bounds2& b1 , const Bounds2& b2) {
    Bounds2 ret;
    ret.p_min = Max(b1.p_min , b2.p_min);
    ret.p_max = Min(b1.p_max , b2.p_max);
    return ret;
  }

  Bounds3::Bounds3() {
    float min_num = std::numeric_limits<float>::lowest();
    float max_num = std::numeric_limits<float>::max();
    p_min = Point3{ max_num , max_num , max_num };
    p_max = Point3{ min_num , min_num , min_num };
  }
      
  bool Bounds3::Overlaps(const Bounds3& other) const {
    bool x = (p_max.x >= other.p_min.x) && (p_min.x <= other.p_min.x);
    bool y = (p_max.y >= other.p_min.y) && (p_min.y <= other.p_min.y);
    bool z = (p_max.z >= other.p_min.z) && (p_min.z <= other.p_min.z);
    return x && y && z;
  }
      
  bool Bounds3::Inside(const Point3& p) const {
    return (p.x >= p_min.x && p.x <= p_max.x &&
            p.y >= p_min.y && p.y <= p_max.y &&
            p.z >= p_min.z && p.z <= p_max.z); 
  }
      
  bool Bounds3::InsideExclusive(const Point3& p) const {
    return (p.x >= p_min.x && p.x < p_max.x &&
            p.y >= p_min.y && p.y < p_max.y &&
            p.z >= p_min.z && p.z < p_max.z); 
  }

  bool Bounds3::IsEmpty() const {
    return p_min.x >= p_max.x ||
           p_min.y >= p_max.y ||
           p_min.z >= p_max.z;
  }

  bool Bounds3::IsDegenerate() const {
    return p_min.x > p_max.x ||
           p_min.y > p_max.y ||
           p_min.z > p_max.z;
  }
    
  float Bounds3::Distance(const Point3& p) const {
    auto dist_2 = DistanceSquared(p);
    return glm::sqrt(dist_2);
  }
      
  float Bounds3::DistanceSquared(const Point3& p) const {
    auto dx = std::max<float>({ 0 , p_min.x - p.x , p.x - p_max.x });
    auto dy = std::max<float>({ 0 , p_min.y - p.y , p.y - p_max.y });
    auto dz = std::max<float>({ 0 , p_min.z - p.z , p.z - p_max.z });
    return glm::pow(dx , 2) + glm::pow(dy , 2) + glm::pow(dz , 2);
  }

  float Bounds3::SurfaceArea() const {
    auto d = Diagonal();
    return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
  }

  float Bounds3::Volume() const {
    auto d = Diagonal();
    return d.x * d.y * d.z;
  }
      
  uint32_t Bounds3::MaxDimension() const {
    auto d = Diagonal();
    if (d.x > d.y && d.x > d.z) {
      return 0;
    } else if (d.y > d.z) {
      return 1;
    } else {
      return 2;
    }
  }
      
  Point3 Bounds3::Lerp(const Point3& t) const {
    return Point3{
      pbr::Lerp(t.x , p_min.x , p_max.x) ,
      pbr::Lerp(t.y , p_min.y , p_max.y) ,
      pbr::Lerp(t.z , p_min.z , p_max.z) ,
    };
  }
      
  glm::vec3 Bounds3::Offset(const Point3& p) const {
    glm::vec3 o = p - p_min;
    if (p_max.x > p_min.x) {
      o.x /= p_max.x - p_min.x;
    }
    if (p_max.y > p_min.y) {
      o.y /= p_max.y - p_min.y;
    }
    if (p_max.z > p_min.z) {
      o.z /= p_max.z - p_min.z;
    }
    return o;
  }
      
  glm::vec3 Bounds3::Diagonal() const {
    return p_max - p_min;
  }
      
  void Bounds3::BoundSphere(Point3& center , float& radius) const {
    center = (p_min + p_max) / 2;
    radius = Inside(center) ?
      Distance(center) : 0;
  }
      
  Bounds3 Bounds3::Union(const Bounds3& b , const Point3& p) {
    Bounds3 ret;
    ret.p_min = Min(b.p_min , p);
    ret.p_max = Max(b.p_max , p);
    return ret;
  }
      
  Bounds3 Bounds3::Union(const Bounds3& b1 , const Bounds3& b2) {
    Bounds3 ret;
    ret.p_min = Min(b1.p_min , b2.p_min);
    ret.p_max = Max(b1.p_max , b2.p_max);
    return ret;
  }
      
  Bounds3 Bounds3::Intersect(const Bounds3& b1 , const Bounds3& b2) {
    Bounds3 ret;
    ret.p_min = Max(b1.p_min , b2.p_min);
    ret.p_max = Min(b1.p_max , b2.p_max);
    return ret;
  }

} // namespace pbr
