#ifndef __A1_Point_hpp__
#define __A1_Point_hpp__

#include "Common.hpp"
#include "DGP/Vector3.hpp"

/* A point in 3D space, with an associated position and normal. */
class Point
{
  private:
    Vector3 position;
    Vector3 normal;

  public:
    /** Default constructor. */
    Point() {}

    /** Initialize with a position and normal. */
    Point(Vector3 const & p, Vector3 const & n = Vector3::zero()) : position(p), normal(n) {}

    /** Get the position of the point. */
    Vector3 const & getPosition() const { return position; }

    /** Set the position of the point. */
    void setPosition(Vector3 const & p) { position = p; }

    /** Get the normal of the point. */
    Vector3 const & getNormal() const { return normal; }

    /** Set the normal of the point. */
    void setNormal(Vector3 const & n) { normal = n; }

}; // class Point

#endif
