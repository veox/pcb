// Simple 2D geometrical types (e.g. vector, line, cirle, ellipse, arc)
// and related tests and methods.

#ifndef	PCB_GEOMETRY_H
#define	PCB_GEOMETRY_H

#include "global.h"

typedef struct {
  Coord x, y;
} Vec;

// FIXME: for greater consistency with other types would maybe express this
// as one point and one offset
typedef struct {
  Vec pa, pb;
} LineSegment;

typedef struct {
  Vec center;
  Coord radius;
} Circle;

// FIXME: should probably express with one corner, one vector to an adjacent
// corner, and one dimension (in other direction).
typedef struct {
  Vec c1, c2, c3, c4;  // c1 is diagonal to c3, and c2 is diagonal to c4
} Rectangle;

// FIXME: this is a goofy way to express an ellipse, should go with center+p_major(vec) + p_minor_mag
// FIXME: should be ArcOfEllipse and reference an Ellipse 
typedef struct {
  Vec center;   // Center of ellipse
  Vec p_major, p_minor;   // Points defining major/minor axes of ellipse
  // Start angle of segment, measuing from +x towards +y, in [0, 2 * M_PI)
  double start_angle;
  // Angular span of segment, measuring in the +x towards +y direction
  double angle_delta;  
} Arc;

double
vec_mag (Vec vec);

double
vec_dot (Vec va, Vec vb);

// Vector from va to vb, aka (vb - va)
Vec
vec_from (Vec va, Vec vb);

// Return va scaled by scale_factor.  Be careful with this: scaling integer
// vectors to small magnitudes can result in a lot of error.  Trying to
// make unit vectors won't work for this reason.
Vec
vec_scale (Vec vec, double scale_factor);

// Return projection of va onto vb.
Vec
vec_proj (Vec va, Vec vb);

Vec
vec_sum (Vec va, Vec vb);

// Return the point on seg closest to pt.
Vec
nearest_point_on_line_segment (Vec pt, LineSegment const *seg);

// Return true iff circles ca and cb intersect.  If pii (Point In
// Intersection) is not NULL, set *pii to a point in the intersection.
bool
circles_intersect (Circle const *ca, Circle const *cb, Vec *pii);

// Return true iff the circle intersects seg.  If pii (Point In Intersection)
// is not NULL, set *pii to a point in the intersection.
bool
circle_intersects_line_segment (
    Circle      const *circle,
    LineSegment const *seg,
    Vec               *pii );

// FIXME: my implementation is lazy and slow, should rotate point
bool
point_is_on_rectangle (Vec point, Rectangle const *rect);

// Return true iff circle intersects the rectangle equivalent to all points
// on line segments of length thickness / 2 orthogonol and incident to seg
// with one end point on seg.  If an intersection is found and pii (Point
// In Intersection) is not NULL, return a point on the intersection in pii.
// As currently implemented, this routing returns a point on the boundry
// of the intersection in pii.
// FIXME: I'm probably not the most efficient either
bool
circle_intersects_rectangle (
    Circle const *circle,
    LineSegment const *seg,
    Coord thickness,
    Vec *pii );

#endif   // PCB_GEOMETRY_H
