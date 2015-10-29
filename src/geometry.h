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

// This is what Arc should be fixed to be FIXME: change them
typedef struct {
  Circle circle;
  double start_angle;   // Measuing from +x towards +y, in [0, 2 * M_PI)
  double angle_delta;   // Measuing from +x towards +y, in [0, 2 * M_PI)
} Arc2;

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

double
normalize_angle_in_radians (double angle);

// Return true iff theta is between start_angle and start_angle + angle_delta
// in radians (winding positive angles from +x towards +y).  No angular
// epsilon is implied here: clients must include one if they need it.
bool
angle_in_span (double theta, double start_angle, double angle_delta);

// Return the point on seg closest to pt.
Vec
nearest_point_on_line_segment (Vec pt, LineSegment const *seg);

// FIXME: The function in this module should be put in a reasonable order

// Return the point on Arc closest to pt.
Vec
nearest_point_on_arc (Vec pt, Arc2 *arc);

// Return the number of points in the intersection of circ and ls (0, 1, or
// 2), assuming no floating point problems.  If the result is non-zero and
// intersections is non-NULL, the intersection point(s) are returned there.
// Degenerate inputs (0-radius circles and 0-length line segments) are not
// allowed.  Detection of the intersection for line segments tangent to circ
// is subject to rounding error and a result of 1 is doubtful in this case.
int
circle_line_segment_intersection (
    Circle const *circ,
    LineSegment const *ls,
    Vec intersection[2] );

// Like circle_line_segment_intersection(), but for an arc of a cirle.
int
arc_line_segment_intersection (
    Arc2 const *arc,
    LineSegment const *ls,
    Vec intersection[2] );

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

// FIXME: my implementation is lazy and slow, should use rotation
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

// circle_intersects_rectangle as it shoudl be // FIXME: replce the other
bool
circle_intersects_rectangle_2 (Circle const *circ, Rectangle *rect, Vec *pii);

// Return the end points of arc in *ep.  Note that arcs spanning > 2 pi
// radians are still considered to have distinct end points according to
// this function.
void
arc_end_points (Arc2 *arc, Vec ep[2]);

#endif   // PCB_GEOMETRY_H
