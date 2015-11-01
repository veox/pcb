// Simple 2D geometrical types (e.g. vector, line, cirle, ellipse, arc)
// and related tests and methods.

#ifndef	PCB_GEOMETRY_H
#define	PCB_GEOMETRY_H

#include "global.h"

typedef struct {
  Coord x, y;
} Vec;

typedef struct {
  Vec pa, pb;
} LineSegment;

typedef struct {
  // corner[0] is diagonal to corner[2], corner[2] is diagonal to corner[3]
  Vec corner[4];
} Rectangle;

typedef struct {
  Vec center;
  Coord radius;
} Circle;

typedef struct {
  Circle circle;
  double start_angle;   // Measuing from +x towards +y, in [0, 2 pi)
  double angle_delta;   // Measuing from +x towards +y, in [0, 2 pi)
} Arc;

double
vec_mag (Vec vec);

// Return vec scaled by scale_factor.  Be careful with this: scaling
// integer vectors to small magnitudes can result in a lot of error.
// Trying to make unit vectors won't work for this reason.
Vec
vec_scale (Vec vec, double scale_factor);

Vec
vec_sum (Vec va, Vec vb);

// Vector from va to vb, aka (vb - va)
Vec
vec_from (Vec va, Vec vb);

double
vec_dot (Vec va, Vec vb);

// Return projection of va onto vb.  Projecting onto a vector of zero
// magnitude will result in a divide-by-zero error.
Vec
vec_proj (Vec va, Vec vb);

// Normalize an angle into the [0, 2 pi) range.
double
normalize_angle_in_radians (double angle);

// Return true iff theta is between start_angle and start_angle + angle_delta
// in radians (winding positive angles from +x towards +y).  No angular
// epsilon is implied here: clients must include one if they need it.
bool
angle_in_span (double theta, double start_angle, double angle_delta);

// Return true iff ppt is in rect (with rect regarded as filled).
bool
point_intersects_rectangle (Vec pt, Rectangle const *rect);

// Return true iff pt is in circ (with circ regarded as filled).
bool
point_intersects_circle (Vec pt, Circle const *circ);

// Return the point on seg closest to pt.
Vec
nearest_point_on_line_segment (Vec pt, LineSegment const *seg);

// Return the point on Arc closest to pt.
Vec
nearest_point_on_arc (Vec pt, Arc const *arc);

// Return true iff filled circle intersects seg.  If pii (Point In
// Intersection) is not NULL, set *pii to a point in the intersection.
// Note that it's considered an intersection if seg lies entirely inside circ.
bool
circle_intersects_line_segment (
    Circle      const *circ,
    LineSegment const *seg,
    Vec               *pii );

// Return true iff filled circ intersects filled rect.  If an intersection
// is found and pii (Point In Intersection) is not NULL, return a point in
// the intersection in pii.  Note that it's considered an intersection if
// one figure lies entirely inside the other.
bool
circle_intersects_rectangle (
    Circle    const *circ,
    Rectangle const *rect,
    Vec             *pii );

// Return true iff circles ca and cb intersect.  If pii (Point In
// Intersection) is not NULL, set *pii to a point in the middle of the
// intersection (treating the circles as filled).
bool
circle_intersects_circle (
    Circle const *ca,
    Circle const *cb,
    Vec          *pii );

// Return the number of points in the intersection of circ and seg (0, 1,
// or 2), assuming no floating point problems.  This function views circ as
// unfilled: if seg lies entirely inside circ no intersection will be found.
// If the result is non-zero and intersections is non-NULL, the intersection
// point(s) are returned there.  Degenerate (zero length) seg arguments
// are treated as point.  Degenerate (radius <= 0) circ arguments aren't
// allowed.  Detection of the intersection for line segments tangent to circ
// is subject to rounding error and a result of 1 is doubtful in this case.
int
circle_line_segment_intersection (
    Circle      const *circ,
    LineSegment const *seg,
    Vec                intersection[2] );

// Return the end points of arc in *ep.  Note that arcs spanning > 2 pi
// radians are still considered to have distinct end points according to
// this function.
void
arc_end_points (Arc *arc, Vec ep[2]);

// Like circle_line_segment_intersection(), but for an arc of a cirle.
int
arc_line_segment_intersection (
    Arc         const *arc,
    LineSegment const *seg,
    Vec                intersection[2] );


#endif   // PCB_GEOMETRY_H
