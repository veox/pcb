// Simple 2D geometrical types (e.g. vector, line, cirle, ellipse, arc)
// and related tests and methods.
//
// The associated .c file is supposed to compile cleanly with at least the
// following GCC options:
//
// -std=c99 -g -O2 -fstrict-aliasing -fstrict-overflow -Wall -Wextra
// -Wcast-align -Wcast-qual -Wfloat-equal -Winit-self -Wmissing-prototypes
// -Wstrict-prototypes -Wpointer-arith -Wstrict-prototypes -Wswitch-default
// -Wswitch-enum -Wunreachable-code -Wwrite-strings -Wmissing-format-attribute
// -Wmultichar -Wnested-externs -Wpointer-arith -Wredundant-decls -Wundef

// The associated .c file should also compile cleanly with -Wconversion,
// provided GEOM_FLOAT_TYPE is wider than GEOM_COORD_TYPE.

#ifndef	GEOMETRY_H
#define	GEOMETRY_H

#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>

// The default coordinate and floating point types used by this module.
// Note that these defaults should never be changed themselves, but only
// overridden (by setting GEOM_COORD_TYPE and/or GEOM_FLOAT_TYPE and friends
// before this header is included).
#define GEOM_DEFAULT_COORD_TYPE long
#define GEOM_DEFAULT_FLOAT_TYPE double

_Static_assert (
    (sizeof (GEOM_DEFAULT_FLOAT_TYPE) > sizeof (GEOM_DEFAULT_COORD_TYPE)),
    "default float type not wider than default coord type" );

// IMPROVEME: GCC at least could do all the garbage below automagically
// with __builtin_choose_expr() and __builtin_types_compatible_p().

// The type used for coordinates.  This module has been tested with this set
// to long.  It could be another integer type, or a floating point type, but
// in the latter case GEOM_ROUND should be defined to be empty.  GEOM_ABS must
// be set to the appropriate absolute value function (e.g. abs, labs, llabs).
#ifndef GEOM_COORD_TYPE
#  define GEOM_COORD_TYPE GEOM_DEFAULT_COORD_TYPE
#  define GEOM_ABS        labs
#endif

// This function is called to return the results of floating point
// calculations to the coordinate grid defined by GEOM_COORD_TYPE.
// If GEOM_COORD_TYPE is defined to be a floating point type, this should be
// defined to be empty, otherwise it must be set to the rounding function
// appropriate for the GEOM_FLOAT_TYPE and GEOM_COORD_TYPE being used
// (e.g. round, roundl, or perhaps lroundl or llroundl).
#ifndef GEOM_ROUND
#  define GEOM_ROUND lround
#endif

// GEOM_FLOAT_TYPE is the floating point type used for angles and squared
// or multiplied coordinate values, dot products, magnitudes, angles, and
// similar (usually intermediate) results.  The other macros here must be
// set to functions appropriate for the GEOM_FLOAT_TYPE in use (e.g. hypotl,
// powl, etc. if GEOM_FLOAT_TYPE is long double).  Note that GEOM_ROUND
// must also be set approropriately, but it has additional implications for
// some GEOM_COORD_TYPE settings and might need to be empty (see above).
// With GCC these could all be selected automagically at compile-time using
// __builtin_choose_expr(), but we're trying to be portable.
#ifndef GEOM_FLOAT_TYPE
#  define GEOM_FLOAT_TYPE GEOM_DEFAULT_FLOAT_TYPE
#  define GEOM_ATAN2      atan2
#  define GEOM_COS        cos
#  define GEOM_FABS       fabs
#  define GEOM_HYPOT      hypot
#  define GEOM_POW        pow
#  define GEOM_SIN        sin
#  define GEOM_SINCOS     sincos
#  define GEOM_SQRT       sqrt
#endif

typedef struct {
  GEOM_COORD_TYPE x, y;
} Vec;

// A Point is-a specific type of Vec.  They have identical data so we don't
// tell the compiler so it can't complain.  Point is used for readability.
#define Point Vec

typedef struct {
  Point pa, pb;
} LineSegment;

typedef struct {
  Point corner[4];   // corner[0] is diagonal to corner[2], etc.
} Rectangle;

typedef struct {
  Point center;
  GEOM_COORD_TYPE radius;
} Circle;

typedef struct {
  Circle circle;
  GEOM_FLOAT_TYPE start_angle;   // Measuing from +x towards +y, in [0, 2 pi)
  GEOM_FLOAT_TYPE angle_delta;   // Measuing from +x towards +y, in [0, 2 pi)
} Arc;

GEOM_FLOAT_TYPE
vec_mag (Vec vec);

// Return vec scaled by scale_factor.  Note that if GEOM_COORD_TYPE is an
// integer type, scaling vectors to small magnitudes can result in a lot
// of error.  Trying to make unit vectors fails badly for integer coordinates.
Vec
vec_scale (Vec vec, GEOM_FLOAT_TYPE scale_factor);

Vec
vec_sum (Vec va, Vec vb);

// Return vector from va to vb, aka (vb - va)
Vec
vec_from (Vec va, Vec vb);

GEOM_FLOAT_TYPE
vec_dot (Vec va, Vec vb);

// Return projection of va onto vb.  Projecting onto a vector of zero
// magnitude will result in a divide-by-zero error.
Vec
vec_proj (Vec va, Vec vb);

// Return angle normalized into the [0, 2 pi) range.
GEOM_FLOAT_TYPE
normalize_angle_in_radians (GEOM_FLOAT_TYPE angle);

// Return true iff theta is between start_angle and start_angle + angle_delta
// in radians (winding positive angles from +x towards +y).
bool
angle_in_span (
    GEOM_FLOAT_TYPE theta,
    GEOM_FLOAT_TYPE start_angle,
    GEOM_FLOAT_TYPE angle_delta );

// Return true iff the angular spans A and B defined by Start Angle A/B and
// Angle Delta A/B overlap.  The angles are normalized before being compared.
// If there is overlap and overlap_start_angle and overlap_angle_delta are
// not NULL, the angular span of the overlap is returned in them.
bool
angular_spans_overlap (
    GEOM_FLOAT_TYPE  start_angle_a,
    GEOM_FLOAT_TYPE  angle_delta_a,
    GEOM_FLOAT_TYPE  start_angle_b,
    GEOM_FLOAT_TYPE  angle_delta_b,
    GEOM_FLOAT_TYPE *overlap_start_angle,
    GEOM_FLOAT_TYPE *overlap_angle_delta );

// Return true iff pt is in rect (with rect regarded as filled).
bool
point_intersects_rectangle (Point pt, Rectangle const *rect);

// Return true iff pt is in circ (with circ regarded as filled).
bool
point_intersects_circle (Point pt, Circle const *circ);

// Return the point on seg closest to pt.
Point
nearest_point_on_line_segment (Point pt, LineSegment const *seg);

// Like nearest_point_on_line_segment(), but seg is assumed to be horizontal
// (parallel to the X axis) so we can go faster.
Point
nearest_point_on_horizontal_line_segment (Point pt, LineSegment const *seg);

// Like nearest_point_on_horizontal_line_segment(), but for horizontal
// line segments.
Point
nearest_point_on_vertical_line_segment (Point pt, LineSegment const *seg);

// Return the point on Arc closest to pt.
Point
nearest_point_on_arc (Point pt, Arc const *arc);

// Return true iff filled circ intersects seg.  If pii (Point In Intersection)
// is not NULL, set *pii to a point in the intersection.  Note that it's
// considered an intersection if seg lies entirely inside circ.
bool
circle_intersects_line_segment (
    Circle      const *circ,
    LineSegment const *seg,
    Point             *pii );

// Return true iff filled circ intersects filled rect.  If an intersection
// is found and pii (Point In Intersection) is not NULL, return a point in
// the intersection in pii.  Note that it's considered an intersection if
// one figure lies entirely inside the other.
bool
circle_intersects_rectangle (
    Circle    const *circ,
    Rectangle const *rect,
    Point           *pii );

// Return true iff filled circles ca and cb intersect.  If pii (Point
// In Intersection) is not NULL, set *pii to a point in the intersection
// (treating the circles as filled).
bool
circle_intersects_circle (
    Circle const *ca,
    Circle const *cb,
    Point        *pii );

// Return the number of points in the intersection of circ and seg (0, 1,
// or 2), assuming no floating point problems.  This function views circ as
// unfilled: if seg lies entirely inside circ no intersection will be found.
// If the result is non-zero and intersection is non-NULL, the intersection
// point(s) are returned there.  Degenerate seg or circ arguments are
// not allowed (i.e. no zero-lenght segs or circles with radius <= 0).
// Note that detection of the intersection for line segments tangent to circ
// is subject to rounding error and a result of 1 is doubtful in this case
// (0 or 2 being more likely).
int
circle_line_segment_intersection (
    Circle      const *circ,
    LineSegment const *seg,
    Point              intersection[2] );

// Return the number of point in the intersection of of ca and cb.  The result
// will be 0, 2, or possibly INT_MAX (but see below).  This function views
// circles as unfilled: if one circle lies entirely inside the other no
// intersection will be found.  If the result is non-zero and intersection
// is non-NULL, the intersection points(s) are returned there.  Degenerate
// (radius <= 0) circle arguments aren't allowed.  Note that the detection
// of the intersection of circles that intersect at exactly one point is
// subject to rounding error and this routine doesn't attempt to detect
// this situation (0 or 2 is always returned, never 1).  The detection of
// identical circles is doubtful if GEOM_COORD_TYPE is a floating point
// type: INT_MAX might be returned (to represent the infinitely many
// intersection points), or some other erroneous result might be returned.
// If GEOM_COORD_TYPE is an integral type, 0 or INT_MAX will be returned
// as appropriate, but the contents of intersection will be meaningless.
int
circle_circle_intersection (
    Circle const *ca,
    Circle const *cb,
    Point         intersection[2] );

// Return the end points of arc in *ep.  Arcs spanning > 2 pi radians are
// still considered to have distinct end points.
void
arc_end_points (Arc const *arc, Point ep[2]);

// Like circle_line_segment_intersection(), but for an arc of a cirle.
int
arc_line_segment_intersection (
    Arc         const *arc,
    LineSegment const *seg,
    Point              intersection[2] );

// Mostly like circle_circle_intersection(), but for arcs of circles.
// Unlike circle_circle_intersection(), this function may return 1 (when
// arcs cross at only one point, but not when arcs are mutually tangent at
// a single point, or when arcs of identical underlying circles intersect
// at a single point).  Intersection detection of arcs of identical circles
// has all the problems described for circle_circle_intersection(), and this
// function is not equipped to return the actual intersection set (another
// potentially interesting arc) in this case.  If GEOM_COORD_TYPE is an
// integer type, intersection detection is mostly correct even for arcs
// of identical underlying circles.  However, arcs of identical underlying
// circles that intersect at exactly one point are subject to rounding error,
// and will only ever yield results of INT_MAX (representing the infinitely
// many intersection points) or 0 (never the mathematically possible 1).
// The contents of intersection are always undefined for arcs of identical
// circles.
int
arc_arc_intersection (
    Arc const *aa,
    Arc const *ab,
    Point      intersection[2] );

#endif   // GEOMETRY_H
