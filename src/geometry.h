// Simple 2D geometrical types (e.g. vector, line, cirle, ellipse, arc)
// and related tests and methods.
//
// The associated .c file is supposed to compile cleanly with at least the
// following GCC options:
//
//   -g -std=c1x -O2 -fstrict-aliasing -fstrict-overflow -Wall -Wextra
//   -Wcast-qual -Wstrict-prototypes -Wno-declaration-after-statement -Wundef
//   -Wfloat-equal -Wpointer-arith -Wcast-align -Wstrict-prototypes
//   -Wmissing-prototypes -Wwrite-strings -Wcast-qual -Wswitch-default
//   -Wswitch-enum -Wunreachable-code -Wconversion -Winline
//   -Wmissing-format-attribute -Wredundant-decls -Wmultichar -Wnested-externs
//   -Winit-self -Wundef -Wpointer-arith

// FIXME: double-check the warning list again, order them nicely, and remove any warnings that are implied by -Wall or -Wextra

// alias makepcb='make -j8 CFLAGS="-Wall -fstrict-aliasing -fstrict-overflow -Wextra -Wcast-qual -Wstrict-prototypes -std=c99 -g -O4 -Wno-declaration-after-statement -Wundef -Wfloat-equal -Wpointer-arith -Wcast-align -Wstrict-prototypes -Wmissing-prototypes -Wwrite-strings -Wcast-qual -Wswitch-default -Wswitch-enum -Wunreachable-code -Wconversion -Winline -Wmissing-format-attribute -Wredundant-decls -Wmultichar -Wnested-externs -Winit-self -Wundef -Wpointer-arith"' ; rm geometry.o ; makepcb geometry.o
//
// The associated .c file should also compile cleanly under -Wconversion,
// provided GEOM_FLOAT_TYPE is wider than GEOM_COORD_TYPE.

#ifndef	GEOMETRY_H
#define	GEOMETRY_H

#include <stdbool.h>

// FIXME: WORK POINT: compile and test this fancy parameterized version, then
// give it a pcb-specific wrapper and more rectangular_part_of_line() there

// FIXME: review this code for compatability
// with the warning options described here:
// http://stackoverflow.com/questions/3375697/useful-gcc-flags-for-c and
// also the ones in our personal lib

// FIXME: does it make sense to have these, given that other defaults
// are still literal (e.g. 'labs')? its sorta confusing but otherwise the
// _Static_assert might fall out of sync...  at the least these should get
// a mention that they aren't the thing to change, which is weird...

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

typedef struct {
  Vec pa, pb;
} LineSegment;

typedef struct {
  // corner[0] is diagonal to corner[2], corner[1] is diagonal to corner[3]
  Vec corner[4];
} Rectangle;

typedef struct {
  Vec center;
  GEOM_COORD_TYPE radius;
} Circle;

typedef struct {
  Circle circle;
  GEOM_FLOAT_TYPE start_angle;   // Measuing from +x towards +y, in [0, 2 pi)
  GEOM_FLOAT_TYPE angle_delta;   // Measuing from +x towards +y, in [0, 2 pi)
} Arc;

GEOM_FLOAT_TYPE
vec_mag (Vec vec);

// Return vec scaled by scale_factor.  Note that scaling integer vectors
// to small magnitudes can result in a lot of error.  Trying to make unit
// vectors won't work for this reason.
Vec
vec_scale (Vec vec, GEOM_FLOAT_TYPE scale_factor);

// Return vec extended by distance.  Like vec_scale(), but additive.
Vec
vec_extend (Vec vec, GEOM_FLOAT_TYPE distance);

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

// Return true iff pt is in rect (with rect regarded as filled).
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

// Return true iff filled circ intersects seg.  If pii (Point In Intersection)
// is not NULL, set *pii to a point in the intersection.  Note that it's
// considered an intersection if seg lies entirely inside circ.
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

// Return true iff filled circles ca and cb intersect.  If pii (Point
// In Intersection) is not NULL, set *pii to a point in the intersection
// (treating the circles as filled).
bool
circle_intersects_circle (
    Circle const *ca,
    Circle const *cb,
    Vec          *pii );

// Return the number of points in the intersection of circ and seg (0, 1,
// or 2), assuming no floating point problems.  This function views circ as
// unfilled: if seg lies entirely inside circ no intersection will be found.
// If the result is non-zero and intersections is non-NULL, the intersection
// point(s) are returned there.  Degenerate (zero length) seg arguments are
// treated as points.  Degenerate (radius <= 0) circ arguments aren't allowed.
// Note that detection of the intersection for line segments tangent to circ
// is subject to rounding error and a result of 1 is doubtful in this case.
int
circle_line_segment_intersection (
    Circle      const *circ,
    LineSegment const *seg,
    Vec                intersection[2] );

// Return the end points of arc in *ep.  Arcs spanning > 2 pi radians are
// still considered to have distinct end points.
void
arc_end_points (Arc const *arc, Vec ep[2]);

// Like circle_line_segment_intersection(), but for an arc of a cirle.
int
arc_line_segment_intersection (
    Arc         const *arc,
    LineSegment const *seg,
    Vec                intersection[2] );

#endif   // GEOMETRY_H
