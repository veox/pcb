// This interface adapts the application-neutral geometry routines and types
// in geometry.h to the pcb-specific types, and extends them slightly with
// additional geometry routines specifice to pcb.  Clients in pcb should
// always include this file, rather than including geometry.h directly.

#ifndef PCB_GEOMETRY_H
#define PCB_GEOMETRY_H

#include "global.h"

// Call the correct round function (lround or llround) for the pcb Coord type.
Coord
pcb_round (double arg);

// Call the correct abs function (abs, labs, or llabs) for the pcb Coord type.
Coord
pcb_abs (Coord arg);

// Set the coordinate type used by geometry.h
#define GEOM_COORD_TYPE Coord
#define GEOM_ABS        pcb_abs
#define GEOM_ROUND      pcb_round

// We're using the default floating point type (double) for geometry.h,
// so we don't need to change it or the related floating point functions.

// It might be nice to ensure that the floating point type used always be
// wider than the coordinate type, to guarantee that we can go from the
// latter to the former without loss, but pcb doesn't do this and it would
// be a significant change to the build requirements.  This would do it:
//_Static_assert (
//    (sizeof (GEOM_DEFAULT_FLOAT_TYPE) > sizeof (Coord)),
//    "floating point type for geometry.h not wider than Coord type" );

// Currently geometry.h #define's Angle to be GEOM_DEFAULT_FLOAT_TYPE
// (which is double), and global.h typedefs Angle to double.  We want to
// be sure the effective meaning is the same to avoid confusion.
#ifdef __GNUC__
_Static_assert (
    __builtin_types_compatible_p (Angle, double),
    "Angle not type-compatible with double before inclusion of geometry.h");
#endif

#include "geometry.h"

// See above comment about Angle, GEOM_DEFAULT_FLOAT_TYPE, and global.h.
#ifdef __GNUC__
_Static_assert (
    __builtin_types_compatible_p (GEOM_DEFAULT_FLOAT_TYPE, double),
    "GEOM_DEFAULT_FLOAT_TYPE not #defined to double after inclusion of"
    "geometry.h" );
#endif

// Convert an start_angle/angle_delta pair using pcb conventions to one
// using normal mathematical conventions (as understood by geometry.h).
// The pcb ArcType type uses degrees, puts 0 degrees on -x axis and measures
// positive angles in the -x towards +y direction.  The geometry.h Arc type
// uses radians, puts 0 degrees on +x and measures positive angles in the
// +x towards +y directon.
void
pcb_to_geometry_angle_range (
    double  pcb_start_angle,
    double  pcb_angle_delta,
    double *geo_start_angle,
    double *geo_angle_delta );

// Like nearest_point_on_line_segment() from geometry.h, but we first check
// if seg is axis-aligned and call a more efficient routine for this case
// if it is.  Usually more than 50% of pcb lines are axis-aligned.
Point
nearest_point_on_probably_axis_aligned_line_segment (
    Point pt,
    LineSegment const *seg );

// Return a new Rectangle consisting of the portion of Line that is a
// rectangle, increased in size by ged (Growth in Each Direction) in all
// four directions parallel to the rectangle edges.  If Line has SQURE_FLAG,
// this rectangle incorporates the square end caps, otherwise it ends where
// the "Line" stops being a rectangle.
Rectangle
rectangular_part_of_line (LineType *Line, Coord ged);

#endif // PCB_GEOMETRY_H
