// Implementation of the interface described in pcb_geometry.h.

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <assert.h>

#include "global.h"
#include "pcb_geometry.h"

Coord
pcb_round (double arg)
{
  // The point of this is to sort out the underlying type of Coord so we
  // know what function to call for rounding.  GCC provides a good way of
  // doing this at compile-time with __builtin_choose_expr(), for other
  // environments we go by the size of the type on first time through at
  // run-time (we assume that Coord is a signed integer type).

#ifdef __GNUC__

  return
    __builtin_choose_expr (
        __builtin_types_compatible_p (Coord, int),
        lround (arg),
        __builtin_choose_expr (
          __builtin_types_compatible_p (Coord, long),
          lround (arg),
          __builtin_choose_expr (
            __builtin_types_compatible_p (Coord, long long),
            llround (arg),
            (void) 0 ) ) );  // If we get here we've failed (at compile-time).

#else

  // FIXME: this way needs tested

  static void *round_func = NULL;
  if ( UNLIKELY (round_func == NULL) ) {
    size_t soc = sizeof (Coord);
    if ( soc == sizeof (int) ) {
      round_func = lround;
    }
    if ( soc == sizeof (long) ) {
      round_func = lround;
    }
    if ( soc == sizeof (long long) ) {
      round_func = llround;
    }
  }

  return ((Coord (*)(double)) round_func) (arg);

#endif

}

Coord
pcb_abs (Coord arg)
{
  // The point of this is to sort out the underlying type of Coord so we
  // know which abs functio to call.  GCC provides a good way of doing this
  // with __builtin_choose_expr(), for other environments we go by the size
  // of the type on first time through at run-time (we assume that Coord is
  // a signed integer type).

#ifdef __GNUC__
  
  return
    __builtin_choose_expr (
        __builtin_types_compatible_p (Coord, int),
        abs (arg),
        __builtin_choose_expr (
          __builtin_types_compatible_p (Coord, long),
          labs (arg),
          __builtin_choose_expr (
            __builtin_types_compatible_p (Coord, long long),
            llabs (arg),
            (void) 0 ) ) );  // If we get here we've failed (at compile-time).

#else

  // FIXME: this way needs tested

  static void *abs_func = NULL;
  if ( UNLIKELY (abs_func == NULL) ) {
    size_t soc = sizeof (Coord);
    if ( soc == sizeof (int) ) {
      abs_func = abs;
    }
    if ( soc == sizeof (long) ) {
      abs_func = labs;
    }
    if ( soc == sizeof (long long) ) {
      abs_func = llabs;
    }
  }

  return ((Coord (*)(Coord)) abs_func) (arg);

#endif

}

void
pcb_to_geometry_angle_range (
    double  pcb_start_angle,
    double  pcb_angle_delta,
    double *geo_start_angle,
    double *geo_angle_delta )
{
  *geo_start_angle = M_PI - ((M_PI / 180.0) * pcb_start_angle);
  *geo_angle_delta = -pcb_angle_delta * (M_PI / 180.0);
}

Point
nearest_point_on_probably_axis_aligned_line_segment (
    Point pt,
    LineSegment const *seg )
{
  if ( seg->pa.y == seg->pb.y ) {
    return nearest_point_on_horizontal_line_segment (pt, seg);
  }
  else if ( seg->pa.x == seg->pb.x ) {
    return nearest_point_on_vertical_line_segment (pt, seg);
  }
  else {
    return nearest_point_on_line_segment (pt, seg);
  }
}

Rectangle
rectangular_part_of_line (LineType *Line, Coord ged)
{
  // This function isn't designed to handle negative growth st the area of
  // the rectangle is 0 or less.
  assert (Line->Thickness / 2 + ged > 0);

  Point   // End points of Line
    pa = { Line->Point1.X, Line->Point1.Y },
    pb = { Line->Point2.X, Line->Point2.Y };
  
  // Vector with direction and mag. of segment, not including end caps
  Vec pa_pb = vec_from (pa, pb);
  
  // Orthogonol Vector (rotated CCW 90 degrees in +x towares +y direction)
  Vec ov = { -pa_pb.y, pa_pb.x };   // Orthogonol Vector (to segment)

  // "Thickness" vector.  
  Vec tv = vec_scale (ov, Line->Thickness / (2.0 * vec_mag (ov)));

  // Cap vector.  Has magnitude of either thickness / 2.0 or 0, and opposite
  // direction of pa_pb
  Vec cv;
  if ( TEST_FLAG (SQUAREFLAG, Line) ) {
    cv = ((Vec) { -tv.y, tv.x }); 
  }
  else {
    cv = ((Vec) { 0, 0 });
  }
  
  // Negatives (other direction) of cv and tv
  Vec ncv = { -cv.x, -cv.y };
  Vec ntv = { -tv.x, -tv.y };

  // Vectors to add ged.  These are used to implement Bloat of find.c
  Vec ob, cdb, nob, ncdb;   // Orthogonal/cap direction bloat, and negatives
  if ( ged != 0 ) {
    ob   = vec_scale (ov, ((double) ged) / vec_mag (ov));   // Orthogonal 
    nob  = (Vec) { -ob.x, -ob.y };   // Negative ob (for other side)
    cdb  = (Vec) { -ob.y,  ob.x};     // Cap-direction bloat
    ncdb = (Vec) {  ob.y, -ob.x};     // Negative cdb (for other side)
  }
  else {
    ob   = (Vec) { 0, 0 };
    nob  = (Vec) { 0, 0 };
    cdb  = (Vec) { 0, 0 };
    ncdb = (Vec) { 0, 0 };

  }
 
  Rectangle result = { 
    { vec_sum (vec_sum (vec_sum (vec_sum (pa, tv),  cv),  ob),  cdb),
      vec_sum (vec_sum (vec_sum (vec_sum (pb, tv),  ncv), ob),  ncdb),
      vec_sum (vec_sum (vec_sum (vec_sum (pb, ntv), ncv), nob), ncdb),
      vec_sum (vec_sum (vec_sum (vec_sum (pa, ntv), cv),  nob), cdb) } };

  return result;
}
