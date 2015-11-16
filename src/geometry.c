// Implementation of the interface described in geometry.h.

#define _GNU_SOURCE

#include <assert.h>
#include <math.h>
#include <stddef.h>
// FIXME: should these headers be included in geometry.h instead, since
// the macros which name the function from them that we want to use are
// defined there?
#include <stdlib.h>

// If clients of the geometry.h module want to change the default types using
// #define directives in a header (rather than with preprocessor -D options),
// that header can be included here so this file gets the desired definitions.

// The pcb-specific type settings happen in this header at the moment,
// so we include this here.
#include "pcb_geometry.h"

#include "geometry.h"

// Readability aliases.  We undef them first to avoid possible clashes,
// since we're only interested in our interpretation of them here.
#undef  ABS
#define ABS    GEOM_ABS
#undef  ATAN2
#define ATAN2  GEOM_ATAN2
#undef  COS
#define COS    GEOM_COS
#undef  CT
#define CT     GEOM_COORD_TYPE
#undef  FABS
#define FABS   GEOM_FABS
#undef  FT
#define FT     GEOM_FLOAT_TYPE
#undef  HYPOT
#define HYPOT  GEOM_HYPOT
#undef  POW
#define POW    GEOM_POW
#undef  ROUND
#define ROUND  GEOM_ROUND
#undef  SIN
#define SIN    GEOM_SIN
#undef  SINCOS
#define SINCOS GEOM_SINCOS
#undef  SQRT
#define SQRT   GEOM_SQRT

GEOM_FLOAT_TYPE
vec_mag (Vec vec)
{
  return HYPOT (vec.x, vec.y);
}

Vec
vec_scale (Vec vec, GEOM_FLOAT_TYPE scale_factor)
{
  Vec result;

  result.x = ROUND (vec.x * scale_factor);
  result.y = ROUND (vec.y * scale_factor);

  return result;
}

Vec
vec_extend (Vec vec, GEOM_FLOAT_TYPE distance)
{
  // FIXME: I'm dead simple but not tested yet

  FT im = vec_mag (vec);   // Initial Magnitude

  return vec_scale (vec, (im + distance) / im);
}

Vec
vec_sum (Vec va, Vec vb)
{
  Vec result;

  result.x = va.x + vb.x;
  result.y = va.y + vb.y;

  return result;
}

Vec
vec_from (Vec va, Vec vb)
{
  Vec result = { vb.x - va.x, vb.y - va.y };
  
  return result;
}

#include <inttypes.h>

GEOM_FLOAT_TYPE
vec_dot (Vec va, Vec vb)
{
  // Note that by letting one of these comversion be implicit, we effectively
  // arrange for a compiler warning under -Wconversion if FT isn't wider
  // than CT.  I think this is a good thing and not something we want to mask.
  return ((FT) va.x) * vb.x + ((FT) va.y) * vb.y;
}

Vec
vec_proj (Vec va, Vec vb)
{
  return vec_scale (vb, vec_dot (va, vb) / vec_dot (vb, vb));
}

GEOM_FLOAT_TYPE
normalize_angle_in_radians (GEOM_FLOAT_TYPE angle)
{
  while ( angle < 0.0 ) {
    angle += 2.0 * M_PI;
  }
  while ( angle >= 2.0 * M_PI ) {
    angle -= 2.0 * M_PI;
  }

  return angle;
}

bool
angle_in_span (
    GEOM_FLOAT_TYPE theta,
    GEOM_FLOAT_TYPE start_angle,
    GEOM_FLOAT_TYPE angle_delta )
{
  if ( angle_delta > 0.0 ) {
    return angle_delta >= normalize_angle_in_radians (theta - start_angle);
  }
  else {
    return -angle_delta >= normalize_angle_in_radians (-theta + start_angle);
  }
}

bool
point_intersects_rectangle (Vec pt, Rectangle const *rect)
{
  // FIXME: rotating and translating point to put rect relatively at origin
  // or at least axis-aligned would be faster

  // Sides as line segments
  LineSegment c1_c2 = { rect->corner[0], rect->corner[1] };
  LineSegment c2_c3 = { rect->corner[1], rect->corner[2] };
  LineSegment c3_c4 = { rect->corner[2], rect->corner[3] };
  LineSegment c4_c1 = { rect->corner[3], rect->corner[0] };

  // Distance between pairs of opposite sides
  FT d_c1_c2_to_c3_c4 = vec_mag (vec_from (rect->corner[0], rect->corner[3]));
  FT d_c2_c3_to_c4_c1 = vec_mag (vec_from (rect->corner[0], rect->corner[1]));
   
  // Nearest Point (to pt) On Sides
  Vec npo_c1_c2 = nearest_point_on_line_segment (pt, &c1_c2);
  Vec npo_c2_c3 = nearest_point_on_line_segment (pt, &c2_c3);
  Vec npo_c3_c4 = nearest_point_on_line_segment (pt, &c3_c4);
  Vec npo_c4_c1 = nearest_point_on_line_segment (pt, &c4_c1);

  // Distances from pt to nearest point on each side
  FT d_pt_c1_c2 = vec_mag (vec_from (pt, npo_c1_c2));
  FT d_pt_c2_c3 = vec_mag (vec_from (pt, npo_c2_c3));
  FT d_pt_c3_c4 = vec_mag (vec_from (pt, npo_c3_c4));
  FT d_pt_c4_c1 = vec_mag (vec_from (pt, npo_c4_c1));

  return (
      d_pt_c1_c2 <= d_c1_c2_to_c3_c4 &&
      d_pt_c3_c4 <= d_c1_c2_to_c3_c4 &&
      d_pt_c2_c3 <= d_c2_c3_to_c4_c1 &&
      d_pt_c4_c1 <= d_c2_c3_to_c4_c1 );
}

bool
point_intersects_circle (Vec pt, Circle const *circ)
{
  // Translate pt st circ is relatively situated at origin.
  pt.x -= circ->center.x;
  pt.y -= circ->center.y;

  return HYPOT (pt.x, pt.y) <= circ->radius;
}

Vec
nearest_point_on_line_segment (Vec pt, LineSegment const *seg)
{
  // FIXME: I don't do anything efficient for horizontal/vertical line
  // segments

  // FIXME: Stack overflow has a somewhat more
  // efficient solution using same basic idea:
  // http://stackoverflow.com/questions/849211/
  // shortest-distance-between-a-point-and-a-line-segment

  Vec spa_spb;   // Vector from segment point a to point b
  Vec spa_pt;    // Vector from segment point a to pt
  Vec ptl;       // Projection of pt onto seg vector (onto spa_spb)
  FT sm;         // Segment Magnitude
  FT pm;         // Projection Magnitude
  Vec pp;        // Projected Point
  FT sppm;       // Magnitude spa_spb + spa_pt (Segment Plus Proj. Mag.)
  Vec result;    // Result to be returned

  // Degenerate case: seg is a point
  if ( seg->pa.x == seg->pb.x && seg->pa.y == seg->pb.y ) {
    result.x = seg->pa.x;
    result.y = seg->pa.y;
    return result;
  }

  spa_spb = vec_from (seg->pa, seg->pb);
  spa_pt  = vec_from (seg->pa, pt);
  ptl     = vec_proj (spa_pt, spa_spb);
  sm      = vec_mag (spa_spb);
  pm      = vec_mag (ptl);
  pp      = vec_sum (seg->pa, ptl);
  sppm    = vec_mag (vec_sum (spa_spb, ptl));
  
  if ( sppm < sm || sppm < pm ) {
    return seg->pa;  // Segment + Projection add desctructively, so end a 
  }
  else if ( pm > sm ) {
    return seg->pb;  // Projection is past other end of segment, so end b
  }
  else {
    return pp;       // Projection landed on line, so projected point
  }
}

Vec
nearest_point_on_arc (Vec pt, Arc const *arc)
{
  Vec cent = arc->circle.center;
  CT  rad  = arc->circle.radius;

  // Translate pt st Arc circle is centered at origin wrt it
  pt.x -= cent.x;
  pt.y -= cent.y;

  FT sa = arc->start_angle;
  FT ad = arc->angle_delta;
  FT ea = sa + ad;   // End Angle.   Note: in [0, 4.0 * M_PI)

  // Nearest Point on Circle
  Vec npoc = vec_scale (pt, rad / vec_mag (pt));

  FT theta_npoc = ATAN2 (npoc.y, npoc.x);

  Vec result;

  // If the nearest point on the underlying circle is part of the arc,
  // the point itself is the result we want, otherwise the end point of
  // the arc closest to the nearest point is the one we want.
  if ( angle_in_span (theta_npoc, sa, ad) ) {
    result = npoc;
  }
  else {
    // We should use sincos() here when it becomes standard
    // End Point A.
    Vec epa = { ROUND (rad * COS (sa)), ROUND (rad * SIN (sa)) };
    // End Point B
    Vec epb = { ROUND (rad * COS (ea)), ROUND (rad * SIN (ea)) };
    FT m_npoc_epa = vec_mag (vec_from (npoc, epa));
    FT m_npoc_epb = vec_mag (vec_from (npoc, epb));
    result  = (m_npoc_epa < m_npoc_epb ? epa : epb);
  }

  // Translate back to true position
  result.x += cent.x;
  result.y += cent.y;

  return result;
}

bool
circle_intersects_line_segment (
    Circle      const *circ,
    LineSegment const *seg,
    Vec               *pii )
{
  Vec np = nearest_point_on_line_segment (circ->center, seg);

  Vec cc_np = vec_from (circ->center, np);

  FT mcc_np = vec_mag (cc_np);

  if ( mcc_np <= circ->radius ) {
    if ( pii != NULL ) {
      *pii = np;
    }
    return true;
  }
  else {
    return false;
  }
}

bool
circle_intersects_rectangle (
    Circle    const *circ,
    Rectangle const *rect,
    Vec             *pii )
{
  Vec cc = circ->center;
  FT cr2 = POW (circ->radius, 2.0);

  // Check if the center of the circle is on the rectangle.  Note that this
  // catches the situation where the circle is entirely inside the rectangle.
  if ( point_intersects_rectangle (circ->center, rect) ) {
    if ( pii != NULL ) {
      *pii = cc;
    }
    return true;
  }

  Vec const *ca = rect->corner;   // Convenience alias for the Corner Array

  // Check if any of the corners of the rect lie inside circ.  Note that
  // this catches the case where the rectangle is entirely inside the circle.
  // FIXME: check that this test work right
  for ( int ii = 0 ; ii < 4 ; ii++ ) {
    Vec tc = { ca[ii].x - cc.x, ca[ii].y - cc.y };   // Translated Corner
    if ( POW (tc.x, 2.0) + POW (tc.y, 2.0) <= cr2 ) {
      if ( pii != NULL ) {
        *pii = ca[ii];
      }
     return true; 
    }
  }
 
  // Line segments between corners of rectangle
  LineSegment c1_c2 = { ca[0], ca[1] };
  LineSegment c2_c3 = { ca[1], ca[2] };
  LineSegment c3_c4 = { ca[2], ca[3] };
  LineSegment c4_c1 = { ca[3], ca[0] };

  // Note that pii (if not NULL) is computed by the first short-circuit true
  // result here.
  return (
      circle_intersects_line_segment (circ, &c1_c2, pii) ||
      circle_intersects_line_segment (circ, &c2_c3, pii) ||
      circle_intersects_line_segment (circ, &c3_c4, pii) ||
      circle_intersects_line_segment (circ, &c4_c1, pii) );
}

bool
circle_intersects_circle (
    Circle const *ca,
    Circle const *cb,
    Vec          *pii )
{

  Vec a_b = vec_from (ca->center, cb->center);   // Vector from a to b
  FT ma_b = vec_mag (a_b);                       // Magnitude of a_b
  FT overlap = ca->radius + cb->radius - ma_b;   // Overlap size (length)

  if ( overlap >= 0.0 ) {
    if ( pii != NULL ) {
      if      ( ca->radius <= overlap ) {
        *pii = ca->center;   // ca is contained in cb
      }
      else if ( cb->radius <= overlap ) {
        *pii = cb->center;   // cb is contained in ca
      }
      else {
        *pii 
          = vec_sum (
              ca->center,
              vec_scale (a_b, (ca->radius - overlap / 2.0) / ma_b) );
      }
    }
    return true;
  }
  else {
    return false;
  }

}

int
circle_line_segment_intersection (
    Circle      const *circ,
    LineSegment const *seg,
    Vec                intersection[2] )
{
  // Translated end point coordinates st circ is relatively situated at (0, 0)
  CT x1 = seg->pa.x - circ->center.x;
  CT y1 = seg->pa.y - circ->center.y;
  CT x2 = seg->pb.x - circ->center.x;
  CT y2 = seg->pb.y - circ->center.y;

  // Degenerate case: seg is a point
  if ( x1 == x2 && y1 == y2 ) {
    if ( point_intersects_circle (seg->pa, circ) ) {
      if ( intersection != NULL ) {
        intersection[0] = seg->pa;
      }
      return 1;
    }
  }

  // Degenerate circles aren't allowed.
  assert (circ->radius > 0);

  // Straight from http://mathworld.wolfram.com/Circle-LineIntersection.html
  CT dx = x2 - x1;
  CT dy = y2 - y1;
  FT dr = HYPOT (dx, dy);
  FT determinant = ((FT) x1) * y2 - ((FT) x2) * y1;
  FT dr2 = POW (dr, 2.0);
  FT discriminant = POW (circ->radius, 2.0) * dr2 - POW (determinant, 2.0);
  if ( discriminant < 0 ) {
    return 0;   // We aren't interested in imaginary intersections
  }
  FT sqrt_disc = SQRT (discriminant);
  FT temp = (dy < 0 ? -1.0 : 1.0) * dx * sqrt_disc;
  FT ip1x = (determinant * dy + temp) / dr2;
  FT ip2x = (determinant * dy - temp) / dr2;
  temp = FABS (dy) * sqrt_disc;
  FT ip1y = (-determinant * dx + temp) / dr2;
  FT ip2y = (-determinant * dx - temp) / dr2;
  
  int result = 0;

  Vec cc = circ->center;   // Convenience alias

  // Now we just need to check which of the intersections are in our segment.
  // We only need to check one coordinate, since the intersection points are
  // known to be on the (infinite) underlying geomatrical line.  However,
  // because ip1x and ip2x are tainted by floating point and our lines are
  // likely to be parallel to the axes, we make sure to check the wider span.
  if ( ABS (dx) >= ABS (dy) ) {
    CT lx = ( x1 < x2 ? x1 : x2);   // Lower x
    CT hx = ( x1 < x2 ? x2 : x1);   // Higher x
    if ( lx <= ip1x && ip1x <= hx ) {
      if ( intersection != NULL ) {
        intersection[result]
          = ((Vec) { ROUND (ip1x) + cc.x, ROUND (ip1y) + cc.y });
      }
      result++;
    }
    if ( lx <= ip2x && ip2x <= hx ) {
      if ( intersection != NULL ) {
        intersection[result]
          = ((Vec) { ROUND (ip2x) + cc.x, ROUND (ip2y) + cc.y });
      }
      result++;
    }
  }
  else {
    CT ly = ( y1 < y2 ? y1 : y2);   // Lower y
    CT hy = ( y1 < y2 ? y2 : y1);   // Higher y
    if ( ly <= ip1y && ip1y <= hy ) {
      if ( intersection != NULL ) {
        intersection[result]
          = ((Vec) { ROUND (ip1x) + cc.x, ROUND (ip1y) + cc.y });
      }
      result++;
    }
    if ( ly <= ip2y && ip2y <= hy ) {
      if ( intersection != NULL ) {
        intersection[result]
          = ((Vec) { ROUND (ip2x) + cc.x, ROUND (ip2y) + cc.y });
      }
      result++;
    }
  }
  
  return result;
}

void
arc_end_points (Arc const *arc, Vec ep[2])
{
  // Sines and cosines of Start Angle/End Angle
  FT
    sin_sa = SIN (arc->start_angle),
    cos_sa = COS (arc->start_angle),
    sin_ea = SIN (arc->start_angle + arc->angle_delta),
    cos_ea = COS (arc->start_angle + arc->angle_delta);

  // It would be nicer to do this but sincos() currently needs _GNU_SOURCE:
  //FT sin_sa, cos_sa, sin_ea, cos_ea;
  //SINCOS (arc->start_angle, &sin_sa, &cos_sa);
  //SINCOS (arc->start_angle + arc->angle_delta, &sin_ea, &cos_ea);

  FT rad = arc->circle.radius;

  CT cx = arc->circle.center.x, cy = arc->circle.center.y;

  ep[0] = ((Vec) { cx + ROUND (rad * cos_sa), cy + ROUND (rad * sin_sa) });
  ep[1] = ((Vec) { cx + ROUND (rad * cos_ea), cy + ROUND (rad * sin_ea) });
}

int
arc_line_segment_intersection (
    Arc         const *arc,
    LineSegment const *seg,
    Vec                intersection[2] )
{
  // Degenerate arcs aren't allowed.
  assert (arc->circle.radius > 0);

  Circle const *uc = &(arc->circle);   // Underlying Circle
   
  // Get intersection count and intersection points 
  Vec ci[2];   // Circle Intersection (up to two points)
  int cic;     // Circle Intersection Count
  cic = circle_line_segment_intersection (uc, seg, ci);

  int result = 0;

  while ( cic > 0 ) {
    bool iioa   // Intersection Is On Arc
      = angle_in_span (
          ATAN2 (ci[cic - 1].y - uc->center.y, ci[cic - 1].x - uc->center.x),
          arc->start_angle,
          arc->angle_delta );
    if ( iioa ) {
      if ( intersection != NULL ) {
        intersection[result] = ci[cic - 1];
      }
      result++;
    }
    cic--;
  }

  return result;
}

