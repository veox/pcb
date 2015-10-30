// Interface of the implementation described in geometry.h.

#define _GNU_SOURCE

#include <assert.h>
#include <math.h>

#include "geometry.h"

double
vec_mag (Vec vec)
{
  // FIXME: this hidden round is weird, should make clients do it at need
  return round (hypot (vec.x, vec.y));
}

double
vec_dot (Vec va, Vec vb)
{
  return ((double) va.x) * vb.x + ((double) va.y) * vb.y;
}

// Vector from va to vb, aka (vb - va)
Vec
vec_from (Vec va, Vec vb)
{
  // Return vector from va to vb.

  Vec result = { vb.x - va.x, vb.y - va.y };
  
  return result;
}

// Return va scaled by scale_factor.  Be careful with this: scaling integer
// vectors to small magnitudes can result in a lot of error.  Trying to
// make unit vectors won't work for this reason.
Vec
vec_scale (Vec vec, double scale_factor)
{
  Vec result;

  result.x = round (vec.x * scale_factor);
  result.y = round (vec.y * scale_factor);

  return result;
}

Vec
vec_proj (Vec va, Vec vb)
{
  return vec_scale (vb, vec_dot (va, vb) / vec_dot (vb, vb));
}

Vec
vec_sum (Vec va, Vec vb)
{
  Vec result;

  result.x = va.x + vb.x;
  result.y = va.y + vb.y;

  return result;
}

double
normalize_angle_in_radians (double angle)
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
angle_in_span (double theta, double start_angle, double angle_delta)
{
  if ( angle_delta > 0.0 ) {
    return angle_delta >= normalize_angle_in_radians (theta - start_angle);
  }
  else {
    return -angle_delta >= normalize_angle_in_radians (-theta + start_angle);
  }
}

Vec
nearest_point_on_line_segment (Vec pt, LineSegment const *seg)
{
  // FIXME: I don't do anything efficient for horizontal/vertical line
  // segments

  // FIXME: I could be rewritten without so many vectors if I'm found to be
  // slow enought to matter

  Vec spa_spb;   // Vector from segment point a to point b
  double sm;     // Segment Magnitude
  Vec spa_pt;    // Vector from segment point a to pt
  Vec ptl;       // Projection of pt onto seg vector (onto spa_spb)
  double pm;     // Projection Magnitude
  Vec pp;        // Projected Point
  double sppm;   // Magnitude spa_spb + spa_pt (Segment Plus Proj. Mag.)
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
nearest_point_on_arc (Vec pt, Arc2 *arc)
{
  Vec cent = arc->circle.center;
  Coord rad = arc->circle.radius;

  // Translate pt st Arc circle is centered at origin wrt it
  pt.x -= cent.x;
  pt.y -= cent.y;

  double sa = arc->start_angle;
  double ad = arc->angle_delta;
  double ea = sa + ad;   // End Angle.   Note: in [0, 4.0 * M_PI)

  // Nearest Point on Circle
  Vec npoc = vec_scale (pt, rad / vec_mag (pt));

  double theta_npoc = atan2 (npoc.y, npoc.x);

  Vec result;

  // If the nearest point on the underlying circle is part of the arc,
  // the point itself is the result we want, otherwise the end point of
  // the arc closest to the nearest point is the one we want.
  if ( angle_in_span (theta_npoc, sa, ad) ) {
    result = npoc;
  }
  else {
    // We should use sincos() here when it becomes standard
    // End Point A.  round() probably isn't really worth it here
    Vec epa = { round (rad * cos (sa)), round (rad * sin (sa)) };
    // End Point B   round() probably isn't really worth it here
    Vec epb = { round (rad * cos (ea)), round (rad * sin (ea)) };
    double m_npoc_epa = vec_mag (vec_from (npoc, epa));
    double m_npoc_epb = vec_mag (vec_from (npoc, epb));
    result  = (m_npoc_epa < m_npoc_epb ? epa : epb);
  }

  // Translate back to true position
  result.x += cent.x;
  result.y += cent.y;

  return result;
}

int
circle_line_segment_intersection (
    Circle const *circ,
    LineSegment const *ls,
    Vec intersection[2] )
{
  // Translated end point coordinates st circ is relatively situated at (0, 0).
  Coord x1 = ls->pa.x - circ->center.x;
  Coord y1 = ls->pa.y - circ->center.y;
  Coord x2 = ls->pb.x - circ->center.x;
  Coord y2 = ls->pb.y - circ->center.y;

  // Degenerate cases. We can't get these right in general, and clients have
  // been warned.
  assert (! ((x1 == x2 && y1 == y2) || (circ->radius == 0)));
    
  // Straight from http://mathworld.wolfram.com/Circle-LineIntersection.html
  Coord dx = x2 - x1;
  Coord dy = y2 - y1;
  double dr = hypot (dx, dy);
  double determinant = ((double) x1) * y2 - ((double) x2) * y1;
  double dr2 = pow (dr, 2.0);
  double discriminant = pow (circ->radius, 2.0) * dr2 - pow (determinant, 2.0);
  if ( discriminant < 0 ) {
    return 0;   // We aren't interested in imaginary intersections
  }
  double sqrt_disc = sqrt (discriminant);
  double temp = (dy < 0 ? -1.0 : 1.0) * dx * sqrt_disc;
  Coord ip1x = round ((determinant * dy + temp) / dr2);
  Coord ip2x = round ((determinant * dy - temp) / dr2);
  temp = fabs (dy) * sqrt_disc;
  Coord ip1y = round ((-determinant * dx + temp) / dr2);
  Coord ip2y = round ((-determinant * dx - temp) / dr2);
  
  int result = 0;

  // FIXME: I think the below mess is no longer worth it and we should
  // instead add vectors and check if they get bigger or smaller.

  // Now we just need to check which of the intersections are in our segment.
  // We only need to check one coordinate, since the intersection points are
  // known to be on the (infinite) underlying geomatrical line.  However,
  // because ip1x and ip2x are tainted by floating point and our lines are
  // likely to be parallel to the axes, we make sure to check the wider span.
  if ( abs (dx) >= abs (dy) ) {
    Coord lx = ( x1 < x2 ? x1 : x2);   // Lower x
    Coord hx = ( x1 < x2 ? x2 : x1);   // Higher x
    if ( lx <= ip1x && ip1x <= hx ) {
      if ( intersection != NULL ) {
        intersection[result]
          = ((Vec) { ip1x + circ->center.x, ip1y + circ->center.y });
      }
      result++;
    }
    if ( lx <= ip2x && ip2x <= hx ) {
      if ( intersection != NULL ) {
        intersection[result]
          = ((Vec) { ip2x + circ->center.x, ip2y + circ->center.y });
      }
      result++;
    }
  }
  else {
    Coord ly = ( y1 < y2 ? y1 : y2);   // Lower y
    Coord hy = ( y1 < y2 ? y2 : y1);   // Higher y
    if ( ly <= ip1y && ip1y <= hy ) {
      if ( intersection != NULL ) {
        intersection[result]
          = ((Vec) { ip1x + circ->center.x, ip1y + circ->center.y });
      }
      result++;
    }
    if ( ly <= ip2y && ip2y <= hy ) {
      if ( intersection != NULL ) {
        intersection[result]
          = ((Vec) { ip2x + circ->center.x, ip2y + circ->center.y });
      }
      result++;
    }
  }
  
  return result;
}

int
arc_line_segment_intersection (
    Arc2 const *arc,
    LineSegment const *ls,
    Vec intersection[2] )
{
  Circle const *uc = &(arc->circle);   // Underlying Circle
   
  // Get intersection count and intersection points 
  Vec ci[2];   // Circle Intersection (up to two points)
  int cic;     // Circle Intersection Count
  cic = circle_line_segment_intersection (uc, ls, ci);

  int result = 0;

  while ( cic > 0 ) {
    bool iioa   // Intersection Is On Arc
      = angle_in_span (
          atan2 (ci[cic - 1].y - uc->center.y, ci[cic - 1].x - uc->center.x),
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

bool
circles_intersect (Circle const *ca, Circle const *cb, Vec *pii)
{

  Vec a_b = vec_from (ca->center, cb->center);      // Vector from a to b
  double ma_b = vec_mag (a_b);                      // Magnitude of a_b
  double overlap = ca->radius + cb->radius - ma_b;     // Overlap size (length)

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

bool
circle_intersects_line_segment (
    Circle      const *circle,
    LineSegment const *seg,
    Vec               *pii )
{
  Vec np = nearest_point_on_line_segment (circle->center, seg);

  Vec cc_np = vec_from (circle->center, np);

  double mcc_np = vec_mag (cc_np);

  if ( mcc_np <= circle->radius ) {
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
point_is_on_rectangle (Vec point, Rectangle const *rect)
{
  // Distance between pairs of opposite sides
  double d_c1_c2_to_c3_c4 = vec_mag (vec_from (rect->c1, rect->c4));
  double d_c2_c3_to_c4_c1 = vec_mag (vec_from (rect->c1, rect->c2));

  // Sides as line segments
  LineSegment c1_c2 = { rect->c1, rect->c2 };
  LineSegment c2_c3 = { rect->c2, rect->c3 };
  LineSegment c3_c4 = { rect->c3, rect->c4 };
  LineSegment c4_c1 = { rect->c4, rect->c1 };
   
  // Nearest Point (to point) On Sides
  Vec npo_c1_c2 = nearest_point_on_line_segment (point, &c1_c2);
  Vec npo_c2_c3 = nearest_point_on_line_segment (point, &c2_c3);
  Vec npo_c3_c4 = nearest_point_on_line_segment (point, &c3_c4);
  Vec npo_c4_c1 = nearest_point_on_line_segment (point, &c4_c1);

  // Distances from point to nearest point on each side
  double d_point_c1_c2 = vec_mag (vec_from (point, npo_c1_c2));
  double d_point_c2_c3 = vec_mag (vec_from (point, npo_c2_c3));
  double d_point_c3_c4 = vec_mag (vec_from (point, npo_c3_c4));
  double d_point_c4_c1 = vec_mag (vec_from (point, npo_c4_c1));

  return (
      d_point_c1_c2 <= d_c1_c2_to_c3_c4 &&
      d_point_c3_c4 <= d_c1_c2_to_c3_c4 &&
      d_point_c2_c3 <= d_c2_c3_to_c4_c1 &&
      d_point_c4_c1 <= d_c2_c3_to_c4_c1 );
  
}

bool
circle_intersects_rectangle (
    Circle      const *circle,
    LineSegment const *seg,
    Coord              thickness,
    Vec               *pii )
{
  Vec pa = seg->pa, pb = seg->pb;   // Convenience aliases

  // Vector with direction and mag. of segment
  Vec pa_pb = vec_from (pa, pb);

  Vec ov = { -pa_pb.y, pa_pb.x };   // Orthogonol Vector (to segment)

  // Corners of rectangle
  double sf  = thickness / (2.0 * vec_mag (ov));   // Scale Factor
  Vec c1 = vec_sum (pa, vec_scale (ov, sf));
  Vec c2 = vec_sum (pb, vec_scale (ov, sf));
  Vec c3 = vec_sum (pb, vec_scale (ov, -sf));
  Vec c4 = vec_sum (pa, vec_scale (ov, -sf));

  // Line segments between corners of rectangle
  LineSegment c1_c2 = { c1, c2 };
  LineSegment c2_c3 = { c2, c3 };
  LineSegment c3_c4 = { c3, c4 };
  LineSegment c4_c1 = { c4, c1 };

  // Check if the center of the circle is on the rectangle.  This catches
  // the situation where the circle is entirely inside the rectangle.
  Rectangle rect = { c1, c2, c3, c4 };
  if ( point_is_on_rectangle (circle->center, &rect) ) {
    *pii = circle->center;
    return true;
  }

  // Note that pii (if not NULL) is computed by the first short-circuit true
  // result here.
  return (
      circle_intersects_line_segment (circle, &c1_c2, pii) ||
      circle_intersects_line_segment (circle, &c2_c3, pii) ||
      circle_intersects_line_segment (circle, &c3_c4, pii) ||
      circle_intersects_line_segment (circle, &c4_c1, pii) );
}

bool
circle_intersects_rectangle_2 (Circle const *circ, Rectangle *rect, Vec *pii)
{
  // Line segments between corners of rectangle
  LineSegment c1_c2 = { rect->c1, rect->c2 };
  LineSegment c2_c3 = { rect->c2, rect->c3 };
  LineSegment c3_c4 = { rect->c3, rect->c4 };
  LineSegment c4_c1 = { rect->c4, rect->c1 };

  // Check if the center of the circle is on the rectangle.  This catches
  // the situation where the circle is entirely inside the rectangle.
  if ( point_is_on_rectangle (circ->center, rect) ) {
    *pii = circ->center;
    return true;
  }

  // Note that pii (if not NULL) is computed by the first short-circuit true
  // result here.
  return (
      circle_intersects_line_segment (circ, &c1_c2, pii) ||
      circle_intersects_line_segment (circ, &c2_c3, pii) ||
      circle_intersects_line_segment (circ, &c3_c4, pii) ||
      circle_intersects_line_segment (circ, &c4_c1, pii) );
}

void
arc_end_points (Arc2 *arc, Vec ep[2])
{
  // Sines and cosines of Start Angle/End Angle
  double
    sin_sa = sin (arc->start_angle),
    cos_sa = cos (arc->start_angle),
    sin_ea = sin (arc->start_angle + arc->angle_delta),
    cos_ea = cos (arc->start_angle + arc->angle_delta);

  // It would be nicer to do this but sincos() currently needs _GNU_SOURCE:
  //double sin_sa, cos_sa, sin_ea, cos_ea;
  //sincos (arc->start_angle, &sin_sa, &cos_sa);
  //sincos (arc->start_angle + arc->angle_delta, &sin_ea, &cos_ea);

  double rad = arc->circle.radius;

  Coord cx = arc->circle.center.x, cy = arc->circle.center.y;

  ep[0] = ((Vec) { cx + round (rad * cos_sa), cy + round (rad * sin_sa) });
  ep[1] = ((Vec) { cx + round (rad * cos_ea), cy + round (rad * sin_ea) });
}
