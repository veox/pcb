// Interface of the implementation described in geometry.h.

#include "geometry.h"

double
vec_mag (Vec vec)
{
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
  return vec_scale (vb, ((double) vec_dot (va, vb)) / vec_dot (vb, vb));
}

Vec
vec_sum (Vec va, Vec vb)
{
  Vec result;

  result.x = va.x + vb.x;
  result.y = va.y + vb.y;

  return result;
}

// FIXME: I don't do anything efficient for horizontal/vertical line segments
Vec
nearest_point_on_line_segment (Vec pt, LineSegment const *seg)
{
  // FIXME: I don't do anything efficient for horizontal/vertical line segments

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

  // FIXME: consider whether this works when pt is on seg

  spa_spb = vec_from (seg->pa, seg->pb);
  spa_pt  = vec_from (seg->pa, pt);
  ptl     = vec_proj (spa_pt, spa_spb);
  sm      = vec_mag (spa_spb);
  pm      = vec_mag (ptl);
  pp      = vec_sum (seg->pa, ptl);
  sppm    = vec_mag (vec_sum (spa_spb, spa_pt));
  
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
