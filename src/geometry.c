// Implementation of the interface described in geometry.h.

// If clients of the geometry.h module want to change the default types using
// #define directives in a header (rather than with preprocessor -D options),
// that header can be included here so this file gets the desired definitions.

// The pcb-specific type settings happen in this header at the moment,
// so we include this here.
#include "pcb_geometry.h"

#include "geometry.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>

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

// FIXME: why am I here at this point?  I should move or vanish
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

Angle
normalize_angle_in_radians (Angle angle)
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
    Angle theta,
    Angle start_angle,
    Angle angle_delta )
{
  if ( angle_delta > 0.0 ) {
    return angle_delta >= normalize_angle_in_radians (theta - start_angle);
  }
  else {
    return -angle_delta >= normalize_angle_in_radians (-theta + start_angle);
  }
}

bool
angular_spans_overlap (
    Angle  start_angle_a,
    Angle  angle_delta_a,
    Angle  start_angle_b,
    Angle  angle_delta_b,
    Angle *overlap_start_angle,
    Angle *overlap_angle_delta )
{
  // Start/End Angle A/B
  FT
    saa = start_angle_a,
    eaa = saa + angle_delta_a,
    sab = start_angle_b,
    eab = sab + angle_delta_b;

  // Start/End of A/B In B/A
  bool
    saib = angle_in_span (saa, sab, angle_delta_b),
    eaib = angle_in_span (eaa, sab, angle_delta_b),
    sbia = angle_in_span (sab, saa, angle_delta_a),
    ebia = angle_in_span (eab, saa, angle_delta_a);

  if ( ! (saib || eaib || sbia || ebia) ) {
    return false;
  }

  if ( overlap_start_angle != NULL ) {
    assert (overlap_angle_delta);  // If either is non-NULL both must be.
    if ( saib && eaib ) {
      *overlap_start_angle = start_angle_a;
      *overlap_angle_delta = angle_delta_a;
    }
    else if ( sbia && ebia ) {
      *overlap_start_angle = start_angle_b;
      *overlap_angle_delta = angle_delta_b;
    }
    else if ( saib && ebia ) {
      *overlap_start_angle = start_angle_a;
      *overlap_angle_delta = eab - start_angle_a;
    }
    else if ( sbia && eaib ) {
      *overlap_start_angle = start_angle_b;
      *overlap_angle_delta = eaa - start_angle_b;
    }
  }
    
  return true; 
}


bool
point_intersects_rectangle (Point pt, Rectangle const *rect)
{
  // We're going to translate then rotate pt (about the origin) such that
  // rect is relatively centered at the origin, with (an arbitrarily chosen)
  // side "A" parallel to the X axis.

  // Direction A Side
  Vec das = vec_from (rect->corner[0], rect->corner[1]);

  // (Rectangle) Length in Direction A/B
  FT lda = vec_mag (das); 
  FT ldb = vec_mag (vec_from (rect->corner[0], rect->corner[3])); 

  // (Rectangle) Center X/Y
  FT cx = 0.0, cy = 0.0;
  for ( int ii = 0 ; ii < 4 ; ii++ ) {
    cx += rect->corner[ii].x; 
    cy += rect->corner[ii].y; 
  }
  cx /= 4.0;
  cy /= 4.0;

  // Angle of X axis with respect to side A
  Angle theta = -ATAN2 (das.y, das.x);
 
  FT sin_th, cos_th;
#ifdef _GNU_SOURCE
  SINCOS (theta, &sin_th, &cos_th);
#else
  sin_th = SIN (theta);
  cos_th = COS (theta);
#endif

  // Translated and rotated version of pt (X/Y Prime).
  FT xp = (pt.x - cx) * cos_th - (pt.y - cy) * sin_th;
  FT yp = (pt.x - cx) * sin_th + (pt.y - cy) * cos_th;
 
  return (FABS (xp) <= lda / 2.0) && (FABS (yp) <= ldb / 2.0);
}

bool
point_intersects_circle (Point pt, Circle const *circ)
{
  // Translate pt st circ is relatively situated at origin.
  pt.x -= circ->center.x;
  pt.y -= circ->center.y;

  return HYPOT (pt.x, pt.y) <= circ->radius;
}

Point
nearest_point_on_line_segment (Point pt, LineSegment const *seg)
{
  // Stack overflow has a somewhat more efficient solution using the same
  // basic idea:
  //
  //   http://stackoverflow.com/questions/849211/
  //   shortest-distance-between-a-point-and-a-line-segment
  //
  // It's also possile that it would be faster to do translate, rotate,
  // nearest_point_on_vertical_line_segment, unrotate, and untranslate,
  // but I doubt its a big savings if any.

  Vec spa_spb;    // Vector from segment point a to point b
  Vec spa_pt;     // Vector from segment point a to pt
  Vec ptl;        // Projection of pt onto seg vector (onto spa_spb)
  FT sm;          // Segment Magnitude
  FT pm;          // Projection Magnitude
  Point pp;       // Projected Point
  FT sppm;        // Magnitude spa_spb + spa_pt (Segment Plus Proj. Mag.)
  Point result;   // Result to be returned

  // Degenerate case: seg is a point
  // FIXME: this is broken for floating point coordinates (can generate
  // warning under advertised options and is bad practice, do something else,
  // but what?
  if ( seg->pa.x == seg->pb.x && seg->pa.y == seg->pb.y ) {
    result.x = seg->pa.x;
    result.y = seg->pa.y;
    return result;
  }

  // FIXME: well this point shows some of the ugliness of supporting
  // integer coordinates.  ptl might come out pretty tiny, so the round
  // back to integer coordinates could introduce big error.  So we could not
  // use Vecs.  Or we could use FVec or something internally.  But how gross.
  // In trying to manage integer coordinates for clients we introduce a bunch
  // of complexity for so little gain.  As usual trying to do multiple things
  // at once is probably just bad.

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

Point
nearest_point_on_horizontal_line_segment (Point pt, LineSegment const *seg)
{
  // Lowest and highest X coordinates of points on seg
  CT lx = seg->pa.x, hx = seg->pb.x, temp;
  if ( lx > hx ) {
    temp = lx;
    lx = hx;
    hx = temp;
  }

  if      ( pt.x > hx ) {
    return (Point) { hx, seg->pa.y };
  }
  else if ( pt.x < lx ) {
    return (Point) { lx, seg->pa.y };
  }
  else {
    return (Point) { pt.x, seg->pa.y };
  }
}

Point
nearest_point_on_vertical_line_segment (Point pt, LineSegment const *seg)
{
  // Lowest and highest Y coordinates of points on seg
  CT ly = seg->pa.y, hy = seg->pb.y, temp;
  if ( ly > hy ) {
    temp = ly;
    ly = hy;
    hy = temp;
  }

  if      ( pt.y > hy ) {
    return (Point) { seg->pa.x, hy };
  }
  else if ( pt.y < ly ) {
    return (Point) { seg->pa.x, ly };
  }
  else {
    return (Point) { seg->pa.x, pt.y };
  }
}

Point
nearest_point_on_arc (Point pt, Arc const *arc)
{
  Point cent = arc->circle.center;
  CT    rad  = arc->circle.radius;

  // Translate pt st Arc circle is centered at origin wrt it
  pt.x -= cent.x;
  pt.y -= cent.y;

  Angle sa = arc->start_angle;
  Angle ad = arc->angle_delta;
  Angle ea = sa + ad;   // End Angle.   Note: not necessarily in [0, 2 * pi]

  // Nearest Point on Circle
  Point npoc = vec_scale (pt, rad / vec_mag (pt));

  Angle theta_npoc = ATAN2 (npoc.y, npoc.x);

  Point result;

  // If the nearest point on the underlying circle is part of the arc,
  // the point itself is the result we want, otherwise the end point of
  // the arc closest to the nearest point is the one we want.
  if ( angle_in_span (theta_npoc, sa, ad) ) {
    result = npoc;
  }
  else {
    FT sin_sa, cos_sa, sin_ea, cos_ea;
#ifdef _GNU_SOURCE
    SINCOS (sa, &sin_sa, &cos_sa);
    SINCOS (ea, &sin_ea, &cos_ea);
#else
    sin_sa = SIN (sa);
    cos_sa = COS (sa);
    sin_ea = SIN (ea);
    cos_ea = COS (ea);
#endif
    // End Point A.
    Point epa = { ROUND (rad * cos_sa), ROUND (rad * sin_sa) };
    // End Point B
    Point epb = { ROUND (rad * cos_ea), ROUND (rad * sin_ea) };
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
    Point             *pii )
{
  Point np = nearest_point_on_line_segment (circ->center, seg);

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
    Point           *pii )
{
  Point cc = circ->center;
  FT cr2 = POW (circ->radius, 2.0);

  // Check if the center of the circle is on the rectangle.  Note that this
  // catches the situation where the circle is entirely inside the rectangle.
  if ( point_intersects_rectangle (circ->center, rect) ) {
    if ( pii != NULL ) {
      *pii = cc;
    }
    return true;
  }

  Point const *ca = rect->corner;   // Convenience alias for the Corner Array

  // Check if any of the corners of the rect lie inside circ.  Note that
  // this catches the case where the rectangle is entirely inside the circle.
  for ( int ii = 0 ; ii < 4 ; ii++ ) {
    Point tc = { ca[ii].x - cc.x, ca[ii].y - cc.y };   // Translated Corner
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
    Point        *pii )
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
    Point              intersection[2] )
{
  // Degenerate circ arguments aren't allowed
  assert (circ->radius >= 0);

  // Translated end point coordinates st circ is relatively situated at (0, 0)
  CT x1 = seg->pa.x - circ->center.x;
  CT y1 = seg->pa.y - circ->center.y;
  CT x2 = seg->pb.x - circ->center.x;
  CT y2 = seg->pb.y - circ->center.y;

  // Straight from http://mathworld.wolfram.com/Circle-LineIntersection.html
  CT dx = x2 - x1;
  CT dy = y2 - y1;
  FT dr = HYPOT (dx, dy);
  assert (dr > 0);   // Because degenerate (0-length) segs aren't allowed
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

  Point cc = circ->center;   // Convenience alias

  // Now we just need to check which of the intersections are in our segment.
  // We only need to check one coordinate, since the intersection points are
  // known to be on the (infinite) underlying geometrical line.  However,
  // because ip1x and ip2x are tainted by floating point and our lines are
  // likely to be parallel to the axes, we make sure to check the wider span.
  if ( ABS (dx) >= ABS (dy) ) {
    CT lx = ( x1 < x2 ? x1 : x2);   // Lower x
    CT hx = ( x1 < x2 ? x2 : x1);   // Higher x
    if ( lx <= ip1x && ip1x <= hx ) {
      if ( intersection != NULL ) {
        intersection[result]
          = ((Point) { ROUND (ip1x) + cc.x, ROUND (ip1y) + cc.y });
      }
      result++;
    }
    if ( lx <= ip2x && ip2x <= hx ) {
      if ( intersection != NULL ) {
        intersection[result]
          = ((Point) { ROUND (ip2x) + cc.x, ROUND (ip2y) + cc.y });
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
          = ((Point) { ROUND (ip1x) + cc.x, ROUND (ip1y) + cc.y });
      }
      result++;
    }
    if ( ly <= ip2y && ip2y <= hy ) {
      if ( intersection != NULL ) {
        intersection[result]
          = ((Point) { ROUND (ip2x) + cc.x, ROUND (ip2y) + cc.y });
      }
      result++;
    }
  }
  
  return result;
}

int
circle_circle_intersection (
    Circle const *ca,
    Circle const *cb,
    Point         intersection[2] )
{
  // Try to detect circles with identical centers.  Here we could presumably
  // get floating point underflow exceptions or similar hideous stuff,
  // we don't even pretend to handle it.
  if ( ca->center.x - cb->center.x == 0 && ca->center.y - cb->center.y == 0 ) {
    if ( ca->radius - cb->radius == 0 ) {
      return INT_MAX;
    }
    else {
      return 0;
    }
  }

  // Circle A/B Radius
  CT car = ca->radius, cbr = cb->radius;  

  // Distance Between Centers
  FT dbc = vec_mag (vec_from (ca->center, cb->center));

  // Straight from http://mathworld.wolfram.com/Circle-CircleIntersection.html.
  // We're going to imagine for now that ca is centered at the origin and cb
  // has it's center on the x axis.
  FT ix = (POW (dbc, 2.0) - POW (cbr, 2.0) + POW (car, 2.0)) / (2.0 * dbc);
  FT clsra   // Chord Length Square Root Argument
    = ( (-dbc + cbr - car) *
        (-dbc - cbr + car) *
        (-dbc + cbr + car) *
        (+dbc + cbr + car) );
  if ( clsra < 0.0 ) {
    return 0;
  }

  // If the intersection points weren't requestion we can return now
  if ( intersection == NULL ) {
    return 2;
  }

  FT cl = SQRT (clsra) / dbc;   // Chord Length

  // Intersection Point A/B X/Y
  FT ipax = ix, ipay = cl / 2.0, ipbx = ix, ipby = -cl / 2.0;

  // Now we have to rotate and translate back to the actual position

  // (xp, yp) is the center of cb relative to an origin at ca->center
  FT xp = cb->center.x - ca->center.x, yp = cb->center.y - ca->center.y;

  Angle theta = ATAN2 (yp, xp);
  FT sin_th, cos_th;
#ifdef _GNU_SOURCE
  SINCOS (theta, &sin_th, &cos_th);
#else
  sin_th = SIN (theta);
  cos_th = COS (theta);
#endif
 
  // (Un)Rotate intersection points back to actual position relative to ca
  FT ipax_ur = ipax * cos_th - ipay * sin_th;
  FT ipay_ur = ipax * sin_th + ipay * cos_th;
  FT ipbx_ur = ipbx * cos_th - ipby * sin_th;
  FT ipby_ur = ipbx * sin_th + ipby * cos_th;

  Point cac = ca->center;   // Convenience alias

  intersection[0]
    = (Point) { ROUND (ipax_ur + cac.x), ROUND (ipay_ur + cac.y) };
  intersection[1]
    = (Point) { ROUND (ipbx_ur + cac.x), ROUND (ipby_ur + cac.y) };

  // As currently implemented we never end up returning one
  return 2;  
}

void
arc_end_points (Arc const *arc, Point ep[2])
{
  // Sines and cosines of Start Angle/End Angle
  FT sin_sa, cos_sa, sin_ea, cos_ea;
#ifdef _GNU_SOURCE
  SINCOS (arc->start_angle, &sin_sa, &cos_sa);
  SINCOS (arc->start_angle + arc->angle_delta, &sin_ea, &cos_ea);
#else
  sin_sa = SIN (arc->start_angle);
  cos_sa = COS (arc->start_angle);
  sin_ea = SIN (arc->start_angle + arc->angle_delta);
  cos_ea = COS (arc->start_angle + arc->angle_delta);
#endif

  FT rad = arc->circle.radius;

  CT cx = arc->circle.center.x, cy = arc->circle.center.y;

  ep[0] = ((Point) { cx + ROUND (rad * cos_sa), cy + ROUND (rad * sin_sa) });
  ep[1] = ((Point) { cx + ROUND (rad * cos_ea), cy + ROUND (rad * sin_ea) });
}

int
arc_line_segment_intersection (
    Arc         const *arc,
    LineSegment const *seg,
    Point              intersection[2] )
{
  // Degenerate arcs aren't allowed.
  assert (arc->circle.radius > 0);

  Circle const *uc = &(arc->circle);   // Underlying Circle
   
  // Get intersection count and intersection points 
  Point ci[2];   // Circle Intersection (up to two points)
  int   cic;     // Circle Intersection Count
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

int
arc_arc_intersection (
    Arc const *aa,
    Arc const *ab,
    Point      intersection[2] )
{
  Angle   // Convenience aliases
    aasa = aa->start_angle,
    aaad = aa->angle_delta,
    absa = ab->start_angle,
    abad = ab->angle_delta;

  // Underlying Circle's Intersection Points and Intersection Count
  Point ucip[2];
  int ucic = circle_circle_intersection (&(aa->circle), &(ab->circle), ucip);

  if ( ucic == 0 ) {
    return 0;
  }

  // If the underlying circles are identical, the question is whether the
  // angular spans of the arcs overlap.
  if ( ucic == INT_MAX ) {
    if ( angular_spans_overlap (aasa, aaad, absa, abad, NULL, NULL) ) {
      // For arcs of identical underlying circles that intersect at exactly
      // one point, this result is wrong.  Clients have been warned though.
      return INT_MAX;
    }
    else {
      return 0;
    }
  }

  // Arc A/B Center
  Point aac = (aa->circle).center, abc = (ab->circle).center;

  // (Underlying Circle) Intersection Point 0/1 On Arc A/B
  bool ip0oaa
    = angle_in_span (ATAN2 (ucip[0].y - aac.y, ucip[0].x - aac.x), aasa, aaad);
  bool ip0oab
    = angle_in_span (ATAN2 (ucip[0].y - abc.y, ucip[0].x - abc.x), absa, abad);
  bool ip1oaa
    = angle_in_span (ATAN2 (ucip[1].y - aac.y, ucip[1].x - aac.x), aasa, aaad);
  bool ip1oab
    = angle_in_span (ATAN2 (ucip[1].y - abc.y, ucip[1].x - abc.x), absa, abad);

  int result = 0;

  if ( ip0oaa && ip0oab ) {
    if ( intersection != NULL ) {
      intersection[0] = ucip[0];
    }
    result++;
  }
  if ( ip1oaa && ip1oab ) {
    if ( intersection != NULL ) {
      intersection[result] = ucip[1];
    }
    result++;
  }

  return result;
}

