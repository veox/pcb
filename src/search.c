/*
 *                            COPYRIGHT
 *
 *  PCB, interactive printed circuit board design
 *  Copyright (C) 1994,1995,1996 Thomas Nau
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *  Contact addresses for paper mail and Email:
 *  Thomas Nau, Schlehenweg 15, 88471 Baustetten, Germany
 *  Thomas.Nau@rz.uni-ulm.de
 *
 */


/* search routines
 * some of the functions use dummy parameters
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <math.h>
#include <setjmp.h>

// FIXME: added for debugging:
#include <unistd.h>
#include "crosshair.h"

#include "global.h"

#include "box.h"
#include "data.h"
#include "draw.h"
#include "error.h"
#include "find.h"
#include "misc.h"
#include "polygon.h"
#include "rtree.h"
#include "search.h"

#ifdef HAVE_LIBDMALLOC
#include <dmalloc.h>
#endif

//#define ENABLE_DEBUG_OUTPUT_THIS_FILE
#ifdef ENABLE_DEBUG_OUTPUT_THIS_FILE
#define DBG(format, ...) printf (format, ## __VA_ARGS__)
#else
#define DBG(format, ...) do { ; } while ( 0 )
#endif

/* ---------------------------------------------------------------------------
 * some local identifiers
 */
static double PosX, PosY;		/* search position for subroutines */
static Coord SearchRadius;
static BoxType SearchBox;
static LayerType *SearchLayer;

/* ---------------------------------------------------------------------------
 * some local prototypes.  The first parameter includes LOCKED_TYPE if we
 * want to include locked types in the search.
 */
static bool SearchLineByLocation (int, LayerType **, LineType **,
				     LineType **);
static bool SearchArcByLocation (int, LayerType **, ArcType **,
				    ArcType **);
static bool SearchRatLineByLocation (int, RatType **, RatType **,
					RatType **);
static bool SearchTextByLocation (int, LayerType **, TextType **,
				     TextType **);
static bool SearchPolygonByLocation (int, LayerType **, PolygonType **,
					PolygonType **);
static bool SearchPinByLocation (int, ElementType **, PinType **,
				    PinType **);
static bool SearchPadByLocation (int, ElementType **, PadType **,
				    PadType **, bool);
static bool SearchViaByLocation (int, PinType **, PinType **,
				    PinType **);
static bool SearchElementNameByLocation (int, ElementType **,
					    TextType **, TextType **,
					    bool);
static bool SearchLinePointByLocation (int, LayerType **, LineType **,
					  PointType **);
static bool SearchPointByLocation (int, LayerType **, PolygonType **,
				      PointType **);
static bool SearchElementByLocation (int, ElementType **,
					ElementType **, ElementType **,
					bool);

/* ---------------------------------------------------------------------------
 * searches a via
 */
struct ans_info
{
  void **ptr1, **ptr2, **ptr3;
  bool BackToo;
  double area;
  jmp_buf env;
  int locked;			/* This will be zero or LOCKFLAG */
  bool found_anything;
  double nearest_sq_dist;
};

static int
pinorvia_callback (const BoxType * box, void *cl)
{
  struct ans_info *i = (struct ans_info *) cl;
  PinType *pin = (PinType *) box;
  AnyObjectType *ptr1 = pin->Element ? pin->Element : pin;

  if (TEST_FLAG (i->locked, ptr1))
    return 0;

  if (!IsPointOnPin (PosX, PosY, SearchRadius, pin))
    return 0;
  *i->ptr1 = ptr1;
  *i->ptr2 = *i->ptr3 = pin;
  longjmp (i->env, 1);
  return 1;			/* never reached */
}

static bool
SearchViaByLocation (int locked, PinType ** Via, PinType ** Dummy1,
		     PinType ** Dummy2)
{
  struct ans_info info;

  /* search only if via-layer is visible */
  if (!PCB->ViaOn)
    return false;

  info.ptr1 = (void **) Via;
  info.ptr2 = (void **) Dummy1;
  info.ptr3 = (void **) Dummy2;
  info.locked = (locked & LOCKED_TYPE) ? 0 : LOCKFLAG;

  if (setjmp (info.env) == 0)
    {
      r_search (PCB->Data->via_tree, &SearchBox, NULL, pinorvia_callback,
		&info);
      return false;
    }
  return true;
}

/* ---------------------------------------------------------------------------
 * searches a pin
 * starts with the newest element
 */
static bool
SearchPinByLocation (int locked, ElementType ** Element, PinType ** Pin,
		     PinType ** Dummy)
{
  struct ans_info info;

  /* search only if pin-layer is visible */
  if (!PCB->PinOn)
    return false;
  info.ptr1 = (void **) Element;
  info.ptr2 = (void **) Pin;
  info.ptr3 = (void **) Dummy;
  info.locked = (locked & LOCKED_TYPE) ? 0 : LOCKFLAG;

  if (setjmp (info.env) == 0)
    r_search (PCB->Data->pin_tree, &SearchBox, NULL, pinorvia_callback,
	      &info);
  else
    return true;
  return false;
}

static int
pad_callback (const BoxType * b, void *cl)
{
  PadType *pad = (PadType *) b;
  struct ans_info *i = (struct ans_info *) cl;
  AnyObjectType *ptr1 = pad->Element;
  double sq_dist;

  DBG ("%s:%i:%s: checkpoint\n", __FILE__, __LINE__, __func__);
  /* Reject locked pads, backside pads (if !BackToo), and non-hit pads */
  if (TEST_FLAG (i->locked, ptr1) ||
      (!FRONT (pad) && !i->BackToo) ||
      !IsPointInPad (PosX, PosY, SearchRadius, pad, NULL))
    return 0;

  /* Determine how close our test-position was to the center of the pad  */
  sq_dist = (PosX - (pad->Point1.X + (pad->Point2.X - pad->Point1.X) / 2)) *
            (PosX - (pad->Point1.X + (pad->Point2.X - pad->Point1.X) / 2)) +
            (PosY - (pad->Point1.Y + (pad->Point2.Y - pad->Point1.Y) / 2)) *
            (PosY - (pad->Point1.Y + (pad->Point2.Y - pad->Point1.Y) / 2));

  /* If this was the closest hit so far, record it */
  if (!i->found_anything || sq_dist < i->nearest_sq_dist)
    {
      *i->ptr1 = ptr1;
      *i->ptr2 = *i->ptr3 = pad;
      i->found_anything = true;
      i->nearest_sq_dist = sq_dist;
    }
  return 0;
}

/* ---------------------------------------------------------------------------
 * searches a pad
 * starts with the newest element
 */
static bool
SearchPadByLocation (int locked, ElementType ** Element, PadType ** Pad,
		     PadType ** Dummy, bool BackToo)
{
  struct ans_info info;

  /* search only if pin-layer is visible */
  if (!PCB->PinOn)
    return (false);
  info.ptr1 = (void **) Element;
  info.ptr2 = (void **) Pad;
  info.ptr3 = (void **) Dummy;
  info.locked = (locked & LOCKED_TYPE) ? 0 : LOCKFLAG;
  info.BackToo = (BackToo && PCB->InvisibleObjectsOn);
  info.found_anything = false;
  r_search (PCB->Data->pad_tree, &SearchBox, NULL, pad_callback, &info);
  return info.found_anything;
}

/* ---------------------------------------------------------------------------
 * searches ordinary line on the SearchLayer 
 */

struct line_info
{
  LineType **Line;
  PointType **Point;
  double least;
  jmp_buf env;
  int locked;
};

static int
line_callback (const BoxType * box, void *cl)
{
  struct line_info *i = (struct line_info *) cl;
  LineType *l = (LineType *) box;

  if (TEST_FLAG (i->locked, l))
    return 0;

  DBG ("%s:%i:%s: checkpoint\n", __FILE__, __LINE__, __func__); 
  if (!IsPointInPad (PosX, PosY, SearchRadius, (PadType *)l, NULL))
    return 0;
  *i->Line = l;
  *i->Point = (PointType *) l;
  longjmp (i->env, 1);
  return 1;			/* never reached */
}


static bool
SearchLineByLocation (int locked, LayerType ** Layer, LineType ** Line,
		      LineType ** Dummy)
{
  struct line_info info;

  info.Line = Line;
  info.Point = (PointType **) Dummy;
  info.locked = (locked & LOCKED_TYPE) ? 0 : LOCKFLAG;

  *Layer = SearchLayer;
  if (setjmp (info.env) == 0)
    {
      r_search (SearchLayer->line_tree, &SearchBox, NULL, line_callback,
		&info);
      return false;
    }
  return (true);
}

static int
rat_callback (const BoxType * box, void *cl)
{
  LineType *line = (LineType *) box;
  struct ans_info *i = (struct ans_info *) cl;

  if (TEST_FLAG (i->locked, line))
    return 0;

  if (TEST_FLAG (VIAFLAG, line) ?
      (Distance (line->Point1.X, line->Point1.Y, PosX, PosY) <=
	   line->Thickness * 2 + SearchRadius) :
      IsPointOnLine (PosX, PosY, SearchRadius, line))
    {
      *i->ptr1 = *i->ptr2 = *i->ptr3 = line;
      longjmp (i->env, 1);
    }
  return 0;
}

/* ---------------------------------------------------------------------------
 * searches rat lines if they are visible
 */
static bool
SearchRatLineByLocation (int locked, RatType ** Line, RatType ** Dummy1,
			 RatType ** Dummy2)
{
  struct ans_info info;

  info.ptr1 = (void **) Line;
  info.ptr2 = (void **) Dummy1;
  info.ptr3 = (void **) Dummy2;
  info.locked = (locked & LOCKED_TYPE) ? 0 : LOCKFLAG;

  if (setjmp (info.env) == 0)
    {
      r_search (PCB->Data->rat_tree, &SearchBox, NULL, rat_callback, &info);
      return false;
    }
  return (true);
}

/* ---------------------------------------------------------------------------
 * searches arc on the SearchLayer 
 */
struct arc_info
{
  ArcType **Arc, **Dummy;
  PointType **Point;
  double least;
  jmp_buf env;
  int locked;
};

static int
arc_callback (const BoxType * box, void *cl)
{
  struct arc_info *i = (struct arc_info *) cl;
  ArcType *a = (ArcType *) box;

  if (TEST_FLAG (i->locked, a))
    return 0;

  if (!IsPointOnArc (PosX, PosY, SearchRadius, a))
    return 0;
  *i->Arc = a;
  *i->Dummy = a;
  longjmp (i->env, 1);
  return 1;			/* never reached */
}


static bool
SearchArcByLocation (int locked, LayerType ** Layer, ArcType ** Arc,
		     ArcType ** Dummy)
{
  struct arc_info info;

  info.Arc = Arc;
  info.Dummy = Dummy;
  info.locked = (locked & LOCKED_TYPE) ? 0 : LOCKFLAG;

  *Layer = SearchLayer;
  if (setjmp (info.env) == 0)
    {
      r_search (SearchLayer->arc_tree, &SearchBox, NULL, arc_callback, &info);
      return false;
    }
  return (true);
}

static int
text_callback (const BoxType * box, void *cl)
{
  TextType *text = (TextType *) box;
  struct ans_info *i = (struct ans_info *) cl;

  if (TEST_FLAG (i->locked, text))
    return 0;

  if (POINT_IN_BOX (PosX, PosY, &text->BoundingBox))
    {
      *i->ptr2 = *i->ptr3 = text;
      longjmp (i->env, 1);
    }
  return 0;
}

/* ---------------------------------------------------------------------------
 * searches text on the SearchLayer
 */
static bool
SearchTextByLocation (int locked, LayerType ** Layer, TextType ** Text,
		      TextType ** Dummy)
{
  struct ans_info info;

  *Layer = SearchLayer;
  info.ptr2 = (void **) Text;
  info.ptr3 = (void **) Dummy;
  info.locked = (locked & LOCKED_TYPE) ? 0 : LOCKFLAG;

  if (setjmp (info.env) == 0)
    {
      r_search (SearchLayer->text_tree, &SearchBox, NULL, text_callback,
		&info);
      return false;
    }
  return (true);
}

static int
polygon_callback (const BoxType * box, void *cl)
{
  PolygonType *polygon = (PolygonType *) box;
  struct ans_info *i = (struct ans_info *) cl;

  if (TEST_FLAG (i->locked, polygon))
    return 0;

  if (IsPointInPolygon (PosX, PosY, SearchRadius, polygon))
    {
      *i->ptr2 = *i->ptr3 = polygon;
      longjmp (i->env, 1);
    }
  return 0;
}


/* ---------------------------------------------------------------------------
 * searches a polygon on the SearchLayer 
 */
static bool
SearchPolygonByLocation (int locked, LayerType ** Layer,
			 PolygonType ** Polygon, PolygonType ** Dummy)
{
  struct ans_info info;

  *Layer = SearchLayer;
  info.ptr2 = (void **) Polygon;
  info.ptr3 = (void **) Dummy;
  info.locked = (locked & LOCKED_TYPE) ? 0 : LOCKFLAG;

  if (setjmp (info.env) == 0)
    {
      r_search (SearchLayer->polygon_tree, &SearchBox, NULL, polygon_callback,
		&info);
      return false;
    }
  return (true);
}

static int
linepoint_callback (const BoxType * b, void *cl)
{
  LineType *line = (LineType *) b;
  struct line_info *i = (struct line_info *) cl;
  int ret_val = 0;
  double d;

  if (TEST_FLAG (i->locked, line))
    return 0;

  /* some stupid code to check both points */
  d = Distance (PosX, PosY, line->Point1.X, line->Point1.Y);
  if (d < i->least)
    {
      i->least = d;
      *i->Line = line;
      *i->Point = &line->Point1;
      ret_val = 1;
    }

  d = Distance (PosX, PosY, line->Point2.X, line->Point2.Y);
  if (d < i->least)
    {
      i->least = d;
      *i->Line = line;
      *i->Point = &line->Point2;
      ret_val = 1;
    }
  return ret_val;
}

/* ---------------------------------------------------------------------------
 * searches a line-point on all the search layer
 */
static bool
SearchLinePointByLocation (int locked, LayerType ** Layer,
			   LineType ** Line, PointType ** Point)
{
  struct line_info info;
  *Layer = SearchLayer;
  info.Line = Line;
  info.Point = Point;
  *Point = NULL;
  info.least = MAX_LINE_POINT_DISTANCE + SearchRadius;
  info.locked = (locked & LOCKED_TYPE) ? 0 : LOCKFLAG;
  if (r_search
      (SearchLayer->line_tree, &SearchBox, NULL, linepoint_callback, &info))
    return true;
  return false;
}

static int
arcpoint_callback (const BoxType * b, void *cl)
{
  ArcType *arc = (ArcType *) b;
  struct arc_info *i = (struct arc_info *) cl;
  int ret_val = 0;
  double d;

  if (TEST_FLAG (i->locked, arc))
    return 0;

  d = Distance (PosX, PosY, arc->Point1.X, arc->Point1.Y);
  if (d < i->least)
    {
      i->least = d;
      *i->Arc = arc;
      *i->Point = &arc->Point1;
      ret_val = 1;
    }

  d = Distance (PosX, PosY, arc->Point2.X, arc->Point2.Y);
  if (d < i->least)
    {
      i->least = d;
      *i->Arc = arc;
      *i->Point = &arc->Point2;
      ret_val = 1;
    }
  return ret_val;
}

/* ---------------------------------------------------------------------------
 * searches an arc-point on all the search layer
 */
static bool
SearchArcPointByLocation (int locked, LayerType **Layer,
                          ArcType **arc, PointType **Point)
{
  struct arc_info info;
  *Layer = SearchLayer;
  info.Arc = arc;
  info.Point = Point;
  *Point = NULL;
  info.least = MAX_ARC_POINT_DISTANCE + SearchRadius;
  info.locked = (locked & LOCKED_TYPE) ? 0 : LOCKFLAG;
  if (r_search
      (SearchLayer->arc_tree, &SearchBox, NULL, arcpoint_callback, &info))
    return true;
  return false;
}
/* ---------------------------------------------------------------------------
 * searches a polygon-point on all layers that are switched on
 * in layerstack order
 */
static bool
SearchPointByLocation (int locked, LayerType ** Layer,
		       PolygonType ** Polygon, PointType ** Point)
{
  double d, least;
  bool found = false;

  least = SearchRadius + MAX_POLYGON_POINT_DISTANCE;
  *Layer = SearchLayer;
  POLYGON_LOOP (*Layer);
  {
    POLYGONPOINT_LOOP (polygon);
    {
      d = Distance (point->X, point->Y, PosX, PosY);
      if (d < least)
	{
	  least = d;
	  *Polygon = polygon;
	  *Point = point;
	  found = true;
	}
    }
    END_LOOP;
  }
  END_LOOP;
  if (found)
    return (true);
  return (false);
}

static int
name_callback (const BoxType * box, void *cl)
{
  TextType *text = (TextType *) box;
  struct ans_info *i = (struct ans_info *) cl;
  ElementType *element = (ElementType *) text->Element;
  double newarea;

  if (TEST_FLAG (i->locked, text))
    return 0;

  if ((FRONT (element) || i->BackToo) && !TEST_FLAG (HIDENAMEFLAG, element) &&
      POINT_IN_BOX (PosX, PosY, &text->BoundingBox))
    {
      /* use the text with the smallest bounding box */
      newarea = (text->BoundingBox.X2 - text->BoundingBox.X1) *
	(double) (text->BoundingBox.Y2 - text->BoundingBox.Y1);
      if (newarea < i->area)
	{
	  i->area = newarea;
	  *i->ptr1 = element;
	  *i->ptr2 = *i->ptr3 = text;
	}
      return 1;
    }
  return 0;
}

/* ---------------------------------------------------------------------------
 * searches the name of an element
 * the search starts with the last element and goes back to the beginning
 */
static bool
SearchElementNameByLocation (int locked, ElementType ** Element,
			     TextType ** Text, TextType ** Dummy,
			     bool BackToo)
{
  struct ans_info info;

  /* package layer have to be switched on */
  if (PCB->ElementOn)
    {
      info.ptr1 = (void **) Element;
      info.ptr2 = (void **) Text;
      info.ptr3 = (void **) Dummy;
      info.area = SQUARE (MAX_COORD);
      info.BackToo = (BackToo && PCB->InvisibleObjectsOn);
      info.locked = (locked & LOCKED_TYPE) ? 0 : LOCKFLAG;
      if (r_search (PCB->Data->name_tree[NAME_INDEX (PCB)], &SearchBox, NULL,
		    name_callback, &info))
	return true;
    }
  return (false);
}

static int
element_callback (const BoxType * box, void *cl)
{
  ElementType *element = (ElementType *) box;
  struct ans_info *i = (struct ans_info *) cl;
  double newarea;

  if (TEST_FLAG (i->locked, element))
    return 0;

  if ((FRONT (element) || i->BackToo) &&
      POINT_IN_BOX (PosX, PosY, &element->VBox))
    {
      /* use the element with the smallest bounding box */
      newarea = (element->VBox.X2 - element->VBox.X1) *
	(double) (element->VBox.Y2 - element->VBox.Y1);
      if (newarea < i->area)
	{
	  i->area = newarea;
	  *i->ptr1 = *i->ptr2 = *i->ptr3 = element;
	  return 1;
	}
    }
  return 0;
}

/* ---------------------------------------------------------------------------
 * searches an element
 * the search starts with the last element and goes back to the beginning
 * if more than one element matches, the smallest one is taken
 */
static bool
SearchElementByLocation (int locked,
			 ElementType ** Element,
			 ElementType ** Dummy1, ElementType ** Dummy2,
			 bool BackToo)
{
  struct ans_info info;

  /* Both package layers have to be switched on */
  if (PCB->ElementOn && PCB->PinOn)
    {
      info.ptr1 = (void **) Element;
      info.ptr2 = (void **) Dummy1;
      info.ptr3 = (void **) Dummy2;
      info.area = SQUARE (MAX_COORD);
      info.BackToo = (BackToo && PCB->InvisibleObjectsOn);
      info.locked = (locked & LOCKED_TYPE) ? 0 : LOCKFLAG;
      if (r_search
	  (PCB->Data->element_tree, &SearchBox, NULL, element_callback,
	   &info))
	return true;
    }
  return false;
}

/* ---------------------------------------------------------------------------
 * checks if a point is on a pin
 */
bool
IsPointOnPin (Coord X, Coord Y, Coord Radius, PinType *pin)
{
  Coord t = PIN_SIZE (pin) / 2;
  if (TEST_FLAG (SQUAREFLAG, pin))
    {
      BoxType b;

      b.X1 = pin->X - t;
      b.X2 = pin->X + t;
      b.Y1 = pin->Y - t;
      b.Y2 = pin->Y + t;
      if (IsPointInBox (X, Y, &b, Radius))
	return true;
    }
  else if (Distance (pin->X, pin->Y, X, Y) <= Radius + t)
    return true;
  return false;
}

/* ---------------------------------------------------------------------------
 * checks if a rat-line end is on a PV
 */
bool
IsPointOnLineEnd (Coord X, Coord Y, RatType *Line)
{
  if (((X == Line->Point1.X) && (Y == Line->Point1.Y)) ||
      ((X == Line->Point2.X) && (Y == Line->Point2.Y)))
    return (true);
  return (false);
}

/* ---------------------------------------------------------------------------
 * checks if a line intersects with a PV
 *
 * let the point be (X,Y) and the line (X1,Y1)(X2,Y2)
 * the length of the line is
 *
 *   L = ((X2-X1)^2 + (Y2-Y1)^2)^0.5
 * 
 * let Q be the point of perpendicular projection of (X,Y) onto the line
 *
 *   QX = X1 + D1*(X2-X1) / L
 *   QY = Y1 + D1*(Y2-Y1) / L
 * 
 * with (from vector geometry)
 *
 *        (Y1-Y)(Y1-Y2)+(X1-X)(X1-X2)
 *   D1 = ---------------------------
 *                     L
 *
 *   D1 < 0   Q is on backward extension of the line
 *   D1 > L   Q is on forward extension of the line
 *   else     Q is on the line
 *
 * the signed distance from (X,Y) to Q is
 *
 *        (Y2-Y1)(X-X1)-(X2-X1)(Y-Y1)
 *   D2 = ----------------------------
 *                     L
 *
 * Finally, D1 and D2 are orthogonal, so we can sum them easily
 * by pythagorean theorem.
 */
bool
IsPointOnLine (Coord X, Coord Y, Coord Radius, LineType *Line)
{
  double D1, D2, L;

  /* Get length of segment */
  L = Distance (Line->Point1.X, Line->Point1.Y, Line->Point2.X, Line->Point2.Y);
  if (L < 0.1)
    return Distance (X, Y, Line->Point1.X, Line->Point1.Y) < Radius + Line->Thickness / 2;

  /* Get distance from (X1, Y1) to Q (on the line) */
  D1 = ((double) (Y - Line->Point1.Y) * (Line->Point2.Y - Line->Point1.Y)
        + (double) (X - Line->Point1.X) * (Line->Point2.X - Line->Point1.X)) / L;
  /* Translate this into distance to Q from segment */
  if (D1 < 0)       D1 = -D1;
  else if (D1 > L)  D1 -= L;
  else              D1 = 0;
  /* Get distance from (X, Y) to Q */
  D2 = ((double) (X - Line->Point1.X) * (Line->Point2.Y - Line->Point1.Y)
        - (double) (Y - Line->Point1.Y) * (Line->Point2.X - Line->Point1.X)) / L;
  /* Total distance is then the pythagorean sum of these */
  return sqrt (D1*D1 + D2*D2) <= Radius + Line->Thickness / 2;
}

/* ---------------------------------------------------------------------------
 * checks if a line crosses a rectangle
 */
bool
IsLineInRectangle (
    Coord X1, Coord Y1, Coord X2, Coord Y2, LineType *Line, PointType *center )
{
  LineType line;

  DBG ("%s:%i:%s: checkpoint\n", __FILE__, __LINE__, __func__);

  /* first, see if point 1 is inside the rectangle */
  /* in case the whole line is inside the rectangle */
  if (X1 < Line->Point1.X && X2 > Line->Point1.X &&
      Y1 < Line->Point1.Y && Y2 > Line->Point1.Y) {
    // FIXME: a more accurate center calculation should be used, this is
    // quick prototype
    if ( center != NULL ) {
      center->X = Line->Point1.X;
      center->Y = Line->Point1.Y;
    }
    return (true);
  }
  /* construct a set of dummy lines and check each of them */
  line.Thickness = 0;
  line.Flags = NoFlags ();

  /* upper-left to upper-right corner */
  line.Point1.Y = line.Point2.Y = Y1;
  line.Point1.X = X1;
  line.Point2.X = X2;
  if (LineLineIntersect (&line, Line, center))
    return (true);

  /* upper-right to lower-right corner */
  line.Point1.X = X2;
  line.Point1.Y = Y1;
  line.Point2.Y = Y2;
  if (LineLineIntersect (&line, Line, center))
    return (true);

  /* lower-right to lower-left corner */
  line.Point1.Y = Y2;
  line.Point1.X = X1;
  line.Point2.X = X2;
  if (LineLineIntersect (&line, Line, center))
    return (true);

  /* lower-left to upper-left corner */
  line.Point2.X = X1;
  line.Point1.Y = Y1;
  line.Point2.Y = Y2;
  if (LineLineIntersect (&line, Line, center))
    return (true);

  return (false);
}

static int /*checks if a point (of null radius) is in a slanted rectangle*/
IsPointInQuadrangle(PointType p[4], PointType *l)
{
  Coord dx, dy, x, y;
  double prod0, prod1;

  dx = p[1].X - p[0].X;
  dy = p[1].Y - p[0].Y;
  x = l->X - p[0].X;
  y = l->Y - p[0].Y;
  prod0 = (double) x * dx + (double) y * dy;
  x = l->X - p[1].X;
  y = l->Y - p[1].Y;
  prod1 = (double) x * dx + (double) y * dy;
  if (prod0 * prod1 <= 0)
    {
      dx = p[1].X - p[2].X;
      dy = p[1].Y - p[2].Y;
      prod0 = (double) x * dx + (double) y * dy;
      x = l->X - p[2].X;
      y = l->Y - p[2].Y;
      prod1 = (double) x * dx + (double) y * dy;
      if (prod0 * prod1 <= 0)
	return true;
    }
  return false;
}
/* ---------------------------------------------------------------------------
 * checks if a line crosses a quadrangle: almost copied from IsLineInRectangle()
 * Note: actually this quadrangle is a slanted rectangle
 */
bool
IsLineInQuadrangle (PointType p[4], LineType *Line, PointType *center)
{
  LineType line;

  /* first, see if point 1 is inside the rectangle */
  /* in case the whole line is inside the rectangle */
  if (IsPointInQuadrangle(p,&(Line->Point1)))
    return true;
  if (IsPointInQuadrangle(p,&(Line->Point2)))
    return true;
  /* construct a set of dummy lines and check each of them */
  line.Thickness = 0;
  line.Flags = NoFlags ();

  /* upper-left to upper-right corner */
  line.Point1.X = p[0].X; line.Point1.Y = p[0].Y;
  line.Point2.X = p[1].X; line.Point2.Y = p[1].Y;
  if (LineLineIntersect (&line, Line, center))
    return (true);

  /* upper-right to lower-right corner */
  line.Point1.X = p[2].X; line.Point1.Y = p[2].Y;
  if (LineLineIntersect (&line, Line, center))
    return (true);

  /* lower-right to lower-left corner */
  line.Point2.X = p[3].X; line.Point2.Y = p[3].Y;
  if (LineLineIntersect (&line, Line, center))
    return (true);

  /* lower-left to upper-left corner */
  line.Point1.X = p[0].X; line.Point1.Y = p[0].Y;
  if (LineLineIntersect (&line, Line, center))
    return (true);

  return (false);
}
/* ---------------------------------------------------------------------------
 * checks if an arc crosses a square
 */
bool
IsArcInRectangle (Coord X1, Coord Y1, Coord X2, Coord Y2, ArcType *Arc)
{
  LineType line;

  /* construct a set of dummy lines and check each of them */
  line.Thickness = 0;
  line.Flags = NoFlags ();

  /* upper-left to upper-right corner */
  line.Point1.Y = line.Point2.Y = Y1;
  line.Point1.X = X1;
  line.Point2.X = X2;
  if (LineArcIntersect (&line, Arc))
    return (true);

  /* upper-right to lower-right corner */
  line.Point1.X = line.Point2.X = X2;
  line.Point1.Y = Y1;
  line.Point2.Y = Y2;
  if (LineArcIntersect (&line, Arc))
    return (true);

  /* lower-right to lower-left corner */
  line.Point1.Y = line.Point2.Y = Y2;
  line.Point1.X = X1;
  line.Point2.X = X2;
  if (LineArcIntersect (&line, Arc))
    return (true);

  /* lower-left to upper-left corner */
  line.Point1.X = line.Point2.X = X1;
  line.Point1.Y = Y1;
  line.Point2.Y = Y2;
  if (LineArcIntersect (&line, Arc))
    return (true);

  return (false);
}

typedef struct {
  Coord x, y;
} Vec;

typedef struct {
  Vec pa, pb;
} LineSegment;

typedef struct {
  Vec center;
  Coord radius;
} Circle;

static double
vector_mag (Vec vec)
{
  return round (sqrt (((double) vec.x) * vec.x + ((double) vec.y) * vec.y));
}

double
vector_dot (Vec va, Vec vb)
{
  return ((double) va.x) * vb.x + ((double) va.y) * vb.y;
}

static Vec
vector_from (Vec va, Vec vb)
{
  // Return vector from va to vb.

  Vec result = { vb.x - va.x, vb.y - va.y };
  
  return result;
}

static Vec
vector_scale (Vec vec, double scale_factor)
{
  // Return va scaled by scale_factor.  Be careful with this: scaling
  // integer vectors to small magnitudes can result in a lot of error.
  // Trying to make unit vectors won't work.

  Vec result;

  result.x = round (vec.x * scale_factor);
  result.y = round (vec.y * scale_factor);

  return result;
}

static Vec
vector_proj (Vec va, Vec vb)
{
  // Return projection of va onto vb.

  return
    vector_scale (vb, ((double) vector_dot (va, vb)) / vector_dot (vb, vb));
}

static Vec
vector_sum (Vec va, Vec vb)
{
  Vec result;

  result.x = va.x + vb.x;
  result.y = va.y + vb.y;

  return result;
}

// FIXME: should take a pointer for seg probably
static Vec
nearest_point_on_line_segment (Vec pt, LineSegment *seg)
{
  // Return the nearest point on seg closest to pt.

  Vec spa_spb, spa_pt, ptl, result;
  double sm, pm, sppm;

  // Degenerate case: seg is a point
  if ( seg->pa.x == seg->pb.x && seg->pa.y == seg->pb.y ) {
    result.x = seg->pa.x;
    result.y = seg->pa.y;
    return result;
  }

  spa_spb = vector_from (seg->pa, seg->pb);

  spa_pt = vector_from (seg->pa, pt);

  ptl = vector_proj (spa_pt, spa_spb);   // Projection To Line

  sm = vector_mag (spa_spb);   // Segment Magnitude

  pm = vector_mag (ptl);   // Projection Magnitude

  Vec pp = vector_sum (seg->pa, ptl);   // Projected Point
  
  gui->add_debug_marker (pp.x, pp.y);

  sppm = vector_mag (vector_sum (spa_spb, spa_pt));   // Segment Plus Proj. Mag.
  
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

static bool
circles_intersect (Circle *ca, Circle *cb, Vec *pii)
{
  // Return true iff circles ca and cb intersect.  If pii (Point In
  // Intersection) is not NULL, return the center of the intersection region.

  Vec a_b = vector_from (ca->center, cb->center);      // Vector from a to b
  double ma_b = vector_mag (a_b);                      // Magnitude of a_b
  double overlap = ca->radius + cb->radius - ma_b;     // Overlap size (length)

  if ( overlap >= 0.0 ) {
    if ( pii != NULL ) {
      *pii = vector_scale (a_b, (ca->radius - overlap / 2.0) / ma_b);
    }
    return true;
  }
  else {
    return false;
  }
}

static bool
circle_intersects_line_segment (Circle *circle, LineSegment *seg, Vec *ip)
{
  // Return true iff the circle intersects seg.  If piin (Point In
  // Intersection) is not NULL, return a point in the intersection.

  Vec np = nearest_point_on_line_segment (circle->center, seg);

  Vec cc_np = vector_from (circle->center, np);

  double mcc_np = vector_mag (cc_np);

  if ( mcc_np <= circle->radius ) {
    printf ("%s:%i:%s: checkpoint\n", __FILE__, __LINE__, __func__); 
    if ( ip != NULL ) {
      *ip = np;
    }
    return true;
  }
  else {
    return false;
  }
}

typedef struct {
  // FIXME; c4 is redundant in a way, change something?
  Vec c1, c2, c3, c4;  // c1 is diagonal to c3, and c2 is diagonal to c4
} Rectangle;

static bool
point_is_on_rectangle (Vec point, Rectangle *rect)
{
  // Distance between pairs of opposite sides
  double d_c1_c2_to_c3_c4 = vector_mag (vector_from (rect->c1, rect->c4));
  double d_c2_c3_to_c4_c1 = vector_mag (vector_from (rect->c1, rect->c2));

  /*
  printf ("rect->c1.x: %li\n", rect->c1.x);
  printf ("rect->c1.y: %li\n", rect->c1.y);
  printf ("rect->c2.x: %li\n", rect->c2.x);
  printf ("rect->c2.y: %li\n", rect->c2.y);
  printf ("rect->c3.x: %li\n", rect->c3.x);
  printf ("rect->c3.y: %li\n", rect->c3.y);
  printf ("rect->c4.x: %li\n", rect->c4.x);
  printf ("rect->c4.y: %li\n", rect->c4.y);

  printf ("%s:%i:%s: point.x: %li\n", __FILE__, __LINE__, __func__, point.x); 
  printf ("%s:%i:%s: point.y: %li\n", __FILE__, __LINE__, __func__, point.y); 

  printf (
      "%s:%i:%s: d_c1_c2_to_c3_c4: %f\n",
      __FILE__, __LINE__, __func__,
      d_c1_c2_to_c3_c4 ); 
  printf (
      "%s:%i:%s: d_c1_c2_to_c3_c4: %f\n",
      __FILE__, __LINE__, __func__,
      d_c2_c3_to_c4_c1 ); 
  */

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

  //gui->add_debug_marker (npo_c1_c2.x, npo_c1_c2.y);

  // Distances from point to nearest point on each side
  double d_point_c1_c2 = vector_mag (vector_from (point, npo_c1_c2));
  double d_point_c2_c3 = vector_mag (vector_from (point, npo_c2_c3));
  double d_point_c3_c4 = vector_mag (vector_from (point, npo_c3_c4));
  double d_point_c4_c1 = vector_mag (vector_from (point, npo_c4_c1));

  // FIXME: WORK POINT: something is wrong about here

  /*
  printf (
      "%s:%i:%s: d_point_c1_c2: %f\n",
      __FILE__, __LINE__, __func__,
      d_point_c1_c2 ); 
  printf (
      "%s:%i:%s: d_point_c2_c3: %f\n",
      __FILE__, __LINE__, __func__,
      d_point_c2_c3 ); 
  printf (
      "%s:%i:%s: d_point_c3_c4: %f\n",
      __FILE__, __LINE__, __func__,
      d_point_c3_c4 ); 
  printf (
      "%s:%i:%s: d_point_c4_c1: %f\n",
      __FILE__, __LINE__, __func__,
      d_point_c4_c1 ); 
      */

  return (
      d_point_c1_c2 <= d_c1_c2_to_c3_c4 &&
      d_point_c3_c4 <= d_c1_c2_to_c3_c4 &&
      d_point_c2_c3 <= d_c2_c3_to_c4_c1 &&
      d_point_c4_c1 <= d_c2_c3_to_c4_c1 );
}

// FIXME: could use const all over the place to doc/sami-enforce what
// doesn't change, ug I dunno, its pretty obvious anyway I think

static bool
circle_intersects_rectangle (
    Circle *circle,
    LineSegment *seg,
    Coord thickness,
    Vec *pii )
{
  // Return true iff circle intersects the rectangle equivalent to all
  // points on line segments of length thickness / 2 orthogonol to seg (with
  // one end point on seg).  If an intersection is found and pii (Point In
  // Intersection) is not NULL, return a point on the intersection in pii.
  // As currently implemented, this routing returns a point on the boundry
  // of the intersection in pii.

  Vec pa = seg->pa, pb = seg->pb;   // Convenience aliases

  // Vector with direction and mag. of segment
  Vec pa_pb = vector_from (pa, pb);

  Vec ov = { -pa_pb.y, pa_pb.x };   // Orthogonol Vector (to segment)

  // Corners of rectangle
  double sf  = thickness / (2.0 * vector_mag (ov));   // Scale Factor
  printf ("%s:%i:%s: sf: %f\n", __FILE__, __LINE__, __func__, sf); 
  Vec c1 = vector_sum (pa, vector_scale (ov, sf));
  Vec c2 = vector_sum (pb, vector_scale (ov, sf));
  Vec c3 = vector_sum (pb, vector_scale (ov, -sf));
  Vec c4 = vector_sum (pa, vector_scale (ov, -sf));

  //gui->add_debug_marker (c1.x, c1.y);
  //gui->add_debug_marker (c2.x, c2.y);
  //gui->add_debug_marker (c3.x, c3.y);
  //gui->add_debug_marker (c4.x, c4.y);

  // Line segments between corners of rectangle
  LineSegment c1_c2 = { c1, c2 };
  LineSegment c2_c3 = { c2, c3 };
  LineSegment c3_c4 = { c3, c4 };
  LineSegment c4_c1 = { c4, c1 };

  // Check if the center of the circle is on the rectangle.  This catches
  // the situation where the circle is entirely inside the rectangle.
  Rectangle rect = { c1, c2, c3, c4 };
  if ( point_is_on_rectangle (circle->center, &rect) ) {
    printf ("%s:%i:%s: CHECKPOINT\n", __FILE__, __LINE__, __func__); 
    *pii = circle->center;
    return true;
  }

  // Note that ip (if not NULL) is computed by the first short-circuit true
  // result here.
  return (
      circle_intersects_line_segment (circle, &c1_c2, pii) ||
      circle_intersects_line_segment (circle, &c2_c3, pii) ||
      circle_intersects_line_segment (circle, &c3_c4, pii) ||
      circle_intersects_line_segment (circle, &c4_c1, pii) );
}

// FIXME: the argument name center is bad, it should be changed to pii
// (Point In Intersection);
bool
IsPointInPad (Coord X, Coord Y, Coord Radius, PadType *Pad, PointType *center)
{
  // Cirlce Around Point, having radius Radius centered at X, Y.  Despite the
  // name of this function it checks for the intersection of a pad and a
  // circle, not a pad and a point.
  Circle circ = { {X, Y}, Radius };

  // Ends of line defining extent of rectangle in one dimension.
  Vec pa = { Pad->Point1.X, Pad->Point1.Y };   // Pad (end) A
  Vec pb = { Pad->Point2.X, Pad->Point2.Y };   // Pad (end) B

  //gui->add_debug_marker (pa.x, pa.y);
  //gui->add_debug_marker (pb.x, pb.y);

  Coord pt = Pad->Thickness;   // Convenience alias

  // Center As Vector (for adapting this fctn interface to Vec interface)
  // FIXME: rename this when we rename center
  Vec cav;  

  printf ("\n\n\n");

  Vec pa_pb = vector_from (pa, pb);
 
  // Cap Vector.  This is a vector in the direction of pa_pb, with magnitude
  // equal to the half the radius or width of the cap.  FIXME: verify this:
  // In pcb PadType objects are of LineType, and LineType objects have width.
  // Pads with rounded caps have caps with radius equal this width.  Thus,
  // they consist of a rectangle of width Pad->thickness and ends Pad-Point1
  // and Point2 in the centers of two sides, unioned with two circles of
  // radius Pad->thickness / 2.  Pads with square caps are similar, but
  // union on a square the width of the pad.  This "Cap Vector" is used
  // to account for these end caps.  We also have a "Reverse Cap Vector"
  // (for the other ent).
  double sf = ((pt + 1) / 2.0) / vector_mag (pa_pb);
  Vec cv = vector_scale (pa_pb, sf);
  Vec rcv = vector_scale (cv, -1.0);
    
  //gui->add_debug_marker (pa_pb.x, pa_pb.y);
  //gui->add_debug_marker (cv.x, cv.y);

  //printf ("%s:%i:%s: checkpoint\n", __FILE__, __LINE__, __func__);
  //printf ("%s:%i:%s: Radius: %li\n", __FILE__, __LINE__, __func__, Radius);
  //printf ("%s:%i:%s: pt: %li\n", __FILE__, __LINE__, __func__, pt);
  
  if ( TEST_FLAG (SQUAREFLAG, Pad) ) {
    // In this case the "pad" is a true rectangle, so here we compute the
    // endpoints of a Rectangular Center Line segment down the center of
    // this rectangle.
    LineSegment rcl = { vector_sum (pa, rcv), vector_sum (pa, cv) };
    if ( circle_intersects_rectangle (&circ, &rcl, pt, &cav) ) {
      if ( center != NULL ) {
        center->X = cav.x;
        center->Y = cav.y;
      }
      return true;
    }
    else {
      return false;
    }
  }
  else {
    printf ("%s:%i:%s: checkpoint\n", __FILE__, __LINE__, __func__); 
    // In this case the rectangular part of the pad runs between the end
    // points in Pad.
    LineSegment rcl = { pa, pb };
    if ( circle_intersects_rectangle (&circ, &rcl, pt, &cav) ) {
      if ( center != NULL ) {
        center->X = cav.x;
        center->Y = cav.y;
      }
      return true;
    }
    else {
      Circle cac = { pa, (pt + 1) / 2 };   // Cap A Circle
      Circle cbc = { pb, (pt + 1) / 2 };   // Cap B Circle
      // Note that center (if not NULL) is computed by the first short-circuit
      // true result here.
      bool result = (
          circles_intersect (&cac, &circ, &cav) ||
          circles_intersect (&cbc, &circ, &cav) );
      if ( result && (center != NULL) ) {
        center->X = cav.x;
        center->Y = cav.y;
      }
      return result;
    }
  }
}


/* ---------------------------------------------------------------------------
 * Check if a circle of Radius with center at (X, Y) intersects a Pad.
 * Written to enable arbitrary pad directions; for rounded pads, too.
 */
//bool
//IsPointInPad (Coord X, Coord Y, Coord Radius, PadType *Pad, PointType *center)
//{
//  double r, Sin, Cos;
//  Coord x; 
//  Coord t2 = (Pad->Thickness + 1) / 2, range;
//  PadType pad = *Pad;
//
//  printf ("%s:%i:%s: checkpoint\n", __FILE__, __LINE__, __func__);
//
//  /* series of transforms saving range */
//  /* move Point1 to the origin */
//  // FIXME: the above comment is not true, the point is move dependeing on
//  // pad position.
//  X -= pad.Point1.X;
//  Y -= pad.Point1.Y;
//
//  pad.Point2.X -= pad.Point1.X;
//  pad.Point2.Y -= pad.Point1.Y;
//  /* so, pad.Point1.X = pad.Point1.Y = 0; */
//  // FIXME: the comment immediately above is not true, and not just because
//  // it uses = rather than ==).  These asserts fail:
//  //assert (pad.Point1.X == 0);
//  //assert (pad.Point1.Y == 0);
//
//  /* rotate round (0, 0) so that Point2 coordinates be (r, 0) */
//  r = Distance (0, 0, pad.Point2.X, pad.Point2.Y);
//  if (r < .1)
//    {
//      Cos = 1;
//      Sin = 0;
//    }
//  else
//    {
//      Sin = pad.Point2.Y / r;
//      Cos = pad.Point2.X / r;
//    }
//  x = X;
//  X = X * Cos + Y * Sin;
//  Y = Y * Cos - x * Sin;
//  /* now pad.Point2.X = r; pad.Point2.Y = 0; */
//  // FIXME: the above comment is wrong, these assertions don't both pass:
//  //assert (pad.Point2.X == r);
//  //assert (pad.Point2.Y == 0);
//
//  /* take into account the ends */
//  if (TEST_FLAG (SQUAREFLAG, Pad))
//    {
//      r += Pad->Thickness;
//      X += t2;
//    }
//  if (Y < 0)
//    Y = -Y;	/* range value is evident now*/
//
//  if (TEST_FLAG (SQUAREFLAG, Pad))
//    {
//      if (X <= 0)
//	{
//	  if (Y <= t2)
//            range = -X;
//          else 
//            {
//              if ( Radius > Distance (0, t2, X, Y) ) {
//                // FIXME: for now we just put it where one end of the pad is,
//                // should make the a better estimate of the center of the
//                // intersection.  If this function was well-named we could
//                // just return the true location of the point, but it isn't:
//                // it actually checks for intersection of pad and circle
//                if ( center != NULL ) {
//                  printf (
//                      "%s:%i:%s: checkpoint\n",
//                      __FILE__, __LINE__, __func__ ); 
//                  center->X = Pad->Point1.X;
//                  center->Y = Pad->Point1.Y;
//                }
//                return true;
//              }
//              else {
//                return false;
//              }
//            }
//	}
//      else if (X >= r)
//	{
//	  if (Y <= t2)
//            range = X - r;
//          else 
//            if ( Radius > Distance (r, t2, X, Y) ) {
//              if ( center != NULL ) {
//                // FIXME: for now we just put it where one end of the pad is,
//                // should make the a better estimate of the center of the
//                // intersection
//                printf (
//                    "%s:%i:%s: checkpoint\n",
//                    __FILE__, __LINE__, __func__ ); 
//                center->X = Pad->Point1.X;
//                center->Y = Pad->Point1.Y;
//              }
//              return true;
//            }
//            else {
//              return false;
//            }
//	}
//      else
//	range = Y - t2;
//    }
//  else/*Rounded pad: even more simple*/
//    {
//      if (X <= 0) {
//        if ( (Radius + t2) > Distance (0, 0, X, Y) ) {
//          if ( center != NULL ) {
//            printf ("%s:%i:%s: checkpoint\n", __FILE__, __LINE__, __func__);
//            // FIXME: needs to find point actually in intersection
//            center->X = Pad->Point1.X;
//            center->Y = Pad->Point1.Y;
//          }
//          return true;
//        }
//        else {
//          return false;
//        }
//      }
//      else if (X >= r) {
//        if ( (Radius + t2) > Distance (r, 0, X, Y) ) {
//          if ( center != NULL ) {
//            printf ("%s:%i:%s: checkpoint\n", __FILE__, __LINE__, __func__);
//            // FIXME: needs to find point actually in intersection
//            center->X = Pad->Point1.X;
//            center->Y = Pad->Point1.Y;
//          }
//          return true;
//        }
//        else {
//          return false;
//        }
//      }
//      else
//	range = Y - t2;
//    }
//  if ( range < Radius ) {
//    if ( center != NULL ) {
//      printf ("%s:%i:%s: checkpoint\n", __FILE__, __LINE__, __func__);
//      // FIXME: needs to find point actually in intersection
//      center->X = Pad->Point2.X;
//      center->Y = Pad->Point2.Y;
//    }
//    return true;
//  }
//  else {
//    return false;
//  }
//}

bool
IsPointInBox (Coord X, Coord Y, BoxType *box, Coord Radius)
{
  Coord width, height, range;

  /* NB: Assumes box has point1 with numerically lower X and Y coordinates */

  /* Compute coordinates relative to Point1 */
  X -= box->X1;
  Y -= box->Y1;

  width =  box->X2 - box->X1;
  height = box->Y2 - box->Y1;

  if (X <= 0)
    {
      if (Y < 0)
        return Radius > Distance (0, 0, X, Y);
      else if (Y > height)
        return Radius > Distance (0, height, X, Y);
      else
        range = -X;
    }
  else if (X >= width)
    {
      if (Y < 0)
        return Radius > Distance (width, 0, X, Y);
      else if (Y > height)
        return Radius > Distance (width, height, X, Y);
      else
        range = X - width;
    }
  else
    {
      if (Y < 0)
        range = -Y;
      else if (Y > height)
        range = Y - height;
      else
        return true;
    }

  return range < Radius;
}

/* TODO: this code is BROKEN in the case of non-circular arcs,
 *       and in the case that the arc thickness is greater than
 *       the radius.
 */
bool
IsPointOnArc (Coord X, Coord Y, Coord Radius, ArcType *Arc)
{
  /* Calculate angle of point from arc center */
  double p_dist = Distance (X, Y, Arc->X, Arc->Y);
  double p_cos = (X - Arc->X) / p_dist;
  Angle p_ang = acos (p_cos) * RAD_TO_DEG;
  Angle ang1, ang2;

  /* Convert StartAngle, Delta into bounding angles in [0, 720) */
  if (Arc->Delta > 0)
    {
      ang1 = NormalizeAngle (Arc->StartAngle);
      ang2 = NormalizeAngle (Arc->StartAngle + Arc->Delta);
    }
  else
    {
      ang1 = NormalizeAngle (Arc->StartAngle + Arc->Delta);
      ang2 = NormalizeAngle (Arc->StartAngle);
    }
  if (ang1 > ang2)
    ang2 += 360;
  /* Make sure full circles aren't treated as zero-length arcs */
  if (Arc->Delta == 360 || Arc->Delta == -360)
    ang2 = ang1 + 360;

  if (Y > Arc->Y)
    p_ang = -p_ang;
  p_ang += 180;

  /* Check point is outside arc range, check distance from endpoints */
  if (ang1 >= p_ang || ang2 <= p_ang)
    {
      Coord ArcX, ArcY;

      ArcX = Arc->X + Arc->Width *
              cos ((Arc->StartAngle + 180) / RAD_TO_DEG);
      ArcY = Arc->Y - Arc->Width *
              sin ((Arc->StartAngle + 180) / RAD_TO_DEG);
      if (Distance (X, Y, ArcX, ArcY) < Radius + Arc->Thickness / 2)
        return true;

      ArcX = Arc->X + Arc->Width *
              cos ((Arc->StartAngle + Arc->Delta + 180) / RAD_TO_DEG);
      ArcY = Arc->Y - Arc->Width *
              sin ((Arc->StartAngle + Arc->Delta + 180) / RAD_TO_DEG);
      if (Distance (X, Y, ArcX, ArcY) < Radius + Arc->Thickness / 2)
        return true;
      return false;
    }
  /* If point is inside the arc range, just compare it to the arc */
  return fabs (Distance (X, Y, Arc->X, Arc->Y) - Arc->Width) < Radius + Arc->Thickness / 2;
}

/* ---------------------------------------------------------------------------
 * searches for any kind of object or for a set of object types
 * the calling routine passes two pointers to allocated memory for storing
 * the results. 
 * A type value is returned too which is NO_TYPE if no objects has been found.
 * A set of object types is passed in.
 * The object is located by it's position.
 *
 * The layout is checked in the following order:
 *   polygon-point, pin, via, line, text, elementname, polygon, element
 *
 * Note that if Type includes LOCKED_TYPE, then the search includes
 * locked items.  Otherwise, locked items are ignored.
 */
int
SearchObjectByLocation (unsigned Type,
			void **Result1, void **Result2, void **Result3,
			Coord X, Coord Y, Coord Radius)
{
  void *r1, *r2, *r3;
  void **pr1 = &r1, **pr2 = &r2, **pr3 = &r3;
  int i;
  double HigherBound = 0;
  int HigherAvail = NO_TYPE;
  int locked = Type & LOCKED_TYPE;
  /* setup variables used by local functions */
  PosX = X;
  PosY = Y;
  SearchRadius = Radius;
  if (Radius)
    {
      SearchBox.X1 = X - Radius;
      SearchBox.Y1 = Y - Radius;
      SearchBox.X2 = X + Radius;
      SearchBox.Y2 = Y + Radius;
    }
  else
    {
      SearchBox = point_box (X, Y);
    }

  if (TEST_FLAG (LOCKNAMESFLAG, PCB))
    {
      Type &= ~ (ELEMENTNAME_TYPE | TEXT_TYPE);
    }
  if (TEST_FLAG (HIDENAMESFLAG, PCB))
    {
      Type &= ~ELEMENTNAME_TYPE;
    }
  if (TEST_FLAG (ONLYNAMESFLAG, PCB))
    {
      Type &= (ELEMENTNAME_TYPE | TEXT_TYPE);
    }
  if (TEST_FLAG (THINDRAWFLAG, PCB) || TEST_FLAG (THINDRAWPOLYFLAG, PCB))
    {
      Type &= ~POLYGON_TYPE;
    }

  if (Type & RATLINE_TYPE && PCB->RatOn &&
      SearchRatLineByLocation (locked,
			       (RatType **) Result1,
			       (RatType **) Result2,
			       (RatType **) Result3))
    return (RATLINE_TYPE);

  if (Type & VIA_TYPE &&
      SearchViaByLocation (locked,
			   (PinType **) Result1,
			   (PinType **) Result2, (PinType **) Result3))
    return (VIA_TYPE);

  if (Type & PIN_TYPE &&
      SearchPinByLocation (locked,
			   (ElementType **) pr1,
			   (PinType **) pr2, (PinType **) pr3))
    HigherAvail = PIN_TYPE;

  if (!HigherAvail && Type & PAD_TYPE &&
      SearchPadByLocation (locked,
			   (ElementType **) pr1,
			   (PadType **) pr2, (PadType **) pr3, false))
    HigherAvail = PAD_TYPE;

  if (!HigherAvail && Type & ELEMENTNAME_TYPE &&
      SearchElementNameByLocation (locked,
				   (ElementType **) pr1,
				   (TextType **) pr2, (TextType **) pr3,
				   false))
    {
      BoxType *box = &((TextType *) r2)->BoundingBox;
      HigherBound = (double) (box->X2 - box->X1) * (double) (box->Y2 - box->Y1);
      HigherAvail = ELEMENTNAME_TYPE;
    }

  if (!HigherAvail && Type & ELEMENT_TYPE &&
      SearchElementByLocation (locked,
			       (ElementType **) pr1,
			       (ElementType **) pr2,
			       (ElementType **) pr3, false))
    {
      BoxType *box = &((ElementType *) r1)->BoundingBox;
      HigherBound = (double) (box->X2 - box->X1) * (double) (box->Y2 - box->Y1);
      HigherAvail = ELEMENT_TYPE;
    }

  for (i = -1; i < max_copper_layer + 1; i++)
    {
      if (i < 0)
	SearchLayer = &PCB->Data->SILKLAYER;
      else if (i < max_copper_layer)
	SearchLayer = LAYER_ON_STACK (i);
      else
	{
	  SearchLayer = &PCB->Data->BACKSILKLAYER;
	  if (!PCB->InvisibleObjectsOn)
	    continue;
	}
      if (SearchLayer->On)
	{
	  if ((HigherAvail & (PIN_TYPE | PAD_TYPE)) == 0 &&
	      Type & POLYGONPOINT_TYPE &&
	      SearchPointByLocation (locked,
				     (LayerType **) Result1,
				     (PolygonType **) Result2,
				     (PointType **) Result3))
	    return (POLYGONPOINT_TYPE);

	  if ((HigherAvail & (PIN_TYPE | PAD_TYPE)) == 0 &&
	      Type & LINEPOINT_TYPE &&
	      SearchLinePointByLocation (locked,
					 (LayerType **) Result1,
					 (LineType **) Result2,
					 (PointType **) Result3))
	    return (LINEPOINT_TYPE);

	  if ((HigherAvail & (PIN_TYPE | PAD_TYPE)) == 0 && Type & LINE_TYPE
	      && SearchLineByLocation (locked,
				       (LayerType **) Result1,
				       (LineType **) Result2,
				       (LineType **) Result3))
	    return (LINE_TYPE);

	    if ((HigherAvail & (PIN_TYPE | PAD_TYPE)) == 0 &&
	      Type & ARCPOINT_TYPE &&
	      SearchArcPointByLocation (locked,
					(LayerType **) Result1,
					(ArcType **) Result2,
					(PointType **) Result3))
	    return (ARCPOINT_TYPE);

	  if ((HigherAvail & (PIN_TYPE | PAD_TYPE)) == 0 && Type & ARC_TYPE &&
	      SearchArcByLocation (locked,
				   (LayerType **) Result1,
				   (ArcType **) Result2,
				   (ArcType **) Result3))
	    return (ARC_TYPE);

	  if ((HigherAvail & (PIN_TYPE | PAD_TYPE)) == 0 && Type & TEXT_TYPE
	      && SearchTextByLocation (locked,
				       (LayerType **) Result1,
				       (TextType **) Result2,
				       (TextType **) Result3))
	    return (TEXT_TYPE);

	  if (Type & POLYGON_TYPE &&
	      SearchPolygonByLocation (locked,
				       (LayerType **) Result1,
				       (PolygonType **) Result2,
				       (PolygonType **) Result3))
	    {
	      if (HigherAvail)
		{
		  BoxType *box =
		    &(*(PolygonType **) Result2)->BoundingBox;
		  double area =
		    (double) (box->X2 - box->X1) * (double) (box->X2 - box->X1);
		  if (HigherBound < area)
		    break;
		  else
		    return (POLYGON_TYPE);
		}
	      else
		return (POLYGON_TYPE);
	    }
	}
    }
  /* return any previously found objects */
  if (HigherAvail & PIN_TYPE)
    {
      *Result1 = r1;
      *Result2 = r2;
      *Result3 = r3;
      return (PIN_TYPE);
    }

  if (HigherAvail & PAD_TYPE)
    {
      *Result1 = r1;
      *Result2 = r2;
      *Result3 = r3;
      return (PAD_TYPE);
    }

  if (HigherAvail & ELEMENTNAME_TYPE)
    {
      *Result1 = r1;
      *Result2 = r2;
      *Result3 = r3;
      return (ELEMENTNAME_TYPE);
    }

  if (HigherAvail & ELEMENT_TYPE)
    {
      *Result1 = r1;
      *Result2 = r2;
      *Result3 = r3;
      return (ELEMENT_TYPE);
    }

  /* search the 'invisible objects' last */
  if (!PCB->InvisibleObjectsOn)
    return (NO_TYPE);

  if (Type & PAD_TYPE &&
      SearchPadByLocation (locked,
			   (ElementType **) Result1,
			   (PadType **) Result2, (PadType **) Result3,
			   true))
    return (PAD_TYPE);

  if (Type & ELEMENTNAME_TYPE &&
      SearchElementNameByLocation (locked,
				   (ElementType **) Result1,
				   (TextType **) Result2,
				   (TextType **) Result3, true))
    return (ELEMENTNAME_TYPE);

  if (Type & ELEMENT_TYPE &&
      SearchElementByLocation (locked,
			       (ElementType **) Result1,
			       (ElementType **) Result2,
			       (ElementType **) Result3, true))
    return (ELEMENT_TYPE);

  return (NO_TYPE);
}

/* ---------------------------------------------------------------------------
 * searches for a object by it's unique ID. It doesn't matter if
 * the object is visible or not. The search is performed on a PCB, a
 * buffer or on the remove list.
 * The calling routine passes two pointers to allocated memory for storing
 * the results. 
 * A type value is returned too which is NO_TYPE if no objects has been found.
 */
int
SearchObjectByID (DataType *Base,
		  void **Result1, void **Result2, void **Result3, int ID,
		  int type)
{
  if (type == LINE_TYPE || type == LINEPOINT_TYPE)
    {
      ALLLINE_LOOP (Base);
      {
	if (line->ID == ID)
	  {
	    *Result1 = (void *) layer;
	    *Result2 = *Result3 = (void *) line;
	    return (LINE_TYPE);
	  }
	if (line->Point1.ID == ID)
	  {
	    *Result1 = (void *) layer;
	    *Result2 = (void *) line;
	    *Result3 = (void *) &line->Point1;
	    return (LINEPOINT_TYPE);
	  }
	if (line->Point2.ID == ID)
	  {
	    *Result1 = (void *) layer;
	    *Result2 = (void *) line;
	    *Result3 = (void *) &line->Point2;
	    return (LINEPOINT_TYPE);
	  }
      }
      ENDALL_LOOP;
    }
  if (type == ARC_TYPE)
    {
      ALLARC_LOOP (Base);
      {
	if (arc->ID == ID)
	  {
	    *Result1 = (void *) layer;
	    *Result2 = *Result3 = (void *) arc;
	    return (ARC_TYPE);
	  }
      }
      ENDALL_LOOP;
    }

  if (type == TEXT_TYPE)
    {
      ALLTEXT_LOOP (Base);
      {
	if (text->ID == ID)
	  {
	    *Result1 = (void *) layer;
	    *Result2 = *Result3 = (void *) text;
	    return (TEXT_TYPE);
	  }
      }
      ENDALL_LOOP;
    }

  if (type == POLYGON_TYPE || type == POLYGONPOINT_TYPE)
    {
      ALLPOLYGON_LOOP (Base);
      {
	if (polygon->ID == ID)
	  {
	    *Result1 = (void *) layer;
	    *Result2 = *Result3 = (void *) polygon;
	    return (POLYGON_TYPE);
	  }
	if (type == POLYGONPOINT_TYPE)
	  POLYGONPOINT_LOOP (polygon);
	{
	  if (point->ID == ID)
	    {
	      *Result1 = (void *) layer;
	      *Result2 = (void *) polygon;
	      *Result3 = (void *) point;
	      return (POLYGONPOINT_TYPE);
	    }
	}
	END_LOOP;
      }
      ENDALL_LOOP;
    }
  if (type == VIA_TYPE)
    {
      VIA_LOOP (Base);
      {
	if (via->ID == ID)
	  {
	    *Result1 = *Result2 = *Result3 = (void *) via;
	    return (VIA_TYPE);
	  }
      }
      END_LOOP;
    }

  if (type == RATLINE_TYPE || type == LINEPOINT_TYPE)
    {
      RAT_LOOP (Base);
      {
	if (line->ID == ID)
	  {
	    *Result1 = *Result2 = *Result3 = (void *) line;
	    return (RATLINE_TYPE);
	  }
	if (line->Point1.ID == ID)
	  {
	    *Result1 = (void *) NULL;
	    *Result2 = (void *) line;
	    *Result3 = (void *) &line->Point1;
	    return (LINEPOINT_TYPE);
	  }
	if (line->Point2.ID == ID)
	  {
	    *Result1 = (void *) NULL;
	    *Result2 = (void *) line;
	    *Result3 = (void *) &line->Point2;
	    return (LINEPOINT_TYPE);
	  }
      }
      END_LOOP;
    }

  if (type == ELEMENT_TYPE || type == PAD_TYPE || type == PIN_TYPE
      || type == ELEMENTLINE_TYPE || type == ELEMENTNAME_TYPE
      || type == ELEMENTARC_TYPE)
    /* check pins and elementnames too */
    ELEMENT_LOOP (Base);
  {
    if (element->ID == ID)
      {
	*Result1 = *Result2 = *Result3 = (void *) element;
	return (ELEMENT_TYPE);
      }
    if (type == ELEMENTLINE_TYPE)
      ELEMENTLINE_LOOP (element);
    {
      if (line->ID == ID)
	{
	  *Result1 = (void *) element;
	  *Result2 = *Result3 = (void *) line;
	  return (ELEMENTLINE_TYPE);
	}
    }
    END_LOOP;
    if (type == ELEMENTARC_TYPE)
      ARC_LOOP (element);
    {
      if (arc->ID == ID)
	{
	  *Result1 = (void *) element;
	  *Result2 = *Result3 = (void *) arc;
	  return (ELEMENTARC_TYPE);
	}
    }
    END_LOOP;
    if (type == ELEMENTNAME_TYPE)
      ELEMENTTEXT_LOOP (element);
    {
      if (text->ID == ID)
	{
	  *Result1 = (void *) element;
	  *Result2 = *Result3 = (void *) text;
	  return (ELEMENTNAME_TYPE);
	}
    }
    END_LOOP;
    if (type == PIN_TYPE)
      PIN_LOOP (element);
    {
      if (pin->ID == ID)
	{
	  *Result1 = (void *) element;
	  *Result2 = *Result3 = (void *) pin;
	  return (PIN_TYPE);
	}
    }
    END_LOOP;
    if (type == PAD_TYPE)
      PAD_LOOP (element);
    {
      if (pad->ID == ID)
	{
	  *Result1 = (void *) element;
	  *Result2 = *Result3 = (void *) pad;
	  return (PAD_TYPE);
	}
    }
    END_LOOP;
  }
  END_LOOP;

  Message ("hace: Internal error, search for ID %d failed\n", ID);
  return (NO_TYPE);
}

/* ---------------------------------------------------------------------------
 * searches for an element by its board name.
 * The function returns a pointer to the element, NULL if not found
 */
ElementType *
SearchElementByName (DataType *Base, char *Name)
{
  ElementType *result = NULL;

  ELEMENT_LOOP (Base);
  {
    if (element->Name[1].TextString &&
	NSTRCMP (element->Name[1].TextString, Name) == 0)
      {
	result = element;
	return (result);
      }
  }
  END_LOOP;
  return result;
}

/* ---------------------------------------------------------------------------
 * searches the cursor position for the type 
 */
int
SearchScreen (Coord X, Coord Y, int Type, void **Result1,
	      void **Result2, void **Result3)
{
  int ans;

  ans = SearchObjectByLocation (Type, Result1, Result2, Result3,
				X, Y, SLOP * pixel_slop);
  return (ans);
}
