
// Support for constant-pixel size debug markers
#define DEBUG_MARKER_RADIUS_PIXELS 8
#define MAX_DEBUG_MARKER_COUNT 1042
typedef struct {
  Coord x, y;
} DebugMarker;
extern DebugMarker debug_markers[MAX_DEBUG_MARKER_COUNT];

void common_gui_draw_pcb_polygon (hidGC gc, PolygonType *poly, const BoxType *clip_box);
void common_fill_pcb_polygon (hidGC gc, PolygonType *poly, const BoxType *clip_box);
void common_thindraw_pcb_polygon (hidGC gc, PolygonType *poly, const BoxType *clip_box);
void common_fill_pcb_pad (hidGC gc, PadType *pad, bool clear, bool mask);
void common_thindraw_pcb_pad (hidGC gc, PadType *pad, bool clear, bool mask);
void common_fill_pcb_pv (hidGC fg_gc, hidGC bg_gc, PinType *pv, bool drawHole, bool mask);
void common_thindraw_pcb_pv (hidGC fg_gc, hidGC bg_gc, PinType *pv, bool drawHole, bool mask);
void common_draw_helpers_init (HID_DRAW *graphics);
