#ifndef __BIRDEYEVIEW_H__
#define __BIRDEYEVIEW_H__

#include "geometry2D.h"

struct track_widths {
    LineSegment upper_segment;
    LineSegment lower_segment;
};

void getPerspectiveTransform(Point2D src[4], Point2D dst[4], float H[3][3]);

Point2D applyPerspectiveTransform(float H[3][3], Point2D p);

struct track_widths getTrackWidths(LineSegment left_segment_, LineSegment right_segment_, float frame_height_);

struct track_widths srcViewToRealView(struct track_widths src_view, float real_track_width_m, float frame_width);


#endif // !__BIRDEYEVIEW_H__
