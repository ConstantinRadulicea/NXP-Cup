#ifndef __BIRDEYEVIEW_H__
#define __BIRDEYEVIEW_H__

#include "geometry2D.h"

void getPerspectiveTransform(Point2D src[4], Point2D dst[4], float H[3][3]);
Point2D applyPerspectiveTransform(float H[3][3], Point2D p);


#endif // !__BIRDEYEVIEW_H__
