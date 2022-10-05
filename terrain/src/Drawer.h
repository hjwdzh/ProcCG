#ifndef TERRAIN_DRAWER_H_
#define TERRAIN_DRAWER_H_

#include "ImageWidget.h"

// associate drawer to a imageWidget to draw and transform
class Drawer
{
public:
	Drawer();
	enum DrawerMode { EMPTY, CIRCLE, ELLIPSE, LINE, BOX };
	enum OperationMode { REST, SCALE, ROTATE, TRANSLATE };

	void SetCanvas(ImageWidget* imageWidget);	// set the target canvas
	void RenderPrimitives();					// render drawing without finalizing
	void FinalizePrimitives();					// finalize drawinng of shape to widget

	cv::Mat DrawPrimitive(DrawerMode mode);		// given recorded transform/shape, draw primitive		
	cv::Mat PrimitiveMask();
	cv::Mat PrimitiveDirection();

	void keyPressEvent(QKeyEvent* event);
	void MouseMove();
	void MouseRelease();
	void MouseCancel();
	void MouseClick();

	std::function<void()> FinalizeCallback;

	DrawerMode drawerMode_, prevDrawerMode_;	// the primitive shape that is drawing/drawed
	OperationMode operationMode_;				// the current transform that is applying
	
	ImageWidget* imageWidget_;			 		// target drawing widget

	cv::Mat restImg_; 							// temp image
	std::vector<Vector2> shapeList, opList; 	// temp shape/transform info

	cv::Scalar color_; 							// drawing color
	Vector2 translation_;						// translation that is applying
	float scale_;								// scale that is applying
	float rotation_;							// rotation nthat is applying
};

#endif