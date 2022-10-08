#include "Drawer.h"

Drawer::Drawer()
{
	drawerMode_ = EMPTY;
	operationMode_ = REST;
	imageWidget_ = 0;
	FinalizeCallback = []{};
}

void Drawer::SetCanvas(ImageWidget* imageWidget)
{
	imageWidget_ = imageWidget;
	// bind events to the drawer member function
	imageWidget_->MouseMove = std::bind(&Drawer::MouseMove, this);
	imageWidget_->MouseClick = std::bind(&Drawer::MouseClick, this);
	imageWidget_->MouseRelease = std::bind(&Drawer::MouseRelease, this);
	imageWidget_->MouseCancel = std::bind(&Drawer::MouseCancel, this);
	// get the previous image from the widget
	restImg_ = imageWidget_->img_.clone();
}

void Drawer::RenderPrimitives()
{
	if (!imageWidget_) return;
	// draw a primitive, visualize it in the widget,
	cv::Mat tempImg = DrawPrimitive(drawerMode_);
	cv::Mat originImg = imageWidget_->img_;
	imageWidget_->SetImage(tempImg);
	// but reset the image data as previous
	imageWidget_->img_ = originImg;
}

void Drawer::FinalizePrimitives()
{
	if (!imageWidget_) return;
	// for line drawing, smooth it
	if (prevDrawerMode_ == LINE && operationMode_ == EMPTY) {
		std::vector<Vector2> tempList = shapeList;
		for (int i = 0; i < shapeList.size(); ++i) {
			float counter = 0;
			Vector2 corner(0, 0);
			for (int j = -10; j <= 10; ++j) {
				int ni = i + j;
				if (ni < 0)
					ni = 0;
				if (ni >= shapeList.size())
					ni = shapeList.size() - 1;
				float d = (tempList[ni] - tempList[i]).norm();
				float weight = exp(-(d * d) / 900.0);
				corner += tempList[ni] * weight;
				counter += weight;
			}
			corner /= (double)counter;
			shapeList[i] = corner;
		}
	}
	// draw a primitive to image, and apply it to the image
	cv::Mat tempImg = DrawPrimitive(prevDrawerMode_);
	restImg_ = imageWidget_->img_.clone();
	imageWidget_->SetImage(tempImg);
	FinalizeCallback();
}

cv::Mat Drawer::PrimitiveMask()
{
	cv::Mat tempImg = imageWidget_->img_.clone();
	imageWidget_->img_ = cv::Mat::zeros(tempImg.rows, tempImg.cols, CV_8UC3);
	cv::Mat res = DrawPrimitive(prevDrawerMode_);
	imageWidget_->img_ = tempImg;
	cv::Mat mask = cv::Mat::zeros(tempImg.rows, tempImg.cols, CV_8U);
	for (int i = 0; i < res.rows; ++i) {
		for (int j = 0; j < res.cols; ++j) {
			auto a = res.at<cv::Vec3b>(i, j);
			if (a.val[0] == color_.val[0] && a.val[1] == color_.val[1] && a.val[2] == color_.val[2]) {
				mask.at<unsigned char>(i, j) = 1;
			}
		}
	}
	return mask;
}

cv::Mat Drawer::PrimitiveDirection()
{
	cv::Mat mask = PrimitiveMask();
	cv::Mat direction(mask.rows, mask.cols, CV_8UC3);
	for (int i = 0; i < direction.rows; ++i) {
		for (int j = 0; j < direction.cols; ++j) {
			if (prevDrawerMode_ == EMPTY || prevDrawerMode_ == LINE || prevDrawerMode_ == ELLIPSE) {
				direction.at<cv::Vec3b>(i, j) = cv::Vec3b(127, 127, 0);
			}
			else if (mask.at<unsigned char>(i, j) == 0) {
				direction.at<cv::Vec3b>(i, j) = cv::Vec3b(127, 127, 127);
			}
			else {
				double angle = rotation_;
				Vector2 xAxis(cos(angle / 180.0 * 3.141592654), sin(angle / 180.0 * 3.141592654));
				if (prevDrawerMode_ == CIRCLE) {
					Vector2 mid = (shapeList[0] + shapeList[1]) * 0.5;
					xAxis = Vector2(j - mid[0], i - mid[1]);
					xAxis.normalize();
				}
				direction.at<cv::Vec3b>(i, j) = cv::Vec3b(xAxis[0] * 127 + 127,
					xAxis[1] * 127 + 127, 255);
			}
		}
	}
	return direction;
}

cv::Mat Drawer::DrawPrimitive(DrawerMode mode)
{
	cv::Mat tempImg = imageWidget_->img_.clone();
	// draw a circle
	if (mode == CIRCLE && shapeList.size() == 2) {
		Vector2 mid = (shapeList[0] + shapeList[1]) * 0.5;
		mid += translation_;
		if (operationMode_ == TRANSLATE && opList.size() == 2) {
			mid += opList[0] - opList[1];
		}
		double radius = (shapeList[0] - shapeList[1]).norm() * 0.5;
		if (radius == 0) return tempImg;
		radius *= scale_;
		if (operationMode_ == SCALE && opList.size() == 2) {
			radius *= exp((opList[1][0] - opList[0][0]) / 100.0);
		}
		cv::circle(tempImg, cv::Point(mid[0], mid[1]), radius, color_, -1, 8, 0);
	}
	// draw a ellipse
	else if (mode == ELLIPSE && shapeList.size() == 2) {
		Vector2 mid = (shapeList[0] + shapeList[1]) * 0.5;
		mid += translation_;
		if (operationMode_ == TRANSLATE && opList.size() == 2) {
			mid += opList[1] - opList[0];
		}
		double angle = rotation_;
		if (operationMode_ == ROTATE && opList.size() == 2) {
			angle += opList[1][1] - opList[0][1];
		}
		double radiusX = std::abs((shapeList[0] - shapeList[1])[0] * 0.5) * scale_;
		double radiusY = std::abs((shapeList[0] - shapeList[1])[1] * 0.5) * scale_;
		if (operationMode_ == SCALE && opList.size() == 2) {
			radiusX *= exp((opList[1][0] - opList[0][0]) / 100.0);
			radiusY *= exp((opList[1][0] - opList[0][0]) / 100.0);
		}
		if (radiusX == 0 || radiusY == 0) return tempImg;
		cv::ellipse(tempImg, cv::Point(mid[0], mid[1]), cv::Size(radiusX, radiusY), angle, 0, 360, color_, -1);
	}
	// draw a rectangle (two triangles)
	else if (mode == BOX && shapeList.size() == 2) {
		if (shapeList[0][0] == shapeList[1][0] || shapeList[0][1] == shapeList[1][1])
			return tempImg;
		Vector2 mid = (shapeList[0] + shapeList[1]) * 0.5;
		mid += translation_;
		if (operationMode_ == TRANSLATE && opList.size() == 2) {
			mid += opList[1] - opList[0];
		}
		double angle = rotation_;
		if (operationMode_ == ROTATE && opList.size() == 2) {
			angle += opList[1][1] - opList[0][1];
		}
		double radiusX = std::abs((shapeList[0] - shapeList[1])[0] * 0.5) * scale_;
		double radiusY = std::abs((shapeList[0] - shapeList[1])[1] * 0.5) * scale_;
		if (operationMode_ == SCALE && opList.size() == 2) {
			radiusX *= exp((opList[1][0] - opList[0][0]) / 100.0);
			radiusY *= exp((opList[1][0] - opList[0][0]) / 100.0);
		}
		if (radiusX == 0 || radiusY == 0) return tempImg;
		Vector2 xAxis(cos(angle / 180.0 * 3.141592654), sin(angle / 180.0 * 3.141592654));
		angle += 90;
		Vector2 yAxis(cos(angle / 180.0 * 3.141592654), sin(angle / 180.0 * 3.141592654));
		if (angle == 90) {
			xAxis = Vector2(1, 0);
			yAxis = Vector2(0, 1);
		}
		Vector2 p00 = mid - radiusX * xAxis - radiusY * yAxis;
		Vector2 p01 = mid + radiusX * xAxis - radiusY * yAxis;
		Vector2 p10 = mid - radiusX * xAxis + radiusY * yAxis;
		Vector2 p11 = mid + radiusX * xAxis + radiusY * yAxis;
		std::vector<std::vector<cv::Point>> arr = {
			{cv::Point(p00[0], p00[1]), cv::Point(p10[0],p10[1]), cv::Point(p01[0],p01[1])},
			{cv::Point(p11[0], p11[1]), cv::Point(p10[0],p10[1]), cv::Point(p01[0],p01[1])}};
		cv::fillPoly(tempImg, arr, color_);
	}
	// draw line strips
	else if (mode == LINE && shapeList.size() > 0) {
		float thickness = scale_;
		if (operationMode_ == SCALE && opList.size() == 2) {
			thickness *= exp((opList[1][0] - opList[0][0]) / 100.0);
		}
		if (thickness < 0.5)
			thickness = 0.5;

		for (int i = 0; i < shapeList.size() - 1; ++i) {
			cv::Point pt0(shapeList[i][0], shapeList[i][1]);
			cv::Point pt1(shapeList[i + 1][0], shapeList[i + 1][1]);
			cv::line(tempImg, pt0, pt1, color_, thickness * 2);
		}
	}
	return tempImg;
}

void Drawer::keyPressEvent(QKeyEvent* event)
{
	// press keys to start primitive or transformation mode
	if (!imageWidget_) return;
	auto defaultParam = [this] {
		shapeList.clear();
		opList.clear();
		prevDrawerMode_ = EMPTY;
		rotation_ = 0;
		translation_ = Vector2(0, 0);
		scale_ = 1;
		imageWidget_->fixed_ = 1;
	};
	switch (event->key()) {
		case Qt::Key_C: // circle
			if (operationMode_ == REST) {
				defaultParam();
				drawerMode_ = CIRCLE;
			}
			break;
		case Qt::Key_E: // ellipse
			if (operationMode_ == REST) {
				defaultParam();
				drawerMode_ = ELLIPSE;
			}
			break;
		case Qt::Key_L: // line
			if (operationMode_ == REST) {
				defaultParam();
				drawerMode_ = LINE;
			}
			break;
		case Qt::Key_B: // box
			if (operationMode_ == REST) {
				defaultParam();
				drawerMode_ = BOX;
			}
			break;
		case Qt::Key_T: // trannslation
			if (prevDrawerMode_ != EMPTY) {
				operationMode_ = TRANSLATE;
				opList.clear();
				opList.push_back(imageWidget_->GetImageCoord());
				opList.push_back(imageWidget_->GetImageCoord());
				imageWidget_->fixed_ = 1;
			}
			break;
		case Qt::Key_R: // trannslation
			if (prevDrawerMode_ != EMPTY) {
				operationMode_ = ROTATE;
				opList.clear();
				opList.push_back(imageWidget_->GetImageCoord());
				opList.push_back(imageWidget_->GetImageCoord());
				imageWidget_->fixed_ = 1;
			}
			break;
		case Qt::Key_S: // trannslation
			if (prevDrawerMode_ != EMPTY) {
				operationMode_ = SCALE;
				opList.clear();
				opList.push_back(imageWidget_->GetImageCoord());
				opList.push_back(imageWidget_->GetImageCoord());
				imageWidget_->fixed_ = 1;
			}
			break;
		default:
			break;
	}
}

void Drawer::MouseClick()
{
	if (!imageWidget_) return;
	Vector2 pix = imageWidget_->GetImageCoord();
	// if it is under the transformation mode, finalize the drawing
	if (drawerMode_ == EMPTY) {
		if (operationMode_ != REST) {
			imageWidget_->SetImage(restImg_);
			opList.back() = pix;
			FinalizePrimitives();
		}
		return;
	}
	// otherwise, simply redraw the primitive and visualize it
	shapeList.clear();
	shapeList.push_back(pix);
	shapeList.push_back(pix);
	RenderPrimitives();
}

void Drawer::MouseMove()
{
	if (!imageWidget_) return;
	Vector2 pix = imageWidget_->GetImageCoord();
	if (shapeList.empty()) return;
	// for drawing mode, simply update the visualization
	if (drawerMode_ == LINE) {
		shapeList.push_back(pix);
		RenderPrimitives();
	}
	else if (drawerMode_ != EMPTY) {
		shapeList.back() = pix;
		RenderPrimitives();
	}
	// otherwise, finalize it (by smoothing line strip)
	else if (prevDrawerMode_ != EMPTY && operationMode_ != REST) {
		opList.back() = pix;
		imageWidget_->SetImage(restImg_);

		cv::Mat tempImg = DrawPrimitive(prevDrawerMode_);
		restImg_ = imageWidget_->img_.clone();
		imageWidget_->SetImage(tempImg);
	}
}

void Drawer::MouseRelease()
{
	if (!imageWidget_) return;
	Vector2 pix = imageWidget_->GetImageCoord();
	// finalize the primitive drawing
	if (drawerMode_ != EMPTY) {
		prevDrawerMode_ = drawerMode_;
		drawerMode_ = EMPTY;
		FinalizePrimitives();
	}
	// finalize the transformation
	else if (prevDrawerMode_ != EMPTY && operationMode_ != REST) {
		imageWidget_->SetImage(restImg_);
		if (operationMode_ == TRANSLATE) {
			translation_ += opList[1] - opList[0];
		}
		else if (operationMode_ == ROTATE) {
			rotation_ += opList[1][1] - opList[0][1];
		}
		else if (operationMode_ == SCALE) {
			scale_ *= exp((opList[1][0] - opList[0][0]) / 100.0);
		}
		opList.clear();
		FinalizePrimitives();
		operationMode_ = REST;
	}
	imageWidget_->fixed_ = 0;
}

void Drawer::MouseCancel()
{
	if (!imageWidget_) return;
	// roll back the transformation
	if (prevDrawerMode_ != EMPTY && operationMode_ != REST) {
		operationMode_ = REST;
		opList.clear();
		imageWidget_->SetImage(restImg_);
		FinalizePrimitives();
		imageWidget_->fixed_ = 0;
		return;
	}
	// roll back the primitive drawing
	imageWidget_->SetImage(restImg_);
	drawerMode_ = EMPTY;
	prevDrawerMode_ = EMPTY;
	imageWidget_->fixed_ = 0;
}