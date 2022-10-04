#ifndef TERRAIN_IMAGEWIDGET_H_
#define TERRAIN_IMAGEWIDGET_H_

#include <QLabel>
#include <QMouseEvent>
#include <QWheelEvent>
#include <opencv2/opencv.hpp>
#include <memory>
#include <unordered_map>
#include <Eigen/Core>
#include <functional>
using Vector2 = Eigen::Vector2d;

// basic image visualization ui
class ImageWidget : public QLabel {
	Q_OBJECT
public:
	explicit ImageWidget(QWidget* parent = 0);
	~ImageWidget();

	// dispklay an rgb image
	void SetGeometry(int x, int y, int width, int height);
	void SetImage(cv::Mat img);
	void Paint();

	void mousePressEvent(QMouseEvent* event);
	void mouseReleaseEvent(QMouseEvent* event);
	void mouseMoveEvent(QMouseEvent* event);
	void wheelEvent(QWheelEvent* event);
	Vector2 GetImageCoord() const {
		return current_mouse_ * scale_ + offset_;
	}
	std::function<void()> MouseClick;
	std::function<void()> MouseMove;
	std::function<void()> MouseCancel;
	std::function<void()> MouseRelease;
	void DisplayImage(cv::Mat img);
	void SetDefaultParameters();
	void resizeEvent(QResizeEvent* event);

	cv::Mat img_;
	double scale_;
	int fixed_;
	int pressed_;
	Vector2 offset_, current_mouse_;
	Vector2 prev_offset_, prev_mouse_;
};

#endif