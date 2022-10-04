#include "ImageWidget.h"

ImageWidget::ImageWidget(QWidget* parent) : QLabel(parent) {
	SetDefaultParameters();
	this->setMouseTracking(true);
	MouseMove = [] {};
	MouseClick = [] {};
	MouseRelease = [] {};
	MouseCancel = [] {};
}

ImageWidget::~ImageWidget() {}

void ImageWidget::SetDefaultParameters()
{
	scale_ = 1;
	offset_ = Vector2(0, 0);
	current_mouse_ = Vector2(0, 0);
	prev_mouse_ = Vector2(0, 0);
	pressed_ = 0;
	fixed_ = 0;
}

void ImageWidget::SetGeometry(int x, int y, int width, int height)
{
	this->setGeometry(x, y, width, height);
	cv::Mat img = cv::Mat::zeros(height, width, CV_8UC3);
	for (int i = 0; i < img.rows; ++i) {
		for (int j = 0; j < img.cols; ++j) {
			img.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255);
		}
	}
	SetImage(img);
}

void ImageWidget::SetImage(cv::Mat img)
{
	img_ = img;
	Paint();
}

void ImageWidget::Paint() {
	if (img_.rows == 0) {
		img_ = cv::Mat::zeros(256, 256, CV_8UC3);
		for (int i = 0; i < 256; ++i) {
			for (int j = 0; j < 256; ++j) {
				img_.at<cv::Vec3b>(i, j) = cv::Vec3b(i, j, 255);
			}
		}
	}
	DisplayImage(img_);
}

void ImageWidget::DisplayImage(cv::Mat img) {
	cv::Mat newImg(img.rows, img.cols, CV_8UC3);
	cv::Mat mapX(img.rows, img.cols, CV_32F);
	cv::Mat mapY(img.rows, img.cols, CV_32F);
	for (int i = 0; i < img.rows; ++i) {
		for (int j = 0; j < img.cols; ++j) {
			mapX.at<float>(i, j) = j * scale_ + offset_[0];
			mapY.at<float>(i, j) = i * scale_ + offset_[1];
		}
	}
	cv::remap(img, newImg, mapX, mapY, cv::INTER_LINEAR);
	QImage imdisplay(newImg.data, newImg.cols, newImg.rows, newImg.step, QImage::Format_RGB888);
	QPixmap pixmap = QPixmap::fromImage(imdisplay);
	this->setPixmap(pixmap);
}

void ImageWidget::resizeEvent(QResizeEvent* event) { Paint(); }

void ImageWidget::mousePressEvent(QMouseEvent* event) {
	if (event->button() == Qt::LeftButton) {
		prev_mouse_ = Vector2(event->x(), event->y());
		current_mouse_ = prev_mouse_;
		MouseClick();
		if (!fixed_) {
			pressed_ = 1;
		}
	}
	else if (event->button() == Qt::RightButton) {
		MouseCancel();
	}
}

void ImageWidget::mouseReleaseEvent(QMouseEvent* event) {
	if (pressed_ || fixed_) {
		pressed_ = 0;
	}
	MouseRelease();
	Paint();
}

void ImageWidget::wheelEvent(QWheelEvent* event) {
	if (fixed_) return;
	current_mouse_ = Vector2(event->x(), event->y());
	Vector2 target = current_mouse_ * scale_ + offset_;
	int degree = event->angleDelta().y();
	double amplifier = 1.03;
	if (degree < 0)
		amplifier = 1.0 / amplifier;
	scale_ *= amplifier;
	offset_ = target - current_mouse_ * scale_;
	Paint();
}

void ImageWidget::mouseMoveEvent(QMouseEvent* event) {
	current_mouse_ = Vector2(event->x(), event->y());
	MouseMove();
	if (!pressed_ || fixed_) return;
	if (img_.cols > 0) {
		Vector2 mouse_temp;
		mouse_temp[0] = (current_mouse_[0] - prev_mouse_[0]) * scale_;
		mouse_temp[1] = (current_mouse_[1] - prev_mouse_[1]) * scale_;
		offset_ -= mouse_temp;
		prev_mouse_ = current_mouse_;
	}
	Paint();
}