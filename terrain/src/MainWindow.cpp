#include "MainWindow.h"

#include "ui_MainWindow.h"

MainWindow::MainWindow(QWidget* parent)
: QMainWindow(parent), ui(new Ui::MainWindow) {
	ui->setupUi(this);
	this->riverViewer = new ImageWidget(this);
	this->riverViewer->SetGeometry(0, 0, 640, 480);
	this->grassViewer = new ImageWidget(this);
	this->grassViewer->SetGeometry(0, 0, 640, 480);
	ui->tabWidget->clear();
	ui->tabWidget->addTab(this->riverViewer, "River");
	ui->tabWidget->addTab(this->grassViewer, "Grass");
	connect(ui->tabWidget, SIGNAL(currentChanged(int)), this, SLOT(TabSelected()));
	drawer.color_ = cv::Scalar(0x38, 0xB3, 0xD0);
	drawer.SetCanvas(this->riverViewer);
	prevViewer = this->riverViewer;
}

MainWindow::~MainWindow() {
	delete ui;
	QApplication::quit();
}

// key events
void MainWindow::keyPressEvent(QKeyEvent* event)
{
	drawer.keyPressEvent(event);
}

void MainWindow::keyReleaseEvent(QKeyEvent* event)
{
	// nothinng
}

void MainWindow::TabSelected()
{
	if (ui->tabWidget->currentIndex() == 0) {
		drawer.color_ = cv::Scalar(0x38, 0xB3, 0xD0);
		drawer.SetCanvas(this->riverViewer);
	}
	else if (ui->tabWidget->currentIndex() == 1) {
		drawer.color_ = cv::Scalar(0x7E, 0xC8, 0x50);
		drawer.SetCanvas(this->grassViewer);
	}
	drawer.imageWidget_->SetImage(prevViewer->img_);
	prevViewer = drawer.imageWidget_;
}