#include "MainWindow.h"

#include "ui_MainWindow.h"

#include <QFileDialog>

MainWindow::MainWindow(QWidget* parent)
: QMainWindow(parent), ui(new Ui::MainWindow) {
	ui->setupUi(this);
	ui->riverButton->setChecked(true);
	this->fieldViewer = new ImageWidget(this);
	this->fieldViewer->SetGeometry(0, 0, 1300, 821);
	this->roadViewer = new ImageWidget(this);
	this->roadViewer->SetGeometry(0, 0, 1300, 821);
	terrain.UpdateElementMask(this->fieldViewer->img_, 1.0);
	this->fieldViewer->SetImage(terrain.VisualizeField());
	ui->tabWidget->clear();
	ui->tabWidget->addTab(this->fieldViewer, "Field");
	ui->tabWidget->addTab(this->roadViewer, "Road");
	connect(ui->tabWidget, SIGNAL(currentChanged(int)), this, SLOT(TabSelected()));
	drawer.color_ = cv::Scalar(0x38, 0xB3, 0xD0);
	drawer.SetCanvas(this->fieldViewer);
	drawer.FinalizeCallback = std::bind(&MainWindow::UpdateFromDrawer, this);
	prevViewer = this->fieldViewer;

	connect(ui->riverButton,SIGNAL(clicked()),this,SLOT(ChangeElement()));
	connect(ui->grassButton,SIGNAL(clicked()),this,SLOT(ChangeElement()));
	connect(ui->populationButton,SIGNAL(clicked()),this,SLOT(ChangeElement()));
	connect(ui->guidanceButton,SIGNAL(clicked()),this,SLOT(ChangeElement()));
	connect(ui->groundButton,SIGNAL(clicked()),this,SLOT(ChangeElement()));

	connect(ui->actionLoadTerrain, SIGNAL(triggered()), this, SLOT(LoadTerrain()));
	connect(ui->actionSaveTerrain, SIGNAL(triggered()), this, SLOT(SaveTerrain()));
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
	// nothing
}

void MainWindow::TabSelected()
{
	/*
	if (ui->tabWidget->currentIndex() == 0) {
		drawer.color_ = cv::Scalar(0x38, 0xB3, 0xD0);
		drawer.SetCanvas(this->fieldViewer);
	}
	else if (ui->tabWidget->currentIndex() == 1) {
		drawer.color_ = cv::Scalar(0x7E, 0xC8, 0x50);
		drawer.SetCanvas(this->roadViewer);
	}
	*/
	drawer.imageWidget_->SetImage(prevViewer->img_);
	prevViewer = drawer.imageWidget_;
}

void MainWindow::ChangeElement()
{
	if (ui->riverButton->isChecked()) {
		drawer.color_ = cv::Scalar(0x38, 0xB3, 0xD0);
	}
	else if (ui->grassButton->isChecked()) {
		drawer.color_ = cv::Scalar(0x7E, 0xC8, 0x50);
	}
	else if (ui->populationButton->isChecked()) {
		drawer.color_ = cv::Scalar(0xFF, 0, 0);
	}
	else if (ui->guidanceButton->isChecked()) {
		drawer.color_ = cv::Scalar(0xFF, 0xFF, 0);
	}
	else if (ui->groundButton->isChecked()) {
		drawer.color_ = cv::Scalar(0xFF, 0xFF, 0xFF);
	}
}

void MainWindow::UpdateFromDrawer()
{
	if (ui->tabWidget->currentIndex() == 0) {
		terrain.UpdateElementMask(this->fieldViewer->img_, 1e-1);
		this->fieldViewer->SetImage(terrain.VisualizeField());
	}
	else if (ui->tabWidget->currentIndex() == 1) {

	}
}

void MainWindow::LoadTerrain()
{
	auto fileName = QFileDialog::getOpenFileName(this,
         tr("Open Terrain File "), QDir::currentPath(), tr("Terrain Files (*.trn)")).toStdString();
}

void MainWindow::SaveTerrain()
{
	auto fileName = QFileDialog::getSaveFileName(this,
         tr("Open Terrain File "), QDir::currentPath(), tr("Terrain Files (*.trn)")).toStdString();
	if (fileName.size() < 4 || fileName.substr(fileName.size() - 4) != ".trn")
		fileName += ".trn";

}