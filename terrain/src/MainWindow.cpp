#include "MainWindow.h"

#include "ui_MainWindow.h"

#include <QFileDialog>
#include <QDoubleValidator>
#include "RoadGenerator.h"

MainWindow::MainWindow(QWidget* parent)
: QMainWindow(parent), ui(new Ui::MainWindow) {
	ui->setupUi(this);
	ui->riverButton->setChecked(true);
	this->fieldViewer = new ImageWidget(this);
	this->fieldViewer->SetGeometry(0, 0, 1300, 821);
	this->roadViewer = new ImageWidget(this);
	this->roadViewer->SetGeometry(0, 0, 1300, 821);
	terrain.UpdateElementMask(this->fieldViewer->img_, cv::Mat(), 1.0);
	this->fieldViewer->SetImage(terrain.VisualizeField());
	this->roadViewer->SetImage(terrain.VisualizeElement());

	ui->tabWidget->clear();
	ui->tabWidget->addTab(this->fieldViewer, "Field");
	ui->tabWidget->addTab(this->roadViewer, "Road");
	connect(ui->tabWidget, SIGNAL(currentChanged(int)), this, SLOT(TabSelected()));
	drawer.color_ = cv::Scalar(0x38, 0xB3, 0xD0);
	drawer.SetCanvas(this->fieldViewer);
	drawer.FinalizeCallback = std::bind(&MainWindow::UpdateFromDrawer, this);
	prevViewer = this->fieldViewer;
	ui->weightSlider->setValue(50);
	weight_ = 0.5;

	// field editor
	connect(ui->riverButton,SIGNAL(clicked()),this,SLOT(ChangeElement()));
	connect(ui->grassButton,SIGNAL(clicked()),this,SLOT(ChangeElement()));
	connect(ui->populationButton,SIGNAL(clicked()),this,SLOT(ChangeElement()));
	connect(ui->guidanceButton,SIGNAL(clicked()),this,SLOT(ChangeElement()));
	connect(ui->groundButton,SIGNAL(clicked()),this,SLOT(ChangeElement()));

	connect(ui->actionLoadTerrain, SIGNAL(triggered()), this, SLOT(LoadTerrain()));
	connect(ui->actionSaveTerrain, SIGNAL(triggered()), this, SLOT(SaveTerrain()));

	connect(ui->weightSlider, SIGNAL(sliderMoved(int)), this, SLOT(SetValue(int)));
	connect(ui->weightSlider, SIGNAL(sliderReleased()), this, SLOT(UpdateWeight()));

	// road editor
	connect(ui->genRoadButton, SIGNAL(clicked()), this, SLOT(GenerateRoad()));
	auto dv = new QDoubleValidator(0.0, 1.0, 4);
	auto di = new QIntValidator();
	ui->spaceEdit->setValidator(dv);
	ui->spaceEdit->setText("0.05");
	ui->stepEdit->setValidator(di);
	ui->stepEdit->setText("2");
	ui->layerEdit->setValidator(di);
	ui->layerEdit->setText("3");

	ui->spaceEdit->setFocusPolicy(Qt::ClickFocus);
	ui->stepEdit->setFocusPolicy(Qt::ClickFocus);
	ui->layerEdit->setFocusPolicy(Qt::ClickFocus);
	connect(ui->spaceEdit, SIGNAL(returnPressed()), this, SLOT(Defocus()));
	connect(ui->stepEdit, SIGNAL(returnPressed()), this, SLOT(Defocus()));
	connect(ui->layerEdit, SIGNAL(returnPressed()), this, SLOT(Defocus()));
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

// field
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
		UpdateStatus("Info: Computing field...");
		cv::Mat direction = drawer.PrimitiveDirection();
		UpdateStatus("Info: Compute field done...");
		terrain.UpdateElementMask(this->fieldViewer->img_, direction, weight_);
		this->fieldViewer->SetImage(terrain.VisualizeField());
		this->roadViewer->SetImage(terrain.VisualizeElement());
	}
	else if (ui->tabWidget->currentIndex() == 1) {

	}
}

void MainWindow::SetValue(int value) {
	weight_ = 1.0f * exp(value / 15.0) / exp(50.0 / 15.0);
	ui->weightLabel->setText(QString("weight: %1").arg(weight_));
}

void MainWindow::UpdateWeight()
{
	cv::Mat mask = drawer.PrimitiveMask();
	terrain.UpdateCurrentWeight(mask, weight_);
	this->fieldViewer->SetImage(terrain.VisualizeField());
	this->roadViewer->SetImage(terrain.VisualizeElement());
}

void MainWindow::TabSelected()
{
	// nothing
}

void MainWindow::Defocus()
{
	ui->spaceEdit->clearFocus();
	ui->stepEdit->clearFocus();
	ui->layerEdit->clearFocus();
}


// road
void MainWindow::GenerateRoad()
{
	UpdateStatus("Info: Generate Road...\n");
	RoadGenerator rg;
	RoadGenerator::RoadGenParam param;
	param.minSpace = ui->spaceEdit->text().toFloat() * terrain.field.validMask_.cols;
	param.layer = 0;
	rg.GenerateRoad(param, terrain.field);

	int numLayers = ui->layerEdit->text().toInt();
	int numStep = ui->stepEdit->text().toInt();
	int span = 1;
	double width = 2.0f;
	std::vector<double> widths;
	widths.push_back(width);
	for (int i = 1; i < numLayers; ++i) {
		span *= numStep;
		printf("Upsample %d: %d\n", i, span);
		rg.UpsampleRoadGraph(i - 1, i, span);
		width *= 2.0f;
		widths.push_back(width);
	}

	rg.SetRoadWidth(widths);
	terrain.road = rg.road;
	UpdateStatus("Info: Road generation done...\n");

	this->roadViewer->SetImage(terrain.VisualizeElement());
	ui->tabWidget->setCurrentIndex(1);
}


// menu
void MainWindow::LoadTerrain()
{
	auto fileName = QFileDialog::getOpenFileName(this,
         tr("Open Terrain File "), QDir::currentPath(), tr("Terrain Files (*.trn)")).toStdString();
	if (fileName.empty())
		return;
	FILE* fp = fopen(fileName.c_str(), "rb");
	terrain.LoadFromFile(fp);
	fclose(fp);
	this->fieldViewer->SetImage(terrain.VisualizeField());
	this->roadViewer->SetImage(terrain.VisualizeElement());
}

void MainWindow::SaveTerrain()
{
	auto fileName = QFileDialog::getSaveFileName(this,
         tr("Open Terrain File "), QDir::currentPath(), tr("Terrain Files (*.trn)")).toStdString();
	if (fileName.empty())
		return;
	if (fileName.size() < 4 || fileName.substr(fileName.size() - 4) != ".trn")
		fileName += ".trn";
	FILE* fp = fopen(fileName.c_str(), "wb");
	terrain.SaveToFile(fp);
	fclose(fp);
}

void MainWindow::UpdateStatus(const char* info) {
	ui->infoLabel->setText(info);
	QApplication::processEvents();
}