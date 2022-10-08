#ifndef TERRAINN_MAINWINDOW_H_
#define TERRAINN_MAINWINDOW_H_

#include <QMainWindow>
#include <QLineEdit>

#include "Drawer.h"
#include "ImageWidget.h"
#include "Terrain.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
	Q_OBJECT

public:
	explicit MainWindow(QWidget* parent = 0);
	~MainWindow();
	void keyPressEvent(QKeyEvent* event);
	void keyReleaseEvent(QKeyEvent* event);
	Ui::MainWindow* ui;
	ImageWidget *fieldViewer, *roadViewer, *prevViewer;
	Drawer drawer;
	Terrain terrain;
	double weight_;
	void UpdateFromDrawer();

	void UpdateStatus(const char* info);
public slots:
	// field
	void ChangeElement();
	void SetValue(int value);
	void UpdateWeight();
	// road
	void GenerateRoad();

	void TabSelected();
	void Defocus();

	// menu
	void LoadTerrain();
	void SaveTerrain();


};

#endif