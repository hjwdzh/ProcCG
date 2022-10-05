#ifndef TERRAINN_MAINWINDOW_H_
#define TERRAINN_MAINWINDOW_H_

#include <QMainWindow>

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
public slots:
	void ChangeElement();
	void TabSelected();
	void SetValue(int value);
	void UpdateWeight();
	void LoadTerrain();
	void SaveTerrain();
};

#endif