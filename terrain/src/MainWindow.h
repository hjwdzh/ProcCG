#ifndef TERRAINN_MAINWINDOW_H_
#define TERRAINN_MAINWINDOW_H_

#include <QMainWindow>

#include "Drawer.h"
#include "ImageWidget.h"

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
	ImageWidget *riverViewer, *grassViewer, *prevViewer;
	Drawer drawer;
public slots:
	void TabSelected();
};

#endif