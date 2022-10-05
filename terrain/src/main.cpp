#include <QApplication>
#include "MainWindow.h"
#include "FlowField.h"
#include "RoadGenerator.h"

int main(int argc, char** argv)
{
	QApplication a(argc, argv);
	MainWindow window;
	window.show();
	return a.exec();
}