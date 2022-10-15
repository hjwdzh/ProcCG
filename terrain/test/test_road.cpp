#include <QApplication>
#include "MainWindow.h"
#include "FlowField.h"
#include "RoadGenerator.h"

int main(int argc, char** argv)
{
	/*
	QApplication a(argc, argv);
	MainWindow window;
	window.show();
	*/

	printf("Initialize field...\n");
	FlowField ff;
	cv::Mat mask(2048, 2048, CV_8U);
	for (int i = 0; i < 2048; ++i) {
		for (int j = 0; j < 2048; ++j) {
			double dis = sqrt((i - 768) * (i - 768) / 4.0 + (j - 512) * (j - 512));
			if (dis < 256 || (j > 1000 && j < 2000 && i > 1800 && i < 1900)) {
				mask.at<unsigned char>(i, j) = 0;
			} else {
				mask.at<unsigned char>(i, j) = 255;
			}
		}
	}

	ff.SetValidMask(mask);

	printf("Optimize orientation...\n");
	ff.CreateOrientationField();

	RoadGenerator rg;
	RoadGenerator::RoadGenParam param;
	param.minSpace = 64;
	param.layer = 0;
	rg.GenerateRoad(param, ff);
	rg.UpsampleRoadGraph(0, 1, 2);
	rg.UpsampleRoadGraph(1, 2, 4);
	rg.SetRoadWidth({2, 3, 5});
	
	cv::Mat roadMask = rg.road.VisualizeRoadMap();
	cv::imwrite("road.png", roadMask);
	return 0;
}