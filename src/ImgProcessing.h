/*
 * ImgProcessing.h
 *
 *  Created on: Mar 9, 2017
 *      Author: gnouchnam
 */

#ifndef SRC_IMGPROCESSING_H_
#define SRC_IMGPROCESSING_H_

#include <fstream>
#include "extOpencvHeader.h"
#include "my_tools.h"
#include "opencv2/video/background_segm.hpp"

class ImgProcessing{

public:
	ofstream osfileForProcessingTime;

private:
	Point roiPts[4];
	Ptr< BackgroundSubtractorMOG2> pMOG2Ratio;
	Ptr< BackgroundSubtractorMOG2> pMOG2grayInput;
	Mat elm, elmMedium, elmSmall;
	Mat grayInput,accInput;
	double recoverValue;


	vector<vector<vector<Point2f> > >  gndTruthForCar;
	vector<vector<vector<Point2f> > >  gndTruthForTruck;
	vector<Point> roiContour;
	ofstream osfileForCar;
	ofstream osfileForTruck;

	//===== tracking =====
	vector<vector<Point> > savedCarCenterPoints;
	vector<vector<double> > savedCarIOUs;
	vector<vector<double> > savedCarErrorDistance;
	vector<double> averageCarIOUs;
	vector<double> averageCarErrorDistance;

	vector<vector<Point> > savedTruckCenterPoints;
	vector<vector<double> > savedTruckIOUs;
	vector<vector<double> > savedTruckErrorDistance;
	vector<double> averageTruckIOUs;
	vector<double> averageTruckErrorDistance;

	int numberOfCars;
public:

	ImgProcessing(){
		pMOG2grayInput	= createBackgroundSubtractorMOG2(1000,16,false);
		pMOG2Ratio 	= createBackgroundSubtractorMOG2(1000,16,false);
		elm			= getStructuringElement(CV_SHAPE_ELLIPSE, Size(15,15));
		elmMedium	= getStructuringElement(CV_SHAPE_ELLIPSE, Size(5,5));
		elmSmall	= getStructuringElement(CV_SHAPE_ELLIPSE, Size(3,3));
		recoverValue = -1;
		osfileForCar.open(OS_FOR_CAR, ofstream::binary);
		osfileForTruck.open(OS_FOR_TRUCK, ofstream::binary);
		osfileForProcessingTime.open(OS_PROCESSING_TIME, ofstream::binary);

		numberOfCars = 0;
	}

	~ImgProcessing(){
		osfileForCar.close();
		osfileForTruck.close();
		osfileForProcessingTime.close();
	}

	Mat mainProcessing(Mat &input, int frameId, Mat *transformMat, Mat * ppmap, Mat *binMask = NULL);
	Mat preProcessing(Mat &input,  Mat *binMask = NULL);
	void testDft(Mat &input, string name);

	void setupPPMap(Mat &input, Mat * binMask = NULL,Mat *transformMat = NULL, Mat *ppmap = NULL, string fileName = "");
	bool checkPtOnROI(Point p);
	bool getCarGroundTruthFromDataFile(string fileName);
	bool getTruckGroundTruthFromDataFile(string fileName);
	void showCarGroundTruthData(Mat &img,size_t frameId);
	void showTruckGroundTruthData(Mat &img, size_t frameId);

	int getNumberOfCars(){
		return numberOfCars;
	}
private:
	void showGroundTruthDataOnPPMap(Mat &img, vector<vector<Point2f> > &gndSet, vector<Point2f> &ptsList,vector<Rect> &boundingList, Scalar lineColor, Mat &transformMat);
	bool getGroundTruthFromDataFile( string fileName, vector<vector<vector<Point2f> > > &importedData);
	void showGroundTruthData(Mat &img, vector<vector<Point2f> > &gndCarsSet, Scalar lineColor);
};




#endif /* SRC_IMGPROCESSING_H_ */
