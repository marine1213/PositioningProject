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

	// perform the main function in this method
	// including: pre-process, cluster separation and occupied space estimation
	// result evaluation function:
	//       - car and truck tracking following ground truth
	//       - IOU and DEER calculation
	//       - draw and show the result
	// added function: preventing interference of height to the estimation result
	// next step: height estimation
	Mat mainProcessing(Mat &input, int frameId, Mat *transformMat, Mat * ppmap, Mat *binMask = NULL);
	// perform pre-process in this method
	// including: minor shadow removal and background subtraction
	Mat preProcessing(Mat &input,  Mat *binMask = NULL);

	// set up perspective map at the beginning
	void setupPPMap(Mat &input, Mat * binMask = NULL,Mat *transformMat = NULL, Mat *ppmap = NULL, string fileName = "");
	// read the MATLAB car ground truth file
	bool getCarGroundTruthFromDataFile(string fileName);
	// read the MATLAB truck ground truth file
	bool getTruckGroundTruthFromDataFile(string fileName);
	// draw the MATLAB car ground truth file
	void showCarGroundTruthData(Mat &img,size_t frameId);
	// draw the MATLAB truck ground truth file
	void showTruckGroundTruthData(Mat &img, size_t frameId);
	// return the number of detected cars on current frame
	int getNumberOfCars(){
		return numberOfCars;
	}

	// just to test - can be deleted
	void testDft(Mat &input, string name);
	// check if next point is on the border of selected ROI or not - can be deleted
	// purpose: remove the contour when one of its point touch the ROI
	bool checkPtOnROI(Point p);
private:
	// draw the ground truth after applied perspective map
	void showGroundTruthDataOnPPMap(Mat &img, vector<vector<Point2f> > &gndSet, vector<Point2f> &ptsList,vector<Rect> &boundingList, Scalar lineColor, Mat &transformMat);
	// read the ground truth data file and save the data to importedData
	bool getGroundTruthFromDataFile( string fileName, vector<vector<vector<Point2f> > > &importedData);
	// draw the ground truth data based on the point information and color
	void showGroundTruthData(Mat &img, vector<vector<Point2f> > &gndCarsSet, Scalar lineColor);
};




#endif /* SRC_IMGPROCESSING_H_ */
