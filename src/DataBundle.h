/*
 * DataBundle.h
 *
 *  Created on: Mar 3, 2018
 *      Author: gnouchnam
 */

#ifndef SRC_DATABUNDLE_H_
#define SRC_DATABUNDLE_H_

#include "extOpencvHeader.h"
#include "extCommon.h"

class EndingPoints{
public:
	void drawPts(Mat &output, vector<vector<Point> > contours, vector<int> ptIds, Scalar ptColor = Scalar(0,255,0));

	vector<int> leftId, rightId, bottomId, topId;
};

class BorderComponents{
public:
	void drawLines(Mat &output, vector<vector<Point2f> > contours, Scalar ptColor = Scalar(0,255,0));

	vector<vector<Point2f> >  mainParts;
	vector<vector<Point2f> >  leftPart;
	vector<vector<Point2f> >  rightPart;
	vector<Vec4f> leftLines;
	vector<Vec4f> rightLines;
	vector<vector<Point2f> >	botLinePoint;
};


class CarInformation{
public:
	vector<Point2f> resultCenterListPts;
	vector<Rect> boundingResultList;
	vector<float> carVirtualHeightsX;
	vector<float> carVirtualHeightsY;
	vector<float> carVirtualWidths;
	vector<float> carWidths;
	vector<float> carLengths;
	vector<Point2f> shadowX, shadowY;	// on baseline to calculate height

	const vector<Rect>& getBoundingResultList() const;
	void setBoundingResultList(const vector<Rect>& boundingResultList);
	const vector<Point2f>& getResultCenterListPts() const;
	void setResultCenterListPts(const vector<Point2f>& resultCenterListPts);
};

class DataBundle {

public:

	//==== temporary matrices data ===
	Mat transformMat, ppMap;

	//==== Result of the final step for contour processing ====
	vector<vector<Point> > finalResultCnts;

	//==== Medium Components ====
//	BottomComponents bottomCompo;

	//==== Car Information =======


	//==== Result Evaluation =====
	vector<Point2f> carGndTruthListPts;
	vector<Rect> boundingCarTruthList;
	vector<Point2f> truckGndTruthListPts;
	vector<Rect> boundingtruckTruthList;

	void clear();

	DataBundle();
	virtual ~DataBundle();
	const Mat& getPpMap() const;
	void setPpMap(const Mat& ppMap);
	const Mat& getTransformMat() const;
	void setTransformMat(const Mat& transformMat);
	const vector<vector<Point> >& getFinalResultCnts() const;
	void setFinalResultCnts(const vector<vector<Point> >& finalResultCnts);

	const vector<Rect>& getBoundingCarTruthList() const;
	void setBoundingCarTruthList(const vector<Rect>& boundingCarTruthList);
	const vector<Rect>& getBoundingtruckTruthList() const;
	void setBoundingtruckTruthList(const vector<Rect>& boundingtruckTruthList);
	const vector<Point2f>& getCarGndTruthListPts() const;
	void setCarGndTruthListPts(const vector<Point2f>& carGndTruthListPts);
	const vector<Point2f>& getTruckGndTruthListPts() const;
	void setTruckGndTruthListPts(const vector<Point2f>& truckGndTruthListPts);
};

#endif /* SRC_DATABUNDLE_H_ */

