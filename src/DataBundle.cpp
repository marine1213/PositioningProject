/*
 * DataBundle.cpp
 *
 *  Created on: Mar 3, 2018
 *      Author: gnouchnam
 */

#include "DataBundle.h"

DataBundle::DataBundle() {
	// TODO Auto-generated constructor stub

}

void DataBundle::clear(){
	finalResultCnts.clear();

	carGndTruthListPts.clear();
	boundingCarTruthList.clear();
	truckGndTruthListPts.clear();
	boundingtruckTruthList.clear();
}

const Mat& DataBundle::getPpMap() const {
	return ppMap;
}

void DataBundle::setPpMap(const Mat& ppMap) {
	this->ppMap = ppMap;
}

const Mat& DataBundle::getTransformMat() const {
	return transformMat;
}

const vector<vector<Point> >& DataBundle::getFinalResultCnts() const {
	return finalResultCnts;
}

void DataBundle::setFinalResultCnts(
		const vector<vector<Point> >& finalResultCnts) {
	this->finalResultCnts = finalResultCnts;
}

void DataBundle::setTransformMat(const Mat& transformMat) {
	this->transformMat = transformMat;
}

DataBundle::~DataBundle() {
	// TODO Auto-generated destructor stub
}

const vector<Rect>& CarInformation::getBoundingResultList() const {
	return boundingResultList;
}

void CarInformation::setBoundingResultList(const vector<Rect>& boundingResultList) {
	this->boundingResultList = boundingResultList;
}

const vector<Point2f>& CarInformation::getResultCenterListPts() const {
	return resultCenterListPts;
}

const vector<Rect>& DataBundle::getBoundingCarTruthList() const {
	return boundingCarTruthList;
}

void DataBundle::setBoundingCarTruthList(
		const vector<Rect>& boundingCarTruthList) {
	this->boundingCarTruthList = boundingCarTruthList;
}

const vector<Rect>& DataBundle::getBoundingtruckTruthList() const {
	return boundingtruckTruthList;
}

void DataBundle::setBoundingtruckTruthList(
		const vector<Rect>& boundingtruckTruthList) {
	this->boundingtruckTruthList = boundingtruckTruthList;
}

const vector<Point2f>& DataBundle::getCarGndTruthListPts() const {
	return carGndTruthListPts;
}

void DataBundle::setCarGndTruthListPts(
		const vector<Point2f>& carGndTruthListPts) {
	this->carGndTruthListPts = carGndTruthListPts;
}

const vector<Point2f>& DataBundle::getTruckGndTruthListPts() const {
	return truckGndTruthListPts;
}

void DataBundle::setTruckGndTruthListPts(
		const vector<Point2f>& truckGndTruthListPts) {
	this->truckGndTruthListPts = truckGndTruthListPts;
}

void CarInformation::setResultCenterListPts(
		const vector<Point2f>& resultCenterListPts) {
	this->resultCenterListPts = resultCenterListPts;
}

void BorderComponents::drawLines(Mat &output, vector<vector<Point2f> > contour, Scalar ptColor) {
	for (size_t cntId = 0; cntId < contour.size(); ++cntId) {
		for (size_t hullPtId = 0; hullPtId < contour[cntId].size(); ++hullPtId) {
			circle(output, contour[cntId][hullPtId],3, ptColor, -1);
			if(hullPtId < contour[cntId].size() - 1)
				line(output, contour[cntId][hullPtId], contour[cntId][hullPtId+1],Scalar(128,255,24),2);
		}
	}
}

void EndingPoints::drawPts(Mat &output, vector<vector<Point> > contours, vector<int> ptIds, Scalar ptColor){
	for (size_t hullPtId = 0; hullPtId < ptIds.size(); ++hullPtId) {
		circle(output, contours[hullPtId][ptIds[hullPtId]],3, ptColor, -1);
	}
}
