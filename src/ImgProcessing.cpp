/*
 * ImgProcessing.cpp
 *
 *  Created on: Mar 9, 2017
 *      Author: gnouchnam
 */

#include "ImgProcessing.h"
#include "Line.h"

void carsSeparation(Mat cntsMat, vector<vector<Point> > *finalResultCnts, Mat *sepResult = NULL);
void analyzeData(vector<vector<Point> > &inputCnts, BorderComponents &bot, BorderComponents &top, Mat *cloneInput);
void showPointsOnMap(DataBundle &data, BorderComponents &botComp, BorderComponents &topComp, CarInformation &ci, Mat *ppMap, Scalar color=Scalar(128,255,24));
// remove not seen vehicles
void removeUncheckedVehicles(vector<vector<Point> > &centerPtList, vector<bool> &checkedList, vector<vector<double> > &iouList,vector<vector<double> > &deerList, ofstream &osFile, vector<double> &meanIouList, vector<double> &meanDeerList);
// track the position of vehicles
void trackVehicles(vector<vector<Point> > &lastCenterpts, vector<Point2f> &newCenterPts, vector<Rect> &boundingList,
		vector<vector<double> > &iouList, vector<vector<double> > &deerList,
		vector<bool> &checkedList, int resId, float iou, float deer );
// fit lines to the contours and draw them on the image
void fitLineToContour(vector<vector<Point2f> > &contours, vector<Vec4f> &outputLines, Mat *img);
void drawLines(vector<Vec4f> &lines, float alpha, Scalar color, Mat &img);
//void correctBottomLine(vector<vector<Point> > &hullCnts, EndingPoints &ep, BorderComponents &bottomPart, Mat *img);

Mat ImgProcessing::mainProcessing(DataBundle &data,Mat &input, int frameId, Mat *binMask) {
	Mat maskPreProc = preProcessing(input, binMask);

	// exit if the preprocessing fails
	if(maskPreProc.rows == 1 && maskPreProc.cols == 1)
		return maskPreProc;

	// input contour and output contour
	// idea is:
	//        - from input contour list, good contour is put into output contour
	//        - if the input contour has problem, it will create more than one intermediate contour
	//        - then, new added contour in output is removed, and two new contours are added.
	vector<vector<Point> > initCnts;
	data.clear();

	// matrix to draw the result
	Mat finalResultMat = input.clone();// =  Mat::zeros(input.size(), CV_8UC3);

	// debug the input
	Mat cloneInput = input.clone();

	// Find contours in moving object mask
 	findContours(maskPreProc.clone(), initCnts, noArray(), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

	for (size_t cntId = 0; cntId < initCnts.size(); cntId++) {
		Mat cntsMat =  Mat::zeros(input.size(), CV_8UC1);
		float area = contourArea(initCnts[cntId]);

		if(area<1000)	// 100 for noises and 10000 to remove small moving objects
			continue;


		data.finalResultCnts.push_back(initCnts[cntId]);
		// draw a mask
		drawContours(cntsMat, initCnts, cntId, Scalar(255), -1, 8, noArray());

		//============================== Cars Separation =========================

		morphologyEx(cntsMat, cntsMat, MORPH_CLOSE, elm);

//		carsSeparation(cntsMat, &(data.finalResultCnts), &finalResultMat);
	}

	//============================== Car model =========================
	Mat ppMapClone = data.getPpMap().clone();
	Mat ppMapTop = ppMapClone.clone();

	BorderComponents botParts, topParts;
	analyzeData(data.finalResultCnts, botParts, topParts, &cloneInput);

	//============== optical flow ===============

	//============== end of optical flow ===============
	CarInformation ci;
//	showPointsOnMap(data, topParts, ci, &ppMapTop, Scalar(234,110,230));
	showPointsOnMap(data, botParts, topParts, ci, &ppMapClone);

	//=========== Result Evaluation ===========

#if SHOW_GROUNDTRUTH_ON_MAP
	Mat *	drawGroundTruth = &ppMapClone;

	for (size_t i = 0; i < data.getFinalResultCnts().size(); ++i) {

		//--- ground truth data ---
		if(frameId > 1){
			// synchronize the input sequence with laser scanner data
			vector<vector<Point2f> > gndCarsSet = gndTruthForCar[(frameId - 2)/2];
			vector<vector<Point2f> > gndTrucksSet = gndTruthForTruck[(frameId - 2)/2];

			showGroundTruthDataOnPPMap(drawGroundTruth, gndCarsSet, data.carGndTruthListPts, data.boundingCarTruthList, Scalar(255,0,0),data.transformMat);
			showGroundTruthDataOnPPMap(drawGroundTruth, gndTrucksSet, data.truckGndTruthListPts, data.boundingtruckTruthList, Scalar(0,255,255),data.transformMat);
		}
	}
#endif
#if NO_ACCURACY_EVALUATION
	groundTruthComparing(data, ci, &ppMapClone);
#endif
	return ppMapClone;
}

void ImgProcessing::groundTruthComparing(DataBundle &data, CarInformation &ci, Mat *ppMapClone){

	/**
	 * the number of vehicles in last detection is saved in lastNumberofCenterPts
	 * In the next loop, the number of vehicles can be increased or decreased
	 */
	size_t lastNumberOfCarCenterPts = savedCarCenterPoints.size();
	size_t lastNumberOfTruckCenterPts = savedTruckCenterPoints.size();
	/**
	 * check if the vehicles in last detection is available in the next detection
	 * If not, it will be removed after this loop
	 */
	vector<bool> checkedCarIdList(lastNumberOfCarCenterPts);
	vector<bool> checkedTruckIdList(lastNumberOfTruckCenterPts);

	for (size_t resId = 0; resId < ci.getResultCenterListPts().size(); ++resId) {

		// if the vehicle center is not inside a contour, skip
		double isInsideROI = pointPolygonTest(roiContour, ci.getResultCenterListPts()[resId], false);
		if(isInsideROI<0)
			continue;

		bool isTruckInGndTruth = false;

		// mapping 2 contours - find minimum distance between 2 types of center points
		double minDistane = -1;
		int minId = -1;
		for (size_t gndId = 0; gndId < data.carGndTruthListPts.size(); ++gndId) {
			double dx = ci.getResultCenterListPts()[resId].x - data.carGndTruthListPts[gndId].x;
			double dy = ci.getResultCenterListPts()[resId].y - data.carGndTruthListPts[gndId].y;
			double thisDistane = sqrt(dx * dx + dy * dy);
			if (minDistane == -1 || minDistane > thisDistane) {
				minDistane = thisDistane;
				minId = gndId;
			}
		}

		if(minDistane >= 100){
			minDistane = -1;
			isTruckInGndTruth = true;
			for (size_t gndId = 0; gndId < data.truckGndTruthListPts.size(); ++gndId) {
				double dx = ci.resultCenterListPts[resId].x- data.truckGndTruthListPts[gndId].x;
				double dy = ci.resultCenterListPts[resId].y- data.truckGndTruthListPts[gndId].y;
				double thisDistane = sqrt(dx * dx + dy * dy);
				if (minDistane == -1 || minDistane > thisDistane) {
					minDistane = thisDistane;
					minId = gndId;
				}
			}
		}

		// Calculate IOU
		double iou, diagonalEccentricRatio;
		// only evaluate with nearest ground truth point and the distance is smaller than 100
		if (minId > -1 && minDistane < 100 && minDistane > -1) {
			Rect bTruthRect;
			Rect bResultRect = ci.boundingResultList[resId];
//			float carVirtualHeightX = ci.carVirtualHeightsX[resId];	// Delta x
//			float carVirtualHeightY = ci.carVirtualHeightsY[resId];	// Delta y
//			float carWidth = ci.carWidths[resId];
//			float carLength = ci.carLengths[resId];
//			Point2f shadowBaseX = ci.shadowX[resId];
//			Point2f shadowBaseY = ci.shadowY[resId];

			// formula
//			float carHeightX = 1/(0.2 + ((shadowBaseX.x) + carWidth)/(carVirtualHeightX - carWidth));
//			float carHeightY = 1/(0.2 + ((shadowBaseY.y) + carLength)/(carVirtualHeightY - carWidth));
//			float carHeight = sqrt(carHeightX*carHeightX + carHeightY*carHeightY);

			if(isTruckInGndTruth)
				bTruthRect = data.boundingtruckTruthList[minId];
			else
				bTruthRect = data.boundingCarTruthList[minId];

			Rect intersectionRect = bTruthRect & bResultRect;
			double intersectArea = intersectionRect.area();
			iou = intersectArea / (bResultRect.area() + bTruthRect.area() - intersectArea);

			double diagonalLine = sqrt(bTruthRect.width*bTruthRect.width + bTruthRect.height+bTruthRect.height);
			diagonalEccentricRatio = minDistane / diagonalLine;
//============================Tracking part===============================
			if(!isTruckInGndTruth){
				trackVehicles(savedCarCenterPoints, ci.resultCenterListPts, ci.boundingResultList, savedCarIOUs, savedCarErrorDistance,
						checkedCarIdList, resId, iou, diagonalEccentricRatio);
			}else{
				trackVehicles(savedTruckCenterPoints, ci.resultCenterListPts, ci.boundingResultList, savedTruckIOUs, savedTruckErrorDistance,
						checkedTruckIdList, resId, iou, diagonalEccentricRatio);
			}
//============================End of Tracking part===============================

			ostringstream oss2;
			oss2 << (int)(iou*100);	// in percent
//			oss2 << (carHeight);
			//Todo problem is here
			if(ppMapClone){
				if(isTruckInGndTruth)
						putText(*ppMapClone, oss2.str(), Point(data.truckGndTruthListPts[minId].x, data.truckGndTruthListPts[minId].y - 10), FONT_HERSHEY_COMPLEX,0.7, Scalar(0, 0, 255), 1, 8);
					else
						putText(*ppMapClone, oss2.str(), Point(data.carGndTruthListPts[minId].x, data.carGndTruthListPts[minId].y - 10), FONT_HERSHEY_COMPLEX,0.7, Scalar(0, 0, 255), 1, 8);
				circle(*ppMapClone, ci.resultCenterListPts[resId], 2, Scalar(96, 255, 96), 2);
			}
		}

	}

	//======== remove not checked center points ============
	removeUncheckedVehicles(savedCarCenterPoints, checkedCarIdList, savedCarIOUs, savedCarErrorDistance, osfileForCar, averageCarIOUs, averageCarIOUs);
	removeUncheckedVehicles(savedTruckCenterPoints, checkedTruckIdList, savedTruckIOUs, savedTruckErrorDistance, osfileForTruck, averageTruckIOUs, averageTruckIOUs);

	//============================================================
	ostringstream oss;
	oss << "No. Vehicles: " << savedCarIOUs.size() + savedTruckIOUs.size();
	if(ppMapClone){
		putText(*ppMapClone, oss.str(), Point(30, 90), FONT_HERSHEY_COMPLEX,  0.8, Scalar(255,255,255),2);
	}

	numberOfCars = savedCarIOUs.size();

}

void drawLines(vector<Vec4f> &lines, float alpha, Scalar color, Mat &img){
	for (int cntId = 0; cntId < (int)lines.size(); ++cntId) {
		if(!(lines.empty())){
			Vec4f aLine = lines[cntId];
			Point p1 = Point(aLine[2]-alpha*aLine[0],aLine[3]-alpha*aLine[1]);
			Point p2 = Point(aLine[2]+alpha*aLine[0],aLine[3]+alpha*aLine[1]);
			line(img, p1, p2,color,4);
		}
	}
}

void fitLineToContour(vector<vector<Point2f> > &contours, vector<Vec4f> &outputLines, Mat *img){
	vector<Vec4f> lines(contours.size());
	for (size_t cntId = 0; cntId < contours.size(); ++cntId) {
		if (!(contours[cntId].empty())) {
			fitLine(contours[cntId], lines[cntId], CV_DIST_L2, 0, 10,0.01);
		}
	}
	outputLines = lines;
	if(img)
		drawLines(lines, 50, Scalar(128,255,209), *img);
}

// join two lines to create the point outside border
void generatePointsFromBorderLines(vector<vector<Point> > &hullCnts, EndingPoints &ep, BorderComponents &bot, Mat *img){
	for (size_t cntId = 0; cntId < bot.leftLines.size(); ++cntId) {
		vector<Point2f> pointList;

		Vec4f mv4f = bot.leftLines[cntId];
		Point2f p1 = Point2f(mv4f[2] + 1000 * mv4f[0], mv4f[3] + 1000 * mv4f[1]);
		Point2f p2 = Point2f(mv4f[2] - 1000 * mv4f[0], mv4f[3] - 1000 * mv4f[1]);
		Line leftLine = Line(p1.x, p1.y, p2.x, p2.y );
		Vec4f mv4f2 = bot.rightLines[cntId];
		Point2f p1r = Point2f(mv4f2[2] + 1000 * mv4f2[0], mv4f2[3] + 1000 * mv4f2[1]);
		Point2f p2r = Point2f(mv4f2[2] - 1000 * mv4f2[0], mv4f2[3] - 1000 * mv4f2[1]);
		Line rightLine = Line(p1r.x, p1r.y, p2r.x, p2r.y );
		float bottomPoint[2];
		rightLine.cross(leftLine, bottomPoint);

		// evaluate ending left point and right point
		Point endingLeftPt = hullCnts[cntId][ep.leftId[cntId]];
//		float leftPtDist = leftLine.distance(endingLeftPt.x, endingLeftPt.y);
		Point endingRightPt = hullCnts[cntId][ep.rightId[cntId]];
//		float rightPtDist = rightLine.distance(endingRightPt.x, endingRightPt.y);

		Line leftRight = Line(endingLeftPt.x, endingLeftPt.y, endingRightPt.x, endingRightPt.y);
		int bottomDirection = leftRight.distance(bottomPoint[0],bottomPoint[1],true);

		bool addedLeft = false;
		// save the points - first, left point
		for (size_t ptId = 0; ptId < hullCnts[cntId].size(); ++ptId) {

			//check direction to bottom
			Point tempPt = hullCnts[cntId][ep.leftId[cntId] + ptId];
			int tempDirection = leftRight.distance(tempPt.x, tempPt.y, true);
			int sign = tempDirection * bottomDirection > 0 ? 1 : -1;

			// estimage point based on distance
			int rectifiedId = (ep.leftId[cntId] + sign * ptId) % hullCnts[cntId].size();
			Point pt = hullCnts[cntId][rectifiedId];
			float distance = leftLine.distance(pt.x,pt.y);

			if(distance < 7){
				pointList.push_back(leftLine.shadow(pt.x,pt.y));
				addedLeft = true;
				break;
			}
		}
		if(!addedLeft){
			pointList.push_back(leftLine.shadow(endingLeftPt.x, endingLeftPt.y));
		}

		bool addedRight = false;

		// save the points - second, the bottom point
		pointList.push_back(Point2f(bottomPoint[0], bottomPoint[1]));

		// save the points - third, the right point
		for (size_t ptId = 0; ptId < hullCnts[cntId].size(); ++ptId) {

			//check direction to bottom
			Point tempPt = hullCnts[cntId][ep.rightId[cntId] + ptId];
			int tempDirection = leftRight.distance(tempPt.x, tempPt.y, true);
			int sign = tempDirection * bottomDirection > 0 ? 1 : -1;

			// estimage point based on distance
			int rectifiedId = (ep.rightId[cntId] + sign * ptId) % hullCnts[cntId].size();
			Point pt = hullCnts[cntId][rectifiedId];
			float distance = rightLine.distance(pt.x,pt.y);

			if(distance < 3){
				pointList.push_back(rightLine.shadow(pt.x,pt.y));
				addedRight = true;
				break;
			}
		}
		if(!addedRight){
			pointList.push_back(rightLine.shadow(endingRightPt.x,endingRightPt.y));
		}


		if(img){ // For debugging
//			line(*img, p1, p2, Scalar(234,31,35),2);
//			line(*img, p1r, p2r, Scalar(234,31,35),2);
			// show estimated vertices of cars
			for (size_t ptId = 0; ptId < pointList.size(); ++ptId) {
				circle(*img, pointList[ptId], 2, Scalar(0,0,255),2);
			}
		}
		bot.botLinePoint.push_back(pointList);
	}
}


void removeUncheckedVehicles(vector<vector<Point> > &centerPtList, vector<bool> &checkedList, vector<vector<double> > &iouList,vector<vector<double> > &deerList, ofstream &osFile, vector<double> &meanIouList, vector<double> &meanDeerList){
	int erasingTimes = 0;
	for (size_t centerPtId = 0; centerPtId < checkedList.size(); ++centerPtId) {
		// browse the list if the car is checked or not
		if(checkedList[centerPtId] == false){
			// check if the number of saved IOU is larger than 5
			// if not, it means the sudden object appeared for a short time
			// and is considered to be spike noise
			if(!iouList.empty() && iouList[centerPtId].size() > 5){
				double sumIOU = 0, sumErrorDistance = 0;
				osFile<<iouList[centerPtId].size()<<'\t';
				for (size_t iouId = 0; iouId < iouList[centerPtId].size(); ++iouId) {
					sumIOU += iouList[centerPtId][iouId];
					sumErrorDistance += deerList[centerPtId][iouId];
				}
				// if the car is not seen anymore its data is calculated and saved to respect file
				double averageIOUval = sumIOU/iouList[centerPtId].size();
				double averageErrorDist = sumErrorDistance/deerList[centerPtId].size();
				meanIouList.push_back(averageIOUval);
				meanDeerList.push_back(averageErrorDist);
				osFile<< averageIOUval<<'\t'<<averageErrorDist<<endl;
			}

			// remove saved coordinates
			centerPtList.erase(centerPtList.begin() + centerPtId - erasingTimes);
			// remove saved IOU
			if(!iouList.empty())
				iouList.erase(iouList.begin() + centerPtId - erasingTimes);

			erasingTimes++;
		}
	}
}

void trackVehicles(vector<vector<Point> > &lastCenterpts, vector<Point2f> &newCenterPts, vector<Rect> &boundingList,
		vector<vector<double> > &iouList, vector<vector<double> > &deerList,
		vector<bool> &checkedList, int resId, float iou, float deer ){
	// check if the center points belong to saved cars or not
	bool isSavedBefore =false;
	size_t savedId;
	for (savedId = 0; savedId < checkedList.size(); ++savedId) {
		Point lastSavedPt = lastCenterpts[savedId].back();
		Rect bResultRect = boundingList[resId];
		bool rectXbool = bResultRect.x < lastSavedPt.x && lastSavedPt.x < bResultRect.x + bResultRect.width;
		bool rectYbool = bResultRect.y < lastSavedPt.y && lastSavedPt.y < bResultRect.y + bResultRect.height;
		if(rectXbool && rectYbool){
			isSavedBefore = true;
			lastCenterpts[savedId].push_back(newCenterPts[resId]);
			checkedList[savedId] = true;
			break;
		}
	}

	// the car is not saved before, add new car
	if (!isSavedBefore) {
		vector<Point> newCarPoints;
		newCarPoints.push_back(newCenterPts[resId]);
		lastCenterpts.push_back(newCarPoints);

		vector<double> newCarIOUs;
		iouList.push_back(newCarIOUs);
		vector<double> newErrorDistance;
		deerList.push_back(newErrorDistance);
	}
	if(savedId < iouList.size()){
		iouList[savedId].push_back(iou);
		deerList[savedId].push_back(deer);
	}
}

void showPointsOnMap(DataBundle &data, BorderComponents &botComp, BorderComponents &topComp, CarInformation &ci, Mat *ppMapClone, Scalar color){

	vector<vector<Point2f> > ptsBotSet = botComp.botLinePoint;
	vector<vector<Point2f> > ptsTopSet = topComp.mainParts;

	//====show map and point===
	vector<Point2f> centerPtsInMap;
	for (size_t linesId = 0; linesId < ptsBotSet.size(); ++linesId) {
		vector<Point2f> botPerspect(ptsBotSet[linesId].size());
		vector<Point2f> topPerspect(ptsTopSet[linesId].size());

		if(botPerspect.size() == 0 || topPerspect.size() == 0)
			continue;

		perspectiveTransform(ptsBotSet[linesId], botPerspect, data.transformMat);
		perspectiveTransform(ptsTopSet[linesId], topPerspect, data.transformMat);

		// find center points
		Point2f firstBtpt = botPerspect.back();
		Point2f secondBtPt = botPerspect[1];
		Point2f lastBtPt = botPerspect.front();
		Point2f centerPt((firstBtpt.x + lastBtPt.x)/2,(firstBtpt.y + lastBtPt.y)/2);

		// saved center point
		if(ppMapClone){
			circle(*ppMapClone, centerPt,3, Scalar(0,255,0), -1);
		}

		centerPtsInMap.push_back(centerPt);
		// find symmetric points
		size_t sizeOfbottomPts = botPerspect.size();
		for (size_t hullPtId = 1; hullPtId < sizeOfbottomPts;++hullPtId) {
			Point2f edgePt = botPerspect[hullPtId];
			Point2f nextPt(2*centerPt.x - edgePt.x, 2*centerPt.y - edgePt.y);
			botPerspect.push_back(nextPt);
		}

		//======= Height Estimating ========


//		float d1 = (secondBtPt.x - firstBtpt.x)*(secondBtPt.x - firstBtpt.x) + (secondBtPt.y - firstBtpt.y)*(secondBtPt.y - firstBtpt.y);
//		float d2 = (secondBtPt.x - lastBtPt.x)*(secondBtPt.x - lastBtPt.x) + (secondBtPt.y - lastBtPt.y)*(secondBtPt.y - lastBtPt.y);
//
//		// Find long edges
//		Line l1 = Line(secondBtPt.x, secondBtPt.y, firstBtpt.x, firstBtpt.y);
//		Line l2 = Line(secondBtPt.x, secondBtPt.y, lastBtPt.x, lastBtPt.y);
//		Line baseLengthLine = d2 > d1? l2 : l1;
//		Line baseWidthLine = d2 > d1? l1 : l2;
//		float width, length;
//		if(d2 > d1){
//			width = sqrt(d1);
//			length = sqrt(d2);
//		}else{
//			width = sqrt(d2);
//			length = sqrt(d1);
//		}
//
//
////		float botDist = baseLine.distance(secondBtPt.x, secondBtPt.y, true);
//		float maxDist = -1;
//		int maxDistId = -1;
//		// Calculate the distances
//		for (size_t ptId = 0; ptId < topPerspect.size(); ++ptId) {
//			float topDist = baseLengthLine.distance(topPerspect[ptId].x, topPerspect[ptId].y, true);
//			float absTopDist = abs(topDist);
//			if (maxDist == -1 || maxDist < absTopDist) {
//				maxDist = absTopDist;
//				maxDistId = ptId;
//			}
//		}
//
//		Point2f topPt = topPerspect[maxDistId];
//		Point2f ptOnBaseLength = baseLengthLine.shadow(topPt.x, topPt.y);
//		Point2f ptOnBaseWidth = baseWidthLine.shadow(topPt.x, topPt.y);
//		float virtualHeightY = sqrt((topPt.x - ptOnBaseWidth.x)*(topPt.x - ptOnBaseWidth.x) + (topPt.y - ptOnBaseWidth.y)*(topPt.y - ptOnBaseWidth.y));
//		circle(*ppMapClone, topPerspect[maxDistId],3, Scalar(255,0,238), -1);
	// Stablize the result - no idea


		// save the data
		//		ci.carVirtualHeightsX.push_back(maxDist);
		//		ci.carVirtualHeightsY.push_back(virtualHeightY);
		//		ci.carWidths.push_back(width);
		//		ci.carLengths.push_back(length);
		//		ci.shadowX.push_back(ptOnBaseLength);
		//		ci.shadowY.push_back(ptOnBaseWidth);

		//==================================
//			mappedCnts.push_back(outputPerspect);
		// FOR TRACKING
		ci.resultCenterListPts.push_back(centerPt);
		Rect bRect = boundingRect(botPerspect);
		ci.boundingResultList.push_back(bRect);


		// show mapped points
		if(ppMapClone){
			for (size_t ppPtId = 0; ppPtId < botPerspect.size() - 1;++ppPtId) {
				line(*ppMapClone, botPerspect[ppPtId], botPerspect[(ppPtId+1)%botPerspect.size()],color,2);
				circle(*ppMapClone, botPerspect[ppPtId],3, Scalar(0,0,255), -1);
			}
//			for (size_t ppPtId = 0; ppPtId < topPerspect.size() - 1;++ppPtId) {
//				line(*ppMapClone, topPerspect[ppPtId], topPerspect[(ppPtId+1)%topPerspect.size()],color,2);
//			}
		}

	}
}

void findEndingPts(vector<vector<Point> > &hullCnts, EndingPoints &ep){
	for (size_t cntId = 0; cntId < hullCnts.size(); ++cntId) {
		int xleft, xright,ybottom, ytop;
		ep.leftId.push_back(-1);
		ep.rightId.push_back(-1);
		ep.bottomId.push_back(-1);
		ep.topId.push_back(-1);
		for (size_t hullPtId = 0; hullPtId < hullCnts[cntId].size(); ++hullPtId) {
			if(ep.leftId.back() == -1 || xleft > hullCnts[cntId][hullPtId].x){
				ep.leftId[cntId] = hullPtId;
				xleft = hullCnts[cntId][hullPtId].x;
			}
			if (ep.rightId.back() == -1|| xright < hullCnts[cntId][hullPtId].x) {
				ep.rightId[cntId]= hullPtId;
				xright = hullCnts[cntId][hullPtId].x;
			}
			if (ep.bottomId.back() == -1|| ybottom < hullCnts[cntId][hullPtId].y) {
				ep.bottomId[cntId]= hullPtId;
				ybottom = hullCnts[cntId][hullPtId].y;
			}
			if (ep.topId.back() == -1|| ytop > hullCnts[cntId][hullPtId].y) {
				ep.topId[cntId]= hullPtId;
				ytop = hullCnts[cntId][hullPtId].y;
			}
		}
		if(ep.rightId.back() == -1)
			ep.rightId.pop_back();
		if(ep.leftId.back() == -1)
			ep.leftId.pop_back();
		if(ep.bottomId.back() == -1)
			ep.bottomId.pop_back();
		if(ep.topId.back() == -1)
			ep.topId.pop_back();
	}
}

void extractLines(vector<vector<Point> > &hullCnts, EndingPoints &ep, BorderComponents &bot, BorderComponents &top, int cutBL = 0, int cutBR = 0, int cutTL = 0, int cutTR = 0){
	for (size_t cntId = 0; cntId < ep.leftId.size(); ++cntId) {
//			int nearep.LeftId = -1;
		Point ptLeft = hullCnts[cntId][ep.leftId[cntId]];
		Point ptRight = hullCnts[cntId][ep.rightId[cntId]];
		Point ptBot = hullCnts[cntId][ep.bottomId[cntId]];
		Point ptTop = hullCnts[cntId][ep.topId[cntId]];

		//======bottom point versus diagonal line=======
		Line diagLine = Line(ptLeft.x,ptLeft.y,ptRight.x,ptRight.y);
		int ptBtDirection = diagLine.distance(ptBot.x,ptBot.y, true);
		//======bottom point versus left-bottom line=======
		Line topBotLine = Line(ptTop.x, ptTop.y, ptBot.x, ptBot.y);
		int leftRightDir = topBotLine.distance(ptRight.x,ptRight.y, true);
		//======get points=======
		vector<Point2f> botPts, botRightPts, botLeftPts;
		vector<Point2f> topPts, topRightPts, topLeftPts;


		if(cutBR == 0){
			botPts.push_back(ptRight);
			botRightPts.push_back(ptRight);
		}

		for (int hullPtId = 0; hullPtId < (int)(hullCnts[cntId].size()); ++hullPtId) {
			Point ptTemp = hullCnts[cntId][hullPtId];
			int ptTempBotDirection = diagLine.distance(ptTemp.x,ptTemp.y,true);
			float tempDirection = ptBtDirection*ptTempBotDirection;
			if(tempDirection > 0){	// same direction with bottom point
				botPts.push_back(ptTemp);
				int ptTempRightDirection = topBotLine.distance(ptTemp.x, ptTemp.y,true);
				if (leftRightDir * ptTempRightDirection <= 0) {
					botLeftPts.push_back(ptTemp);
				} else {
					botRightPts.push_back(ptTemp);
				}
			}
			if(tempDirection < 0){
				topPts.push_back(ptTemp);
				int ptTempRightDirection = topBotLine.distance(ptTemp.x, ptTemp.y,true);
				if (leftRightDir * ptTempRightDirection <= 0) {
					topLeftPts.push_back(ptTemp);
				} else {
					topRightPts.push_back(ptTemp);
				}
			}
		}


		if(cutBL == 0){
			botPts.push_back(ptLeft);
			botLeftPts.push_back(ptLeft);
		}

		if(cutBL > 0 && botLeftPts.size() > 10){
			int removedSize = (int)(botLeftPts.size() * cutBL/100.0);
			for (int ptId = 0; ptId < removedSize; ++ptId) {
				botLeftPts.pop_back();
			}
		}

		if(cutBR > 0 && botRightPts.size() > 10){
			int removedSize = (int)(botRightPts.size() * cutBR/100.0);
			botRightPts.erase(botRightPts.begin(), botRightPts.begin() + removedSize);
		}

		if(cutTR > 0 && botRightPts.size() > 10){
			int removedSize = (int)(topRightPts.size() * cutTR/100.0);
			for (int ptId = 0; ptId < removedSize; ++ptId) {
				topRightPts.pop_back();
				topPts.pop_back();
			}
		}

		if(cutTL > 0 && botLeftPts.size() > 10){
			int removedSize = (int)(topLeftPts.size() * cutTL/100.0);
			topLeftPts.erase(topLeftPts.begin(), topLeftPts.begin() + removedSize);
			topPts.erase(topPts.begin(), topPts.begin() + removedSize);
		}

		bot.mainParts.push_back(botPts);
		bot.leftPart.push_back(botLeftPts);
		bot.rightPart.push_back(botRightPts);

		top.mainParts.push_back(topPts);
		top.leftPart.push_back(topLeftPts);
		top.rightPart.push_back(topRightPts);
	}
}

void analyzeData(vector<vector<Point> > &inputCnts, BorderComponents &bot, BorderComponents &top,Mat *cloneInput){

	vector<vector<Point> > hullCnts(inputCnts.size());
	vector<vector<Point2f> > bottomLines, leftLines, rightLines;

	for (size_t cntId = 0; cntId < hullCnts.size(); ++cntId) {
		convexHull(inputCnts[cntId], hullCnts[cntId]);
	}

	//==============first method bottom line extraction===============

	//===== find ending left and ending right separating contour into 2 parts ===
	EndingPoints ep;
	findEndingPts(hullCnts, ep);

	//==== extract only bottom line from hull contour =====
	extractLines(hullCnts, ep, bot, top, 15, 15, 0 ,0);

	vector<vector<Point2f> > topLayerPts, edgeLines;//, innerLines;
	vector<vector<float> > shownDebugData;

	fitLineToContour(bot.leftPart, bot.leftLines, cloneInput);
	fitLineToContour(bot.rightPart, bot.rightLines, cloneInput);

	generatePointsFromBorderLines(hullCnts, ep, bot, cloneInput);
//	correctBottomLine(hullCnts, ep,bot, cloneInput);
//	estimateHiddenLine(hullCnts, ep,bot, cloneInput);

	//====show marked points======
	if(cloneInput){
		Scalar ptColor = Scalar(0,255,0); //Scalar(255,180,160);
	//	Scalar ptLeft = Scalar(255,255,0);
		Scalar ptBottom = Scalar(0,0,255);
		Scalar ptRight = Scalar(255,0,255);
		Scalar ptTop = Scalar(0,255,255);

//		bot.drawLines(*cloneInput, bot.correctedMainPart, ptColor);

		ep.drawPts(*cloneInput, hullCnts, ep.leftId, ptColor);
		ep.drawPts(*cloneInput, hullCnts, ep.rightId, ptRight);
		ep.drawPts(*cloneInput, hullCnts, ep.topId, ptBottom);
		ep.drawPts(*cloneInput, hullCnts, ep.bottomId, ptTop);
	}
		imshow("hull pts Input clone", *cloneInput);


}

void carsSeparation(Mat cntsMat, vector<vector<Point> > *finalResultCnts, Mat *sepResult){

	//================= startOfSep ============ Cars separation ============================
	// distance transform to separate 2 cars
	Mat distanceImage = Mat::zeros(cntsMat.size(), CV_8UC1);
	distanceTransform(cntsMat, distanceImage, DIST_L2, 5);
	normalize(distanceImage, distanceImage, 0,255, NORM_MINMAX );

	// threshold distance transform to apply separation
	double maxNormDistance;
	minMaxLoc(distanceImage, 0, &maxNormDistance);
	threshold(distanceImage, distanceImage, 0.55 * maxNormDistance,255,THRESH_TOZERO);

	Mat distImg8UC1;
	distanceImage.convertTo(distImg8UC1, CV_8UC1, 255,0);

	// find markers for watershed by contours
	vector<vector<Point> > movCntsDist;
	findContours(distImg8UC1.clone(), movCntsDist, noArray(), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

	vector<vector<Point> > hullDistance(movCntsDist.size());
	vector<vector<int> > hullDistanceId(movCntsDist.size());
	vector<vector<Vec4i> > defects(movCntsDist.size());
	for (size_t i = 0; i < movCntsDist.size(); i++) {
		convexHull(movCntsDist[i], hullDistance[i], false);
		convexHull(movCntsDist[i], hullDistanceId[i], false);
		if(hullDistanceId[i].size() > 3)
			convexityDefects(movCntsDist[i],hullDistanceId[i], defects[i]);
	}

	Point separatingPts[2];
	bool isSeparatingLineExist = false;
	/// Draw contours + hull results
	size_t len = movCntsDist.size();
	for (size_t i = 0; i < len; i++) {

		cv::Point2f posOld, posOlder;
		cv::Point2f f1stDerivative, f2ndDerivative;

		vector<int> defectIdList;

		for (size_t j = 0; j < defects[i].size(); j++)
	    {
	    	Vec4i& v = defects[i][j];

	    	// check if farId is between startId and endId or not
	    	int startEndDist = (int)(len + (v[1] - v[0])) % len;
	    	int startFarDist = (int)(len + (v[2] - v[0])) % len;

	    	if(startEndDist < startFarDist)
	    		// the farId is outside of start - end Id
	    		continue;
            int faridx = v[2]; Point ptFar(movCntsDist[i][faridx]);
            //ignore the big size vehicle ( bus )
			double directTest = pointPolygonTest(hullDistance[i], movCntsDist[i][faridx],false);

            // depth is approximately equal distance with point polygonTest
            // but may include the points that are outside of hull -- #stupid
	        float depth = v[3] / 256.0 * directTest;
	        if (depth > 4) //  filter defects by depth, e.g more than 10 - 4 is recommended
	        {
	        	defectIdList.push_back(j);
	        }

	    }
		RotatedRect rect = minAreaRect(movCntsDist[i]);

		if (defectIdList.size() > 2){
			// if there are 3 points or more in defect point list
			// find 2 nearest point to the center of this mov
			double min1stDistVal, min2ndDistVal;
			int min1stDistId = -1, min2ndDistId = -1;
			for (size_t defectId = 0; defectId < defectIdList.size(); ++defectId) {
				int faridx = defects[i][defectIdList[defectId]][2];
				Point ptFar(movCntsDist[i][faridx]);
				double d = sqrt(pow(ptFar.x - rect.center.x , 2) + pow(ptFar.y - rect.center.y , 2));
				if(min1stDistId == -1 || min1stDistVal > d){
					min1stDistVal = d;
					min1stDistId = defectId;
				}else if(min2ndDistId == -1 || min2ndDistVal > d){
					min2ndDistVal = d;
					min2ndDistId = defectId;
				}
			}
			// found two nearest to center points
			int faridx0 = defects[i][defectIdList[min1stDistId]][2];
			separatingPts[0] = movCntsDist[i][faridx0];
			int faridx1 = defects[i][defectIdList[min2ndDistId]][2];
			separatingPts[1] = movCntsDist[i][faridx1];

		}else if (defectIdList.size() == 2){

			int faridx0 = defects[i][defectIdList[0]][2];
			separatingPts[0] = movCntsDist[i][faridx0];
			int faridx1 = defects[i][defectIdList[1]][2];
			separatingPts[1] = movCntsDist[i][faridx1];
		}
		if(defectIdList.size() > 1){
			isSeparatingLineExist = true;
		}
	}

	// draw separating line and find the distance fragment again
	if(isSeparatingLineExist){
		movCntsDist.clear();
//			imshow("before separating points applied",distImg8UC1);
		line(distImg8UC1, separatingPts[0], separatingPts[1], Scalar(0),3,8);
//			imshow("after separating points applied",distImg8UC1);
		findContours(distImg8UC1, movCntsDist, noArray(), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
//			waitKey(0);
	}

	// number of contour is number of separated cars
	if(movCntsDist.size() > 1){
		if(finalResultCnts)
			(*finalResultCnts).pop_back();

		Mat markers = Mat::zeros(cntsMat.rows, cntsMat.cols, CV_32SC1);

		// draw area of markers
		for (size_t cntIdDist = 0; cntIdDist < movCntsDist.size();cntIdDist++) {
			drawContours(markers, movCntsDist, static_cast<int>(cntIdDist),Scalar::all(static_cast<int>(cntIdDist) + 1), -1);
		}
		circle(markers, Point(5, 5), 3, CV_RGB(255, 255, 255), -1);

		Mat img8UC3;
		cvtColor(cntsMat, img8UC3, CV_GRAY2BGR);
		watershed(img8UC3, markers);

		Mat mark = Mat::zeros(markers.size(), CV_8UC1);
		markers.convertTo(mark, CV_8UC1);
		bitwise_not(mark, mark);
//			imshow("markers", mark);

		//======================== Separation result analyzing ============================

		for (size_t markerId = 1; markerId <= movCntsDist.size(); ++markerId) {
			Mat markerExtractedMat = Mat::zeros(cntsMat.size(), CV_8UC1);

			for (int i = 0; i < markers.rows; i++) {
				for (int j = markers.cols - 1; j >= 0; j--) {
					size_t index = markers.at<int>(i, j);
					if (index == markerId){
						markerExtractedMat.at<char>(i, j) = 255;
					}
					else
						markerExtractedMat.at<char>(i, j) = 0;
				}
			}

			vector<vector<Point> > separatedCnts;
			findContours(markerExtractedMat.clone(), separatedCnts, noArray(), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

			for (size_t sepCntId = 0; sepCntId < separatedCnts.size(); ++sepCntId) {
				if(finalResultCnts)
					(*finalResultCnts).push_back(separatedCnts[sepCntId]);
				if(sepResult)
					drawContours(*sepResult,separatedCnts,sepCntId, Scalar(255,196,0),2);
			}
		}
	}
#if NO_SUPPORT_PICTURES
	if(sepResult)
		imshow("sepResult", *sepResult);
#endif
//=========== endofSep ===========

}

Mat ImgProcessing::preProcessing(Mat &input, Mat *binMask){
	Mat outRatio0p5;
	Mat outRatio1;

	// gray conversion
	cvtColor(input, grayInput, COLOR_BGR2GRAY);



	if(binMask)
		bitwise_and(grayInput,*binMask,grayInput);

	if(accInput.data == NULL){
		accInput = grayInput.clone();
		accInput0p5 = accInput*0.5;
		recoverValue = mean(grayInput).val[0];
	}

	Mat ratioImg0p5 = abs(grayInput / accInput0p5);
	ratioImg0p5 = ratioImg0p5 * (int)recoverValue;
	Mat ratioImg1 = abs(grayInput / accInput);
	ratioImg1 = ratioImg1 * (int)recoverValue*3;

#if NO_SUPPORT_PICTURES
	imshow("ratio image0p5",ratioImg0p5);
	imshow("ratio image1",ratioImg1);
#endif

	pMOG2Ratio0p5->apply(ratioImg0p5, outRatio0p5, 0);//.0013);
	pMOG2Ratio1->apply(ratioImg1, outRatio1, 0.0013);

	Mat binBgSub = Mat::zeros(grayInput.rows,grayInput.cols,CV_8UC1);
	Mat blankBin = binBgSub.clone();
	//====================== Minor shadow result comparing area ==========================

//	morphologyEx(outCnts, outCnts, MORPH_CLOSE, elmMedium);
//	Mat showedInputbg = Mat::zeros(input.rows, input.cols, CV_8UC1);
//	bitwise_and(grayInput, outCnts, showedInputbg);
//
//	vector<vector<Point> > bgCnt;
// 	findContours(outCnts.clone(), bgCnt, noArray(), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
// 	for (size_t cntId = 0; cntId < bgCnt.size(); ++cntId) {
// 		drawContours(inputCopy, bgCnt, cntId, Scalar(0,0,255), 2, 8, noArray());
// 		drawContours(binBgSub, bgCnt, cntId, Scalar(255), -1, 8, noArray());
//	}
//	imshow("bg to compare", showedInputbg);
	//======================

	morphologyEx(outRatio0p5, outRatio0p5, MORPH_CLOSE, elmMedium);
	morphologyEx(outRatio1, outRatio1, MORPH_CLOSE, elmMedium);

	Mat draw0p5;
	cvtColor(outRatio0p5,draw0p5,COLOR_GRAY2BGR);
	carsSeparation(outRatio0p5, NULL, &draw0p5);

	vector<vector<Point> > ratio0p5Cnt, ratio1Cnt;
 	findContours(outRatio0p5.clone(), ratio0p5Cnt, noArray(), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
 	findContours(outRatio1.clone(), ratio1Cnt, noArray(), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

 	Mat out0p5clone;
	cvtColor(outRatio0p5,out0p5clone,COLOR_GRAY2BGR);

	Mat blankBGR = Mat::zeros(grayInput.rows,grayInput.cols,CV_8UC3);
	vector<vector<Point> > hullTemp(ratio0p5Cnt.size()), preHullObj;
 	for (size_t cntId0p5 = 0; cntId0p5 < ratio0p5Cnt.size(); ++cntId0p5) {
 		convexHull(ratio0p5Cnt[cntId0p5], hullTemp[cntId0p5], false);

 		vector<Point> tempObj;
 		for (size_t cntId1p = 0; cntId1p < ratio1Cnt.size(); ++cntId1p) {
 			for (size_t ptId1p = 0; ptId1p < ratio1Cnt[cntId1p].size(); ++ptId1p) {
				int testResult = pointPolygonTest(hullTemp[cntId0p5],ratio1Cnt[cntId1p][ptId1p],false);
				if(testResult >= 0){
					tempObj.push_back(ratio1Cnt[cntId1p][ptId1p]);
				}
			}
		}
		if (tempObj.size() > 0) {
			preHullObj.push_back(tempObj);
		}
	}
	if (preHullObj.size() > 0 && preHullObj[0].size()) {
		vector<vector<Point> > hullObj(preHullObj.size());

		for (size_t cntId = 0; cntId < preHullObj.size(); ++cntId) {
			convexHull(preHullObj[cntId], hullObj[cntId], false);
			drawContours(blankBGR, hullObj, cntId, Scalar(255,255,255), -1, 8, noArray());
			drawContours(blankBin, hullObj, cntId, Scalar(255), -1, 8, noArray());
		}
#if NO_SUPPORT_PICTURES
		imshow("test", blankBGR);
#endif
	}
//	imshow("ratio bg mog", out0p5clone);

	return blankBin;
}

bool ImgProcessing::getCarGroundTruthFromDataFile(string fileName){
	return getGroundTruthFromDataFile(fileName, gndTruthForCar);
}

bool ImgProcessing::getTruckGroundTruthFromDataFile( string fileName){
	return getGroundTruthFromDataFile(fileName, gndTruthForTruck);
}

bool ImgProcessing::getGroundTruthFromDataFile(string fileName, vector<vector<vector<Point2f> > > &importedData) {
//	Mat ppSetLayer;
//	input.copyTo(ppSetLayer);

	ifstream gndFile;
	string dataLine;
	gndFile.open(fileName.c_str());
	size_t counting = 1;

	if (gndFile.is_open()) {
		getline(gndFile, dataLine);
		cout << "Reading Roi: " << dataLine << '\n';
		for (size_t i = 0; gndFile.good() /*i < 4*/; ++i, counting++) {
			char c[10];
			// check frame sync symbol
			gndFile >> c;
			cout<<c<<endl;
			if(c[0] != '*')
				continue;
			// check sync number for a frame
			size_t syncNum[2];
			gndFile >> syncNum[0];
			if(syncNum[0] != counting)
				continue;
			// read number of elements in one block
			gndFile >> syncNum[1];

			// read a block of data for one frame
			vector<vector<Point2f> > frameGndTruth;
			for (size_t blockId = 0; blockId < syncNum[1]; ++blockId) {
				// check sync symbol
				gndFile >> c;
				if(c[0] != '#')
					break;
				cout<<c;
				// check order
				size_t frameId;
				gndFile >> frameId;
				if(frameId != blockId+1)
					break;
				// read points
				vector<Point2f> tempPtsList;
				for (int ptId = 0; ptId < 4; ++ptId) {
					gndFile >> c;
					if (c[0] != '>')
						break;
					Point2f tempPt;
					gndFile >> tempPt.x;
					gndFile >> tempPt.y;
					tempPtsList.push_back(tempPt);
				}
				frameGndTruth.push_back(tempPtsList);
			}

			importedData.push_back(frameGndTruth);
		}
		gndFile.close();
		return false;
	} else {
		cout << "Unable to open file" << endl;
		return true;
	}
}

void ImgProcessing::showGroundTruthDataOnPPMap(Mat *img, vector<vector<Point2f> > &gndSet, vector<Point2f> &ptsList,vector<Rect> &boundingList, Scalar lineColor, Mat &transformMat){
	for (size_t carId = 0; carId < gndSet.size(); ++carId) {
		vector<Point2f> lines = gndSet[carId];
		vector<Point2f> gndTruthpp(lines.size());
		double perimeter = arcLength(lines,true);
		if(perimeter > 800)
			continue;

		perspectiveTransform(lines, gndTruthpp, transformMat);
		RotatedRect rr = minAreaRect(gndTruthpp);
		Rect bTruthRect = boundingRect(gndTruthpp);

		ptsList.push_back(rr.center);
		boundingList.push_back(bTruthRect);

		if(img){
			for (size_t ptId = 0; ptId < gndTruthpp.size(); ++ptId) {
				line(*img, gndTruthpp[ptId], gndTruthpp[(ptId+1)%gndTruthpp.size()],lineColor, 2);
				circle(*img, gndTruthpp[ptId], 2, Scalar(96, 128, 255),2);
			}
		}

	}
}

void ImgProcessing::showCarGroundTruthData(Mat &img,size_t frameId){
	showGroundTruthData(img, gndTruthForCar[frameId], Scalar(255, 0, 0));
}

void ImgProcessing::showTruckGroundTruthData(Mat &img, size_t frameId){
	showGroundTruthData(img, gndTruthForTruck[frameId], Scalar(0, 255, 255));
}

void ImgProcessing::showGroundTruthData(Mat &img, vector<vector<Point2f> > &gndSet, Scalar lineColor){

	for (size_t carId = 0; carId < gndSet.size(); ++carId) {
		vector<Point2f> lines = gndSet[carId];
		double perimeter = arcLength(lines,true);
		if(perimeter > 800)
			continue;
		for (size_t ptId = 0; ptId < lines.size(); ++ptId) {
			line(img, lines[ptId], lines[(ptId+1)%lines.size()],lineColor, 2);
			circle(img, lines[ptId], 2, Scalar(96, 128, 255),2);
		}
	}
}

void select_points(int event, int x, int y, int flags, void* userdata)
{
	if (event == EVENT_LBUTTONDOWN){//&& flags == EVENT_FLAG_CTRLKEY
		cout << "Left button clicked pos (x,y): " << x << "," << y << endl;
		Point2f *p = (Point2f *)userdata;
		p->x = x;
		p->y = y;
	}
}

void getPointsWithMouse(Mat &input, Point2f pts[4]){
	Mat ppSetLayer;
	input.copyTo(ppSetLayer);

	pts[0] = Point2f();
	imshow("Perspective Settings",ppSetLayer);

	for(int i = 0; i<4;){

		setMouseCallback("Perspective Settings",select_points,&pts[i]);


		if( pts[i].x>0 && pts[i].y>0 ){
			if(i>0)
				line(ppSetLayer,Point(pts[i].x,pts[i].y),Point(pts[i-1].x,pts[i-1].y),Scalar(0,0,255),2);
			if(i==3)
				line(ppSetLayer,Point(pts[0].x,pts[0].y),Point(pts[i].x,pts[i].y),Scalar(0,0,255),2);
			circle(ppSetLayer,Point(pts[i].x,pts[i].y),2,Scalar(0,255,0),2);
			imshow("Perspective Settings",ppSetLayer);
			i++;
		}
//		waitKey(1);
	}
}

bool getPointsFromFile(Mat &input, Point2f *pts, string fileName){
	Mat ppSetLayer;
	input.copyTo(ppSetLayer);

	ifstream roiFile;
	string dataLine;
	roiFile.open(fileName.c_str());
	if (roiFile.is_open()) {
		getline(roiFile, dataLine);
		cout << "Reading Roi: "<< dataLine << '\n';
		for (size_t i = 0; i < 4; ++i) {
			roiFile >> pts[i].x;
			cout << pts[i].x << "-";
			roiFile >> pts[i].y;
			cout << pts[i].y << "\n";

			if (i > 0)
				line(ppSetLayer, Point(pts[i].x, pts[i].y),Point(pts[i - 1].x, pts[i - 1].y), Scalar(0, 0, 255),2);
			if (i == 3)
				line(ppSetLayer, Point(pts[0].x, pts[0].y),Point(pts[i].x, pts[i].y), Scalar(0, 0, 255), 2);
			circle(ppSetLayer, Point(pts[i].x, pts[i].y), 2, Scalar(0, 255, 0),2);
			imshow("Perspective Settings", ppSetLayer);
		}
		roiFile.close();
		return false;
	}
	else{
		cout << "Unable to open file"<<endl;
		return true;
	}
}

void ImgProcessing::setupPPMap(Mat &input, Mat * binMask,Mat *transformMat, Mat *ppmap, string fileName){
	Point2f pts[4],mapPt[4], extVal[4];
	bool isNotError = (fileName.length() != 0);
	if(isNotError )
		isNotError = getPointsFromFile(input, pts, fileName);
	if(isNotError)
		getPointsWithMouse(input,pts);

	if(transformMat){
		mapPt[0] = Point2f(0, 0);
		mapPt[1] = Point2f(MAP_WIDTH, 0);
		mapPt[2] = Point2f(MAP_WIDTH, MAP_HEIGHT);
		mapPt[3] = Point2f(0, MAP_HEIGHT);

		extVal[0] = Point2f(30, 30);
		extVal[1] = Point2f(-50, 30);
		extVal[2] = Point2f(-50, -50);
		extVal[3] = Point2f(30, -50);
		(getPerspectiveTransform(pts,mapPt)).copyTo(*transformMat);
	}

	if (binMask) {
		(Mat(Mat::zeros(input.rows,input.cols,CV_8UC1))).copyTo(*binMask);
		for (int i = 0; i < 4; ++i) {
			roiPts[i] = Point(pts[i].x, pts[i].y );
			roiContour.push_back(mapPt[i] + extVal[i]);
		}
		Scalar color;
		if(binMask->channels() == 1)
			color = Scalar(255);
		else if(binMask->channels() == 3)
			color = Scalar(255,255,255);
		fillConvexPoly(*binMask, roiPts, 4, color);

//		imshow("mask", *binMask);
	}
	if(ppmap && transformMat){
		warpPerspective(input, (*ppmap), (*transformMat), Size(MAP_WIDTH, MAP_HEIGHT));
	}
#if !SHOW_MAP_SETTINGS
	destroyWindow("Perspective Settings");
#endif
}

bool ImgProcessing::checkPtOnROI(Point p){
	for (size_t i = 0; i < 4; ++i) {
		Point nextPt = roiPts[(i+1)%4];
		Point dPt1 = p - roiPts[i];
		Point dPt2 = nextPt - p;
		double dx = dPt1.x*1.0 / dPt2.x;
		double dy = dPt1.y*1.0 / dPt2.y;
		if(dx == dy)
			return true;
	}
	return false;
}
