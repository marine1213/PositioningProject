/*
 * ImgProcessing.cpp
 *
 *  Created on: Mar 9, 2017
 *      Author: gnouchnam
 */

#include "ImgProcessing.h"
#include "Line.h"

Mat ImgProcessing::mainProcessing(Mat &input, int frameId, Mat *transformMat, Mat * ppmap, Mat *binMask) {
	Mat maskPreProc = preProcessing(input, binMask);

	// exit if the preprocessing fails
	if(maskPreProc.rows == 1 && maskPreProc.cols == 1)
		return maskPreProc;


	vector<vector<Point> > initCnts, finalResultCnts;
	vector<Vec4i> movHier;
	Mat finalResultMat = input.clone();// =  Mat::zeros(input.size(), CV_8UC3);

	Mat cntsMergedMat = Mat::zeros(input.size(), CV_8UC1);
	Mat distanceMergedMat = Mat::zeros( input.size(), CV_8UC1 );
	Mat drawing = Mat::zeros( input.size(), CV_8UC3 );
	Mat cloneInput = input.clone();

	// Find contours in mov mask
 	findContours(maskPreProc.clone(), initCnts, movHier, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);


	for (size_t cntId = 0; cntId < initCnts.size(); cntId++) {
		Mat cntsMat =  Mat::zeros(input.size(), CV_8UC1);
		float area = contourArea(initCnts[cntId]);

		if(area<1000)	// 100 for noises and 10000 to remove small moving objects
			continue;


		finalResultCnts.push_back(initCnts[cntId]);
		// draw a mask
		drawContours(cntsMat, initCnts, cntId, Scalar(255), -1, 8, movHier);

		//============================== Cars anti-fragmenting =========================

		morphologyEx(cntsMat, cntsMat, MORPH_CLOSE, elm);

//		imshow("cont mat", cntsMat);
		bitwise_or(cntsMergedMat, cntsMat,cntsMergedMat);


		//================= startOfSep ============ Cars separation ============================
		// distance transform to separate 2 cars
		Mat distanceImage = Mat::zeros(input.size(), CV_8UC1);
		distanceTransform(cntsMat, distanceImage, DIST_L2, 5);
		normalize(distanceImage, distanceImage, 0,255, NORM_MINMAX );

		// threshold distance transform to apply separation
		double maxNormDistance;
		minMaxLoc(distanceImage, 0, &maxNormDistance);
		threshold(distanceImage, distanceImage, 0.55 * maxNormDistance,255,THRESH_TOZERO);

		Mat distImg8UC1;
		distanceImage.convertTo(distImg8UC1, CV_8UC1, 255,0);
		// show distance transform image
		 bitwise_xor(distanceMergedMat,distImg8UC1,distanceMergedMat);
//		 imshow("distance imge", distanceMergedMat);

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

			RNG rng(12345);
			Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255),rng.uniform(0, 255));
			drawContours(drawing, movCntsDist, i, color, 1, 8, vector<Vec4i>(),0, Point());
			drawContours(drawing, hullDistance, i, color, 1, 8, vector<Vec4i>(), 0,Point());


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

					//===== end of curvature calculation

		            circle(drawing, ptFar, 2, Scalar(0, 255, 0), 1);
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
			finalResultCnts.pop_back();

			Mat markers = Mat::zeros(input.rows, input.cols, CV_32SC1);

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
				Mat markerExtractedMat = Mat::zeros(input.size(), CV_8UC1);

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
					finalResultCnts.push_back(separatedCnts[sepCntId]);
					drawContours(finalResultMat,separatedCnts,sepCntId, Scalar(255,196,0),2);
				}
//				imshow("show mat", markerExtractedMat);
			}
		}
	//=========== endofSep ===========

	}

	//=========== Result Evaluation ===========
//	vector<vector<Point2f> > mappedCnts;
	vector<Point2f> resultCenterListPts;
//	vector<double> areaResultList;
	vector<Rect> boundingResultList;

	vector<Point2f> carGndTruthListPts;
//	vector<double> areaGndList;
	vector<Rect> boundingCarTruthList;
	vector<Point2f> truckGndTruthListPts;
	vector<Rect> boundingtruckTruthList;
	//============================== Flatten the contour =========================

	Mat bottomLineMat = Mat::zeros(grayInput.size(), CV_8UC1);
	Mat hullPtsmat = Mat::zeros(grayInput.size(), CV_8UC3);
	Mat ppMapClone = (*ppmap).clone();

	vector<vector<Point> > hullCnts(finalResultCnts.size());

	for (size_t i = 0; i < finalResultCnts.size(); ++i) {//bug here
		Mat cntTempMat = Mat::zeros(grayInput.size(), CV_8UC1);

		convexHull(finalResultCnts[i], hullCnts[i]);
		drawContours(cntTempMat, hullCnts, i, Scalar(255), 1, 8, noArray());
		drawContours(hullPtsmat, hullCnts, i, Scalar(255,255,255), 1, 8, noArray());

		//==============first method bottom line extraction===============
		vector<int> endingLeftId, endingRightId, endingBottomId; //, endingTopId;
		vector<vector<Point2f> > bottomLines, topLayerPts, edgeLines;//, innerLines;
		vector<vector<float> > shownDebugData;

		Scalar ptColor = Scalar(0,255,0); //Scalar(255,180,160);
		Scalar ptLeft = Scalar(255,255,0);
		Scalar ptBottom = Scalar(0,0,255);
		Scalar ptRight = Scalar(255,0,255);
		Scalar ptTop = Scalar(0,255,255);
		//===== find ending left and ending right separating contour into 2 parts ===
		for (size_t hullLineId = 0; hullLineId < hullCnts.size(); ++hullLineId) {
			int xleft, xright,ybottom, ytop;
			endingLeftId.push_back(-1);
			endingRightId.push_back(-1);
			endingBottomId.push_back(-1);
//			endingTopId.push_back(-1);
			for (size_t hullPtId = 0; hullPtId < hullCnts[hullLineId].size(); ++hullPtId) {
				if(endingLeftId.back() == -1 || xleft > hullCnts[hullLineId][hullPtId].x){
					endingLeftId[hullLineId] = hullPtId;
					xleft = hullCnts[hullLineId][hullPtId].x;
				}
				if (endingRightId.back() == -1|| xright < hullCnts[hullLineId][hullPtId].x) {
					endingRightId[hullLineId]= hullPtId;
					xright = hullCnts[hullLineId][hullPtId].x;
				}
				if (endingBottomId.back() == -1|| ybottom < hullCnts[hullLineId][hullPtId].y) {
					endingBottomId[hullLineId]= hullPtId;
					ybottom = hullCnts[hullLineId][hullPtId].y;
				}
//				if (endingTopId.back() == -1|| ytop > hullCnts[hullLineId][hullPtId].y) {
//					endingTopId[hullLineId]= hullPtId;
//					ytop = hullCnts[hullLineId][hullPtId].y;
//				}
			}
			if(endingRightId.back() == -1)
				endingRightId.pop_back();
			if(endingLeftId.back() == -1)
				endingLeftId.pop_back();
			if(endingBottomId.back() == -1)
				endingBottomId.pop_back();
//			if(endingTopId.back() == -1)
//				endingTopId.pop_back();
		}


		//==================== Analyze the corner of the car =====================
		// find the actual side points with height estimation
		for (size_t hullLineId = 0; hullLineId < endingLeftId.size(); ++hullLineId) {
			Point sidePts[2] = {
					hullCnts[hullLineId][endingLeftId[hullLineId]],
					hullCnts[hullLineId][endingRightId[hullLineId]]
			};
			Point ptBotPt = hullCnts[hullLineId][endingBottomId[hullLineId]];
//			Point ptTopPt = hullCnts[hullLineId][endingTopId[hullLineId]];
			// find nearer point to the bottom
			float minDistOf2vertices = -1;
			int minId = -1;
			for (int i = 0; i < 2; ++i) {
				float d = ((ptBotPt.y - sidePts[i].y)^2) + ((ptBotPt.x - sidePts[i].x)^2);
				if(minId == -1 || d < minDistOf2vertices){
					minDistOf2vertices = d;
					minId = i;
				}
			}
			Line sideLine = Line(ptBotPt.x, ptBotPt.y, sidePts[1-minId].x, sidePts[1-minId].y);
			float sideLineLength = sqrt(((ptBotPt.x - sidePts[1-minId].x)^2) + ((ptBotPt.y -  sidePts[1-minId].y)^2));
			//todo: find the most distance poitn
			int sideId[2] = {
					endingLeftId[hullLineId],
					endingRightId[hullLineId]
			};
			int startId, endId = endingBottomId[hullLineId];
			if(sideId[1-minId] < endId){
				startId = sideId[1-minId];
			}else{
				startId = endId;
				endId = sideId[1-minId];
			}
			float maxRatioOfDistAndDiag = -1;
			int maxIdPt = -1;
			vector<Point2f> oneEdgeLine;
			vector<float> itemData;
			for (int ptId = startId; ptId <= endId; ++ptId) {
				Point tempPt = hullCnts[hullLineId][ptId];
				float tempDist = sideLine.distance(tempPt.x, tempPt.y)/sideLineLength;
//				printf("\nThe tempDist of point %d car %d has dist %f",ptId,hullLineId,tempDist);
				if(maxRatioOfDistAndDiag == -1 || tempDist >= maxRatioOfDistAndDiag){
					maxRatioOfDistAndDiag = tempDist;
					maxIdPt = ptId;
				}
			}
			oneEdgeLine.push_back(hullCnts[hullLineId][maxIdPt]);
			itemData.push_back(maxRatioOfDistAndDiag);
//			printf("==========The max tempDist is %f", maxDistOfptList);
			if(maxRatioOfDistAndDiag > 1.1){

				float minDiffAngle = -1;
				int minDiffIdPt = -1;
				for (int ptId = maxIdPt; ptId < endId; ++ptId) {
					Point tempPt = hullCnts[hullLineId][ptId];
					// Large angle
					Line maxToBtPt = Line(tempPt.x, tempPt.y, ptBotPt.x, ptBotPt.y);
					Line maxToFarPt = Line(tempPt.x, tempPt.y, ptBotPt.x, ptBotPt.y);
					float angleDeltaLarge = maxToBtPt.getAngle() - maxToFarPt.getAngle();
					// Adjacent angle
					Point frontPt = hullCnts[hullLineId][ptId+1];
					Line maxToFront = Line(tempPt.x, tempPt.y, frontPt.x, frontPt.y);
					Point prevPt = hullCnts[hullLineId][ptId-1];
					Line maxToPrev = Line(tempPt.x, tempPt.y, prevPt.x, prevPt.y);
					float angleDeltaAdjacent = maxToPrev.getAngle() - maxToFront.getAngle();
					float tempAngle = abs(angleDeltaAdjacent - angleDeltaLarge);
					// adjacent angle
					if(minDiffAngle == -1 || tempAngle >= minDiffAngle){
						minDiffAngle = tempAngle;
						minDiffIdPt = ptId;
					}
				}

				if(minId == 0){
					endingRightId[hullLineId] = minDiffIdPt;
				}else{
					endingLeftId[hullLineId] = minDiffIdPt;
				}
			}
//			Point dPt = ptBotPt - sidePts[minId];
//			// Copy Points
//
//			vector<Point2f> oneCarLine;
//			for (size_t ptId = 0; ptId < bottomLines[hullLineId].size(); ++ptId) {
//				Point ptTemp = bottomLines[hullLineId][ptId];
//				Point newTempPt = ptTemp - dPt;
//				oneCarLine.push_back(newTempPt);
//			}

//			innerLines.push_back(oneCarLine);
			edgeLines.push_back(oneEdgeLine);
//			vector<Point2f> eachCarPts;
//			eachCarPts.push_back(ptTopPt);
//			eachCarPts.push_back(sidePts[minId]);
//			topLayerPts.push_back(eachCarPts);

			shownDebugData.push_back(itemData);
		}

		//==== extract only bottom line from hull contour =====
		for (size_t hullLineId = 0; hullLineId < endingLeftId.size(); ++hullLineId) {
//			int nearEndingLeftId = -1;
			Point pt1 = hullCnts[hullLineId][endingLeftId[hullLineId]];
			Point pt2 = hullCnts[hullLineId][endingRightId[hullLineId]];
			Line diagLine = Line(pt1.x,pt1.y,pt2.x,pt2.y);
			Point ptBotPt = hullCnts[hullLineId][endingBottomId[hullLineId]];
			int ptBtDirection = diagLine.distance(ptBotPt.x,ptBotPt.y, true);
			// Angles between

			vector<Point2f> hullLine;
			hullLine.push_back(pt2);
			for (size_t hullPtId = 0; hullPtId < hullCnts[hullLineId].size(); ++hullPtId) {
				Point ptTemp = hullCnts[hullLineId][hullPtId];
				int ptTempDirection = diagLine.distance(ptTemp.x,ptTemp.y,true);
				if(ptBtDirection*ptTempDirection > 0){
					hullLine.push_back(ptTemp);
//					size_t leftEndPtId = endingLeftId[hullLineId];
//					if(hullPtId == (size_t)(leftEndPtId + 2) || hullPtId == leftEndPtId - 2){
//						nearEndingLeftId = hullPtId;
//					}
				}
			}
			hullLine.push_back(pt1);
//			endingLeftId[hullLineId] = nearEndingLeftId;
			bottomLines.push_back(hullLine);
		}
		//====show marked points======
		for (size_t hullLineId = 0; hullLineId < bottomLines.size(); ++hullLineId) {
			for (size_t hullPtId = 0; hullPtId < bottomLines[hullLineId].size(); ++hullPtId) {
				circle(hullPtsmat, hullCnts[hullLineId][hullPtId],3, ptColor, -1);
				circle(cloneInput, hullCnts[hullLineId][hullPtId],3, ptColor, -1);
				if(hullPtId < bottomLines[hullLineId].size() - 1)
					line(cloneInput, hullCnts[hullLineId][hullPtId], hullCnts[hullLineId][hullPtId+1],Scalar(128,255,24),2);
			}
		}
		for (size_t hullPtId = 0; hullPtId < endingLeftId.size(); ++hullPtId) {
			circle(hullPtsmat, hullCnts[hullPtId][endingLeftId[hullPtId]],3, ptColor, -1);
			circle(cloneInput, hullCnts[hullPtId][endingLeftId[hullPtId]],3, ptColor, -1);
		}
		for (size_t hullPtId = 0; hullPtId < endingRightId.size(); ++hullPtId) {
			circle(hullPtsmat, hullCnts[hullPtId][endingRightId[hullPtId]],3, ptColor, -1);
			circle(cloneInput, hullCnts[hullPtId][endingRightId[hullPtId]],3, ptColor, -1);
		}
		for (size_t hullPtId = 0; hullPtId < endingBottomId.size(); ++hullPtId) {
			circle(hullPtsmat, hullCnts[hullPtId][endingBottomId[hullPtId]],3, ptBottom, -1);
			circle(cloneInput, hullCnts[hullPtId][endingBottomId[hullPtId]],3, ptBottom, -1);
		}
//		for (size_t hullPtId = 0; hullPtId < endingTopId.size(); ++hullPtId) {
//			circle(hullPtsmat, hullCnts[hullPtId][endingTopId[hullPtId]],3, ptTop, -1);
//			circle(cloneInput, hullCnts[hullPtId][endingTopId[hullPtId]],3, ptTop, -1);
//		}
//		for (size_t hullPtId = 0; hullPtId < innerLines.size(); ++hullPtId) {
//			for (size_t ptId = 0; ptId < innerLines[hullPtId].size(); ++ptId) {
//				circle(hullPtsmat, innerLines[hullPtId][ptId],3, ptTop, -1);
//				circle(cloneInput, innerLines[hullPtId][ptId],3, ptTop, -1);
//			}
//		}
		for (size_t hullPtId = 0; hullPtId < edgeLines.size(); ++hullPtId) {
			for (size_t ptId = 0; ptId < edgeLines[hullPtId].size(); ++ptId) {
				circle(hullPtsmat, edgeLines[hullPtId][ptId], 3, ptTop, -1);
				circle(cloneInput, edgeLines[hullPtId][ptId], 3, ptTop, -1);
				ostringstream oss2;
				oss2 << (shownDebugData[hullPtId][ptId]);
				putText(cloneInput, oss2.str(), edgeLines[hullPtId][ptId], FONT_HERSHEY_COMPLEX,0.7, Scalar(0, 0, 255), 1, 8);
			}
		}
//		imshow("hull pts", hullPtsmat);
		imshow("hull pts Input clone", cloneInput);
//		waitKey(0);

		//====show map and point===
		vector<Point2f> centerPtsInMap;
		for (size_t linesId = 0; linesId < bottomLines.size(); ++linesId) {
			vector<Point2f> outputPerspect(bottomLines[linesId].size());
//			vector<Point2f> ppTop(topLayerPts.size());
			if(outputPerspect.size() == 0)
				continue;

			perspectiveTransform(bottomLines[linesId], outputPerspect, (*transformMat));
//			perspectiveTransform(topLayerPts[linesId], ppTop, (*transformMat));

			// find center points
			Point2f firstBtpt = outputPerspect[outputPerspect.size() - 1];
			Point2f lastBtPt = outputPerspect[0];
			Point2f centerPt((firstBtpt.x + lastBtPt.x)/2,(firstBtpt.y + lastBtPt.y)/2);

			// save center point
			circle(ppMapClone, centerPt,3, Scalar(0,255,0), -1);

//			for (unsigned id = 0; id < topLayerPts.size(); ++id) {
//				circle(ppMapClone, ppTop[id], 3, Scalar(0,255,255), -1);
//			}

			centerPtsInMap.push_back(centerPt);
			// find symmetric points
			size_t lengthOfbottomPts = outputPerspect.size();
			for (size_t hullPtId = 0; hullPtId < lengthOfbottomPts;++hullPtId) {
				Point2f firstPt = outputPerspect[hullPtId];
				Point2f nextPt(2*centerPt.x - firstPt.x, 2*centerPt.y - firstPt.y);
				outputPerspect.push_back(nextPt);
			}

//			mappedCnts.push_back(outputPerspect);
			resultCenterListPts.push_back(centerPt);
//			double areaFull = contourArea(outputPerspect);
//			areaResultList.push_back(areaFull);
			Rect bRect = boundingRect(outputPerspect);
			boundingResultList.push_back(bRect);

			// show mapped points
			for (size_t ppPtId = 0; ppPtId < outputPerspect.size();++ppPtId) {
/*//				circle(ppMapClone, outputPerspect[ppPtId], 3, Scalar(0,0,255), -1);*/
				line(ppMapClone, outputPerspect[ppPtId], outputPerspect[(ppPtId+1)%outputPerspect.size()],Scalar(128,255,24),2);
			}
/*
//			circle(ppMapClone, outputPerspect[outputPerspect.size() - 1],3, ptRight, -1);
//			circle(ppMapClone, outputPerspect[0],3, ptLeft, -1);
*/
		}

		//--- ground truth data ---
		if(frameId > 1){

			vector<vector<Point2f> > gndCarsSet = gndTruthForCar[(frameId - 2)/2];
			vector<vector<Point2f> > gndTrucksSet = gndTruthForTruck[(frameId - 2)/2];

			showGroundTruthDataOnPPMap(ppMapClone, gndCarsSet, carGndTruthListPts, boundingCarTruthList, Scalar(255,0,0),*transformMat);
			showGroundTruthDataOnPPMap(ppMapClone, gndTrucksSet, truckGndTruthListPts, boundingtruckTruthList, Scalar(0,255,255),*transformMat);
		}
	}

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

	for (size_t resId = 0; resId < resultCenterListPts.size(); ++resId) {

		// if the vehicle center is not inside a contour, skip
		double isInsideROI = pointPolygonTest(roiContour, resultCenterListPts[resId], false);
		if(isInsideROI<0)
			continue;

		bool isTruckInGndTruth = false;

		// mapping 2 contours - find minimum distance between 2 types of center points
		double minDistane = -1;
		int minId = -1;
		for (size_t gndId = 0; gndId < carGndTruthListPts.size(); ++gndId) {
			double dx = resultCenterListPts[resId].x - carGndTruthListPts[gndId].x;
			double dy = resultCenterListPts[resId].y - carGndTruthListPts[gndId].y;
			double thisDistane = sqrt(dx * dx + dy * dy);
			if (minDistane == -1 || minDistane > thisDistane) {
				minDistane = thisDistane;
				minId = gndId;
			}
		}

		if(minDistane >= 100){
			minDistane = -1;
			isTruckInGndTruth = true;
			for (size_t gndId = 0; gndId < truckGndTruthListPts.size(); ++gndId) {
				double dx = resultCenterListPts[resId].x- truckGndTruthListPts[gndId].x;
				double dy = resultCenterListPts[resId].y- truckGndTruthListPts[gndId].y;
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
			Rect bResultRect = boundingResultList[resId];

			if(isTruckInGndTruth)
				bTruthRect = boundingtruckTruthList[minId];
			else
				bTruthRect = boundingCarTruthList[minId];


			Rect intersectionRect = bTruthRect & bResultRect;
			double intersectArea = intersectionRect.area();
			iou = intersectArea / (bResultRect.area() + bTruthRect.area() - intersectArea);

			double diagonalLine = sqrt(bTruthRect.width*bTruthRect.width + bTruthRect.height+bTruthRect.height);
			diagonalEccentricRatio = minDistane / diagonalLine;
//============================Car Tracking part===============================
			if(!isTruckInGndTruth){
				// check if the center points belong to saved cars or not
				bool isCarSavedBefore =false;
				size_t carSavedId;
				for (carSavedId = 0; carSavedId < lastNumberOfCarCenterPts; ++carSavedId) {
					Point lastSavedPt = savedCarCenterPoints[carSavedId].back();
					Rect bResultRect = boundingResultList[resId];
					bool rectXbool = bResultRect.x < lastSavedPt.x && lastSavedPt.x < bResultRect.x + bResultRect.width;
					bool rectYbool = bResultRect.y < lastSavedPt.y && lastSavedPt.y < bResultRect.y + bResultRect.height;
					if(rectXbool && rectYbool){
						isCarSavedBefore = true;
						savedCarCenterPoints[carSavedId].push_back(resultCenterListPts[resId]);
						checkedCarIdList[carSavedId] = true;
						break;
					}
				}

				// the car is not saved before, add new car
				if (!isCarSavedBefore) {
					vector<Point> newCarPoints;
					newCarPoints.push_back(resultCenterListPts[resId]);
					savedCarCenterPoints.push_back(newCarPoints);

					vector<double> newCarIOUs;
					savedCarIOUs.push_back(newCarIOUs);
					vector<double> newErrorDistance;
					savedCarErrorDistance.push_back(newErrorDistance);
				}
				if(carSavedId < savedCarIOUs.size()){
					savedCarIOUs[carSavedId].push_back(iou);
					savedCarErrorDistance[carSavedId].push_back(diagonalEccentricRatio);
				}
			}
//============================End of Car Tracking part===============================
//============================Truck Tracking part===============================
			else{
				// check if the center points belong to saved trucks or not
				bool isTruckSavedBefore =false;
				size_t truckSavedId;
				for (truckSavedId = 0; truckSavedId < lastNumberOfTruckCenterPts; ++truckSavedId) {
					Point lastSavedPt = savedTruckCenterPoints[truckSavedId].back();
					Rect bResultRect = boundingResultList[resId];
					bool rectXbool = bResultRect.x < lastSavedPt.x && lastSavedPt.x < bResultRect.x + bResultRect.width;
					bool rectYbool = bResultRect.y < lastSavedPt.y && lastSavedPt.y < bResultRect.y + bResultRect.height;
					if(rectXbool && rectYbool){
						isTruckSavedBefore = true;
						savedTruckCenterPoints[truckSavedId].push_back(resultCenterListPts[resId]);
						checkedTruckIdList[truckSavedId] = true;
						break;
					}
				}

				// the Truck is not saved before, add new Truck
				if (!isTruckSavedBefore) {
					vector<Point> newTruckPoints;
					newTruckPoints.push_back(resultCenterListPts[resId]);
					savedTruckCenterPoints.push_back(newTruckPoints);

					vector<double> newTruckIOUs;
					savedTruckIOUs.push_back(newTruckIOUs);
					vector<double> newErrorDistance;
					savedTruckErrorDistance.push_back(newErrorDistance);
				}
				if(truckSavedId < savedTruckIOUs.size()){
					savedTruckIOUs[truckSavedId].push_back(iou);
					savedTruckErrorDistance[truckSavedId].push_back(diagonalEccentricRatio);
				}
			}
//============================End of Truck Tracking part===============================
//			rectangle(ppMapClone, intersectionRect,Scalar(128,255,196),-1,8);
//			rectangle(ppMapClone, boundingTruthList[minId],Scalar(0,255,196),2,8);
//			areaRatio = (int) (areaResultList[resId] / areaGndList[minId] * 1000);

//			osfile<<iou<<"\t"<<minDistane<<"\t"<<distanceRatio<<endl;
			ostringstream oss2;
			oss2 << (int)(iou*100);
			//Todo problem is here
			if(isTruckInGndTruth)
				putText(ppMapClone, oss2.str(), Point(truckGndTruthListPts[minId].x, truckGndTruthListPts[minId].y - 10), FONT_HERSHEY_COMPLEX,0.7, Scalar(0, 0, 255), 1, 8);
			else
				putText(ppMapClone, oss2.str(), Point(carGndTruthListPts[minId].x, carGndTruthListPts[minId].y - 10), FONT_HERSHEY_COMPLEX,0.7, Scalar(0, 0, 255), 1, 8);
			circle(ppMapClone, resultCenterListPts[resId], 2, Scalar(96, 255, 96), 2);
		}

//		ostringstream oss;
//		oss<< (int)(areaRatio)<<'%';
//		putText(ppMapClone, oss.str(), Point(gndTruthListPts[minId].x, gndTruthListPts[minId].y + 10), FONT_HERSHEY_COMPLEX, 0.7, Scalar(0,0,255),1,8);
//		circle(ppMapClone, gndTruthListPts[minId], 2, Scalar(96, 128, 255),2);
	}

	//======== remove not checked car center points ============
	int carErasingTimes = 0;
	for (size_t carCenterPtId = 0; carCenterPtId < checkedCarIdList.size(); ++carCenterPtId) {
		// browse the list if the car is checked or not
		if(checkedCarIdList[carCenterPtId] == false){
			// check if the number of saved IOU is larger than 5
			// if not, it means the sudden object appeared for a short time
			// and is considered to be spike noise
			if(savedCarIOUs[carCenterPtId].size() > 5){
				double sumIOU = 0, sumErrorDistance = 0;
				osfileForCar<<savedCarIOUs[carCenterPtId].size()<<'\t';
				for (size_t iouId = 0; iouId < savedCarIOUs[carCenterPtId].size(); ++iouId) {
					sumIOU += savedCarIOUs[carCenterPtId][iouId];
					sumErrorDistance += savedCarErrorDistance[carCenterPtId][iouId];
//					osfile<<savedIOUs[centerPtId][iouId]<<'\t';
//					osfileForCar<<savedCarErrorDistance[carCenterPtId][iouId]<<'\t';
				}
				double averageIOUval = sumIOU/savedCarIOUs[carCenterPtId].size();
				double averageErrorDist = sumErrorDistance/savedCarErrorDistance[carCenterPtId].size();
				averageCarIOUs.push_back(averageIOUval);
				averageCarErrorDistance.push_back(averageErrorDist);
				osfileForCar<< averageIOUval<<'\t'<<averageErrorDist<<endl;
//				osfileForCar<<endl;
			}

			// remove saved coordinates
			savedCarCenterPoints.erase(savedCarCenterPoints.begin() + carCenterPtId - carErasingTimes);
			// remove saved IOU
			savedCarIOUs.erase(savedCarIOUs.begin() + carCenterPtId - carErasingTimes);

			carErasingTimes++;
		}
	}
	//======== remove not checked truck center points ============
	int truckErasingTimes = 0;
	for (size_t truckCenterPtId = 0; truckCenterPtId < checkedTruckIdList.size(); ++truckCenterPtId) {
		// browse the list if the Truck is checked or not
		if(checkedTruckIdList[truckCenterPtId] == false){
			// check if the number of saved IOU is larger than 10
			// if not, it means the sudden object appeared for a short time
			// and is considered to be spike noise
			if(savedTruckIOUs[truckCenterPtId].size() > 10){
				double sumIOU = 0, sumErrorDistance = 0;
				osfileForTruck<<savedTruckIOUs[truckCenterPtId].size()<<'\t';
				for (size_t iouId = 0; iouId < savedTruckIOUs[truckCenterPtId].size(); ++iouId) {
					sumIOU += savedTruckIOUs[truckCenterPtId][iouId];
					sumErrorDistance += savedTruckErrorDistance[truckCenterPtId][iouId];
//					osfileForTruck<<savedTruckErrorDistance[truckCenterPtId][iouId]<<'\t';
				}
				double averageIOUval = sumIOU/savedTruckIOUs[truckCenterPtId].size();
				double averageErrorDist = sumErrorDistance/savedTruckErrorDistance[truckCenterPtId].size();
				averageTruckIOUs.push_back(averageIOUval);
				averageTruckErrorDistance.push_back(averageErrorDist);
				osfileForTruck<< averageIOUval<<'\t'<<averageErrorDist<<endl;
//				osfileForTruck<<endl;
			}

			// remove saved coordinates
			savedTruckCenterPoints.erase(savedTruckCenterPoints.begin() + truckCenterPtId - truckErasingTimes);
			// remove saved IOU
			savedTruckIOUs.erase(savedTruckIOUs.begin() + truckCenterPtId - truckErasingTimes);

			truckErasingTimes++;
		}
	}
	//============================================================
	ostringstream oss;
	oss << "No. Vehicles: " << savedCarIOUs.size() + savedTruckIOUs.size();
	putText(ppMapClone, oss.str(), Point(30, 90), FONT_HERSHEY_COMPLEX,  0.8, Scalar(255,255,255),2);

	numberOfCars = savedCarIOUs.size();

//	imshow("pp map", ppMapClone);
//	waitKey(0);
//	imshow("hull pts", hullPtsmat);
/*

//	for (size_t i = 0; i < finalResultCnts.size(); ++i) {
//		RotatedRect rotRect = minAreaRect(finalResultCnts[i]);
//		Point2f pts[4];
//		rotRect.points(pts);
//
//		size_t minyId = 0;
//		int miny = pts[minyId].y;
//		for (size_t i = 1; i < 4; ++i) {
//			if (pts[i].y < miny) {
//				minyId = i;
//				miny = pts[i].y;
//			}
//		}
//
//		//		cvtColor(cannyImg,showMat,COLOR_GRAY2BGR);
//		for (size_t i = 0; i < 4; ++i) {
////			if (minyId != i && minyId != i + 1) {
//				line(finalResultMat, pts[i], pts[(i + 1) % 4], Scalar(0, 0, 255), 1,CV_AA);
////			}
////			if (minyId != i)
//				circle(finalResultMat, pts[i], 3, Scalar(0, 196, 255));
//		}
//	}
//	imshow("detection",finalResultMat);
*/
//	imshow("cnts mat mask",cntsMergedMat);
//	imshow("convexHull distance",drawing);
//	imshow("Distance image", distanceMergedMat);
//	return finalResultMat;
	return ppMapClone;
}

Mat ImgProcessing::preProcessing(Mat &input, Mat *binMask){
	Mat outRatio;
	Mat outCnts;

	// gray conversion
	cvtColor(input, grayInput, COLOR_BGR2GRAY);


	if(binMask)
		bitwise_and(grayInput,*binMask,grayInput);
/*
//	imshow("gray input", grayInput);

//	equalizeHist(grayInput, grayInput);
//	imshow("equalized", grayInput);

//	Mat edgesMat;
//	Canny(grayInput, edgesMat, 30,250);
//	imshow("edges", edgesMat);

//	Mat grayInputFloat;
//	grayInput.convertTo(grayInputFloat, CV_32FC1, 1.0/255);
*/
	if(accInput.data == NULL){
		accInput = grayInput.clone()/2;
/*
//		double maxVal;
//		minMaxLoc(accInput, 0, &maxVal);
*/

		recoverValue = mean(accInput).val[0];

//		cout<<"recover value = "<< recoverValue<<endl;
//		return Mat::zeros(1,1,CV_8UC1);
	}


	Mat ratioImg = abs(grayInput / accInput);
//	double averageRatio = mean(ratioImg).val[0];
//	cout<<"average ratio value = "<< averageRatio<<endl;
	ratioImg = ratioImg * (int)recoverValue;

//	imshow("ratio image",ratioImg);
//	colorPixelPicker(ratioImg,"ratio");


//	Mat thresRatio;
//	threshold(ratioImg, thresRatio,70, 255, CV_THRESH_BINARY);

//	imshow("bthres ratio",thresRatio);

	// background extraction
	/**
	 * previous method: grayInput
	 * new method: ratioImg
	 */
	pMOG2Ratio->apply(ratioImg, outRatio, 0);//.0013);
	pMOG2grayInput->apply(grayInput, outCnts, 0.0013);

//	imshow("edges sub", outCnts);
//	morphologyEx(outCnts, outCnts, MORPH_OPEN, elmMedium);
//	morphologyEx(outCnts, outCnts, MORPH_CLOSE, elm);
//	imshow("edges morph", outCnts);

	Mat inputCopy = input.clone();
	Mat binBgSub = Mat::zeros(grayInput.rows,grayInput.cols,CV_8UC1);
	Mat binRatio = binBgSub.clone();
	//====================== Minor shadow result comparing area ==========================

	morphologyEx(outCnts, outCnts, MORPH_CLOSE, elmMedium);
	Mat showedInputbg = Mat::zeros(input.rows, input.cols, CV_8UC1);
	bitwise_and(grayInput, outCnts, showedInputbg);

	vector<vector<Point> > bgCnt;
 	findContours(outCnts.clone(), bgCnt, noArray(), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
 	for (size_t cntId = 0; cntId < bgCnt.size(); ++cntId) {
 		drawContours(inputCopy, bgCnt, cntId, Scalar(0,0,255), 2, 8, noArray());
 		drawContours(binBgSub, bgCnt, cntId, Scalar(255), -1, 8, noArray());
	}
//	imshow("bg to compare", showedInputbg);
	//======================

	morphologyEx(outRatio, outRatio, MORPH_CLOSE, elmMedium);
	Mat showedInputRatio = Mat::zeros(input.rows, input.cols, CV_8UC1);
	bitwise_and(grayInput, outRatio, showedInputRatio);

	vector<vector<Point> > ratioCnt;
 	findContours(outRatio.clone(), ratioCnt, noArray(), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	vector<vector<Point> > hullCnt(ratioCnt.size());
 	for (size_t cntId = 0; cntId < ratioCnt.size(); ++cntId) {
 		drawContours(inputCopy, ratioCnt, cntId, Scalar(255,255,0), 2, 8, noArray());
 		convexHull(ratioCnt[cntId], hullCnt[cntId], false);
 		drawContours(binRatio, hullCnt, cntId, Scalar(255), -1, 8, noArray());
	}
//	imshow("mov to compare", showedInputRatio);
//	imshow("input to compare", inputCopy);

	//===================================================================================
//	bitwise_and(binRatio, binBgSub, binRatio);
//	imshow("Ratio And BgSub", binRatio);

//	vector<vector<Point> > movCnts;
//	findContours(outCnts.clone(), movCnts, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

//	for (size_t i = 0; i < movCnts.size(); ++i) {
////		float area = contourArea(movCnts[i]);
////		if(area > 10)
//		drawContours(outCnts, movCnts, i, Scalar(255), -1, 8, noArray());
//		imshow("edges morph", outCnts);
////		waitKey(0);
//	}


//	cv::Mat bg = cv::Mat(input.size(), CV_8UC1);
//	pMOG2->getBackgroundImage(bg);

//	imshow("before open", output);
	// close area with glass
//	morphologyEx(outRatio, outRatio, MORPH_ERODE, elmOpen);



//	morphologyEx(outCnts, outCnts, MORPH_OPEN, elmSmall);
//	Mat erodeRatio;
//	morphologyEx(outRatio, erodeRatio, MORPH_ERODE, elmOpen);
//
//	imshow("open dge ratio img", outCnts);
	return outRatio; //- correct output
// 	return outCnts;
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
void ImgProcessing::showGroundTruthDataOnPPMap(Mat &img, vector<vector<Point2f> > &gndSet, vector<Point2f> &ptsList,vector<Rect> &boundingList, Scalar lineColor, Mat &transformMat){
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


		for (size_t ptId = 0; ptId < gndTruthpp.size(); ++ptId) {
			line(img, gndTruthpp[ptId], gndTruthpp[(ptId+1)%gndTruthpp.size()],lineColor, 2);
			circle(img, gndTruthpp[ptId], 2, Scalar(96, 128, 255),2);
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
