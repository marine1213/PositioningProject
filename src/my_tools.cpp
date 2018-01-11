/*
 * my_tools.cpp
 *
 *  Created on: Sep 30, 2016
 *      Author: gnouchnam
 */

#include "my_tools.h"

Mat m,m2, origin;

double thres = 1, upthres = 16;
void hueSelection(vector<Mat> inputHLS) {
	// extract yellow area
	char c = 'a';

	while (c != 27) {
		c = waitKey(10);
		if (c == 'b')
			thres += (thres < 255) ? 1 : 0;
		if (c == 'c')
			thres -= (thres > 1) ? 1 : 0;
		if (c == 'g')
			upthres += (upthres < 255) ? 1 : 0;
		if (c == 'd')
			upthres -= (upthres > 1) ? 1 : 0;

		cout << "[" << thres << "] -> [" << upthres << "]\n";
		Mat yellowMask, yellowImg;
		inRange(inputHLS[0], Scalar(thres), Scalar(upthres), yellowMask);
		imshow("yellowMask", yellowMask);

	}
}
void mouseHandler(int event, int x, int y, int flags, void* param) {
	int c = m.at<unsigned char>(y, x);
	cout << c << endl;
	switch (event) {

	case EVENT_LBUTTONUP:
		cout << "left click:"<<c << endl;
		break;
	}
}

void colorPixelPicker(Mat &input, string name) {
	m = input.clone();
	imshow("color picker" + name, input);
	setMouseCallback("color picker" + name, mouseHandler);
}

Point prevPt;
void drawOnClick(int event, int x, int y, int flags, void* param) {
	Mat temp = m2.clone();
	ostringstream ss,ss2;

	circle(temp,Point(x,y),2,Scalar(0,255,0),2);
	ss <<"["<<x<<","<<y<<"]";
	putText(temp, ss.str(), Point(x, y + 30), FONT_HERSHEY_COMPLEX, 0.5,Scalar(255, 255, 255), 1, CV_AA);
	imshow("pixel picker",temp);

	switch (event) {

	case EVENT_LBUTTONUP:
		m2 = temp;
		prevPt = Point(x,y);
		break;
	case EVENT_RBUTTONDOWN:
		circle(m2,Point(x,y),2,Scalar(0,255,0),2);
		ss2 <<"["<<x<<","<<y<<"]";
		putText(m2, ss2.str(), Point(x, y + 30), FONT_HERSHEY_COMPLEX, 0.5,Scalar(255, 255, 255), 1, CV_AA);
		line(m2,Point(x,y),Point(prevPt.x, prevPt.y),Scalar(0,0,255),2);
		imshow("pixel picker",m2);
		break;
	}
}

void pixelPicker(Mat &input, bool clear){
	if(clear)
		m2 = origin.clone();
	else{
		origin = input.clone();
		m2 = input.clone();
		imshow("pixel picker",m2);
		setMouseCallback("pixel picker",drawOnClick);
	}
}
