#include "main.h"

int main(int argc, char** args){

	// assertions
	assert((VIDEO + CAMERA + STATIC_IMAGE) == 1 && "Only one type of input should be enable at a time");
	init(argc,args);

	char c = 'a';
	bool isFistFrame = true;
	Mat img, binMask;
	Mat transformMat, ppMap;
	ImgProcessing proc;

	// Read ground truth data
	proc.getCarGroundTruthFromDataFile(CAR_GROUNDTRUTH_FILEPATH);
	proc.getTruckGroundTruthFromDataFile(TRUCK_GROUNDTRUTH_FILEPATH);

	clock_t startTimer, processTimer;
	float sumOfTime = 0, averageOfTime = 0;
	int lastSecond;
	int fpsCounting = 0;
	int fpsShowing = 0;

	bool pauseProcess = false;
	for(int frameId = 0;c != 27; frameId++, fpsCounting++){
		img = readImage();
		if(img.empty())
			break;
		imshow("Preview", img);

		if(isFistFrame){
			proc.setupPPMap(img, &binMask,&transformMat,&ppMap,ROI_FILEPATH);
			isFistFrame = false;
			startTimer = clock();
		}

		processTimer = clock();
		Mat output = proc.mainProcessing(img, frameId, &transformMat, &ppMap, &binMask);
		float processingTime = (float)(clock() - processTimer)* 1000 / CLOCKS_PER_SEC;
		sumOfTime += processingTime;

		if(frameId > 1){
//			proc.showCarGroundTruthData(output,(frameId-2)/2);
//			proc.showTruckGroundTruthData(output,(frameId-2)/2);
		}
		// show FPS
		float timePassed = (float) (clock()- startTimer) / CLOCKS_PER_SEC;
		if(lastSecond - (int)timePassed < 0){
			lastSecond++;
			fpsShowing = fpsCounting;
			averageOfTime = sumOfTime / fpsCounting;
			proc.osfileForProcessingTime << averageOfTime << '\t' << proc.getNumberOfCars() << '\n';
			fpsCounting = 0;
			sumOfTime = 0;
		}
		ostringstream osFps, osTime;
		osFps << "FPS: " << fpsShowing;
		osTime << "TimePF: " << averageOfTime << " ms";
		putText(output, osFps.str(), Point(30,30), FONT_HERSHEY_COMPLEX, 0.8, Scalar(255,255,255),2);
		putText(output, osTime.str(), Point(30,60), FONT_HERSHEY_COMPLEX, 0.8, Scalar(255,255,255),2);
		imshow("Output", output);

		if(pauseProcess){
			c = waitKey(0);
			pauseProcess = false;
		}
		else{
			cout<< frameId <<endl;
			if(frameId<1655)
				c = waitKey(1);
			else
				c = waitKey(WAIT_TIME);
			if(c == 'p')
				pauseProcess = true;
		}

	}
	onExit();
}


