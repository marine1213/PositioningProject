#include "main.h"

#define MAX_COUNT 250
#define DELAY_T 20
#define PI 3.1415

int main(int argc, char** args){

	// assertions
	assert((VIDEO + CAMERA + STATIC_IMAGE) == 1 && "Only one type of input should be enable at a time");
	init(argc,args);

	char c = 'a';
	bool isFistFrame = true;
	Mat img, binMask;
	ImgProcessing proc;
	DataBundle data;

	// Read ground truth data
	proc.getCarGroundTruthFromDataFile(CAR_GROUNDTRUTH_FILEPATH);
	proc.getTruckGroundTruthFromDataFile(TRUCK_GROUNDTRUTH_FILEPATH);

	// set up parameters: time, frame id, frame rate
	clock_t startTimer, processTimer;
	float sumOfTime = 0, averageOfTime = 0;
	int lastSecond;
	int fpsCounting = 0;
	int fpsShowing = 0;

	bool pauseProcess = false;
	// main processing loop
	// the loop will stop when the input sequence runs out or ESC key is pressed
	for(int frameId = 0;c != 27; frameId++, fpsCounting++){
		// read one frame
		img = readImage();
		// if it is empty, error happened, end the program
		if(img.empty())
			break;
#if SHOW_PREVIEW
		// if no error happened, show the preview of input
		imshow("Preview", img);
#endif

		// set up perspective parameter on the first frame
		if(isFistFrame){
			// The ROI_FILEPATH is the path to the list of 4 points
			// If it is null, a screen will appear to ask the user input 4 point coordinates
			// 4 points are the vertices of the working area (intersection in this case)
			proc.setupPPMap(img, &binMask,&(data.transformMat),&(data.ppMap),ROI_FILEPATH);
			isFistFrame = false;
			// get the starting second for timer
			startTimer = clock();
		}
		// get current time
		processTimer = clock();
		// receive the output from image processing file
		Mat output = proc.mainProcessing(data, img, frameId, &binMask);
		// calculate processing time in second
		float processingTime = (float)(clock() - processTimer)* 1000 / CLOCKS_PER_SEC;
		sumOfTime += processingTime;

		if(frameId > 1 && SHOW_ORIGINAL_GROUNDTRUTH){
//			draw the ground truth to input
			proc.showCarGroundTruthData(img,(frameId-2)/2);
			proc.showTruckGroundTruthData(img,(frameId-2)/2);
			imshow("gnd truth", img);
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
		// write information to the output image
		ostringstream osFps, osTime;
		osFps << "FPS: " << fpsShowing;
		osTime << "TimePF: " << averageOfTime << " ms";
		putText(output, osFps.str(), Point(30,30), FONT_HERSHEY_COMPLEX, 0.8, Scalar(255,255,255),2);
		putText(output, osTime.str(), Point(30,60), FONT_HERSHEY_COMPLEX, 0.8, Scalar(255,255,255),2);
		imshow("Output", output);

		// pause the process for debugging
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
	// release memory
	onExit();
}


