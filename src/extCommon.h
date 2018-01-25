#ifndef _COMMON_H_
#define _COMMON_H_


#include <iostream>
#include <cassert>
#include <algorithm>
#include <cstdio>
#include <sstream>

using namespace std;

// Type of input (enable for an input only)
#define STATIC_IMAGE				1
#define VIDEO						0
#define CAMERA						0


// Dataset configurations
#define VIDEO_FILE					"../Dataset/Intersection dataset/Custom Dataset/Jodoin/Sherbrooke/sherbrooke_video.avi"
#define DATA_PATH					"../Dataset/Intersection dataset/Kwangsoo-dataset/"
#define DATASET_DATA				DATA_PATH"Sequence1d/"
#define DATASET_DIR					DATASET_DATA"KAB_SK_1_undist/"
#define ROI_FILEPATH				DATASET_DATA"Roi"
#define CAR_GROUNDTRUTH_FILEPATH	DATASET_DATA"groundTruthData.txt"
#define TRUCK_GROUNDTRUTH_FILEPATH	DATASET_DATA"groundTruthDataTruck.txt"

// Image size
#define SIZE_HOR					960
#define SIZE_VER					540

#define MAP_WIDTH					522
#define MAP_HEIGHT					586

// Performance Ouput
#define OS_FOR_CAR					DATASET_DATA"OuputStreamForCar.txt"
#define OS_FOR_TRUCK				DATASET_DATA"OutputStreamForTruck.txt"
#define OS_PROCESSING_TIME			DATASET_DATA"OutputStreamForCarProcessingTime.txt"
#endif //_COMMON_H_
