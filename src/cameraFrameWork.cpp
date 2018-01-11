/*
 * cameraFrameWork.cpp
 *
 *  Created on: Dec 31, 2016
 *      Author: gnouchnam
 */

#include "cameraFrameWork.h"


VideoCapture capture;

std::vector<std::string> listFiles;
unsigned int fileId;

std::vector<std::string> read_directory(const std::string& path =std::string());

// Predefined Functions
void init(int argc, char** args){
#if STATIC_IMAGE
	listFiles = read_directory (DATASET_DIR);
	fileId = 0;
#elif CAMERA
	int cameraId = 0;
	if(argc > 1)
		cameraId = atoi(args[1]);
	capture = VideoCapture(cameraId);
	assert(capture.isOpened() && "Cannot open the camera");
#elif VIDEO
	capture = VideoCapture(VIDEO_FILE);
	assert(capture.isOpened() && "Cannot open video file");
#endif
}

int isDirectory(const char *path) {
   struct stat statbuf;
   if (stat(path, &statbuf) != 0)
       return 0;
   return S_ISDIR(statbuf.st_mode);
}
// read_directory()
//   Return an ASCII-sorted vector of filename entries in a given directory.
//   If no path is specified, the current working directory is used.
//
//   Always check the value of the global 'errno' variable after using this
//   function to see if anything went wrong. (It will be zero if all is well.)
//
std::vector<std::string> read_directory(const std::string& path) {
	std::vector<std::string> result;
	dirent* de;
	DIR* dp;
	dp = opendir(path.empty() ? "." : path.c_str());
	assert(dp && "Cannot open image directory");
	if (dp) {
		while (true) {
			de = readdir(dp);
			if (de == NULL)
				break;
			if(isDirectory(de->d_name))
				continue;
			result.push_back(std::string(de->d_name));
		}
		closedir(dp);
		std::sort(result.begin(), result.end());
	}
	return result;
}

void onExit(){
#if STATIC_IMAGE

#endif
}

Mat readImage()
{
	Mat input;
#if STATIC_IMAGE
	if(fileId < listFiles.size()){
		string imgPath = DATASET_DIR + listFiles[fileId];
		cout<<"\nReading Img:"<<listFiles[fileId]<<endl;
		input = imread(imgPath);
		fileId++;
	}
#elif VIDEO || CAMERA
	capture >> input;
#endif
	if(input.empty())
		perror("");
	else{
//		int newRows = COLS/(input.cols*1.0/input.rows);
//		resize(input, input, Size(COLS,newRows));
//		resize(input, input, Size(input.cols/4*3,input.rows/4*3));
//		GaussianBlur(input, input, cv::Size(5,5), 0.3);
	}
	return input;
}
