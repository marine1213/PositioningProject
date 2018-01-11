/*
 * main_imgProcessingLib.h
 *
 *  Created on: Oct 28, 2016
 *      Author: gnouchnam
 */

#ifndef MAIN_IMGPROCESSINGLIB_H_
#define MAIN_IMGPROCESSINGLIB_H_

#include <dirent.h>
#include <sys/stat.h>
#include "extCommon.h"
#include "extOpencvHeader.h"


// Variables, structures and classes
#if STATIC_IMAGE
#define 	WAIT_TIME			1//2000
#elif VIDEO || CAMERA
#define 	WAIT_TIME			1
#endif

Mat readImage();

// Function headers
void init(int argc, char** args);
void onExit();


#endif /* MAIN_IMGPROCESSINGLIB_H_ */
