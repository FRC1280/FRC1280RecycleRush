#ifndef VISION_H_
#define VISION_H_

#include "WPILib.h"

//#define USE_USB_CAM
#define USE_AXIS_CAM

#ifdef USE_USB_CAM

class Vision
{
	public:
		Vision(void);
		void processImage(void);
		bool getIsBright(void) { return isBright; }
	private:
		USBCamera *camera;
		bool isBright;
};

#endif

#ifdef USE_AXIS_CAM

class Vision
{
	public:
		Vision(void);
		void processImage();
		bool getIsBright() { return isBright; }
	private:
		void getImage();

		bool checkPix(int x, int y);
		HSLValue getPix(int x, int y);

		AxisCamera *camera;
		HSLImage *image;

		static const int wdth=640, hght=480; //width and height of the image

		HSLValue valueMax, valueMin;

		bool isBright;
};

#endif

#endif
