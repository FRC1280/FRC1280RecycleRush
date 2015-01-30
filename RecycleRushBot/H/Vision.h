#ifndef VISION_H_
#define VISION_H_

#include "WPILib.h"

class Vision
{
	public:
		Vision(void);
		void processImage(void);
		bool getIsBright(void) { return isBright; }
		bool getCameraWorking(void) { return cameraWorking; }
	private:
		AxisCamera *camera;
		bool cameraWorking;
		bool isBright;
};

#endif
