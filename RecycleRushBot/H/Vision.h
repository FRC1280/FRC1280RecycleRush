#ifndef VISION_H_
#define VISION_H_

#include "WPILib.h"

class Vision
{
	public:
		Vision(void);
		void processImage(void);
		bool getIsBright(void) { return isBright; }
	private:
		AxisCamera *camera;
		bool isBright;
};

#endif
