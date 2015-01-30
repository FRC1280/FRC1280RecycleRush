#include "../H/Vision.h"

Vision::Vision(void)
{
	isBright=0;

	camera = 0;

	camera = new AxisCamera("10.12.80.20"); //camera ip was found using the axis camera ip utility

	cameraWorking=(camera!=0);

	if (cameraWorking) {
		camera->WriteBrightness(30);
		camera->WriteWhiteBalance(AxisCamera::kWhiteBalance_Hold);
		camera->WriteCompression(0);
		camera->WriteColorLevel(100);
		camera->WriteMaxFPS(10);
		camera->WriteResolution(AxisCamera::kResolution_320x240);
	}
}

void Vision::processImage(void)
{
	int i;
	int brightSum=0;
	Point pixel;
	PixelValue value;

	/*if(cameraWorking && camera->IsFreshImage())
	{
		Image *image = camera->GetImage();

		for (i=0; i<400; ++i)
		{
			pixel.x=rand()%320;
			pixel.y=rand()%240;

			frcGetPixelValue (image, pixel, &value);

			brightSum+=value.hsl.L-128;
		}

		delete image;
	}*/

	isBright=(brightSum>0);
}
