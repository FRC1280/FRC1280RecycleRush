#include "../H/Vision.h"

Vision::Vision(void)
{
	isBright=0;

	camera = new AxisCamera("10.12.80.22"); //camera ip was found using the axis camera ip utility

	camera->WriteBrightness(30);
	camera->WriteWhiteBalance(AxisCamera::kWhiteBalance_Hold);
	camera->WriteCompression(0);
	camera->WriteColorLevel(100);
	camera->WriteMaxFPS(0); //send as many frames as possible
	camera->WriteResolution(AxisCamera::kResolution_640x480);
}

void Vision::processImage(void)
{
	int i;

	HSLImage *image;

	int brightSum=0;
	Point pixel;
	PixelValue value;

	if(camera->IsFreshImage())
	{
		//isBright=1;

		image = new HSLImage;

		if (camera->GetImage(image))
		{
			for (i=0; i<400; ++i)
			{
				pixel.x=rand()%40;
				pixel.y=rand()%40;

				frcGetPixelValue (image->GetImaqImage(), pixel, &value);

				brightSum+=value.hsl.L-64;
			}
		}

		if (image)
			delete image;
	}

	isBright=(brightSum>0);
}
