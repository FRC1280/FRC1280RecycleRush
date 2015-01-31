#include "../H/Vision.h"

#ifdef USE_USB_CAM

Vision::Vision(void)
{
	isBright=0;

	camera = new USBCamera("cam0", 0);
	camera->OpenCamera();
	camera->StartCapture();
}

void Vision::getImage(void)
{
	int i;

	HSLImage *image;

	int brightSum=0;
	Point pixel;
	PixelValue value;

	//isBright=1;

	image = new HSLImage;

	camera->GetImage(image->GetImaqImage());

	for (i=0; i<400; ++i)
	{
		pixel.x=rand()%40;
		pixel.y=rand()%40;

		frcGetPixelValue (image->GetImaqImage(), pixel, &value);

		brightSum+=value.hsl.L-64;
	}

	delete image;

	isBright=(brightSum>0);
}

#endif

#ifdef USE_AXIS_CAM

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

	valueMin.H=64;
	valueMin.S=64;
	valueMin.L=64;

	valueMax.H=192;
	valueMax.S=192;
	valueMax.L=192;
}

void Vision::getImage(void)
{
	if(camera->IsFreshImage())
	{
		image = new HSLImage;

		if (camera->GetImage(image))
		{

		}
	}
}

#endif

void Vision::processImage ()
{
	getImage();

	const int xIncrement=5; //the number of pixels to move x while first searching for the target
	const int yIncrement=5; //the number of pixels to move y
	int x, y;

	if (image)
	{

		x=0; y=0;

		while (getPix)

		delete image;
	}
}

inline bool Vision::checkPix(int x, int y) //checks to see if a pixel fits the criteria
{
	bool output=1;

	HSLValue value=getPix(x, y);

	if (value.S<16) //so we don't count hue if there is not enough saturation to measure it properly
	{
		if (valueMin.H<=valueMax.H) //the following makes it possible to have a range start in purple and end in red
		{
			if (value.H<valueMin.H || value.H>valueMax.H)
				output=0;

		} else
		{
			if (value.H>valueMin.H && value.H<valueMax.H)
				output=0;
		}
	}

	if (value.S<valueMin.S || value.S>valueMax.S)
		output=0;

	if (value.L<valueMin.L || value.L>valueMax.L)
		output=0;

	return output;
}

inline HSLValue Vision::getPix(int x, int y)
{
	Point pixel;
	PixelValue value;

	pixel.x=x;
	pixel.y=y;

	frcGetPixelValue (image->GetImaqImage(), pixel, &value);

	return value.hsl;
}
