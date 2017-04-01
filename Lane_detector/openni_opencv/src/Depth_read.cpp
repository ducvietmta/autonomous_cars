#include <OpenNI.h>
#include "Viewer.h"


int main(int argc, char** argv)
{
	openni::Status rc = openni::STATUS_OK;

	openni::Device device1, device2;
	openni::VideoStream depth1, depth2;
	rc = openni::OpenNI::initialize();
	openni::Array<openni::DeviceInfo> deviceList;
	openni::OpenNI::enumerateDevices(&deviceList);

	const char* device1Uri;
	const char* device2Uri;

