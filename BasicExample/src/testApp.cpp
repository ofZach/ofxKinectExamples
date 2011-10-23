#include "testApp.h"

void testApp::setup() {
	ofSetVerticalSync(true);
	kinect.init();
	//kinect.init(true);  // shows infrared instead of RGB video image
	//kinect.init(false, false);  // disable infrared/rgb video iamge (faster fps)
	kinect.open();
}

void testApp::update() {
	kinect.update();
}

void testApp::draw() {
	ofSetColor(255);
	kinect.drawDepth(0, 0, 400, 300);	
	kinect.draw(400, 0, 400, 300);
	
	// different ways of accessing the depth data
	unsigned short* rawDepthPixels = kinect.getRawDepthPixels(); // raw 11-bit data
	float* distancePixels = kinect.getDistancePixels(); // distance in centimeters
	unsigned char* depthPixels = kinect.getDepthPixels(); // normalized to 0-255
}

void testApp::exit() {
	kinect.close();
}
