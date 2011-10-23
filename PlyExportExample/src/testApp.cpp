#include "testApp.h"

const float FovX = 1.0144686707507438;
const float FovY = 0.78980943449644714;
const float XtoZ = tanf(FovX / 2) * 2;
const float YtoZ = tanf(FovY / 2) * 2;
const unsigned int Xres = 640;
const unsigned int Yres = 480;

ofVec3f ConvertProjectiveToRealWorld(float x, float y, float z) {
	return ofVec3f((x / Xres - .5f) * z * XtoZ,
								 (y / Yres - .5f) * z * YtoZ,
								 z);
}

void exportPlyCloud(string filename, ofMesh& cloud) {
	ofFile ply;
	if (ply.open(filename, ofFile::WriteOnly)) {		
		// write the header
		ply << "ply" << endl;
		ply << "format binary_little_endian 1.0" << endl;
		ply << "element vertex " << cloud.getVertices().size() << endl;
		ply << "property float x" << endl;
		ply << "property float y" << endl;
		ply << "property float z" << endl;
		ply << "end_header" << endl;
		
		// write all the vertices
		vector<ofVec3f>& surface = cloud.getVertices();
		for(int i = 0; i < surface.size(); i++) {
			if (surface[i].z != 0) {
				// write the raw data as if it were a stream of bytes
				ply.write((char*) &surface[i], sizeof(ofVec3f));
			}
		}
	}
}

void testApp::setup() {
	ofSetVerticalSync(true);
	kinect.init(false, false);  // disable infrared/rgb video iamge (faster fps)
	kinect.open();
}

void testApp::update() {
	kinect.update();
	if(kinect.isFrameNew()) {
		int width = kinect.getWidth();
		int height = kinect.getHeight();
		float* distancePixels = kinect.getDistancePixels(); // distance in centimeters
		cloud.clear();
		cloud.setMode(OF_PRIMITIVE_POINTS);
		for(int y = 0; y < height; y++) {
			for(int x = 0; x < width; x++) {
				int i = y * width + x;
				float z = distancePixels[i];
				if(z != 0) { // ignore empty depth pixels
					cloud.addVertex(ConvertProjectiveToRealWorld(x, y, z));
				}
			}
		}
	}
	if(exportPly) {
		exportPlyCloud("cloud.ply", cloud);
		exportPly = false;
	}
}

void testApp::draw() {
	ofBackground(0);
	
	ofSetColor(255);
	kinect.drawDepth(0, 0, 400, 300);
	
	easyCam.begin();
	int width = kinect.getWidth();
	int height = kinect.getHeight();
	ofScale(1, -1, -1); // orient the point cloud properly
	ofTranslate(0, 0, -150); // rotate about z = 150 cm
	cloud.drawVertices();
	easyCam.end();
}

void testApp::exit() {
	kinect.close();
}

void testApp::keyPressed(int key) {
	if(key == ' ') {
		exportPly = true;
	}
}