#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxBox2d.h"
#include "Particle.h"
#include "ContactListener.h"
#include "ofxContourAnalysis.h"
#include "ofxVectorMath.h"

//writeTXT
#include <iostream>
#include <fstream>
using namespace std;

#define NUM_GUITAR 16
#define NUM_PIANO 8
#define NUM_PARTICLES 60



class ballPond : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed  (int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
    
        void resetBalls();
        void getSizeOf2DArray(vector <ofPoint> nums);
    
        
        //Write TXT
        ofstream myfile;
        
        //Debug Function
        void drawDebugText(int dX, int dY, int dW, int dH);
        ofTrueTypeFont	verdana;
        
        void screenCapture();
        void distCapture();
        
        //Kinect Stuff
        ofxKinect				kinect;
        
        //OpenCv
        ofxCvColorImage			colorImg;
        ofxCvGrayscaleImage 	grayImage;
        ofxCvGrayscaleImage 	grayThresh;
        ofxCvGrayscaleImage 	grayThreshFar;
        ofxCvContourFinder		contourFinder;
        bool					bThreshWithOpenCV;	
        int						nearThreshold;
        int						farThreshold;
        int						angle;
        
        //my variables
        unsigned char * tmpPixels;
        unsigned char * depthGreyVal;
        ofPoint blob1;
        ofPoint blob2;
        ofPoint blob1Size;
        ofPoint blob2Size;
        int resX;
        int resY;
        int distBlob;
        int minBlobSize;
        int maxBlobsize;
        float particleRadius;
        
        
        //Boolean
        bool bShowDepth;
        bool bShowContour;
        bool bShowRGB;
        bool bShowGray;
        bool bShowBlob;
        bool bShowSkeleton;
        bool bDebug;
        bool bSimpleContourPoint;
        bool bDistBlob;
        
        //BOX2D 
        void resized(int w, int h);
        
        float px, py;
        bool							bDrawLines;
        bool							bMouseForce;
        
        ofxBox2d						box2d;			//	the box2d world
        vector <ofxBox2dCircle>	circles;		//	default box2d circles
        //vector	<ofxBox2dPolygon>	polygons;		//	defalut box2d polgons
        //vector    <ofxBox2dLine>		lines;			//	default box2d lines (hacked)
        vector		<Particle>			particles;		//	this is a custom particle the extends a cirlce
        
        ofxBox2dCircle					ballJoints[5];	//	ball joints
        ofxBox2dJoint					joints[5];		//	box2d joints
        
        //	collision points
        vector		< vector<ofPoint> > v_arr;
        vector		<int>				blobPts;
        vector		<ofxBox2dLine>		lineStripBlob;
        int								clearLineStrip;
        
        //Contour Work
        //ofxCvContourFinder				contour;
        ofxContourAnalysis				contourAnalysis;
        
        vector		<ofPoint>			simpleContour;
        vector	<vector <ofPoint> >		createContour; //2dArray
        float							simpleAmount;
        bool							bReversePoints;
        
        //Sound
        ofSoundPlayer ambient;
        vector		<ofSoundPlayer>		guitar;
        vector		<ofSoundPlayer>		piano;
        
        
        //Background
        ofImage background;
        
        //ContactListener
        ContactListener	contacts;
        
        //const b2ContactPoint* point;
        void box2dContactEventHandler ( const b2ContactPoint* point );
        
        //screencapture
        ofImage myImage;
        int mySeconds;
        int count;
        
        int tempFrameRate;
        int saveTime;
        int tempTime;
        
        int tempFrameRateSec;
        int saveTimeSec;
        int tempTimeSec;


		
};
