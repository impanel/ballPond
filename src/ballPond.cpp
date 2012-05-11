#include "ballPond.h"

//--------------------------------------------------------------
void ballPond::setup(){
    
    ofSetFrameRate(60);
    
    //setup
	bShowDepth = false;
	bShowContour = false;
	bShowGray = false;
	bShowBlob = false;
	bShowRGB = false;
	bShowSkeleton = false;
	bDebug = false;
	bDistBlob = false;
    
    bThreshWithOpenCV = true;
	
	minBlobSize = 3000;
	
	resX = ofGetWidth();
	resY = ofGetHeight();
	
	particleRadius = 30.0;
	
    //kinect
	kinect.init();
	kinect.setVerbose(true);
	kinect.open();
    // zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);
    //depth thresholds
    nearThreshold = 197;
	farThreshold  = 58;
	
    //openCV
	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThresh.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);
		
	//Box2D
	box2d.init();
	box2d.setGravity(0, 10);
	box2d.createBounds(0, 0, ofGetWidth(), ofGetHeight());
	box2d.checkBounds(true);
	box2d.setFPS(60.0);
	
	
	ofxBox2dLine lineStrip;
	for (int i = 0; i < 10; i++) {
		lineStripBlob.push_back(lineStrip);
	}
	clearLineStrip = 0;
	
	//Contour Analysis -> simplify
	simpleAmount = 3.2;
	//true = particles are outside
	bReversePoints = true;
	
	//ContactListener
	//contacts.Add(point, particles);
	box2d.getWorld() -> SetContactListener(&contacts);
	
	//Create Particles
	for( int i=0; i <= NUM_PARTICLES; i++ )
	{
		particles.push_back( Particle());
		Particle& circle = particles.back();
		
		float mass		= 7.0;
		float bounce	= 0.73;
		float friction	= 0.1;
		float radius	= particleRadius;
		
		circle.setPhysics( mass, bounce, friction );
		circle.setup( box2d.getWorld(), ofRandom( 0, ofGetWidth() ), ofRandom( 0, ofGetHeight() ), radius, false );
		circle.setVelocity( ofRandom( -10, 10 ), ofRandom( -10, 10 ) );
		
		//orange
		if (i < NUM_PARTICLES/5) {
			circle.r = 248;
			circle.g = 120;
			circle.b = 6;	
			circle.groupId = 1;
		}
		//purple
		else if(i >= NUM_PARTICLES/5 && i < (NUM_PARTICLES/5) * 2)
		{
			circle.r = 160;
			circle.g = 14;
			circle.b = 242;
			circle.groupId = 2;
		}
		//green
		else if(i >= (NUM_PARTICLES/5) * 2 && i < (NUM_PARTICLES/5) * 3)
		{
			circle.r = 152;
			circle.g = 189;
			circle.b = 8;
			circle.groupId = 3;
		}
		//magenta
		else if(i >= (NUM_PARTICLES/5) * 3 && i < (NUM_PARTICLES/5) * 4)
		{
			circle.r = 200;
			circle.g = 29;
			circle.b = 89;
			circle.groupId = 4;
		}
		//blue
		else if(i >= (NUM_PARTICLES/5) * 4 && i <= NUM_PARTICLES)
		{
			circle.r = 40;
			circle.g = 157;
			circle.b = 224;	
			circle.groupId = 5;
		}
	}
	
	contacts.init(particles);
    
	//Sound
	/*
    ambient.loadSound("ambient_loop.mp3");
	ambient.setLoop(true);
	ambient.setVolume(0.75f);
	ambient.play();
    */
	
	for (int i = 0; i < NUM_GUITAR; i++) {
		ofSoundPlayer guitarSound;
		guitar.push_back(guitarSound);
		if(i < 8)
			guitar[i].setVolume(0.75f);
		else {
			guitar[i].setVolume(0.4f);
		}
		guitar[i].setMultiPlay(true);
	}
	
	//GUITAR SOUND
	guitar[0].loadSound("Gitarre/gitarre1.mp3");
	guitar[1].loadSound("Gitarre/gitarre2.mp3");
	guitar[2].loadSound("Gitarre/gitarre3.mp3");
	guitar[3].loadSound("Gitarre/gitarre4.mp3");
	guitar[4].loadSound("Gitarre/gitarre5.mp3");
	guitar[5].loadSound("Gitarre/gitarre6.mp3");
	guitar[6].loadSound("Gitarre/gitarre7.mp3");
	guitar[7].loadSound("Gitarre/gitarre8.mp3");
	guitar[8].loadSound("Piano/1.mp3");
	guitar[9].loadSound("Piano/2.mp3");
	guitar[10].loadSound("Piano/3.mp3");
	guitar[11].loadSound("Piano/4.mp3");
	guitar[12].loadSound("Piano/5.mp3");
	guitar[13].loadSound("Piano/6.mp3");
	guitar[14].loadSound("Piano/7.mp3");
	guitar[15].loadSound("Piano/8.mp3");
	
	//Background Image
	background.loadImage("background_2.jpg");
	
	//load Font for Debug text
	verdana.loadFont("verdana.ttf",8, false, true);
	verdana.setLineHeight(16.0f);
	
	//create TXT file
	myfile.open(ofToDataPath("distance.txt").c_str(), ios::out | ios::app | ios::binary);
	
	mySeconds = ofGetSeconds();
	
	tempFrameRate = int(ofGetFrameRate()*50);
	saveTime = int(ofGetFrameRate()*10);
	tempTime = 0;
	
	tempFrameRateSec = int(ofGetFrameRate()*5);
	saveTimeSec = int(ofGetFrameRate()*1);
	tempTimeSec = 0;
    
}

//--------------------------------------------------------------
void ballPond::update(){
    
    kinect.update();
	
    
	grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
	
	grayThreshFar = grayImage;
	grayThresh = grayImage;
	grayThresh.threshold(nearThreshold, true);
	grayThreshFar.threshold(farThreshold);
	cvAnd(grayThresh.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
	
	//update the cv image
	grayImage.flagImageChanged();
    
	contourFinder.findContours(grayImage, minBlobSize, (kinect.width*kinect.height)/2, 3, true);
	
    
	vector <ofPoint> temp;
  
	if (contourFinder.blobs.size() > 0) 
	{
		//Contour Analysis
		for (int i = 0; i < contourFinder.blobs.size(); i++) 
        {
			
			v_arr.push_back(temp);
			ofPoint p;
			//ofxBox2dLine lineStrip;
			//lineStripBlob.push_back(lineStrip);
			lineStripBlob[i].setWorld(box2d.getWorld());
			lineStripBlob[i].clear();
			//lineStripBlob.clear();
			
            if(bReversePoints) {
				contourAnalysis.simplify(contourFinder.blobs[i].pts, simpleContour, simpleAmount); //BIG MEMORY ALLOCATION LEAK *SOLVED*
               
				blobPts.push_back(simpleContour.size()-1);
                for (int j = 0; j < blobPts[i]; j++) 
                {
					v_arr[i].push_back(0);
				}
				
				for(int j = blobPts[i]; j >= 0; j--) 
                {
                    //for (int j=0; j<blobPts[i]; j++) {
					v_arr[i].push_back(0);
					v_arr[i][j].x = simpleContour[j].x;
					v_arr[i][j].y = simpleContour[j].y;
					p.x = v_arr[i][j].x;
					p.y = v_arr[i][j].y;
					lineStripBlob[i].addPoint(ofMap(p.x, 0, 640, 0, resX, false), ofMap(p.y, 0, 480, 0, resY, false));
				}
			}
			// good to go :)
			lineStripBlob[i].createShape();
		}
		
		//Blobs Center/Width/Distance
		blob1.x = contourFinder.blobs[0].centroid.x;
		blob1.y = contourFinder.blobs[0].centroid.y;
		blob1Size.x = contourFinder.blobs[0].boundingRect.width;
		blob1Size.y = contourFinder.blobs[0].boundingRect.height;
		
		if (contourFinder.blobs.size() == 2){
			blob2.x = contourFinder.blobs[1].centroid.x;
			blob2.y = contourFinder.blobs[1].centroid.y;
			blob2Size.x = contourFinder.blobs[1].boundingRect.width;
			blob2Size.y = contourFinder.blobs[1].boundingRect.height;
			
			if (blob1.x < blob2.x) {
				distBlob = ofDist((blob1.x + blob1Size.x/2),0,(blob2.x - blob2Size.x/2),0);
			}else {
				distBlob = ofDist((blob1.x - blob1Size.x/2),0,(blob2.x + blob2Size.x/2),0);
			}
		}else {
			bDistBlob = true;
		}
        
	}
    
	if (contourFinder.nBlobs < clearLineStrip) 
    {
		//cout << "nblobs: " << contourFinder.nBlobs << " clearLineStrip: " << clearLineStrip << endl;
		for (int i = contourFinder.nBlobs; i < clearLineStrip; i++) 
        {
			lineStripBlob[i].clear();
			//cout << "lineStripBlob[" << i << "].clear()" << endl;
		}
	}
	
	clearLineStrip = contourFinder.nBlobs;
	//cout << " clearLineStrip: " << clearLineStrip <<  " = nblobs: " << contourFinder.nBlobs <<   " = blobs.size(): " << contourFinder.blobs.size() << endl;
	
	//Box2d
	box2d.update();
	
	//if particles are outside the window set new position
	for( int i = 0; i < particles.size(); i++ )
	{
		particles[i].update();
	}
	
	//Sound
	ofSoundUpdate();
	//ContactHandler
}

//--------------------------------------------------------------
void ballPond::draw(){
    
    ofSetColor(255, 255, 255, 255);
	background.draw(0, 0);
	
	
	if (bDebug) {
		if (bShowRGB) {
			kinect.draw(0, 0, ofGetWidth(), ofGetHeight());
		}
		else if(bShowGray)
		{
			grayImage.draw(0, 0, ofGetWidth(), ofGetHeight());
		}
		else if(bShowDepth)
		{
			kinect.drawDepth(0, 0, ofGetWidth(), ofGetHeight());
		}
		else if(bShowContour)
		{
			contourFinder.draw(0, 0, ofGetWidth(), ofGetHeight());
		}
	}
    
	//draw particles
	ofFill();
	ofEnableSmoothing();
	for (int i = 0; i < particles.size(); i++) {
		particles[i].draw();
	}
	ofDisableSmoothing();
	ofNoFill();
	
	//draw contour
    ofSetLineWidth(2);
	for(int i=0; i<contourFinder.blobs.size(); i++) {
		ofBeginShape();
		
		//ofCurveVertex(ofMap(v_arr[i][0].x, 0, 640, 0, resX, false), ofMap(v_arr[i][0].y, 0, 480, 0, resY, false));
		for(int j = blobPts[i]; j>=0; j--) {
            //for (int j=0; j<blobPts[i]; j++) {
			ofSetColor(255, 255, 255);
			ofPoint p;
			p.x = v_arr[i][j].x;
			p.y = v_arr[i][j].y;
			//ofRect(ofMap(p.x, 0, 640, 0, resX, false)-2, ofMap(p.y, 0, 480, 0, resY, false)-2, 4, 4);
			ofCurveVertex(ofMap(p.x, 0, 640, 0, resX, false), ofMap(p.y, 0, 480, 0, resY, false));
		}
		ofCurveVertex(ofMap(v_arr[i][blobPts[i]].x, 0, 640, 0, resX, false), ofMap(v_arr[i][blobPts[i]].y, 0, 480, 0, resY, false));
		ofCurveVertex(ofMap(v_arr[i][blobPts[i]-1].x, 0, 640, 0, resX, false), ofMap(v_arr[i][blobPts[i]-1].y, 0, 480, 0, resY, false));
		
		ofEndShape(true);
	}
	
	px = mouseX;
	py = mouseY;
	
	// DEBUG
	if (bDebug) {
		//drawDebugText(0, 300, 1024, 568);
		drawDebugText(0, 300, 1280, 568);
	}
	
	screenCapture();
	distCapture();
	//cout << "--------------" << endl;
	//cout << "blobPts[]: " << blobPts.size() << endl;
	//cout << "v_arr[]: " << v_arr.size() << endl;
	for(int i=0; i<contourFinder.blobs.size(); i++) {
		//cout << "v_arr[][]: " << v_arr[i].size() << endl;
		v_arr[i].clear();
		//cout << "v_arr[][]: " << v_arr[i].size() << endl;
	}
	blobPts.clear();
	v_arr.clear();
	//cout << "lineStripBlob[]: " << lineStripBlob.size() << endl;
	//cout << "blobPts[]: " << blobPts.size() << endl;
	//cout << "v_arr[]: " << v_arr.size() << endl;
}

//--------------------------------------------------------------
void ballPond::keyPressed(int key){
    
    
	if (key == 'r') {
		resetBalls();
		
	}
    
	if (key == 'd') {
		if (!bDebug) {
			bDebug = true;
		}else {
			bDebug = false;
		}
	}
	if (bDebug) {
		if (key == '1') 
		{
			if (!bShowRGB) 
			{
				bShowRGB = true;
				bShowDepth = false;
				bShowContour = false;
				bShowGray = false;
			}
			else 
			{
				bShowRGB = false;
			}
		}
		
		if (key == '3') {
			if (!bShowDepth) {
				bShowDepth = true;
				bShowRGB = false;
				bShowContour = false;
				bShowGray = false;
			}else {
				bShowDepth = false;
			}
		}
		
		if (key == '4') {
			if (!bShowContour) {
				bShowContour = true;
				bShowDepth = false;
				bShowRGB = false;
				bShowGray = false;
			}else {
				bShowContour = false;
			}
		}
		
		if (key == '2') {
			if (!bShowGray) {
				bShowGray = true;
				bShowDepth = false;
				bShowRGB = false;
				bShowContour = false;
			}else {
				bShowGray = false;
			}
		}
		
		if (key == '5') {
			if (!bShowBlob) {
				bShowBlob = true;
			}else {
				bShowBlob = false;
			}
		}
		//-----
		if (key == OF_KEY_RIGHT)
		{
			farThreshold ++;
			if (farThreshold > 255) farThreshold = 255;
		}
		if (key == OF_KEY_LEFT) 
		{
			farThreshold --;
			if (farThreshold < 0) farThreshold = 0;
		}
		if (key == OF_KEY_UP)
		{
			nearThreshold ++;
			if (nearThreshold > 255) nearThreshold = 255;
		}
		if (key == OF_KEY_DOWN) 
		{
			nearThreshold --;
			if (nearThreshold < 0) nearThreshold = 0;
		}
		if (key == '+') 
		{
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
		}
		if (key == '-') 
		{
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
		}
		if (key == '<') 
		{
			minBlobSize-= 50;
			if(minBlobSize< 0) minBlobSize = 0;
		}
		if (key == '>') 
		{
			minBlobSize+=50;
			//if(minBlobSize > ) minBlobSize = 0;
		}
		
		
	}


}

//--------------------------------------------------------------
void ballPond::keyReleased(int key){

}

//--------------------------------------------------------------
void ballPond::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ballPond::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ballPond::mousePressed(int x, int y, int button){
    
    float r = ofRandom(4, 20);		// a random radius 4px - 20px
	ofxBox2dCircle circle;
	circle.setPhysics(3.0, 0.53, 0.1);
	circle.setup(box2d.getWorld(), mouseX, mouseY, r);
	circles.push_back(circle);
	cout << "mouse x: " << mouseX << " mouse y: " << mouseY << endl;
	//guitar[0].play();

}

//--------------------------------------------------------------
void ballPond::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ballPond::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ballPond::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ballPond::dragEvent(ofDragInfo dragInfo){ 

}

//--------------------------------------------------------------
void ballPond::drawDebugText(int dX, int dY, int dW, int dH)
{
	ofSetColor(0, 0, 0, 127);
	ofFill();
	string debugString;
	debugString = "--- DEBUG MENU ---\n\nFramerate: " + ofToString(ofGetFrameRate(), 1) 
	+ "\nKinect RGB: " + ofToString(bShowRGB) + " (press 1)\n"
	+ "Kinect Gray: " + ofToString(bShowGray) + " (press 2)\n"
	+ "Kinect Depth: " + ofToString(bShowDepth) + " (press 3)\n"
	+ "Kinect Contour: " + ofToString(bShowContour) + " (press 4)\n"
	+ "Kinect Skeleton: " + ofToString(bShowSkeleton) + " (press 5)\n"
	+ "Kinect Blob: " + ofToString(bShowBlob) + " (press 6)\n"
	+ "----------------------------------------\n"
	+ "blobs: " + ofToString(contourFinder.nBlobs) + "\n"
	+ "clearLineStrip: " + ofToString(clearLineStrip) + "\n"
	+ "Blob1: " + ofToString(blob1Size.x) + "\n"
	+ "Blob2: " + ofToString(blob2Size.x) + "\n"
	+ "distBlob: " + ofToString(distBlob) + "\n"
	+ "far threshold: " + ofToString(farThreshold) + " (press LEFT or RIGHT)\n"
	+ "near threshold: " + ofToString(nearThreshold) + " (press DOWN or UP)\n"
	+ "tilt angle: " + ofToString(angle) + " (press - or +)\n"
	+ "Minimum Pixels to be a Blob: " + ofToString(minBlobSize) + " (press < or >)\n"
	+ "position of the first particle: " + ofToString(particles[0].getPosition().x) + " " + ofToString(particles[0].getPosition().y) + "\n"
	+ "----------------------------------------\n"
	;
	
	if (contourFinder.blobs.size() > 0) {	
		int contourPoints = v_arr[0].size();
		debugString += "Blob 1 has: " + ofToString(contourPoints) + " poits"+ "\n";
	}
	
	ofSetRectMode(OF_RECTMODE_CORNER);
	ofRect(dX, dY, dW, dH);
	ofNoFill();
	ofSetRectMode(OF_RECTMODE_CENTER);
	ofSetColor(0, 255, 0);
	verdana.drawString(debugString, dX + 10, dY + 20);
	ofSetRectMode(OF_RECTMODE_CORNER);
}


//--------------------------------------------------------------
void ballPond::resetBalls()
{
	for( int i=0; i <= NUM_PARTICLES; i++ )
	{
		particles[i].setPosition(ofRandom( 0, ofGetWidth() ), ofRandom( 0, ofGetHeight() ));
	}
	
}

//--------------------------------------------------------------
void ballPond::getSizeOf2DArray(vector <ofPoint> nums){ 
	int sizeOne = sizeof(nums)/sizeof(nums[0]); //grš§e des ersten array
	int sizeTwo = sizeof(nums)/sizeof(nums[0][0])/sizeOne; //grš§e der zweiten array
}

//--------------------------------------------------------------
void ballPond::screenCapture(){
    
	if (tempTime > tempFrameRate) {
		tempTime = 0;
		tempFrameRate = int(ofGetFrameRate()*50);
		saveTime = int(ofGetFrameRate()*10);
	}
	
	if (tempTime == 0 ||
		tempTime == saveTime ||
		tempTime == saveTime*2 ||
		tempTime == saveTime*3 ||
		tempTime == saveTime*4 ||
		tempTime == saveTime*5
		){
		
		//myImage.grabScreen(0,0,1024,768);
		myImage.grabScreen(0,0,1280,1024);
		myImage.saveImage("pic/ballpond_"+ofToString(count)+".png");
        
		count++;
	}
	
	tempTime++;
}

//--------------------------------------------------------------
void ballPond::distCapture(){
	
	if (tempTimeSec > tempFrameRateSec) {
		tempTimeSec = 0;
		tempFrameRateSec = int(ofGetFrameRate()*5);
		saveTimeSec = int(ofGetFrameRate()*1);
	}
	
	if (tempTimeSec == 0 ||
		tempTimeSec == saveTimeSec ||
		tempTimeSec == saveTimeSec*2 ||
		tempTimeSec == saveTimeSec*3 ||
		tempTimeSec == saveTimeSec*4 ||
		tempTimeSec == saveTimeSec*5
		){
		
		if (bDistBlob == true) {
			myfile << ofToString( ofGetElapsedTimef(),0) << "    0" << endl;
			//cout << ofToString( ofGetElapsedTimef(),0) << "    0" << endl;
			bDistBlob = false;
		}else {
			myfile << ofToString( ofGetElapsedTimef(),0) << "    " << distBlob << endl;
			//cout << ofToString( ofGetElapsedTimef(),0) << "    " << distBlob << endl;
		}
	}
	
	tempTimeSec++;
}
