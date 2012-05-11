/*
 *  ContactListener.h
 *  kinectBallPit
 *
 *  Created by impanel on 7/3/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "ofxBox2d.h"
#include "ofMain.h"
#include "Particle.h"
#include "ballPond.h"


#ifndef CONTACTLISTENER_H
#define CONTACTLISTENER_H

class ballPond;

class ContactListener : public b2ContactListener
{
public:
	void Add(const b2ContactPoint* point, vector <Particle> _particle);		
	void Add(const b2ContactPoint* point);		
	
	
	//change this so it takes pointers
	void box2dContactEventHandler	(const b2ContactPoint* point, vector <Particle> _particle);
	void box2dContactEventHandler	(const b2ContactPoint* point);
	
	void init(vector <Particle> _particles);
    
	void Remove(const b2ContactPoint* point);
	
	vector <Particle> particles;
	ballPond* appPtr;
	
	vector <b2Vec2> contact_points;
	vector <b2Vec2> contact_velocities;
    
};
#endif