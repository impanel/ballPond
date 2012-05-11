/*
 *  ContactListener.cpp
 *  kinectBallPit
 *
 *  Created by impanel on 7/3/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "ContactListener.h"


//--------------------------------------------------------------
void ContactListener::init(vector <Particle> _particles)
{
	particles = _particles;
}


//--------------------------------------------------------------
//extern kinectBallPitApp *myApp;

//void ContactListener::Add(const b2ContactPoint* point , vector <Particle> _particle){
void ContactListener::Add(const b2ContactPoint* point){
	/*
     b2Vec2 p = point->position;
     p *= OFX_BOX2D_SCALE;
     contact_points.push_back(p);
     p = point->velocity;
     contact_velocities.push_back(p);
     */
	
	appPtr = ((ballPond*)ofGetAppPtr());
	//appPtr->box2dContactEventHandler( point );
	
	//box2dContactEventHandler(point, appPtr->particles);
	
	box2dContactEventHandler(point);
}

//--------------------------------------------------------------
void ContactListener::Remove(const b2ContactPoint* point){
	
}


//--------------------------------------------------------------
//void ContactListener::box2dContactEventHandler(const b2ContactPoint* point, vector <Particle> _particle)
void ContactListener::box2dContactEventHandler(const b2ContactPoint* point)
{
	for( int i = 0; i < appPtr->particles.size(); i++ )
	{
		Particle& particle = appPtr->particles[ i ];
        for( b2Shape* s = particle.body->GetShapeList(); s; s = s->GetNext())
        {
            if(point->shape1 == s)
            {
                if (particle.getPosition().y < ofGetHeight()/3) {
                    int randNum1 = ofRandom(0, 7);
                    int randNum2 = ofRandom(0, 16);
                    if (randNum1 == 1) {
                        //appPtr->guitar[randNum2].play();
                    }
                }
            }
        }
    }
}
