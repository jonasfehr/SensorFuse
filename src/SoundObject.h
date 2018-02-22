//
//  SoundObjects.h
//  WAVE_SensorFuse
//
//  Created by Frederik Tollund Juutilainen on 26/01/2017.
//
//

#ifndef SoundObjects_h
#define SoundObjects_h

#include "MsaPhysics2D.h"
#include <limits>

using namespace msa::physics;

class SoundObject {
public:
    SoundObject(World2D_ptr* world, ofVec2f initPos){
        this->world = world;
        this->particle = (*world)->makeParticle();
        this->particle->setRadius(0.3);
        this->particle->moveTo(initPos);
        this->particle->disableCollision();
        this->particle->setMass(10);
        this->particle->addVelocity(ofVec2f(ofRandom(-1., 1.),ofRandom(-1., 1.)));
    }
    
    void draw(){
        ofSetColor(ofColor::crimson);
        ofFill();
        ofDrawCircle(particle->getPosition(), particle->getRadius());

    }
    
    void update(float changeFactor, float targetAvgVelocity){
        ofVec2f velocity = getVelocity();
        velocity.operator*=(ofVec2f(changeFactor));
        velocity.operator*=(ofVec2f(1.01,.99)); // make them move sideways
        float maxSpeed = targetAvgVelocity*3;
        float minSpeed = targetAvgVelocity/2;
        if(velocity.length() > maxSpeed){ // restrict maximum speed
            velocity = velocity.getNormalized().operator*=(maxSpeed);
        }else if(velocity.length() < minSpeed){ // make them move
            velocity += ofVec2f(ofRandom(-.1, 0.1),ofRandom(-0.1, 0.1));
            velocity = velocity.getNormalized().operator*=(minSpeed);
        }
        setVelocity(velocity);
    }
    
    Attraction2D_ptr createAttraction(Particle2D_ptr* attractor){
        Attraction2D_ptr attraction = (*world)->makeAttraction(this->particle, (*attractor), attractionStrength);
        // set constrains for attraction
        attraction->setMaxDistance(maxDistAttraction);
        attraction->setMinDistance(minDistAttraction);
        return attraction;
    }
    
    void repelOtherSoundObject(SoundObject* repeller){
        Attraction2D_ptr repulsion = (*world)->makeAttraction(this->particle, repeller->particle, repulsionStrength);
        repulsion->setMaxDistance(maxDistRepulsion);
        repulsion->setMinDistance(minDistRepulsion);
    }
    
    void setOccupied(bool b){
        occupied = b;
    }
    
    bool isOccupied(){
        return occupied;
    }
    
    ofVec2f getPosition(){
        return this->particle->getPosition();
    }
    
    ofVec2f getVelocity(){
        return this->particle->getVelocity()*ofGetFrameRate();
    }

    void setVelocity(ofVec2f vel){
        return this->particle->setVelocity(vel/ofGetFrameRate());
    }
    
    Particle2D_ptr getParticle(){
        return particle;
    }
    
private:
    Particle2D_ptr particle;
    float attractionStrength = 0.1;
    float repulsionStrength = -0.005;
    float distThreshold = 30.0;
    float maxDistAttraction = 20.0f;
    float minDistAttraction = 0.5f;
    float maxDistRepulsion = 5.0f;
    float minDistRepulsion = 0.5f;
    World2D_ptr* world;
    std::string getPositionString(){
        stringstream stream;
        stream << fixed << setprecision(0) << this->particle->getPosition().x << ", " << this->particle->getPosition().y;
        return stream.str();
    }
    
    bool occupied = false;
};


#endif /* SoundObjects_h */
