//
//  DistributionGen.h
//  SensorFuse
//
//  Created by Jonas Fehr on 19/02/2018.
//

#pragma once

#include "ActivityMap.h"
#include "defines.h"

class Mover{
public:
    glm::vec2 pos;
    glm::vec2 vel;
    glm::vec2 accel;
    
    Mover(){
        pos = glm::vec2(ofRandom(TOTAL_LENGTH),ofRandom(TOTAL_WIDTH));
        vel = glm::vec2(ofRandom(-1, 1),ofRandom(-1, 1));//ofRandom(-1, 1);
    }
    
    void addAccel(glm::vec2 accel){
        this->accel += accel;
        
        
    }
    
    void update(float maxSpeed, float yReduction){
        vel += accel;
        accel = glm::vec2(0.);
        vel = glm::clamp(vel, glm::vec2(-maxSpeed), glm::vec2(maxSpeed));
        
        //        if( (pos.y > 5. && vel.y > 0) || (pos.y < -5. && vel.y < 0) ){
        //            vel.y *= -0.9;
        //            if(pos.y >  5.0) pos.y = 5.;
        //            if(pos.y < -5.) pos.y -= -5;
        ////            cout << "pos: " << pos << " vel: " << vel << endl;
        //        }
        
        pos += vel*ofGetLastFrameTime();
        pos.y *=yReduction;
        if(pos.x > HALF_TOTAL_LENGTH) pos.x =  -HALF_TOTAL_LENGTH;
        if(pos.x <  -HALF_TOTAL_LENGTH) pos.x = HALF_TOTAL_LENGTH;
        
        
        if(pos.y >  TOTAL_WIDTH/2) pos.y =  -TOTAL_WIDTH/2;
        if(pos.y < -TOTAL_WIDTH/2) pos.y =   TOTAL_WIDTH/2;
        
        
    }
    
};

#define MAXMOVERS 20

class DistributionMap{
public:
    ActivityMap * activityMap;
    
    vector<Mover> movers;
    
    string name;
    
    ofFloatImage imgDistribution;
    ofxSyphonServer syphonOut;
    
    ofParameterGroup parameterGroup;
    ofParameter<bool> deactivate;
    ofParameter<float> numAgents;
    
    ofParameter<float> flowfield;
    ofParameter<float> flowSpeed;
    ofParameter<float> flowZoom;
    
    
    ofParameter<float> neighbordist;
    
    ofParameter<float> cohesion;
    ofParameter<float> align;
    ofParameter<float> desiredseparation;
    ofParameter<float> separation;
    ofParameter<float> confidist;
    ofParameter<float> confidingness;
    ofParameter<float> amtSlow;
    ofParameter<float> amtMid;
    ofParameter<float> amtFast;
    ofParameter<float> seekCenter;
    
    
    ofParameter<float> maxForce;
    ofParameter<float> maxSpeed;
    ofParameter<float> maxSpeedSum;

    float flowTime = 0.;
    ofFloatImage flowImg;
    
    
    DistributionMap(){};
    
    void setup(string name, ActivityMap * actMap){
        this->name = name;
        activityMap = actMap;
        
        for(int i = 0; i < 20; i++){
            Mover mover;
            movers.push_back(mover);
        }
        
        syphonOut.setName("DistributionMap");
        
        setupParameterGroup(name);
        
        imgDistribution.allocate(MAXMOVERS, 1, OF_IMAGE_COLOR_ALPHA);
        imgDistribution.setColor(ofFloatColor(0.,0.));
        
        flowImg.allocate(TOTAL_LENGTH, TOTAL_WIDTH, OF_IMAGE_COLOR_ALPHA);
        
    }
    
    void update(){
        imgDistribution.setColor(ofFloatColor(0.,0.));
        int i = 0;
        for( auto & m : movers){
            if(i<numAgents){
                // FLOCKING
                glm::vec2 center = glm::vec2(0.);
                glm::vec2 sumVel = glm::vec2(0.);
                glm::vec2 sumSep = glm::vec2(0.);
                int count = 0;
                int countSeparate = 0;
                
                
                
                for ( int iM = 0; iM < movers.size(); iM++){
                    
                    float d = distance(m.pos,movers.at(iM).pos);
                    
                    if ((d > 0) && (d < neighbordist)) {
                        center += movers.at(iM).pos;
                        sumVel += movers.at(iM).vel;
                        count+=1;
                    }
                    if ((d > 0) && (d < desiredseparation)) {
                        glm::vec2 diff = m.pos-movers.at(iM).pos;
                        diff = normalize(diff);
                        diff /= d;
                        sumSep += diff;
                        countSeparate+=1;
                    }
                }
                
                // SEPARATE
                if (countSeparate > 0) {
                    sumSep /= countSeparate;
                    sumSep = normalize(sumSep);
                    sumSep *=maxSpeed;
                    
                    glm::vec2 steer = sumSep - m.vel;
                    steer = glm::clamp(steer, glm::vec2(-maxForce), glm::vec2(maxForce));
                    m.addAccel(steer*float(separation));
                }
                
                if (count > 0) {
                    
                    // ALIGN
                    sumVel /= count;
                    sumVel = normalize(sumVel);
                    sumVel *=maxSpeed;
                    
                    glm::vec2 steer = sumVel - m.vel;
                    steer = glm::clamp(steer, glm::vec2(-maxForce), glm::vec2(maxForce));
                    m.addAccel(steer*float(align));
                    
                    // COHESION
                    center /= count;
                    glm::vec2 desired = center-m.pos;
                    desired = normalize(desired);
                    desired *= maxSpeed;
                    
                    steer = desired-m.vel;
                    steer = glm::clamp(steer, glm::vec2(-maxForce), glm::vec2(maxForce));
                    m.addAccel(steer*float(cohesion));
                }
                
                // CONFINDINGNESS
                
                count = 0;
                glm::vec2 sumConf = glm::vec2(0.);
                int aPos = 0;
                
                for (auto & a : activityMap->activity) {
                    //            for(int indx = 0; indx < activityMap->activity.size(); indx++){
                    //                ActivityAccumulator a = activityMap->getActivity(indx);
                    //                float weight = 0.;
                    //                if(glm::round(m.pos.x) != indx) weight =  1./float(glm::round(m.pos.x)-indx);
                    //              cout << "index: " << indx<< " weight: "<<  weight << endl;
                    float posActivity = (aPos*2.)-INSTALLATION_LENGTH/2;
                    
                    float d = distance(m.pos, glm::vec2(posActivity, 0.));
                    aPos++;
                    
                    float amount = (a.second.val_slow*amtSlow + a.second.val_mid*amtMid + a.second.val_fast*amtFast)/(amtSlow+amtMid+amtFast);
                    
                    if ((d > 0) && (d < confidist) && amount > 0.1) {
                        
                        glm::vec2 desired = glm::vec2(posActivity, 0.)-m.pos;
                        desired = normalize(desired);
                        desired *= amount;
                        
                        sumConf += desired;
                        count++;
                    }
                    //                m.addAccel(glm::vec2(-weight*amount/numOfActivities, 0.));
                }
                if (count > 0) {
                    // CONFINDINGNESS
                    sumConf /= count;
                    sumConf = normalize(sumConf);
                    sumConf *= maxSpeed;
                    
                    glm::vec2 steer = sumConf-m.vel;
                    steer = glm::clamp(steer, glm::vec2(-maxForce), glm::vec2(maxForce));
                    m.addAccel(steer*float(confidingness));
                }
            }
            
            // ADD NOISE
            flowTime += flowSpeed/1000.;
            float noiseFieldX = ofNoise((m.pos.x*flowZoom), (m.pos.y*flowZoom), flowTime)-0.5;
            float noiseFieldY = ofNoise((m.pos.x*flowZoom), (m.pos.y*flowZoom), flowTime + 132345)-0.5;
            
            glm::vec2 noiseField = normalize(glm::vec2(noiseFieldX,noiseFieldY));
            
            // force towards center
            float d = distance(m.pos, glm::vec2(m.pos.x, 0.));
            glm::vec2 diff = glm::vec2(m.pos.x, 0.)-m.pos;
            diff = normalize(diff);
            diff *= d*d/float(HALF_TOTAL_WIDTH*HALF_TOTAL_WIDTH);
            
            noiseField.y = abs(noiseField.y)*diff.y;
            
            noiseField *= maxSpeed;
            
            
            glm::vec2 steer = noiseField-m.vel;
            steer = glm::clamp(steer, glm::vec2(-maxForce), glm::vec2(maxForce));
            m.addAccel(steer*float(flowfield));
            
            
            
            // seekCenter
            
            //            float d = distance(m.pos, glm::vec2(m.pos.x, 0.));
            //            glm::vec2 diff = glm::vec2(m.pos.x, 0.)-m.pos;
            //            diff = normalize(diff);
            //            diff *= d*d/25.;
            //            steer = glm::vec2(0.);
            //            steer.y = diff.y-m.vel.y;
            //            steer = glm::clamp(steer, glm::vec2(-maxForce), glm::vec2(maxForce));
            //            m.addAccel(steer*float(seekCenter));
            //            cout << d << ", " << steer*float(seekCenter) << endl;
            
            i++;
        }
        
        i = 0;
        for( auto & m : movers){
            if(i<numAgents){
                m.update(maxSpeedSum, 1.-seekCenter);
                float isActive = (i<numAgents)? 1. : 0.;
                imgDistribution.setColor(i, 0, ofFloatColor(m.pos.x/TOTAL_LENGTH+0.5, m.pos.y/TOTAL_WIDTH+0.5, m.vel.x, isActive));
            }
            i++;
            
        }
        
        if(deactivate) imgDistribution.setColor(ofFloatColor(0.,0.));
        imgDistribution.update();
        
    }
    
    void publish(){
        // PUBLISH OUTPUT
        ofSetColor(255);
        ofFill();
        syphonOut.publishTexture(&imgDistribution.getTexture());
    }
    
    ofTexture& getTexture(){ return imgDistribution.getTexture();}
    
    void drawFlowField(int x, int y, int w, int h){
            flowImg.setColor(ofFloatColor(0.,0.));
            
            for(int xI = -HALF_TOTAL_LENGTH; xI < HALF_TOTAL_LENGTH; xI++){
                for(int yI = -HALF_TOTAL_WIDTH; yI < HALF_TOTAL_WIDTH; yI++){
                    float noiseFieldX = ofNoise((xI*flowZoom), (yI*flowZoom), flowTime)-0.5;
                    float noiseFieldY = ofNoise((xI*flowZoom), (yI*flowZoom), flowTime + 132345)-0.5;
                    
                    glm::vec2 noiseField = normalize(glm::vec2(noiseFieldX,noiseFieldY));
                    
                    // force towards center
                    float d = distance(glm::vec2(xI,yI), glm::vec2(xI, 0.));
                    glm::vec2 diff = glm::vec2(xI, 0.)-glm::vec2(xI,yI);
                    diff = normalize(diff);
                    diff *= d*d/25.;
                    
                    noiseField.y = abs(noiseField.y)*diff.y;
                    noiseField *= maxSpeed;
                    flowImg.setColor(xI+HALF_TOTAL_LENGTH, yI+HALF_TOTAL_WIDTH, ofFloatColor(noiseFieldX+0.5, 0., noiseFieldY+0.5, .3));
                }
            }
            flowImg.update();
            flowImg.draw(x-w/2, y-h/2, w, h);
    }
    
    void drawFlock(int x, int y, int w, int h){
        
        ofSetColor(255);
        ofFill();
        ofSetColor(ofColor::blue);
        
        ofPushMatrix();
        {
            ofTranslate(x, y);
            int i = 0;
            for(auto & m: movers){
                if(i<numAgents) ofDrawCircle(glm::vec2(m.pos.x/TOTAL_LENGTH*w,(m.pos.y/TOTAL_WIDTH)*h), 3);
                i++;
            }
        }
        ofPopMatrix();
    }
    
    void draw(int x, int y, int w, int h){
        
        ofSetColor(255);
        imgDistribution.draw(x,y,w,h);
    }
    
    void setupParameterGroup(string name){
        
        parameterGroup.setName(name);
        parameterGroup.add(deactivate.set("deactivate", false));
        parameterGroup.add(numAgents.set("numAgents", 5, 1, MAXMOVERS));
        
        parameterGroup.add(flowfield.set("flowfield", 0.1, 0., 1.));
        parameterGroup.add(flowSpeed.set("flowSpeed", 0.01, 0., 1.));
        parameterGroup.add(flowZoom.set("flowZoom", 0.1, 0., 1.));
        
        
        parameterGroup.add(desiredseparation.set("desiredseparation", 2, 0., 40));
        parameterGroup.add(separation.set("separation", 0.8, 0., 2.));
        parameterGroup.add(neighbordist.set("neighbordist", 20, 0., 40.));
        parameterGroup.add(align.set("align", 0.8, 0., 2.));
        parameterGroup.add(cohesion.set("cohesion", 0.8, 0., 2.));
        parameterGroup.add(confidist.set("confidist", 10, 0., 40.));
        parameterGroup.add(amtSlow.set("amtSlow", 1, 0., 1.));
        parameterGroup.add(amtMid.set("amtMid", 1, 0., 1.));
        parameterGroup.add(amtFast.set("amtFast", 1, 0., 1.));
        parameterGroup.add(confidingness.set("confidingness", 0.8, -2., 2.));
        parameterGroup.add(seekCenter.set("seekCenter", 0.8, 0., 1.));
        
        parameterGroup.add(maxForce.set("maxForce", 0.1, 0., 5.));
        parameterGroup.add(maxSpeed.set("maxSpeed", 0.1, 0., 5.));
        parameterGroup.add(maxSpeedSum.set("maxSpeedSum", 0.1, 0., 5.));

    }
    
    ofParameterGroup& getParameterGroup(){ return parameterGroup; }
    string& getName(){ return name; }
    
};

