//
//  SensorFuse
//
//  Created by Keith Lim on 3/1/17.
//  small and reasonable edits by Frederik Juutilainen on several dates in january
//
// Models the interactive state of Gates and Sensors and attempts to interperate where Users are, and return that as OSC GPS co-ordinates

#include "ofApp.h"



//--------------------------------------------------------------
void ofApp::setup(){
    ofSetFrameRate(60);
    // listen on the given port
    cout << "listening for osc messages on port " << PORT << "\n";
    receiver.setup( PORT );
    senderVisual.setup( SENDHOST, SENDPORT_VISUAL );
    senderAudio.setup( SENDHOST, SENDPORT_AUDIO );
    receiverFlocking.setup( RECEIVEPORT_FLOCKING );
    senderFlocking.setup( SENDHOST, SENDPORT_FLOCKING );
    
    
    current_msg_string = 0;
    
    // Setup
    ofLogToFile("logs/log_ " + ofToString(ofGetDay()) + "_" + ofToString(ofGetMonth()) + "_" + ofToString(ofGetYear()) +  ".txt", true);
    
    // setup gui
    setupGUI();
    
    // setup world for physics
    world = World2D::create();
    world->setGravity(ofVec2f(0,0));
    world->setWorldSize(ofVec2f(0,0), ofVec2f(INSTALLATION_LENGTH+SPACING_ENDS*2, INSTALLATION_WIDTH+SPACING_SIDE*2));
    world->enableCollision();
    world->setDrag(1);
    
    //fadetime on the message display
    fadeTime = 20.0f;
    
    // Artnet
    ofJson jsonAddresses = ofLoadJson("artnetAddresses.json");
    
    for (auto& address : jsonAddresses) {
        artnetAddrs.push_back(ofToInt(address.get<std::string>()));
    }
    
    
    //init Sensor objects in the artnetAddrs list with artnet name
    int i = 0;
    for(auto& address : artnetAddrs){
        // Create gate
        ofVec2f position = ofVec2f((2.0*i)+SPACING_ENDS, INSTALLATION_WIDTH/2+SPACING_SIDE);
        gates[address] = GateSF(i, ofToString(address),position,&users,&world, &guiParameters, &senderVisual, &soundObjects);
        i++;
    }
    
    // Add soundobjects
    for(int i = 0; i < NUMBER_OF_SOUNDOBJECTS; i++){
        ofVec2f initPos = ofVec2f(ofRandom(0,INSTALLATION_LENGTH+SPACING_ENDS*2), ofRandom(0, INSTALLATION_WIDTH+SPACING_SIDE*2));
        soundObjects.push_back(SoundObject(&world, initPos));
    }
    
    // Create idle particle with attraction for soundobject
    idleParticle = world->makeParticle();
    idleParticle->disableCollision()->setRadius(0.1)->moveTo(ofVec2f(44,7));
    
    for(auto& s : soundObjects){
        world->makeAttraction(idleParticle, s.getParticle(), 0.0f);
    }
    
    
    
    // Create small amount of repulsion to other sound objects
    for (int i = 0; i<soundObjects.size(); i++) {
        for (int j = i+1; j<soundObjects.size(); j++) {
            soundObjects.at(i).repelOtherSoundObject(&soundObjects.at(j));
        }
    }
    
    gateDisplay.resize(NUM_GATE_DISPLAY);
    
    
    // Activity map:
    
    activityMap.setup("activityMap", artnetAddrs);
    guiActivity.setup(activityMap.parameterGroup, "settingsActivity", 20+gui.getWidth(),10);
    
    
    distributionMap.setup("distMap",&activityMap);
    guiDistribution.setup(distributionMap.parameterGroup, "settingsActivity", 30+gui.getWidth()+guiActivity.getWidth(), 10);
    
    
    gui.loadFromFile("settings.xml");
    guiActivity.loadFromFile("settingsActivity.xml");
    guiDistribution.loadFromFile("settingsDistribution.xml");
    
}

//--------------------------------------------------------------
void ofApp::update(){
    // Set title to FrameRate
    std::string title;
    title+= "Wave_SensorFuse - FPS: ";
    title+=ofToString(ofGetFrameRate());
    ofSetWindowTitle(title);
    
    // MSA update for physics simulation
    world->update();
    
    // Move idleParticle
    idleParticle->moveTo(ofVec2f(ofRandom(2,78), ofRandom(1,6)));
    
    // Keep Some speed in SoundObjects
    // estimate some kind of energy measure -> average velocity on x
    float avgVelocity;
    for(auto & soundObject : soundObjects){
        avgVelocity += soundObject.getVelocity().length();
    }
    avgVelocity /= soundObjects.size();
    
    //    float targetAvgVelocity = 15.0f;
    float pFactor = 0.05; // how fast does it change
    
    float deltaVelocity = targetAvgVelocity-avgVelocity;
    float changeFactor = 0;
    
    if(deltaVelocity<0) changeFactor = 1-(abs(deltaVelocity)*pFactor);
    if(deltaVelocity>0) changeFactor = 1+(abs(deltaVelocity)*pFactor);
    
    // Update all users
    for(auto & soundObject : soundObjects){
        soundObject.update(changeFactor, targetAvgVelocity);
    }
    
    // Delete dead users
    vector<User>::iterator it = users.begin();
    while(it != users.end()) {
        (*it).update();
        if((*it).hasTravelledForTooLongNow() || (*it).getTimeSinceLastActivation() > 4.) {
            (*it).killParticles();
            it = users.erase(it);
        }
        else ++it;
    }
    
    //PARSE OSC
    uint64_t elapsedTime = ofGetElapsedTimeMillis();
    while( receiver.hasWaitingMessages() ){
        ofxOscMessage m;
        receiver.getNextMessage( m );
        
        string msg_string;
        msg_string = m.getAddress(); //expect "/BeamBreak/[artnetaddr] [0-1]"
        msgTokens = ofSplitString(msg_string, "/", true); //ignore (leading) empty token = true
        
        
        if(msgTokens[0] == "BeamBreak"){
            //convert artnet string for easy array access
            int artnet = ofToInt(msgTokens[1]);
            //get value of BeamBreak, 0=false, 1=true
            int value = m.getArgAsInt32(0);
            if(value != 1 && value != 0)
            {
                ofLog(OF_LOG_ERROR) << ofGetTimestampString() << " - Faulty value " << value << " received from address: " << artnet;
                value = 0;
            } // no errors if wrong messages are received
            long timeTriggered = ofGetElapsedTimeMillis();
            
            //add trigger value and timestamp to sensor@artnetAddr
            // check if entry exists in map
            map<int,GateSF>::iterator i = gates.find(artnet);
            if (!(i == gates.end())) {
                gates[artnet].activate();
                activityMap.activity[artnet].trigger(elapsedTime);
            }
            
            msg_string += " value=";
            msg_string += ofToString(value);
            msg_string += " time=";
            msg_string += ofToString(ofGetElapsedTimeMillis());
            
        }
    }
    
    if(ofGetElapsedTimeMillis()/25 != oldMillis){
        for(int i = 0; i < soundObjects.size(); i++){
            ofxOscMessage m;
            m.setAddress("/SoundObject/" + ofToString(i));
            ofVec2f pos = soundObjects.at(i).getPosition();
            m.addFloatArg(pos.x - SPACING_ENDS);
            m.addFloatArg(pos.y - (SPACING_SIDE+INSTALLATION_WIDTH/2));
            senderVisual.sendMessage(m);
            senderAudio.sendMessage(m);
        }
        
        for(auto& u : users){
            // send user position
            if(abs(u.getVelocity()) > 0){
                ofxOscMessage m;
                m.setAddress("/User/"+u.getId());
                m.addFloatArg(u.getPosition().x - SPACING_ENDS); // position
                m.addFloatArg(u.getLifespan());
                m.addFloatArg(u.getVelocity());
                senderVisual.sendMessage(m);
            }
        }
        oldMillis = ofGetElapsedTimeMillis()/25;
    }
    
    activityMap.update();
    distributionMap.update();
    
    // simulation
    
    if(simulation && ofRandom(1.)<possibility){
        int addr = artnetAddrs[ofRandom(40)];
        gates[addr].activate();
        activityMap.activity[addr].trigger(elapsedTime);
    }
    
    // RECEIVE OSC FLOCKING
    while( receiverFlocking.hasWaitingMessages() ){
        ofxOscMessage m;
        receiverFlocking.getNextMessage( m );
        
        string msg_string;
        msg_string = m.getAddress();
        msgTokens = ofSplitString(msg_string, "/", true);

        if(msgTokens[0] == "Flocking"){
            if( (m.getNumArgs() == (distributionMap.parameterGroup.size()-1)) ){
                for(int i = 1; i<m.getNumArgs(); i++ ){
                    distributionMap.parameterGroup.getFloat(i) = ofMap(m.getArgAsFloat(i-1), 0, 1., distributionMap.parameterGroup.getFloat(i).getMin(), distributionMap.parameterGroup.getFloat(i).getMax());
                }
                
            } else if( msgTokens.size() == 2){

                if(msgTokens[1] == "numAgents"){
                    distributionMap.numAgents = m.getArgAsFloat(0) < 20 ? m.getArgAsFloat(0) : 20 ;
                }else if(msgTokens[1] == "flowfield"){
                    distributionMap.flowfield = m.getArgAsFloat(0);
                }else if(msgTokens[1] == "flowSpeed"){
                    distributionMap.flowSpeed = m.getArgAsFloat(0);
                }else if(msgTokens[1] == "flowZoom"){
                    distributionMap.flowZoom = m.getArgAsFloat(0);
                }else if(msgTokens[1] == "desiredseparation"){
                    distributionMap.desiredseparation = m.getArgAsFloat(0);
                }else if(msgTokens[1] == "separation"){
                    distributionMap.separation = m.getArgAsFloat(0);
                }else if(msgTokens[1] == "neighbordist"){
                    distributionMap.neighbordist = m.getArgAsFloat(0);
                }else if(msgTokens[1] == "align"){
                    distributionMap.align = m.getArgAsFloat(0);
                }else if(msgTokens[1] == "cohesion"){
                    distributionMap.cohesion = m.getArgAsFloat(0);
                }else if(msgTokens[1] == "confidist"){
                    distributionMap.confidist = m.getArgAsFloat(0);
                }else if(msgTokens[1] == "amtSlow"){
                    distributionMap.amtSlow = m.getArgAsFloat(0);
                }else if(msgTokens[1] == "amtFast"){
                    distributionMap.amtFast = m.getArgAsFloat(0);
                }else if(msgTokens[1] == "amtMid"){
                    distributionMap.amtMid = m.getArgAsFloat(0);
                }else if(msgTokens[1] == "confidingness"){
                    distributionMap.confidingness = m.getArgAsFloat(0);
                }else if(msgTokens[1] == "seekCenter"){
                    distributionMap.seekCenter = m.getArgAsFloat(0);
                }else if(msgTokens[1] == "maxForce"){
                    distributionMap.maxForce = m.getArgAsFloat(0);
                }else if(msgTokens[1] == "maxSpeed"){
                    distributionMap.maxSpeed = m.getArgAsFloat(0);
                }else if(msgTokens[1] == "maxSpeedSum"){
                    distributionMap.maxSpeedSum = m.getArgAsFloat(0);
                }
            }
        }


    }
    
    if(int(ofGetElapsedTimef())%2 == 0 && sendFlag){ // sends all 10 seconds
        sendFlag = false;
        // SEND FLOCKING
        ofxOscMessage m;
        m.setAddress("/Flocking");
        for(int i = 1; i<distributionMap.parameterGroup.size(); i++ ){
            m.addFloatArg(ofMap(distributionMap.parameterGroup.getFloat(i), distributionMap.parameterGroup.getFloat(i).getMin(), distributionMap.parameterGroup.getFloat(i).getMax(), 0., 1.));
        }
        senderFlocking.sendMessage(m);
    }
    if(int(ofGetElapsedTimef())%10 == 1) sendFlag = true;
}


//--------------------------------------------------------------
void ofApp::draw(){
    ofBackground(ofColor::dimGray);
    int scale = 10;
    int x = ofGetWidth()/2;
    int y = guiDistribution.getHeight()+10+(ofGetHeight()-guiDistribution.getHeight()-10)/2;
    
    if(drawFlowField) distributionMap.drawFlowField(x, y,TOTAL_LENGTH*scale, TOTAL_WIDTH*scale);
    
    
    if(drawGatesToggle || drawUsersToggle){
        ofPushMatrix();
        {
            ofTranslate(x, y);
            ofScale(scale,scale);
            ofTranslate(-INSTALLATION_LENGTH/2-SPACING_ENDS, -INSTALLATION_WIDTH/2-SPACING_SIDE);
            
            if(drawGatesToggle){
                for(auto& g : gates){
                    g.second.draw();
                }
            }
            
            if(drawUsersToggle){
                for(auto& u : users){
                    u.draw();
                }
            }
            
            if(drawSoundObjects){
                for(auto& s : soundObjects){
                    s.draw();
                }
            }
            
            ofNoFill();
            ofSetColor(ofColor::antiqueWhite);
            ofDrawRectangle(0, 0, INSTALLATION_LENGTH+SPACING_ENDS*2, INSTALLATION_WIDTH+SPACING_SIDE*2); // Draw borders of world;
        }
        ofPopMatrix();
    }
    
    
    
    // GUI
    if(!hideGui){
        gui.draw();
        guiActivity.draw();
        guiDistribution.draw();
        
        int posX = 40+gui.getWidth()+guiActivity.getWidth()+guiDistribution.getWidth();
        std::string info;
        info+="no. of users:\n";
        info+=ofToString(users.size());
        ofSetColor(ofColor::darkRed);
        ofDrawBitmapStringHighlight(info,posX, 25);
    }
    
    
    if(drawActivityMap) activityMap.draw(x-(INSTALLATION_LENGTH/2+1)*scale, y-(INSTALLATION_WIDTH/2)*scale,(INSTALLATION_LENGTH+2)*scale, INSTALLATION_WIDTH*scale);
    
    if(drawFlock) distributionMap.drawFlock(x, y,TOTAL_LENGTH*scale, TOTAL_WIDTH*scale);
    
    distributionMap.draw(30+gui.getWidth()+guiActivity.getWidth(),15+guiDistribution.getHeight(), guiDistribution.getWidth(),10);
    
    activityMap.publish();
    distributionMap.publish();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    if(key == 'g' || key == 'G'){
        hideGui = !hideGui;
    }
    if(key == 'l'){
        gui.loadFromFile("settings.xml");
        guiActivity.loadFromFile("settingsActivity.xml");
        guiDistribution.loadFromFile("settingsDistribution.xml");
    }
    if(key == 's'){
        gui.saveToFile("settings.xml");
        guiActivity.saveToFile("settingsActivity.xml");
        guiDistribution.saveToFile("settingsDistribution.xml");
    }
    
    // Activate gate sensors based on key
    if(key-48 > 0 && key-48 < gates.size()){
        int address = artnetAddrs.at(key-48);
        gates[address].activate();
        activityMap.activity[address].trigger(ofGetElapsedTimeMillis());
        
    }
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
    
}

//--------------------------------------------------------------
void ofApp::setupGUI(){
    guiParameters.setName("GUI");
    guiParameters.add(targetAvgVelocity.set("targetAvgVelocity", 5.,0.1,20));
    guiParameters.add(timingThreshold.set("timing threshold (ms)", 400,70,4000));
    guiParameters.add(distanceThreshold.set("dist threshold (m)", 2.2,2.0,4.5));
    guiParameters.add(debounceLower.set("debounce lower (ms)",100,20,400));
    guiParameters.add(debounceHigher.set("debounce higher (ms)",200,40,700));
    guiParameters.add(drawGatesToggle.set("draw gates", true));
    guiParameters.add(drawUsersToggle.set("draw users", true));
    guiParameters.add(drawSoundObjects.set("drawSoundObjects", true));
    guiParameters.add(drawActivityMap.set("drawActivityMap users", true));
    guiParameters.add(drawFlowField.set("drawFlowField", true));
    guiParameters.add(drawFlock.set("drawFlock", true));
    guiParameters.add(doUsersAttract.set("doUsersAttract", true));
    guiParameters.add(simulation.set("simulation", true));
    guiParameters.add(possibility.set("possibility",0.1,0,1.));
    gui.setup(guiParameters);
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
    
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){
    
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){
    
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){
    
}

