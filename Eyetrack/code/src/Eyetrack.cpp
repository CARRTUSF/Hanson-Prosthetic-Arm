// %BANNER_BEGIN%
// ---------------------------------------------------------------------
// %COPYRIGHT_BEGIN%
//
// Copyright (c) 2018 Magic Leap, Inc. All Rights Reserved.
// Use of this file is governed by the Creator Agreement, located
// here: https://id.magicleap.com/creator-terms
//
// %COPYRIGHT_END%
// ---------------------------------------------------------------------
// %BANNER_END%

// %SRC_VERSION%: 1
#include <Eyetrack.h>
#include <ctime>
#include <cstdio>
#include <cstdlib>
#include <vector>


using namespace lumin;
using namespace lumin::ui;
using namespace glm;
using namespace std;

namespace {
	Text2dNode* Distance;
	Text2dNode* InitialTarget;
	Text2dNode* FinalTarget;
	Text2dNode* date;

	Node* Laser;
	Node* TargetingModel;
	Node* Target1;
	Node* Target2;

	TransformNode* transformNodeInitialTarget;
	TransformNode* transformNodeFinalTarget;
	TransformNode* transformNodeDistance;
	TransformNode* transformNodeLaser;
	TransformNode* transformNodeTargetingModel;
	TransformNode* transformNodeTarget1;
	TransformNode* transformNodeTarget2;
	TransformNode* nodeDate;

	clock_t start;
	double duration;


	vec3 datePos(0.0f, 0.0f, 0.0f);
	quat dateRot(0, 0, 0, 0);
	vec3 laserPosition(0.0f, 0.0f, -1.0f); //used to Hold Eye Position
	vec3 textPositionT1(0.0f, 0.0f, 0.0f);
	vec3 textPositionT2(0.0f, 0.0f, 0.0f);
	vec3 textPositionDist(0.0f, 0.0f, 0.0f);
	vec3 NewValueEyePosition(0.0f, 0.0f, 0.0f);
	vec3 OldValueEyePosition(0.0f, 0.0f, 0.0f);
	vec3 AvgValueEyePosition(0.0f, 0.0f, 0.0f);
	vec3 headposePosition(0.0f, 0.0f, 0.0f);
	vec3 headposeUpVector(0.0f, 0.0f, 0.0f);
	vec3 headposeForwardVector(0.0, 0.0, 0.0);
	vec3 direction(0.0f, 0.0f, 0.0f);
	quat laserRotation(0.0f, 0.0f, 0.0f, 0.0f); //used to Hold Eye Rotation
	quat textRotationTrue(0.0f, 0.0f, 0.0f, 0.0f); //used for Text Rotation
	quat headRotationTrue(0.0f, 0.0f, 0.0f, 0.0f); //used to Hold Head Rotation
	quat headRotationMod(0.0f, 0.0f, 0.0f, 0.0f);
	vec3 tmp (0.0f, 0.0f, 0.0f);
	EyeTrackingEventData* result;
	bool updated = false;
	int state = 0;
	glm::vec3 posA, posB, eyeFixation, eyePosition, textPositionTrue, EyeFixation_prismPosition, FilteredPosition, Fixation, filteredFixation;
	glm::quat eyeRotation, textRotationT1mod, textRotationT2mod, textRotationDistmod;
	glm::mat4 prismWorldTransform;

	double InitialX;
	double InitialY;
	double InitialZ;
	double FinalX;
	double FinalY;
	double FinalZ;
	double leftconfidence;
	double rightconfidence;
	float len;

	float StoredX[15] = { 0.0 };
	float StoredY[15] = { 0.0 };
	float StoredZ[15] = { 0.0 };
	vector <vec3> buffer;
	float SumX = 0.0;
	float SumY = 0.0;
	float SumZ = 0.0;
	float fx;
	float fy;
	float fz;

}


Eyetrack::Eyetrack() {
	ML_LOG(Debug, "Eyetrack Constructor.");

	// Place your constructor implementation here.
}

Eyetrack::~Eyetrack() {
	ML_LOG(Debug, "Eyetrack Destructor.");

	// Place your destructor implementation here.
}

const glm::vec3 Eyetrack::getInitialPrismSize() const {
	return glm::vec3(10.0f, 10.0f, 10.0f);
}

void Eyetrack::createInitialPrism() {
	prism_ = requestNewPrism(getInitialPrismSize());
	if (!prism_) {
		ML_LOG(Error, "Eyetrack Error creating default prism.");
		abort();
	}
	prismSceneManager_ = new PrismSceneManager(prism_);
}

int Eyetrack::init() {

	ML_LOG(Debug, "Eyetrack Initializing.");

	createInitialPrism();
	lumin::ui::Cursor::SetScale(prism_, 0.03f);
	spawnInitialScenes();
	// Place your initialization here.

	// Eye Laser initialization
	Laser = prism_->findNode("Laser", prism_->getRootNode());
	transformNodeLaser = static_cast<TransformNode*>(Laser);
	transformNodeLaser->moveTo(laserPosition, 0.0f, 0);
	transformNodeLaser->setVisible(false);



	// Eye Tracking initialization
	eyetrackingRetainer_ = prism_->retainEyeTrackingUpdates();
	Distance = static_cast<Text2dNode*>(prism_->findNode("Distance", prism_->getRootNode()));
	InitialTarget = static_cast<Text2dNode*>(prism_->findNode("InitialTarget", prism_->getRootNode()));
	FinalTarget = static_cast<Text2dNode*>(prism_->findNode("FinalTarget", prism_->getRootNode()));
	TargetingModel = prism_->findNode("TargetingModel", prism_->getRootNode());
	Target1 = prism_->findNode("Target1", prism_->getRootNode());
	Target2 = prism_->findNode("Target2", prism_->getRootNode());
	nodeDate = static_cast<Text2dNode*>(prism_->findNode("date", prism_->getRootNode()));

	transformNodeInitialTarget = static_cast<lumin::TransformNode*>(InitialTarget);
	transformNodeFinalTarget = static_cast<lumin::TransformNode*>(FinalTarget);
	transformNodeDistance = static_cast<lumin::TransformNode*>(Distance);
	transformNodeTargetingModel = static_cast<lumin::TransformNode*>(TargetingModel);
	transformNodeTarget1 = static_cast<lumin::TransformNode*>(Target1);
	transformNodeTarget2 = static_cast<lumin::TransformNode*>(Target2);
	//date->setText("dd/mm/yyyy");
	//nodeDate->setVisible(true);
	transformNodeTargetingModel->setVisible(true);
	transformNodeFinalTarget->setVisible(false);
	transformNodeDistance->setVisible(false);
	transformNodeTarget1->setVisible(false);
	transformNodeTarget2->setVisible(false);

	return 0;
}

int Eyetrack::deInit() {
	ML_LOG(Debug, "Eyetrack Deinitializing.");

	// Place your deinitialization here.

	return 0;
}

void Eyetrack::spawnInitialScenes() {

	// Iterate over all the exported scenes
	for (auto& exportedSceneEntry : scenes::externalScenes) {

		// If this scene was marked to be instanced at app initialization, do it
		const SceneDescriptor &sd = exportedSceneEntry.second;
		if (sd.getInitiallySpawned()) {
			lumin::Node* const spawnedRoot = prismSceneManager_->spawn(sd);
			if (spawnedRoot) {
				if (!prism_->getRootNode()->addChild(spawnedRoot)) {
					ML_LOG(Error, "Eyetrack Failed to add spawnedRoot to the prism root node");
					abort();
				}
			}
		}
	}
}

bool Eyetrack::updateLoop(float fDelta) {

	// Place your update here.

	glm::mat4 prism_inverse_matrix = glm::inverse(prism_->getTransform());

	//if (updated) {

	prismWorldTransform = prism_->getTransform();
	vec3 fwd(0.0f, 0.0f, 1.0f);

	textRotationTrue = findRotation(-headposeForwardVector, headposeUpVector);

	//Text Stuff - Stable
	textRotationTrue = glm::inverse(prism_->getRotation()) * textRotationTrue;
	vec3 direction2 = textRotationTrue * fwd;
	textPositionT1 = prism_inverse_matrix * glm::vec4(headposePosition, 1.0f);
	textPositionT1 = textPositionT1 - direction2 * 0.5f;
	textPositionT1.x -= .05;

	textRotationT1mod = textRotationTrue;
	textRotationT1mod.z = 0;

	transformNodeInitialTarget->setLocalPosition(textPositionT1);
	transformNodeInitialTarget->setLocalRotation(textRotationT1mod);



	textPositionT2 = prism_inverse_matrix * glm::vec4(headposePosition, 1.0f);// *getHeadposeWorldPosition();
	textPositionT2 = textPositionT2 - direction2 * 0.5f;
	textPositionT2.y += float(0.05);
	textPositionT2.x -= .05;
	textRotationT2mod = textRotationTrue;
	textRotationT2mod.z = 0;

	transformNodeFinalTarget->setLocalPosition(textPositionT2);
	transformNodeFinalTarget->setLocalRotation(textRotationT2mod);


	textPositionDist = prism_inverse_matrix * glm::vec4(headposePosition, 1.0f);
	textPositionDist = textPositionDist - direction2 * 0.5f;
	textPositionDist.y += float(0.075);
	textPositionDist.x -= .05;
	textRotationDistmod = textRotationTrue;
	textRotationDistmod.z = 0;

	transformNodeDistance->setLocalPosition(textPositionDist);
	transformNodeDistance->setLocalRotation(textRotationDistmod);

	//eyetracking stuff

	headRotationMod.w = headRotationTrue.w;
	headRotationMod.x = headRotationTrue.x;
	headRotationMod.y = headRotationTrue.y;
	headRotationMod.z = 0;


	EyeFixation_prismPosition = glm::vec3(glm::inverse(prismWorldTransform) * glm::vec4(eyeFixation, 1)); //calculation of eye gaze position

	//Filter Function
	
	if (buffer.size() == 5) {
		for (int i = 0; i < 5; i++) {
			tmp += buffer[i]/5;
		}
		filteredFixation = tmp;
		tmp = vec3(0.0f, 0.0f, 0.0f);
		buffer.resize(0);
	}
	else {
		buffer.push_back(EyeFixation_prismPosition);
	}


	//transformNodeTargetingModel->setLocalPosition(EyeFixation_prismPosition);
	transformNodeTargetingModel->setLocalPosition(filteredFixation);
	transformNodeTargetingModel->setLocalRotation(headRotationMod); //Make Text Face the user


	if (state == 0) {

		InitialTarget->setText("Press the trigger to begin targetting");
	}
	
	if (state == 1) {
		//update the T1 position while in scan mode
		posA = EyeFixation_prismPosition;
		InitialTarget->setText("T1 Coord Scanning... (" + std::to_string(posA.x) + "," + std::to_string(posA.y) + "," + std::to_string(posA.z) + ")");
	}
	
	if (state == 2) {
		//update the T2 position while in scan mode
		posB = EyeFixation_prismPosition;
		FinalTarget->setText("T2 Coord Scanning... (" + std::to_string(posB.x) + "," + std::to_string(posB.y) + "," + std::to_string(posB.z) + ")");
	}
	
	if (state == 3) {
		//check what's up with the distance, it looks weird
		len = length(posA - posB);
		Distance->setText("Distance: " + std::to_string(len) + " meters");

		ifstream myfilein(BaseApp::getWritablePath() + "CarloMLTrack.txt");
		string line, text;
		text = "";

	}

	// Return true for your app to continue running, false to terminate the app.
	return true;
}

bool Eyetrack::eventListener(lumin::ServerEvent* event) {

	// Place your event handling here.

	if (event->isInputEventType()) {

		// Checking for Button Taps
		bool goState = false;
		bool run = false;
		InputEventData* inputEventData = static_cast<InputEventData*>(event);
		KeyInputEventData* keyEventData = static_cast<KeyInputEventData*>(inputEventData);


		if (keyEventData->keyCode() == input::KeyCodes::AKEYCODE_EX_TRIGGER) {
			//prismWorldTransform = prism_->getTransform();
			//EyeFixation_prismPosition = glm::vec3(glm::inverse(prismWorldTransform) * glm::vec4(eyeFixation, 1));
			if (inputEventData->getEventType() == input::EventType::EVENT_KEY_UP) {
				
				if (state == 0) {
					state = 1;
					return false;
				}

				if (state == 1) {

					state = 2;
					transformNodeTargetingModel->setVisible(true);
					transformNodeTarget1->setVisible(true);
					transformNodeTarget1->setLocalPosition(posA);
					transformNodeTarget1->setLocalRotation(headRotationMod);
					InitialTarget->setText("T1 Coord Locked!! (" + std::to_string(posA.x) + "," + std::to_string(posA.y) + "," + std::to_string(posA.z) + ")");
					transformNodeFinalTarget->setVisible(true);
					return false;
				}

				if (state == 2) {

					state = 3;
					transformNodeTargetingModel->setVisible(true);
					transformNodeTarget2->setVisible(true);
					transformNodeTarget2->setLocalPosition(EyeFixation_prismPosition);
					transformNodeTarget2->setLocalRotation(headRotationMod);
					posB = EyeFixation_prismPosition;
					FinalTarget->setText("T2 Coord Locked!! (" + std::to_string(posB.x) + "," + std::to_string(posB.y) + "," + std::to_string(posB.z) + ")");
					transformNodeDistance->setVisible(true);



					return false;
				
				}

				if (state == 3) {

					state = 4;
					transformNodeTargetingModel->setVisible(false);
					transformNodeInitialTarget->setVisible(true);
					transformNodeFinalTarget->setVisible(false);
					transformNodeDistance->setVisible(true);
					transformNodeTarget1->setVisible(false);
					transformNodeTarget2->setVisible(false);
					Distance->setText("RE-AQUIRING TARGET");
					InitialTarget->setText("Press Trigger to begin");

					ofstream myfileout(BaseApp::getWritablePath() + "CarloMLTrack.txt");
					myfileout << InitialTarget->getText() + "\n";
					myfileout << FinalTarget->getText() + "\n";
					myfileout.close();

					return false;

				}

				if (state == 4) {

					state = 1;
					transformNodeInitialTarget->setVisible(true);
					transformNodeTargetingModel->setVisible(true);
					return false;
				
				}



				//switch (state) {
				//case 1: // State = 1
				//	posA = EyeFixation_prismPosition;
				//	InitialTarget->setText("T1 Coord Scanning... (" + std::to_string(posA.x) + "," + std::to_string(posA.y) + "," + std::to_string(posA.z) + ")");
				//	break;
				//case 2: // State = 2
				//	InitialTarget->setText("T1 Coord Locked!! (" + std::to_string(posA.x) + "," + std::to_string(posA.y) + "," + std::to_string(posA.z) + ")");
				//	FinalTarget->setText("Press trigger to begin T2 scanning");
				//	FinalTarget->setVisible(true);
				//	break;
				//case 3:
				//	state = 4;
				//	FinalTarget->setText("T2 Coord Scanning... (" + std::to_string(posB.x) + "," + std::to_string(posB.y) + "," + std::to_string(posB.z) + ")");
				//	Distance->setVisible(true);
				//	break;
				//case 4:
				//	state = 5;
				//	FinalTarget->setText("T2 Coord Locked!! (" + std::to_string(posB.x) + "," + std::to_string(posB.y) + "," + std::to_string(posB.z) + ")");
				//	Distance->setText("Distance: " + std::to_string(len) + " meters");
				//	break;
				//case 5:
				//	state = 2;
				//	Distance->setText("RE-AQUIRING TARGETS");
				//	FinalTarget->setVisible(false);
				//	InitialTarget->setText("T1 Coord Tommy Scanning... (" + std::to_string(posA.x) + "," + std::to_string(posA.y) + "," + std::to_string(posA.z) + ")");
				//	break;

				//}
			}

		}

	}
	// Get EyeTracking Data

	else {
		result = static_cast<EyeTrackingEventData*>(event);

		leftconfidence = result->getEyeTrackingLeftEyeConfidence();
		rightconfidence = result->getEyeTrackingRightEyeConfidence();
		eyeFixation = result->getEyeTrackingFixationPosition();

		eyePosition = result->getEyeTrackingRightEyePosition();
		eyeRotation = result->getEyeTrackingRightEyeRotation();

		laserPosition = result->getEyeTrackingLeftEyePosition();
		laserRotation = result->getEyeTrackingLeftEyeRotation();

		textPositionTrue = result->getEyeTrackingLeftEyePosition();
		textRotationTrue = result->getEyeTrackingLeftEyeRotation();

		headRotationTrue = result->getEyeTrackingRightEyeRotation();

		headposePosition = getHeadposeWorldPosition();
		headposeUpVector = getHeadposeWorldUpVector();
		headposeForwardVector = getHeadposeWorldForwardVector();

	}

	// Return true if the event is consumed.
	return false;
}


glm::quat Eyetrack::findRotation(const glm::vec3& direction, const glm::vec3& up) {

	glm::vec3 col1, col2, col3;


	vec3 d = normalize(direction);
	glm::vec3 x = cross(up, d);
	x = normalize(x);
	glm::vec3 y = cross(d, x);
	y = normalize(y);

	float qw = sqrt(1 + x.x + y.y + d.z) / 2;
	float qx = (d.y - y.z) / (4 * qw);
	float qy = (x.z - d.x) / (4 * qw);
	float qz = (y.x - x.y) / (4 * qw);

	return glm::quat(qw, -qx, -qy, qz);
}