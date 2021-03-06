// -- WARNING -- WARNING -- WARNING -- WARNING -- WARNING -- WARNING --
//
// THE CONTENTS OF THIS FILE IS GENERATED BY CODE AND
// ANY MODIFICATIONS WILL BE OVERWRITTEN
//
// -- WARNING -- WARNING -- WARNING -- WARNING -- WARNING -- WARNING --

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

#pragma once

#include <SpawnedSceneBase.h>
#include <SpawnedSceneHandlers.h>

#include <lumin/node/ModelNode.h>
#include <lumin/node/Text2dNode.h>

namespace scenes {

  namespace NewScene {
  
    namespace externalNodes {
      extern const std::string InitialTarget;
      extern const std::string FinalTarget;
      extern const std::string Distance;
      extern const std::string Laser;
      extern const std::string TargetingModel;
    } 

    struct SpawnedScene : public SpawnedSceneBase {
      SpawnedScene(const SceneDescriptor& sceneDescriptor, lumin::Node* root);
      ~SpawnedScene();
      lumin::Text2dNode* InitialTarget;
      lumin::Text2dNode* FinalTarget;
      lumin::Text2dNode* Distance;
      lumin::ModelNode* Laser;
      lumin::ModelNode* TargetingModel;
    };  

    SpawnedSceneBase* createSpawnedScene(const SceneDescriptor& sceneDescriptor, lumin::Node* root);
    SpawnedSceneHandlers* createSpawnedSceneHandlers(SpawnedSceneBase& spawnedScene);

    extern const SceneDescriptor descriptor; 
  }
}

