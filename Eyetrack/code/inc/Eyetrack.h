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

#include <lumin/LandscapeApp.h>
#include <lumin/Prism.h>
#include <lumin/event/ServerEvent.h>
#include <SceneDescriptor.h>
#include <PrismSceneManager.h>
#define GLM_ENABLE_EXPERIMENTAL 1
#include <glm/gtx/transform.hpp>
#include <cstdlib>
#include <ml_head_tracking.h>
#include <ml_perception.h>
#include <lumin/node/RootNode.h>
#include <lumin/ui/Cursor.h>
#include <ml_logging.h>
#include <scenes.h>
#include <string>
#include <PrismSceneManager.h>
#include <lumin/ui/UiKit.h>
#include <lumin/event/InputEventData.h>
#include <util.h>
#include <iostream>
#include "glm/ext.hpp" // For calculating the distance between two points
#include <lumin/event/EyeTrackingEventData.h>
#include <lumin/event/KeyInputEventData.h>
#include <lumin/event/ControlPose6DofInputEventData.h>


/**
 * Eyetrack Landscape Application
 */
class Eyetrack : public lumin::LandscapeApp {
public:
  /**
   * Constructs the Landscape Application.
   */
  Eyetrack();

  /**
   * Destroys the Landscape Application.
   */
  virtual ~Eyetrack();

  /**
   * Disallows the copy constructor.
   */
  Eyetrack(const Eyetrack&) = delete;

  /**
   * Disallows the move constructor.
   */
  Eyetrack(Eyetrack&&) = delete;

  /**
   * Disallows the copy assignment operator.
   */
  Eyetrack& operator=(const Eyetrack&) = delete;

  /**
   * Disallows the move assignment operator.
   */
  Eyetrack& operator=(Eyetrack&&) = delete;

protected:
  /**
   * Initializes the Landscape Application.
   * @return - 0 on success, error code on failure.
   */
  int init() override;

  /**
   * Deinitializes the Landscape Application.
   * @return - 0 on success, error code on failure.
   */
  int deInit() override;

  /**
   * Returns the initial size of the Prism
   * Used in createPrism().
   */
  const glm::vec3 getInitialPrismSize() const;

  /**
   * Creates the prism, updates the private variable prism_ with the created prism.
   */
  void createInitialPrism();

  /**
   * Initializes and creates the scene of all scenes marked as initially instanced
   */
  void spawnInitialScenes();

  /**
   * Run application login
   */
  virtual bool updateLoop(float fDelta) override;

  /**
   * Handle events from the server
   */
  virtual bool eventListener(lumin::ServerEvent* event) override;

  glm::quat findRotation(const glm::vec3& direction, const glm::vec3& up);

private:
  lumin::Prism* prism_ = nullptr;  // represents the bounded space where the App renders.
  PrismSceneManager* prismSceneManager_ = nullptr;

  std::shared_ptr<lumin::PrismDataHandle> eyetrackingRetainer_; // to store eyetracking events
};

