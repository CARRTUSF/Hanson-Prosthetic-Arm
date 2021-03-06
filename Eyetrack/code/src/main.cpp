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
#include <ml_logging.h>
#include <lumin/ui/UiKit.h>



int main(int argc, char **argv)
{
  ML_LOG(Debug, "Eyetrack Starting.");
  Eyetrack myApp;
  return myApp.run();
}

