#include "CGL/CGL.h"
#include "CGL/viewer.h"

#include "lenstester.h"

#include <iostream>
#include <string>
#include <unistd.h>

using namespace std;
using namespace CGL;


int main( int argc, char** argv ) {


  // create application
  LensTester *app  = new LensTester();

  // create viewer
  Viewer viewer = Viewer();

  // set renderer
  viewer.set_renderer(app);

  // init viewer
  viewer.init();

  // start viewer
  viewer.start();

  return 0;

}
