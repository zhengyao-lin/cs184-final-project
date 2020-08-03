#ifndef CGL_LENSTESTER_H
#define CGL_LENSTESTER_H


// STL
#include <string>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <string>
#include <vector>
// libCGL
#include "CGL/CGL.h"
#include "CGL/renderer.h"
#include "CGL/osdtext.h"
#include "CGL/vector2D.h"

#include "lens/lens.h"
#include "pathtracer/camera.h"

namespace CGL {


class LensTester : public Renderer {
 public:

  LensTester();
  ~LensTester( ) { }

  void init( void ) ;

  void render( void ) ;

  void resize( size_t w, size_t h ) ;

  std::string name( void );

  std::string info( void );

  void cursor_event( float x, float y );

  void scroll_event( float offset_x, float offset_y );

  void mouse_event( int key, int event, unsigned char mods );

  void keyboard_event( int key, int event, unsigned char mods ) ;

  void use_hdpi_reneder_target() { use_hdpi = true; }

private:

  void draw_lens(Lens &lens) ;
  void draw_trace(vector<Vector3D> &trace);

  // Internal event system //

  float mouseX, mouseY;
  enum e_mouse_button {
    LEFT   = MOUSE_LEFT,
    RIGHT  = MOUSE_RIGHT,
    MIDDLE = MOUSE_MIDDLE
  };

  size_t screenW;
  size_t screenH;

  Vector2D point1;
  Vector2D point2;

  bool gl_window;
  
  Camera camera;
  std::vector<std::vector<Vector3D>> traces;
  bool save_trace;
  int numrays, rayspace;
  double zoom;

};

} // namespace CGL

#endif // LENSTESTER_H
