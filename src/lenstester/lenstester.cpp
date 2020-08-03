#include <cassert>

#include "lens/lens.h"
#include "lenstester.h"

using namespace std;

namespace CGL {

LensTester::LensTester() {
  Lens *lens = camera.get_current_lens();
  assert(lens && "no current lens");

  double front = fabs(lens->elts.back().center - lens->elts.back().radius);
  double back = fabs(lens->elts.front().center - lens->elts.front().radius);
  double rad = max(back,front);
  zoom = rad * 4;
  point1 = Vector2D(zoom, 0);
  point2 = Vector2D(back, 0);

  numrays = 2;
  rayspace = 20;
}

void LensTester::init() {
  if (gl_window) {
    // Lighting needs to be explicitly enabled.
    glEnable(GL_LIGHTING);

    // Enable anti-aliasing and circular points.
    glEnable( GL_LINE_SMOOTH );
    glEnable( GL_POLYGON_SMOOTH );
    glEnable(GL_POINT_SMOOTH);
    glHint( GL_LINE_SMOOTH_HINT, GL_NICEST );
    glHint( GL_POLYGON_SMOOTH_HINT, GL_NICEST );
    glHint(GL_POINT_SMOOTH_HINT,GL_NICEST);

  }

}

void LensTester::draw_lens(Lens &lens) {
  glColor4f(0, 1, 0, 1);
  Vector2D prev;
  bool do_prev = false;
  for (LensElement &elt : lens.elts) {
    if (!elt.radius) {
      continue;
    }
    double theta_max = asin(.5 * elt.aperture / fabs(elt.radius));
    double dt = theta_max / 25.;
    glBegin(GL_LINE_STRIP);
    for (double t = -theta_max; t <= theta_max; t += dt)
      glVertex2f((elt.center - elt.radius * cos(t))/zoom, (0 - elt.radius * sin(t))/zoom);
    glEnd();

    Vector2D next((elt.center - elt.radius * cos(elt.radius > 0 ? theta_max:-theta_max))/zoom, 
                        (0 - elt.radius * sin(elt.radius > 0 ? theta_max:-theta_max))/zoom);
    if (do_prev) {
      glBegin(GL_LINES);
      glVertex2f(prev.x,prev.y);
      glVertex2f(next.x,next.y);
      glVertex2f(prev.x,-prev.y);
      glVertex2f(next.x,-next.y);
      glEnd();
    }
    do_prev = elt.ior > 1.;
    prev = next;
  }
}

void LensTester::draw_trace(vector<Vector3D> &trace) {
  
  glBegin(GL_LINE_STRIP);
  for (Vector3D v : trace)
    glVertex2f(v.z / zoom, v.x / zoom);
  glEnd();
}


void LensTester::render() {
  Lens *lens = camera.get_current_lens();
  assert(lens && "no current lens");

  vector<vector<Vector3D>> curr_traces;
  for (int dp = -numrays * rayspace; dp <= numrays * rayspace; dp += rayspace){
      Vector2D point1_ = point1;
      Vector2D point2_ = point2;
      point2_.y += dp * ((float)screenH)/(float)screenW / zoom;

      Vector3D o(point1_.y, 0, point1_.x);
      Vector3D d(point2_.y-point1.y, 0, point2_.x-point1_.x);
      bool backwards = o.z < 0;
      if ((d.z > 0) ^ backwards) {
        d = -d;
        o = Vector3D(point2_.y, 0, point2_.x);
      }
      d.normalize();
      Ray r(o,d);
      vector<Vector3D> trace;

      trace.push_back(r.o);
      if (backwards) 
        lens->trace_backwards(r,&trace);
      else
        lens->trace(r,&trace);
      trace.push_back(r.o + 2000*r.d);

      curr_traces.push_back(trace);
  }


  glPushAttrib(GL_VIEWPORT_BIT);
  glViewport(0, 0, screenW, screenH);

  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  glOrtho(-1, 1, -((float)screenH)/(float)screenW, ((float)screenH)/(float)screenW, 0, 1);

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  glTranslatef(0, 0, -1);

  // -- Black with opacity .8;


  glDisable(GL_DEPTH_TEST);
  glDisable(GL_LIGHTING);

  glColor4f(1, 0, 0, 1);
  for (vector<Vector3D> &trace : curr_traces) {
    draw_trace(trace);
  }

  glColor4f(0, 0, 1, 1);
  for (vector<Vector3D> &vec : traces) {
    draw_trace(vec);
  }
  if (save_trace) {
    for (vector<Vector3D> &trace : curr_traces)
      traces.push_back(trace);
    save_trace = false;
  }


  draw_lens(*lens);

}

void LensTester::resize(size_t w, size_t h) {
  screenW = w;
  screenH = h;
}


string LensTester::name() {
  return "LensTester";
}

string LensTester::info() {
  return "LensTester";
}


void LensTester::cursor_event(float x, float y) {

  mouseX = x;
  mouseY = y;
  point2 = Vector2D((mouseX/screenW-.5)*2. * zoom, 
        ((float)screenH)/(float)screenW*(.5-mouseY/screenH)*2. * zoom);
}

void LensTester::scroll_event(float offset_x, float offset_y) {
  zoom *= exp(offset_y / 10.);
}

void LensTester::mouse_event(int key, int event, unsigned char mods) {
  switch(event) {
    case EVENT_PRESS:
      switch(key) {
        case MOUSE_LEFT:
          point1 = Vector2D((mouseX/screenW-.5)*2. * zoom, 
                  ((float)screenH)/(float)screenW*(.5-mouseY/screenH)*2. * zoom);
          cout << "[LensTester] Fixed point is now " << point1 << endl;
          // mouse_pressed(LEFT);
          break;
        case MOUSE_RIGHT:
          // mouse_pressed(RIGHT);
          break;
        case MOUSE_MIDDLE:
          // mouse_pressed(MIDDLE);
          break;
      }
      break;
    case EVENT_RELEASE:
      switch(key) {
        case MOUSE_LEFT:
          // mouse_released(LEFT);
          break;
        case MOUSE_RIGHT:
          // mouse_released(RIGHT);
          break;
        case MOUSE_MIDDLE:
          // mouse_released(MIDDLE);
          break;
      }
      break;
  }
}

void LensTester::keyboard_event(int key, int event, unsigned char mods) {
  if (event != EVENT_PRESS) 
    return;

  if (key >= '1' && key <= '4') {
    camera.mount_lens(key-'1');
    return;
  }
  switch (key) {
    case '-':
      if (numrays >= 1) numrays--;
      break;
    case '=':
      numrays++;
      break;
    case '[':
      if (rayspace >= 5) rayspace /= 2;
      break;
    case ']':
      rayspace *= 2;
      break;
    case ' ':
     save_trace = true;
     break;
    case 'R':
      traces.clear();
      break;
  }
}


} // namespace CGL
