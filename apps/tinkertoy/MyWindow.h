#ifndef _MYWINDOW_
#define _MYWINDOW_

#include "dart/gui/SimWindow.h"
#include "Particle.h"

class MyWorld;
class Particle;

class MyWindow : public dart::gui::SimWindow {
 public:
  MyWindow();
  virtual ~MyWindow();
    
  // Override these virtual functions defined in yui::Win3D
  virtual void displayTimer(int _val);
  virtual void draw();
  virtual void keyboard(unsigned char key, int x, int y);
  virtual void click(int button, int state, int x, int y);
  virtual void drag(int x, int y);
  virtual void move(int x, int y);
    
  MyWorld* getWorld() {
    return mWorld;
  }

  void setWorld(MyWorld *_world) {
    mWorld = _world;
  }

protected:
	typedef enum WINDOW_MODE {
		MODE_SELECT,
		MODE_INSERT_PARTICLE,
		MODE_ADD_CONSTRAINT,
		MODE_MOVE,
		MODE_SPRING,
		NUM_MODES
	} WinMode;

  bool mPlaying;
  int mFrame;

  Particle* mSelected;
  double mSelectDist;
    
  MyWorld *mWorld;

  WinMode mWinMode;
  bool mShowMenu;

  // Keys for user input options
  char mKeyMenu;
  char mKeySelect;
  char mKeyInsert;
  char mKeyAddConstraint;
  char mKeySpring;
  char mKeyMove;

  virtual Particle* getClosest(int x, int y);
  virtual void selectClosest(int x, int y);
  virtual void deselectClosest();
  virtual void applyForce(Particle *p, Eigen::Vector3d force);
  virtual void showMenu(float x, float y);
};

#endif
