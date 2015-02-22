#include <iostream>
#include "MyWindow.h"
#include "MyWorld.h"
#include "Particle.h"
#include "dart/gui/GLFuncs.h"

#define KEY_MENU ('m')
#define KEY_RESET ('r')
#define KEY_SELECT ('s')
#define KEY_INSERT_PARTICLE ('i')
#define KEY_ADD_CONSTRAINT ('c')
#define KEY_SPRING ('p')
#define KEY_MOVE ('M')

using namespace Eigen;

Vector4d gRed(0.9, 0.2, 0.2, 1.0);
Vector4d gBlue(0.2, 0.2, 0.9, 1.0);

MyWindow::MyWindow() : SimWindow() {
  mBackground[0] = 1.0;
  mBackground[1] = 1.0;
  mBackground[2] = 1.0;
  mBackground[3] = 1.0;
		
  mPlaying = false;
            
  mPersp = 30.f;
  mTrans[1] = 0.f;
  mTrans[2] = -500;
  mFrame = 0;
  mDisplayTimeout = 5;

  mSelected = NULL;
  mSelectDist = 0.05;

  mWinMode = MODE_SELECT;

  mShowMenu = false;
  mKeyMenu = KEY_MENU;
  mKeyReset = KEY_RESET;
  mKeySelect = KEY_SELECT;
  mKeyInsert = KEY_INSERT_PARTICLE;
  mKeyAddConstraint = KEY_ADD_CONSTRAINT;
  mKeySpring = KEY_SPRING;
  mKeyMove = KEY_MOVE;
}

MyWindow::~MyWindow() {
}

void MyWindow::displayTimer(int _val) {
  // Apply spring force to selected particle
  if (mWinMode == MODE_SPRING && mMouseDown && mPlaying && mSelected != NULL) {
	Vector3d mouse((mMouseX * 1.0 / GlutWindow::mWinWidth - 0.5) * (0.2 / 0.1875), (0.5 - mMouseY * 1.0 / GlutWindow::mWinHeight) * (0.2 / 0.25), 0.0);
	applyForce(mSelected, 80.0 * (mouse - mSelected->mPosition));
	//	  std::cout << "Applying spring force" << std::endl;
  }
  mWorld->simulate();
  glutPostRedisplay();
  mFrame++;
  if(mPlaying)
    glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
}

void MyWindow::draw() {
  // Draw particles
  for (int i = 0; i < mWorld->getNumParticles(); i++)
    mWorld->getParticle(i)->draw(mRI);

  glDisable(GL_LIGHTING);
  mRI->setPenColor(Vector4d(0.3, 0.3, 0.3, 1.0));
  mRI->pushMatrix();
  

  for (int i = 0; i < mWorld->getNumConstraints(); i++) {
	  const Constraint *c = mWorld->getConstraint(i);
	  if (c->type == Constraint::CONSTRAINT_CIRCLE) {
		  // Draw a circle
		  glBegin(GL_LINE_LOOP);
		  double rad = 3.14 / 180.0;
		  double radius = ((const CircleConstraint*)c)->mRadius;
		  for (int j = 0; j < 360; j++) {
			  double angle = j * rad;
			  glVertex3d(radius * cos(angle), radius * sin(angle), 0.0);
		  }
		  glEnd();
	  }
	  else if (c->type == Constraint::CONSTRAINT_DISTANCE) {
		  // Draw a line through the particles constrianed by distance
		  const Particle *p1 = c->p;
		  const Particle *p2 = ((const DistanceConstraint*)c)->p_other;
		  glBegin(GL_LINES);
		  glVertex3f(p1->mPosition[0], p1->mPosition[1], p1->mPosition[2]);
		  glVertex3f(p2->mPosition[0], p2->mPosition[1], p2->mPosition[2]);
		  glEnd();
	  }
  }

  mRI->popMatrix();
  glEnable(GL_LIGHTING);

  // Display the frame count in 2D text
  char buff[64];
  sprintf(buff,"%d",mFrame);
  std::string frame(buff);
  glDisable(GL_LIGHTING);
  glColor3f(0.0,0.0,0.0);
  dart::gui::drawStringOnScreen(0.02f,0.02f,frame);

  // Show menu on screen
  showMenu(0.01f, 0.97f);

  // Show current mode
  char modeBuffs[NUM_MODES][16] = { "SELECT", "INSERT", "ADD CONSTRAINT", "MOVE", "SPRING" };
  std::string modeString(modeBuffs[mWinMode]);
  dart::gui::drawStringOnScreen(0.85, 0.02, modeString);

  glEnable(GL_LIGHTING);
}

void MyWindow::showMenu(float x, float y) {
	char formatBuff[32], menuBuff[16], resBuff[32], selBuff[32], insBuff[32];
	char lClickBuff[64];
	if (mShowMenu) {
		sprintf(formatBuff, "Action <MODE>: \'Key\'");
		sprintf(menuBuff, "Hide Menu: \'%c\'", mKeyMenu);
		sprintf(resBuff, "Reset World: \'%c\'", mKeyReset);
		sprintf(selBuff, "Select Mode: \'%c\'", mKeySelect);
		sprintf(insBuff, "Insert Mode: \'%c\'", mKeyInsert);
		std::string f(formatBuff);
		std::string m(menuBuff);
		std::string r(resBuff);
		std::string s(selBuff);
		std::string i(insBuff);
		float lineSpacing = 0.035f;
		glColor3f(0.0, 0.0, 0.0);
		dart::gui::drawStringOnScreen(x, y, m);
		dart::gui::drawStringOnScreen(x, y - lineSpacing, r);
		dart::gui::drawStringOnScreen(x, y - lineSpacing * 2, s);
		dart::gui::drawStringOnScreen(x, y - lineSpacing * 3, i);

		switch (mWinMode) {
		case MODE_SELECT:
			sprintf(lClickBuff, "Left Mouse: Select a particle");
			break;
		case MODE_INSERT_PARTICLE:
			sprintf(lClickBuff, "Left Mouse: Insert a particle");
			break;
		case MODE_ADD_CONSTRAINT:
			sprintf(lClickBuff, "Left Mouse: Choose second particle");
			break;
		default:
			sprintf(lClickBuff, "");
			break;
		}
		std::string lClick(lClickBuff);
		dart::gui::drawStringOnScreen(0.6f, 0.97f, lClick);
	}
	else {
		sprintf(menuBuff, "Show Menu: \'%c\'", mKeyMenu);
		std::string m(menuBuff);
		dart::gui::drawStringOnScreen(x, y, m);
	}
}

void MyWindow::keyboard(unsigned char key, int x, int y) {
	switch (key){
	case ' ': // Use space key to play or stop the motion
		mPlaying = !mPlaying;
		if (mPlaying)
			glutTimerFunc(mDisplayTimeout, refreshTimer, 0);
		break;
	case KEY_RESET:
		mWorld->~MyWorld();
		mWorld = new MyWorld();
		mFrame = 0;
		break;
	case KEY_MENU:
		mShowMenu = !mShowMenu;
		break;
	case KEY_SELECT:
		mWinMode = MODE_SELECT;
		break;
	case KEY_INSERT_PARTICLE:
		deselectClosest();
		mWinMode = MODE_INSERT_PARTICLE;
		break;
	case KEY_ADD_CONSTRAINT:
		if (mSelected != NULL) {
			mWinMode = MODE_ADD_CONSTRAINT;
		}
		else {
			std::cout << "Select a particle first!" << std::endl;
		}
		break;
	case KEY_SPRING:
		if (mSelected != NULL) {
			mWinMode = MODE_SPRING;
		}
		else {
			std::cout << "Select a particle first!" << std::endl;
		}
		break;
	case 'b':
		break;
	default:
		Win3D::keyboard(key, x, y);
	}
	glutPostRedisplay();
}

void MyWindow::click(int button, int state, int x, int y) {
	mMouseDown = !mMouseDown;
	Vector3d mouse((x * 1.0 / GlutWindow::mWinWidth - 0.5) * (0.2 / 0.1875), (0.5 - y * 1.0 / GlutWindow::mWinHeight) * (0.2 / 0.25), 0.0);
	Particle *p;
	if (mMouseDown){
		if (button == GLUT_LEFT_BUTTON) {
			std::cout << "Left Click" << std::endl;
			switch (mWinMode) {
			case MODE_SELECT:
				deselectClosest();
				selectClosest(x, y);
				break;
			case MODE_INSERT_PARTICLE:
				mWorld->addParticle(mouse);
				break;
			case MODE_ADD_CONSTRAINT:
				p = getClosest(x, y);
				if (p != NULL) {
					mWorld->addConstraint(new DistanceConstraint(mSelected, p));
					deselectClosest();
					mWinMode = MODE_SELECT;
				}
				break;
			default:
				break;
			}
		}
		else if (button == GLUT_RIGHT_BUTTON || button == GLUT_MIDDLE_BUTTON) {
			std::cout << "RIGHT Click" << std::endl;
			/*
			double xx = (x * 1.0 / GlutWindow::mWinWidth - 0.5) * (0.2 / 0.1875);
			double yy = (0.5 - y * 1.0 / GlutWindow::mWinHeight) * (0.2 / 0.25);
			mWorld->addParticle(Eigen::Vector3d(xx, yy, 0));
			*/
		}

		mMouseX = x;
		mMouseY = y;
	}
	glutPostRedisplay();
}

void MyWindow::drag(int x, int y) {
  double deltaX = x - mMouseX;
  double deltaY = y - mMouseY;

  mMouseX = x;
  mMouseY = y;
//  std::cout << "Drag by (" << deltaX << ", " << deltaY << ")" << std::endl;
  Vector3d mouse((x * 1.0 / GlutWindow::mWinWidth - 0.5) * (0.2 / 0.1875), (0.5 - y * 1.0 / GlutWindow::mWinHeight) * (0.2 / 0.25), 0.0);
  if (!mPlaying && mSelected != NULL && !mSelected->mConstrained) {
	  mSelected->mPosition = mouse;
  }

  glutPostRedisplay();
}

void MyWindow::move(int x, int y) {

}

Particle* MyWindow::getClosest(int x, int y) {
	Particle *p = NULL;
	Vector3d mouse((x * 1.0 / GlutWindow::mWinWidth - 0.5) * (0.2 / 0.1875), (0.5 - y * 1.0 / GlutWindow::mWinHeight) * (0.2 / 0.25), 0.0);
	double distance, minDistance = mSelectDist;
	int index = -1;
	for (int i = 0; i < mWorld->getNumParticles(); i++) {
		distance = sqrt((mouse - mWorld->getParticle(i)->mPosition).dot(mouse - mWorld->getParticle(i)->mPosition));
		if (distance < minDistance) {
			minDistance = distance;
			p = mWorld->getParticle(i);
			index = i;
		}
	}

	return p;
}

void MyWindow::selectClosest(int x, int y) {
	Particle* p = getClosest(x, y);

	if (p != NULL) {
		mSelected = p;
		mSelected->mColor = gBlue;
	}
}

void MyWindow::deselectClosest() {
	if (mSelected != NULL) {
		mSelected->mColor = gRed;
		mSelected = NULL;
	}
}

void MyWindow::applyForce(Particle *p, Vector3d force) {
	p->mAccumulatedForce += force;
}
