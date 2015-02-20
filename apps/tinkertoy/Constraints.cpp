#include "Constraints.h"
#include "Particle.h"

double CircleConstraint::C() const {
	return 0.5 * p->mPosition.dot(p->mPosition);
}