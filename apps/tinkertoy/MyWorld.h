#ifndef _MYWORLD_
#define _MYWORLD_

#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <memory>
//#include "Constriants.h"

class Particle;
class Constraint;

class MyWorld {
public:
	MyWorld();

	virtual ~MyWorld();
	
	int getNumParticles();

	Particle* getParticle(int _index) {
		return mParticles[_index];
	}

	void addParticle(Eigen::Vector3d position);

	// TODO: your simulation code goes here
	void simulate();
	void updateForces();
	void resetForces();
	void updateConstraintParams();
	void calculateConstraints();
	void applyConstraints();

	void addConstraint(Constraint* constraint);
	const Constraint* getConstraint(int index);
	int getNumConstraints();

protected:
	std::vector<Particle*> mParticles;
	const int mMaxParticles = 5;

	Eigen::VectorXd q;
	Eigen::VectorXd dq;
	Eigen::VectorXd Q;
	Eigen::MatrixXd W;
	Eigen::VectorXd lambda;
	Eigen::MatrixXd J;
	Eigen::MatrixXd dJ;
	Eigen::MatrixXd Qh;
	Eigen::MatrixXd C;
	Eigen::MatrixXd dC;

	int nConstraints;

	std::vector<Constraint*> constraints;
	std::vector< std::vector< int >> constraintTable;

private:
	void initMats();

	inline void print_matrices() {
		//std::cout << "q:" << std::endl << q << std::endl << "-----------" << std::endl;
		//std::cout << "dq:" << std::endl << dq << std::endl << "-----------" << std::endl;
		//std::cout << "Q:" << std::endl << Q << std::endl << "-----------" << std::endl;
		//std::cout << "W:" << std::endl << W << std::endl << "-----------" << std::endl;
		std::cout << "J:" << std::endl << J << std::endl << "-----------" << std::endl;
		std::cout << "dJ:" << std::endl << dJ << std::endl << "-----------" << std::endl;
	}
};

#endif
