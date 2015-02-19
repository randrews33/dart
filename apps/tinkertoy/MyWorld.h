#ifndef _MYWORLD_
#define _MYWORLD_

#include <vector>
#include <iostream>
#include <Eigen/Dense>

class Particle;

class MyWorld {
public:
	MyWorld(int _numParticles);

	virtual ~MyWorld();
	
	int getNumParticles() {
		return mParticles.size();
	}

	Particle* getParticle(int _index) {
		return mParticles[_index];
	}

	void addParticle(Eigen::Vector3d position);

	// TODO: your simulation code goes here
	void simulate();
	void updateForces();
	void updateConstraintParams();
	void calculateConstraints();
	void applyConstraints();

protected:
	std::vector<Particle*> mParticles;

	Eigen::VectorXd q;
	Eigen::VectorXd dq;
	Eigen::VectorXd Q;
	Eigen::MatrixXd W;
	//Eigen::ArrayXd W;
	Eigen::VectorXd lambda;
	Eigen::MatrixXd J;
	Eigen::MatrixXd dJ;
	Eigen::MatrixXd Qh;
	Eigen::MatrixXd C;
	Eigen::MatrixXd dC;

	int nConstraints;

	// Temporary, for storing distance values of distance-based constraints
	// First value will correspond to second particle, because current assumption is that the
	// first particle is NOT operating under a  distance-based but circle-based constraint
	std::vector<float> distances;

private:
	void initMats();

	inline void print_matrices() {
		//std::cout << "q:" << std::endl << q << std::endl << "-----------" << std::endl;
		//std::cout << "dq:" << std::endl << dq << std::endl << "-----------" << std::endl;
		//std::cout << "Q:" << std::endl << Q << std::endl << "-----------" << std::endl;
		std::cout << "W:" << std::endl << W << std::endl << "-----------" << std::endl;
		std::cout << "J:" << std::endl << J << std::endl << "-----------" << std::endl;
		std::cout << "dJ:" << std::endl << dJ << std::endl << "-----------" << std::endl;
	}
};

#endif
