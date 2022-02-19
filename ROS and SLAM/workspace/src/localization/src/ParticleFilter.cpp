#include "localization/ParticleFilter.h"
#include "localization/Util.h"

#include "tf/tf.h"

using namespace std;

ParticleFilter::ParticleFilter(int numberOfParticles) {
	this->numberOfParticles = numberOfParticles;

	// initialize particles
	for (int i = 0; i < numberOfParticles; i++) {
		this->particleSet.push_back(new Particle());
	}

	// this variable holds the estimated robot pose
	this->bestHypothesis = new Particle();

	// at each correction step of the filter only the laserSkip-th beam of a scan should be integrated
	this->laserSkip = 5;

	// distance map used for computing the likelihood field
	this->distMap = NULL;
}

ParticleFilter::~ParticleFilter() {
	// delete particles
	for (int i = 0; i < numberOfParticles; i++) {
		Particle* p = this->particleSet[i];
		delete p;
	}

	this->particleSet.clear();

	if (this->likelihoodField)
		delete[] this->likelihoodField;

	delete this->bestHypothesis;

	if (this->distMap)
		delete[] this->distMap;
}

int ParticleFilter::getNumberOfParticles() {
	return this->numberOfParticles;
}

std::vector<Particle*>* ParticleFilter::getParticleSet() {
	return &(this->particleSet);
}

void ParticleFilter::initParticlesUniform() {
    //get map properties
    int mapWidth, mapHeight;
    double mapResolution;
    this->getLikelihoodField(mapWidth, mapHeight,mapResolution);

	// TODO: here comes your code
	int i, numberOfParticles;
	double x, y, theta, weight;
	
	numberOfParticles = this->getNumberOfParticles();

	weight = 1.0 / numberOfParticles;

	for (i = 0; i < numberOfParticles; i++) {
		
		x = Util::uniformRandom(0, mapWidth * mapResolution);
		y = Util::uniformRandom(0, mapHeight * mapResolution);
		theta = Util::uniformRandom(-M_PI, M_PI);

		this->particleSet[i]->x = x;
		this->particleSet[i]->y = y;
		this->particleSet[i]->theta = theta;
		this->particleSet[i]->weight = weight;
	}
}

void ParticleFilter::initParticlesGaussian(double mean_x, double mean_y,
		double mean_theta, double std_xx, double std_yy, double std_tt) {
	// TODO: here comes your code
	int i, numberOfParticles;
	double x, y, theta, weight;
	
	numberOfParticles = this->getNumberOfParticles();

	weight = 1.0 /numberOfParticles;

	for (i = 0; i < numberOfParticles; i++) {
		
		x = Util::gaussianRandom(mean_x, std_xx);
		y = Util::gaussianRandom(mean_y, std_yy);
		theta = Util::gaussianRandom(mean_theta, std_tt);

		// x = Util::gaussianRandom(4.1, 0.2);
		// y = Util::gaussianRandom(6.5, 0.2);
		// theta = Util::gaussianRandom(4.5, 0.5);

		this->particleSet[i]->x = x;
		this->particleSet[i]->y = y;
		this->particleSet[i]->theta = theta;
		this->particleSet[i]->weight = weight;
	}

}

/**
 *  Initializes the likelihood field as our sensor model.
 */
void ParticleFilter::setMeasurementModelLikelihoodField(
		const nav_msgs::OccupancyGrid& map, double zRand, double sigmaHit) {
	ROS_INFO("Creating likelihood field for laser range finder...");

	// create the likelihood field - with the same discretization as the occupancy grid map
	this->likelihoodField = new double[map.info.height * map.info.width];
	this->likelihoodFieldWidth = map.info.width;
	this->likelihoodFieldHeight = map.info.height;
	this->likelihoodFieldResolution = map.info.resolution;

    // calculates the distance map and stores it in member variable 'distMap'
	// for every map position it contains the distance to the nearest occupied cell.
	calculateDistanceMap(map);

    // Here you have to create your likelihood field
	// HINT0: sigmaHit is given in meters. You have to take into account the resolution of the likelihood field to apply it.
	// HINT1: You will need the distance map computed 3 lines above
	// HINT2: You can visualize it in the map_view when clicking on "show likelihood field" and "publish all".
	// HINT3: Storing probabilities in each cell between 0.0 and 1.0 might lead to round-off errors, therefore it is
	// good practice to convert the probabilities into log-space, i.e. storing log(p(x,y)) in each cell. As a further
	// advantage you can simply add the log-values in your sensor model, when you weigh each particle according the
	// scan, instead of multiplying the probabilities, because: log(a*b) = log(a)+log(b).

	// TODO: here comes your code

	int index;
	double pHit, sigmaSq;

	//Convert the sigma to map res since its given in metres
	sigmaSq = sigmaHit / this->likelihoodFieldResolution;

	for (int x = 0; x < this->likelihoodFieldWidth; x++) {
		for(int y = 0; y < this->likelihoodFieldHeight; y++) {
			index = computeMapIndex(this->likelihoodFieldWidth, this->likelihoodFieldHeight, x, y);
			pHit = Util::gaussian(distMap[index], sigmaSq, 0.0);
			this->likelihoodField[index] = log((1 - zRand) * pHit + zRand);
		}
	}
	
	ROS_INFO("...DONE creating likelihood field!");
}

void ParticleFilter::calculateDistanceMap(const nav_msgs::OccupancyGrid& map) {
	// calculate distance map = distance to nearest occupied cell
	distMap = new double[likelihoodFieldWidth * likelihoodFieldHeight];
	int occupiedCellProbability = 90;
	// initialize with max distances
	for (int x = 0; x < likelihoodFieldWidth; x++) {
		for (int y = 0; y < likelihoodFieldHeight; y++) {
			distMap[x + y * likelihoodFieldWidth] = 32000.0;
		}
	}
	// set occupied cells next to unoccupied space to zero
	for (int x = 0; x < map.info.width; x++) {
		for (int y = 0; y < map.info.height; y++) {
			if (map.data[x + y * map.info.width] >= occupiedCellProbability) {
				bool border = false;
				for (int i = -1; i <= 1; i++) {
					for (int j = -1; j <= 1; j++) {
						if (!border && x + i >= 0 && y + j >= 0 && x + i
								< likelihoodFieldWidth && y + j
								< likelihoodFieldHeight && (i != 0 || j != 0)) {
							if (map.data[x + i + (y + j) * likelihoodFieldWidth]
									< occupiedCellProbability && map.data[x + i
									+ (y + j) * likelihoodFieldWidth] >= 0)
								border = true;
						}
						if (border)
							distMap[x + i + (y + j) * likelihoodFieldWidth]
									= 0.0;
					}
				}
			}
		}
	}
	// first pass -> SOUTHEAST
	for (int x = 0; x < likelihoodFieldWidth; x++)
		for (int y = 0; y < likelihoodFieldHeight; y++)
			for (int i = -1; i <= 1; i++)
				for (int j = -1; j <= 1; j++)
					if (x + i >= 0 && y + j >= 0 && x + i
							< likelihoodFieldWidth && y + j
							< likelihoodFieldHeight && (i != 0 || j != 0)) {
						double v = distMap[x + i + (y + j)
								* likelihoodFieldWidth] + ((i * j != 0) ? 1.414
								: 1);
						if (v < distMap[x + y * likelihoodFieldWidth]) {
							distMap[x + y * likelihoodFieldWidth] = v;
						}
					}

	// second pass -> NORTHWEST
	for (int x = likelihoodFieldWidth - 1; x >= 0; x--)
		for (int y = likelihoodFieldHeight - 1; y >= 0; y--)
			for (int i = -1; i <= 1; i++)
				for (int j = -1; j <= 1; j++)
					if (x + i >= 0 && y + j >= 0 && x + i
							< likelihoodFieldWidth && y + j
							< likelihoodFieldHeight && (i != 0 || j != 0)) {
						double v = distMap[x + i + (y + j)
								* likelihoodFieldWidth] + ((i * j != 0) ? 1.414
								: 1);
						if (v < distMap[x + y * likelihoodFieldWidth]) {
							distMap[x + y * likelihoodFieldWidth] = v;
						}
					}
}

double* ParticleFilter::getLikelihoodField(int& width, int& height,
		double& resolution) {
	width = this->likelihoodFieldWidth;
	height = this->likelihoodFieldHeight;
	resolution = this->likelihoodFieldResolution;

	return this->likelihoodField;
}

/**
 *  A generic measurement integration method that invokes some specific observation model.
 *  Maybe in the future, we add some other model here.
 */
void ParticleFilter::measurementModel(
		const sensor_msgs::LaserScanConstPtr& laserScan) {
	likelihoodFieldRangeFinderModel(laserScan);
}

/**
 *  Method that implements the endpoint model for range finders.
 *  It uses a precomputed likelihood field to weigh the particles according to the scan and the map.
 */
void ParticleFilter::likelihoodFieldRangeFinderModel(
		const sensor_msgs::LaserScanConstPtr & laserScan) {

	// TODO: here comes your code
	int scanCount = laserScan->ranges.size();

	double angleMin = laserScan->angle_min;

	double angleIncrement = laserScan->angle_increment;

	double rangeMin = laserScan->range_min;

	double rangeMax = laserScan->range_max;

	int numberOfParticles, index;

	double x, y, theta, scanTheta, p, q, currentRange, currentWeightSum, currentWeight, sumOfWeights = 0.0;

	numberOfParticles = this->getNumberOfParticles();	


	//For each particle, we try to calculate the weight for each scan
	for (int j = 0; j < numberOfParticles; j++) {
		currentWeightSum = 0.0;

		for (int i = 0; i < scanCount; i = i + this->laserSkip) {

			currentRange = laserScan->ranges[i];
			scanTheta = angleMin + (i * angleIncrement);

			if (currentRange > rangeMin && currentRange < rangeMax) {

				x = this->particleSet[j]->x;
				y = this->particleSet[j]->y;
				theta = this->particleSet[j]->theta;

				p = (currentRange * cos(theta + scanTheta) + x) / this->likelihoodFieldResolution;

				q = (currentRange * sin(theta + scanTheta) + y) / this->likelihoodFieldResolution;

				//Out of bounds check for sanity
				if (p >= 0 && q >= 0 && p < this->likelihoodFieldWidth && q < this->likelihoodFieldHeight) {

					index = computeMapIndex(this->likelihoodFieldWidth, this->likelihoodFieldHeight, p, q);
					
					currentWeightSum += this->likelihoodField[index];

				} else {
					//Well, reduce the prob by chaining a lower than 1 multiplier
					currentWeightSum += log(0.65);
				}
			}
	
		}

		// For each scan, we try to determine the probability of the endpoint
		// being the point closest point according to the likelihood field.
		// The total probability for a particle is given by the product of
		// individual probability of each scan.
		this->particleSet[j]->weight = this->particleSet[j]->weight + exp(currentWeightSum);

		//Total sum of weights needed for normalization
		sumOfWeights += this->particleSet[j]->weight;

	}

	//Normalize the weights so that the sum is equal to 1
	for (int j = 0; j < numberOfParticles; j++) {
		this->particleSet[j]->weight = this->particleSet[j]->weight / sumOfWeights;
	}
}

void ParticleFilter::setMotionModelOdometry(double alpha1, double alpha2,
		double alpha3, double alpha4) {
	this->odomAlpha1 = alpha1;
	this->odomAlpha2 = alpha2;
	this->odomAlpha3 = alpha3;
	this->odomAlpha4 = alpha4;

}

/**
 *  A generic motion integration method that invokes some specific motion model.
 *  Maybe in the future, we add some other model here.
 */
void ParticleFilter::sampleMotionModel(double oldX, double oldY,
		double oldTheta, double newX, double newY, double newTheta) {
	sampleMotionModelOdometry(oldX, oldY, oldTheta, newX, newY, newTheta);
}

/**
 *  Method that implements the odometry-based motion model.
 */
void ParticleFilter::sampleMotionModelOdometry(double oldX, double oldY,
		double oldTheta, double newX, double newY, double newTheta) {
	// TODO: here comes your code
	
	int numberOfParticles;

	numberOfParticles = this->getNumberOfParticles();
	
	double deltaTrans, deltaRot1, deltaRot2, sample1, sample2, sample3, delTrans, delRot1, delRot2;

	double particleX, particleY, particleTheta, dekh;

	deltaTrans = sqrt(pow(newX - oldX, 2) + pow(newY - oldY, 2));
	deltaRot1 = Util::diffAngle(oldTheta, atan2(newY - oldY, newX - oldX));
	deltaRot2 = Util::normalizeTheta(newTheta - oldTheta - deltaRot1);

	if (deltaRot1 == deltaRot1 && deltaRot2 == deltaRot2) {

		for (int i = 0; i < numberOfParticles; i++) {

			sample1 = Util::gaussianRandom(0, odomAlpha1 * abs(deltaRot1) + odomAlpha2 * deltaTrans);

			sample2 = Util::gaussianRandom(0, odomAlpha3 * abs(deltaTrans) + odomAlpha4 * abs(deltaRot1 + deltaRot2));

			sample3 = Util::gaussianRandom(0, odomAlpha1 * abs(deltaRot2) + odomAlpha2 * deltaTrans);

			delTrans = deltaTrans + sample2;

			delRot1 = deltaRot1 + sample1;
			
			delRot2 = deltaRot2 + sample3;

			particleX = this->particleSet[i]->x;
			particleY = this->particleSet[i]->y;
			particleTheta = this->particleSet[i]->theta;

			this->particleSet[i]->x = particleX + delTrans * cos((particleTheta + delRot1));

			this->particleSet[i]->y = particleY + delTrans * sin((particleTheta + delRot1));

			this->particleSet[i]->theta = Util::normalizeTheta(particleTheta + (delRot1 + delRot2));
		}
	}

}

/**
 *  The stochastic importance resampling.
 */
void ParticleFilter::resample() {
	// TODO: here comes your code

	std::vector<Particle*> newParticleSet;

	int numberOfParticles, i = 0, bestHypothesisIndex;

	numberOfParticles = this->getNumberOfParticles();

	double maxWeight = -99999, sumOfWeights = 0.0, r, currentSum = 0.0, U;

	r = Util::uniformRandom(0, 1.0/numberOfParticles);

	currentSum = this->particleSet[i]->weight;

	for (int m = 0; m < numberOfParticles; m++) {
		U = r + m * (1.0/numberOfParticles);

		while(U > currentSum) {
			i = i + 1;
			currentSum = currentSum + this->particleSet[i]->weight;
		}

		Particle* toInsertParticle = new Particle(this->particleSet[i]);

		newParticleSet.push_back(toInsertParticle);
	}

	for (int k = 0; k < numberOfParticles; k++) {
		this->particleSet[k]->x = newParticleSet[k]->x;
		this->particleSet[k]->y = newParticleSet[k]->y;
		this->particleSet[k]->theta = newParticleSet[k]->theta;
		this->particleSet[k]->weight = newParticleSet[k]->weight;

		if (maxWeight < this->particleSet[k]->weight) {
			maxWeight = this->particleSet[k]->weight;
			bestHypothesisIndex = k;
		}
	}

	//The best hypothesis is simply the particle with the max weight

	this->bestHypothesis->x = this->particleSet[bestHypothesisIndex]->x;
	this->bestHypothesis->y = this->particleSet[bestHypothesisIndex]->y;
	this->bestHypothesis->theta = this->particleSet[bestHypothesisIndex]->theta;

}

Particle* ParticleFilter::getBestHypothesis() {
	return this->bestHypothesis;
}

// added for convenience
int ParticleFilter::computeMapIndex(int width, int height, int x,
		int y) {
	return x + y * width;
}

