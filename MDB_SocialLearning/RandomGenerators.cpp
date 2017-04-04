#include "RandomGenerators.h"
#include <boost/version.hpp>
#include <sys/time.h>
#include <time.h>
#include <stdio.h>

RandomGenerator::RandomGenerator()
{
	refCount = 0;
}

RandomGenerator::~RandomGenerator()
{

}

void RandomGenerator::operator delete (void* p)
{
	RandomGenerator* pc = static_cast<RandomGenerator*>(p);
	pc->refCount--;

	if (pc->refCount == 0)
		delete pc;
}

long RandomGenerator::getSeed()
{
	long myseed;
	struct timeval t;
	if (gettimeofday(&t,NULL)<0) {
		perror("Error getting time of day...");
	}
	myseed=(long)(t.tv_usec*(t.tv_sec%1000));
	if (myseed<0)
		myseed=-myseed;

	return myseed;	
}


void RandomGenerator::InitializeRandomSeed()
{

	// Initialise the random seed using the time in microseconds

	long myseed;
	struct timeval t;
	if (gettimeofday(&t,NULL)<0) {
		perror("Error getting time of day...");
	}
	myseed=(long)(t.tv_usec*(t.tv_sec%1000));
	if (myseed<0)
		myseed=-myseed;
	srand48(myseed);
	srandom(myseed);
}


/********************* GaussianGenerator *************************/

GaussianGenerator* GaussianGenerator::instance = NULL;

GaussianGenerator::GaussianGenerator(double mean, double std):
		normal_dist(mean, std),
		normal(gen,normal_dist)
{
    local = false;

    gen.seed(getSeed());
}

GaussianGenerator::~GaussianGenerator() 
{ 
    if (!local)
	instance = NULL;
}


GaussianGenerator* GaussianGenerator::getInstance() 
{
	if (!instance) {
		instance = new GaussianGenerator();
	}
	instance->refCount++;

	return instance;
}

void GaussianGenerator::setParameters(double _mean, double _std)
{
#if BOOST_VERSION > 104700
    normal_dist.param(boost::normal_distribution<double>::param_type(_mean, _std));
#else
    std::cerr << "WARNING: GaussianGenerator::setParameters : Your boost version("<<BOOST_VERSION<<") does not support this feature. Upgrade it if you need it." << std::endl;
#endif
}


GaussianGenerator* GaussianGenerator::getLocalInstance()
{
    GaussianGenerator* ret = new GaussianGenerator();
    ret->local = true;
    return ret;
}

double GaussianGenerator::operator()() 
{
	return normal();
}

/************************* UNIFORM DISTRIBUTION ********************************/

UniformGenerator* UniformGenerator::instance = NULL;

UniformGenerator::UniformGenerator(double min, double max):
	uniform_dist(min, max),
	uniform(gen,uniform_dist)
{
    gen.seed(getSeed());
}

UniformGenerator::~UniformGenerator() 
{ 
	instance = NULL;
}


UniformGenerator* UniformGenerator::getInstance() 
{
	if (!instance) {
		instance = new UniformGenerator();
	}

	instance->refCount++;
	return instance;
}

double UniformGenerator::operator()() 
{
	return uniform();
}


/************************* UNIFORM INTEGER DISTRIBUTION ********************************/

UniformIntegerGenerator* UniformIntegerGenerator::instance = NULL;

UniformIntegerGenerator::UniformIntegerGenerator()
{
    gen.seed(getSeed());
}   

UniformIntegerGenerator::~UniformIntegerGenerator()
{
        instance = NULL;
}
   
unsigned UniformIntegerGenerator::operator()()
{
    return gen();
}

UniformIntegerGenerator* UniformIntegerGenerator::getInstance()
{
	if (!instance) {
		instance = new UniformIntegerGenerator();
	}

	instance->refCount++;
	return instance;
    
}

