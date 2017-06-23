#ifndef __RANDOM_GENERATORS__
#define __RANDOM_GENERATORS__

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/linear_congruential.hpp>
#include <boost/random/random_number_generator.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_real.hpp>

namespace MDB_Social {

    class RandomGenerator
    {
     protected:
            long unsigned refCount;

            RandomGenerator();
            ~RandomGenerator();

     public:

            void operator delete (void* p);

            long getSeed();

            static void InitializeRandomSeed();

    };

    class GaussianGenerator : public RandomGenerator
    {
     private:
            boost::mt19937 gen;    // Mersenne Twister Generator
            boost::normal_distribution<double> normal_dist;  //Normal distribution
            boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> > normal;  // everything together as one

            GaussianGenerator(double mean = 0.0, double std = 1.0);

            bool local;
     public:
            static GaussianGenerator* instance;
            GaussianGenerator* localInstance;
            ~GaussianGenerator();

            double operator()();

            void setParameters(double _mean, double _std);

            static GaussianGenerator* getInstance();
            static GaussianGenerator* getLocalInstance();
    };

    class UniformGenerator : public RandomGenerator
    {
     private:
            boost::mt19937 gen;    // Mersenne Twister Generator
            boost::uniform_real<double> uniform_dist;  //Normal distribution
            boost::variate_generator<boost::mt19937&, boost::uniform_real<double> > uniform;  // everything together as one

            UniformGenerator(double min = 0.0, double max = 1.0);

            bool local;
     public:
            static UniformGenerator* instance;
            ~UniformGenerator();

            double operator()();

            static UniformGenerator* getInstance();
            static UniformGenerator* getLocalInstance();

    };

    class UniformIntegerGenerator : public RandomGenerator
    {
    private:
            boost::mt19937 gen;    // Mersenne Twister Generator

            bool local;

            UniformIntegerGenerator();
    public:
            static UniformIntegerGenerator* instance;
            ~UniformIntegerGenerator();

            unsigned operator()();

            static UniformIntegerGenerator* getInstance();
            static UniformIntegerGenerator* getLocalInstance();

    };
}

#endif
