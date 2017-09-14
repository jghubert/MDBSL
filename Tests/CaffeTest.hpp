/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   CaffeTest.hpp
 * Author: Julien Hubert
 *
 * Created on August 15, 2017, 5:04 PM
 */

#ifndef CAFFETEST_HPP
#define CAFFETEST_HPP

#include "../MDB_SocialLearning/Settings.h"
#include "../MDB_SocialLearning/CaffeDeepNet.h"

namespace MDB_Social {

    class CaffeDeepNetTest: public ::testing::Test {
    protected:
        CaffeDeepNet caffe;
        Settings* settings;
        
        CaffeDeepNetTest() {
            settings = SettingsLibrary::getInstance();
            settings->setParameterSources("UnitTest.cfg", 0, NULL);

            caffe.registerParameters("CaffeDeepNet");

        }
        
        virtual ~CaffeDeepNetTest() {
            delete settings;
        }
        
        virtual void SetUp() {
            ASSERT_TRUE(settings->processAllParameters("UnitTest.cfg", 0, NULL));
         
            caffe.loadParameters("CaffeDeepNet");
            
        }
//        
//        virtual void TearDown() {
//            
//        }
    };

// Put the tests here

    TEST_F(CaffeDeepNetTest, XORTEST) {
        unsigned numData = 4;
        std::vector<double> inputs = {0, 0, 0, 1, 1, 0, 1, 1};
        std::vector<double> labels = {0, 1, 1, 0};
//        std::vector<double> inputs = {0, 0, 0, 1, 1, 0};
//        std::vector<double> labels = {0, 1, 1};
        
        caffe.setTrainingSet(numData, inputs, labels);
        caffe.train();
        std::vector<double> outputs = labels;
//        std::vector<double> outputs = caffe.run(numData, inputs, labels);
        caffe.run(numData, inputs, outputs);
        
        EXPECT_LE( std::fabs(outputs[0] - labels[0]), 0.1 ) << "Expected value for {0,0} = " << labels[0] << " ; Network value = " << outputs[0];
        EXPECT_LE( std::fabs(outputs[1] - labels[1]), 0.1 ) << "Expected value for {0,0} = " << labels[1] << " ; Network value = " << outputs[1];
        EXPECT_LE( std::fabs(outputs[2] - labels[2]), 0.1 ) << "Expected value for {0,0} = " << labels[2] << " ; Network value = " << outputs[2];
        EXPECT_LE( std::fabs(outputs[3] - labels[3]), 0.1 ) << "Expected value for {0,0} = " << labels[3] << " ; Network value = " << outputs[3];
        
        for (unsigned i=0; i<numData; ++i)
            std::cout << "Expected value for {0,0} = " << labels[i] << " ; Network value = " << outputs[i] << std::endl;
    }
    
}


#endif /* CAFFETEST_HPP */

