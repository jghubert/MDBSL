/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   ResourceLibrary.hpp
 * Author: Julien Hubert
 *
 * Created on August 11, 2016, 6:16 PM
 */

#ifndef RESOURCELIBRARY_HPP
#define RESOURCELIBRARY_HPP

//#include "GeneticAlgorithm.hpp"
#include <unordered_map>
#include "PolicyMemory.hpp"
#include <string>

namespace MDB_Social {

    class Policy;
    class ValueFunction;
    class TraceMemory;
    class ValueFunctionMemory;
    class GAFitness;
    class GeneticAlgorithm;
    class SocialManagerClient;
//    class PolicyMemory;
    
    class ResourceLibraryData
    {
    private:
        Policy* policy;
        ValueFunction* vf;
        TraceMemory* tm;
        ValueFunctionMemory* vfm;
        PolicyMemory* pm;
        GAFitness* gafitness;
        GeneticAlgorithm* ga;
        SocialManagerClient* smclient;
        
        bool socialMode;
        std::string workingDirectory;
        
    public:
        ResourceLibraryData() {
            policy = NULL;
            vf = NULL;
            tm = NULL;
            pm = NULL;
            vfm = NULL;
            gafitness = NULL;
            ga = NULL;
            smclient = NULL;
            
            socialMode = false;
            workingDirectory = '.';
        }
        
        void setValueFunction(ValueFunction* _vf) {
            vf = _vf;
        }
        
        ValueFunction* getValueFunction() const {
            return vf;
        }
        
        void setPolicy(Policy* p) {
            policy = p;
        }
        
        Policy* getPolicy() const {
            return policy;
        }
        
        void setTraceMemory(TraceMemory* _tm) {
            tm = _tm;
        }
        
        TraceMemory* getTraceMemory() const {
            return tm;
        }
        
        void setValueFunctionMemory(ValueFunctionMemory* _vfm) {
            vfm = _vfm;
        }
        
        ValueFunctionMemory* getValueFunctionMemory() const {
            return vfm;
        }
        
        void setPolicyMemory(PolicyMemory* _pm) {
            pm = _pm;
        }
        
        PolicyMemory* getPolicyMemory() const {
            return pm;
        }
        
        void setGAFitness(GAFitness* _fitness) {
            gafitness = _fitness;
        }
        
        GAFitness* getGAFitness() const {
            return gafitness;
        }
        
        void setGeneticAlgorithm(GeneticAlgorithm* _ga) {
            ga = _ga;
        }
        
        GeneticAlgorithm* getGeneticAlgorithm() const {
            return ga;
        }
        
        void setSocialManagerClient(SocialManagerClient* _smclient) {
            smclient = _smclient;
        }
        
        SocialManagerClient* getSocialManagerClient() const {
            return smclient;
        }
        
        void setSocialMode(bool mode) {
            socialMode = mode;
        }
        
        bool getSocialMode () const {
            return socialMode;
        }
        
        void setWorkingDirectory(std::string cwd) {
            workingDirectory = cwd;
        }

        std::string getWorkingDirectory() const {
            return workingDirectory;
        }
        
    };
    
    class ResourceLibrary
    {
    private:

        int long refCount;
        
        std::unordered_map<std::string, ResourceLibraryData*> library;
        
        ResourceLibrary() {
            refCount = 0;
        }
        
        
        
    public:
        
        static ResourceLibrary* instance;
        
        virtual ~ResourceLibrary() {
            
        }

        void operator delete (void* p) {
            ResourceLibrary* pc = static_cast<ResourceLibrary*>(p);
            pc->refCount--;

            if (!pc->refCount) {
                for (auto it = pc->library.begin(); it != pc->library.end(); ++it)
                    delete it->second;
                pc->library.clear();
                instance = NULL;
            }
            
        }
        
        static ResourceLibraryData* getInstance(std::string id="Default") {
            if (instance == NULL) {
                instance = new ResourceLibrary();
                ResourceLibraryData* tmp = new ResourceLibraryData();
                instance->library.insert(std::make_pair(id, tmp));
            }
            
            ResourceLibraryData* ret;
            if (instance->library.count(id) > 0)
                ret = instance->library[id];
            else {
                ret = new ResourceLibraryData();
                instance->library.insert(std::make_pair(id, ret));
            }
            instance->refCount++;
            return ret;
        }
        
    };
    
}

#endif /* RESOURCELIBRARY_HPP */

