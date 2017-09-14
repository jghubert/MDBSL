/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Settings.h
 * Author: Julien Hubert
 *
 * Created on June 20, 2016, 3:10 PM
 */

#ifndef SETTINGS_H
#define SETTINGS_H

#include <iostream>
#include <fstream>
#include <boost/program_options.hpp>
#include <unordered_map>
#include <list>
#include <functional>
#include <utility>
#include <typeinfo>
#include "colormod.hpp"

#ifndef MDBSL_FIELD_SEPARATOR
#define MDBSL_FIELD_SEPARATOR " "
#endif

namespace MDB_Social {
    
    namespace po = boost::program_options;
    
    class Settings {
    private:
        po::options_description fileDescriptions;
        po::options_description cmdLineDescriptions;
        po::options_description mainDescriptions;
        po::variables_map* settingLibrary;
                
        std::list<std::pair<std::function<void(void*)>, void*> > callbacks; 
        
        std::string configFilename;
        int argc;
        char** argv;
        
        bool loaded;
        
        enum data_type {
            UNKNOWN = 0,
            INTEGER,
            DOUBLE,
            STRING,
            BOOL
        };

        enum data_type identifyValueType(const std::string& value) const;

        void registerUnknownKeywords(const std::vector<po::basic_option<char> >& keywords, po::options_description& desc);

        std::pair<std::string, std::string> findInFile(const char* name);
        std::pair<std::string, std::string> findInArgv(const char* name);
        
                
    public:

        Settings();
        virtual ~Settings();

        void registerCallbackFunction(void (callMeWhenLoaded)(void*), void*);

        void setParameterSources(const char* _filename, int _argc, char* _argv[]);
        
        bool processAllParameters(const char* _filename, int _argc, char* _argv[]);
        bool isKeywordRegistered(const char* name) const;

        bool printHelpIfRequested() const;
        bool saveSettingsToFileRequested() const;
        
        template<typename T> bool registerParameter(const char* name, const char* description=NULL) {
            try {
                fileDescriptions.add_options() (name, description);
            } catch(po::duplicate_option_error) {
                    std::cerr << __FILE__ << "(l." << __LINE__ << "): Warning: Duplicate option." << std::endl;
                    return false;
            }
            return true;
        }

        template<typename T> bool registerParameter(const char* name, T defaultValue, const char* description=NULL) {
            try {
                fileDescriptions.add_options() (name, po::value<T>()->default_value(defaultValue), description);
            } catch(po::duplicate_option_error) {
                    std::cerr << __FILE__ << "(l." << __LINE__ << "): Warning: Duplicate option." << std::endl;
                    return false;
            }
            return true;
        }

        
        template<typename T> bool registerAndRetrieveParameter(T& returnValue, const char* name, T defaultValue, const char* description=NULL)
        {
            if (configFilename.empty() && argc == 0)
                return false;
            
            std::pair<std::string, std::string> param;

            // First, parse the argv as they replace the value from the file.
            if (argc > 2) 
                param = findInArgv(name);

            if (param.first.empty() && !configFilename.empty())
                param = findInFile(name);
            
            if (param.first.empty()) {
                returnValue = defaultValue;
            }
            else {
                // Need to convert the std::string to typename T.
                // Functions I can use:
                // numbers: strtoul (unsigned)
                if (typeid(T) == typeid(int)) {
                    returnValue = std::stoi(param.second);
                }
                else  if (typeid(T) == typeid(long int)) {
                    returnValue = std::stol(param.second);
                }
                else if (typeid(T) == typeid(unsigned) || typeid(T) == typeid(unsigned long)) {
                    returnValue = std::stoul(param.second);
                }
                else if (typeid(T) == typeid(std::string)) {
                    returnValue = param.second;
                }
                else if (typeid(T) == typeid(float)) {
                    returnValue = std::stof(param.second);
                }
                else if (typeid(T) == typeid(double)) {
                    returnValue = std::stod(param.second);
                }
                else if (typeid(T) == typeid(bool)) {
                    returnValue = (param.second == "true" || param.second == "1");
                }
                else {
                    std::cerr << "Settings::registerAndRetrieveParameter: Unknown type." << std::endl;
                    return false;
                }
                
            }
            return registerParameter<T>(name, defaultValue, description);
            
        }
                    
        template<typename T> std::pair<std::string, T> value(const char* name) const {
            std::pair<std::string, T> ret;
            if (!loaded) {
                Color::Modifier cdefault(Color::FG_DEFAULT);
                Color::Modifier red(Color::FG_RED);
                std::cerr << red << "ERROR: Settings::value : Configuration not yet read and requesting the value for " << name << cdefault << std::endl;
                return ret;
            }
            if (settingLibrary->count(name)) {
                auto it = settingLibrary->find(name);
                if (it != settingLibrary->end()) {
                    ret.first = it->first;
                    try {
                        ret.second = it->second.as<T>();
                    } catch(boost::bad_any_cast) {
                        std::cerr << "Settings: Error with the type of the variable " << it->first << std::endl;
                        exit(1);
                    }
                }
            }   

            return ret;
        }

        bool saveToStream(std::ofstream& out) const;
        
        bool saveToFile(const char* filename) const;
        
    };
    
    class SettingsLibrary {
    private:
        std::unordered_map<std::string, Settings*> library;
        
        SettingsLibrary();
        
        int long refCount;
    public:
        virtual ~SettingsLibrary();
        void operator delete (void* p);
        
        static SettingsLibrary* instance;
        static Settings* getInstance(std::string id="Default");
        
        bool saveToFile(const char* filename) const;
    };
    
    
    template<> 
    bool Settings::registerAndRetrieveParameter(unsigned& returnValue, const char* name, unsigned defaultValue, const char* description);
    
}
#endif /* SETTINGS_H */

