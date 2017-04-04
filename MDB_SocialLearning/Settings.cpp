/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Settings.cpp
 * Author: Julien Hubert
 * 
 * Created on June 20, 2016, 3:10 PM
 */

#include "Settings.h"
#include <iostream>
#include <string>
#include <fstream>
#include "colormod.hpp"

namespace MDB_Social {

    SettingsLibrary* SettingsLibrary::instance = NULL;

    Settings::Settings()
    {
        loaded = false;

        argc = 0;
        argv = NULL;
        
        settingLibrary = new po::variables_map();
        cmdLineDescriptions.add_options() ("help,h", "Print this help");
        cmdLineDescriptions.add_options() ("dumpSettings,d", po::value<std::string>()->default_value(std::string("settings.log")), "Save the configuration in specified filename");
        
    }
    
    Settings::~Settings()
    {
        delete settingLibrary;
    }
    
    SettingsLibrary::SettingsLibrary() {
        refCount = 0;
        
    }

    void SettingsLibrary::operator delete (void* p)
    {
        SettingsLibrary* pc = static_cast<SettingsLibrary*>(p);
        pc->refCount--;
        
        if (!pc->refCount) {
            for (auto it = pc->library.begin(); it != pc->library.end(); ++it)
                delete it->second;
            pc->library.clear();
            instance = NULL;
        }
    }
    
    
    SettingsLibrary::~SettingsLibrary() {
    }
    
    bool SettingsLibrary::saveToFile(const char* filename) const
    {
        std::ofstream out(filename, std::ios_base::trunc | std::ios_base::out);
        if (out.is_open()) {
            for (auto it = library.begin(); it != library.end(); ++it) {
                out << "// "<< it->first << std::endl;
                it->second->saveToStream(out);
                std::cout << std::endl;
            }
            return true;
        }
        else {
            std::cerr << "SettingsLibrary: Impossible to open filename " << filename << std::endl;
            return false;
        }
    }
    
    
    Settings* SettingsLibrary::getInstance(std::string id) {
        if (!instance) {
            instance = new SettingsLibrary;
            Settings* tmp = new Settings();
            instance->library.insert(std::make_pair(id, tmp));
        }
        Settings* ret;
        if (instance->library.count(id) > 0)
            ret = instance->library[id];
        else {
            ret = new Settings();
            instance->library.insert(std::make_pair(id, ret));
        }
        
        instance->refCount++;
        return ret;
    }

    void Settings::setParameterSources(const char* _filename, int _argc, char* _argv[])
    {
        if (_filename)
            configFilename = std::string(_filename);
        else {
            std::cerr << Color::Modifier(Color::FG_RED) << "Settings: ERROR: no configuration filename provided. Using defaults and arguments only. If no argument is provided, it will crash." << Color::Modifier(Color::FG_DEFAULT) << std::endl;
        }
        argc = _argc;
        argv = _argv;
    }

    std::pair<std::string, std::string> Settings::findInFile(const char* name)
    {
        std::pair<std::string, std::string> ret;
        std::ifstream infile(configFilename);
        if (!infile.is_open())
            return ret;
        
        
        std::string line;
        std::string section;
        size_t word, equal;

        std::string strname(name);
        // parse the name to find the section if exists
        word = strname.find('.');
        if (word != std::string::npos) {
            section = strname.substr(0, word);
            strname.erase(0, word+1);
        }


        bool found = false;
        std::string currentSection;
        do {
            std::getline(infile, line);
//            std::cout << " **** line = " << line << std::endl;
            if (infile.good()) {
                // Let's clean the line from starting and ending with spaces
                unsigned spacecount = 0;
                while (spacecount < line.size() && line[spacecount] == ' ')
                    ++spacecount;
                if (spacecount < line.size()) {
                    line.erase(0, spacecount);
                    spacecount = line.size()-1;
                    while (line[spacecount] == ' ')
                        spacecount--;
                    line.erase(spacecount+1);
                }
                
                if (line[0] == '[') {
                    // new section
                    currentSection.clear();
                    currentSection.reserve(line.size());
                    for (unsigned i=1; i<line.size() && line[i] !=']'; ++i) {
                        if (line[i] != ' ')
                            currentSection.push_back(line[i]);
                    }
//                    std::cout << " **** currentSection = " << currentSection << " ; section = " << section << std::endl;
                        
                }
                else {
                    word = std::string::npos;
                    if (currentSection.empty()) {
                        word = line.find(name);
                    }
                    else if (!section.empty() && currentSection == section) {
                        word = line.find(strname);
                    }
                    
                    if (word != std::string::npos) {
                        equal = line.find("=");
                        if (equal != std::string::npos) {
                            // We found a match.
                            ret.first = section+'.'+strname;
                            ret.second = line.substr(equal+1);
//                            std::cout << "  ***** ret = " << ret.first << " : " << ret.second << std::endl;
                            spacecount = 0;
                            while (spacecount < ret.second.size() && ret.second[spacecount] == ' ')
                                ++spacecount;
                            if (spacecount < ret.second.size())
                                ret.second.erase(0, spacecount);
                            found = true;
                        }
                    }
                }
            }
            
        } while (!found && infile.good());
        
        
        infile.close();
        return ret;
    }

    std::pair<std::string, std::string> Settings::findInArgv(const char* name) 
    {
        std::pair<std::string, std::string> ret;
        
        if (argc <= 2)   // Need the name of the executable and the name of a configuration file
            return ret;
        
        bool found = false;
        char* current;
        unsigned index = 0;
        for (unsigned i=2; i<argc && !found; ++i) {
            current = argv[i];
            index = 0;
            if (current[index] == '-') {
                if (current[index+1] == '-') 
                    index+=2;
                
                while (current[index] != '\0' && name[index] != '\0' && current[index]==name[index])
                    index++;
                
                found = (current[index]==name[index]);   // should be \0 == \0 if found
                if (found) {
                    if (i+1 < argc && argv[i+1][0] != '-') {
                        ret.first = std::string(name);
                        ret.second = std::string(argv[i+1]);
                    }
                }
            }
        }
        return ret;
    }
    
    enum Settings::data_type Settings::identifyValueType(const std::string& value) const
    {
        enum data_type type = UNKNOWN;
        
        if (!value.length())
            return type;
        
        if (value == "true" || value == "false")
            return BOOL;
        
        bool decimal = false;
        bool numbers = true;

        // Numbers: starts with -, has e{-,+}
        // TODO : add support for e{-,+}
        
        unsigned i = value[0]=='-' ? 1 : 0;   // skip - if present at start
        for (; i<value.length() && numbers; ++i) {
            if (value[i] == '.')
                if (decimal)   // 2 decimal dots have been detected
                    numbers = false;
                else
                    decimal = true;
            else {
                if (value[i] <'0' || value[i] > '9')
                    numbers = false;
            }
        }

        if (!numbers)
            type = STRING;
        else if (decimal)
            type = DOUBLE;
        else
            type = INTEGER;

        return type;
    }

    void Settings::registerCallbackFunction(void (callMeWhenLoaded)(void*), void* c)
    {
        callbacks.push_back(std::make_pair(callMeWhenLoaded, c));
        if (loaded)
            callMeWhenLoaded(c);
    }

    
    void Settings::registerUnknownKeywords(const std::vector<po::basic_option<char> >& keywords, po::options_description& desc)
    {
        for (unsigned i=0; i<keywords.size(); ++i) {

            if (keywords[i].unregistered) {
                // Try to identify the type and add the keyword if not registered prior
                std::cout << "Settings::registerUnknownKeywords : keywords[i].value[0] = " << keywords[i].value[0] << std::endl;
                enum data_type dt = identifyValueType(keywords[i].value[0]);
                switch(dt) {
                    case STRING:
                        desc.add_options() (keywords[i].string_key.c_str(), "");
                        break;
                    case INTEGER:
                        desc.add_options() (keywords[i].string_key.c_str(), po::value<int>(), "");
                        break;
                    case DOUBLE:
                        desc.add_options() (keywords[i].string_key.c_str(), po::value<double>(), "");
                        break;
                    case BOOL:
                        desc.add_options() (keywords[i].string_key.c_str(), po::value<bool>(), "");
                        break;
                    default:
                        std::cerr << "Settings: Ignoring unknown keyword [" << keywords[i].string_key << "]." << std::endl;
                        break;
                }
            }
        }
        
    }
        
    bool Settings::processAllParameters(const char* filename, int argc, char* argv[])
    {
        // We first parse the command line, and then the files. 
        
        // First create a common repository
        mainDescriptions.add(fileDescriptions);
        mainDescriptions.add(cmdLineDescriptions);
        
        po::basic_parsed_options<char> file_parsed_options(&mainDescriptions);
        po::basic_parsed_options<char> cmd_parsed_options(&mainDescriptions);
        settingLibrary->clear();
        try {
            if (argc > 1) {
                auto style = po::command_line_style::allow_short | po::command_line_style::allow_dash_for_short | po::command_line_style::short_allow_adjacent | po::command_line_style::long_allow_adjacent | po::command_line_style::allow_long;
                cmd_parsed_options = po::command_line_parser(argc,argv).allow_unregistered().style(style).options(mainDescriptions).run();
                registerUnknownKeywords(cmd_parsed_options.options, mainDescriptions);
                cmd_parsed_options = po::command_line_parser(argc,argv).allow_unregistered().style(style).options(mainDescriptions).run();
                po::store(cmd_parsed_options, *settingLibrary);
            }

            if (filename) {
                file_parsed_options = po::parse_config_file<char>(filename, mainDescriptions, true);
                registerUnknownKeywords(file_parsed_options.options, mainDescriptions);
                file_parsed_options = po::parse_config_file<char>(filename, mainDescriptions, true);
                po::store(file_parsed_options, *settingLibrary);
            }
            

            po::notify(*settingLibrary);
            
        }
        catch (po::error& e) {  //unknown_option
            std::cerr << "Settings: Error with the parameters: " << e.what() << std::endl;
            return false;
        }

        loaded = true;
        
        for (std::list<std::pair<std::function<void(void*)>, void*> >::iterator it = callbacks.begin(); it != callbacks.end(); ++it)
            it->first(it->second);
        
        return true;
        
    }
    
    bool Settings::isKeywordRegistered(const char* name) const
    {
        return settingLibrary->count(name) > 0;
    }

    bool Settings::printHelpIfRequested() const
    {
        if (settingLibrary->count("help") || settingLibrary->count("h") || settingLibrary->count("help,h")) {
            std::cout << mainDescriptions << std::endl;
            return true;
        }
        else
            return false;
        
    }

    bool Settings::saveSettingsToFileRequested() const
    {
        if (settingLibrary->count("dumpSettings") || settingLibrary->count("d") || settingLibrary->count("dumpSettings,d")) {
//            std::string filename;
            auto it = settingLibrary->find("dumpSettings");
            if (it == settingLibrary->end()) {
                it = settingLibrary->find("d");
                if (it == settingLibrary->end())
                    it = settingLibrary->find("dumpSettings,d");
            }
            saveToFile(it->second.as<std::string>().c_str());
            return true;
        }
        else
            return false;
        
    }
    
    
    bool Settings::saveToStream(std::ofstream& out) const
    {
        for (auto it = settingLibrary->begin(); it != settingLibrary->end(); ++it) {
            out << it->first << " = ";  //as<std::string>()
            if (it->second.value().type() == boost::typeindex::type_id<std::string>())
                    out << it->second.as<std::string>();
            else if (it->second.value().type() == boost::typeindex::type_id<unsigned>()) {
                    out << it->second.as<unsigned>();
            }
            else if (it->second.value().type() == boost::typeindex::type_id<float>()) {
                    out << it->second.as<float>();
            }
            else if (it->second.value().type() == boost::typeindex::type_id<double>()) {
                    out << it->second.as<double>();
            }
            else if (it->second.value().type() == boost::typeindex::type_id<unsigned long>()) {
                    out << it->second.as<unsigned long>();
            }
            else if (it->second.value().type() == boost::typeindex::type_id<long int>()) {
                    out << it->second.as<long int>();
            }
            else if (it->second.value().type() == boost::typeindex::type_id<int>()) {
                    out << it->second.as<int>();
            }
            else if (it->second.value().type() == boost::typeindex::type_id<bool>()) {
                    out << it->second.as<bool>();
            }
            out << std::endl;
//            it->second.value().type()
        }
        return true;
    }

    bool Settings::saveToFile(const char* filename) const
    {
        std::ofstream out(filename, std::ios_base::app | std::ios_base::out);
        if (out.is_open())
            return saveToStream(out);
        else {
            std::cerr << "Settings: Impossible to open file " << filename << std::endl;
            return false;
        }
    }
    
    
    template<> 
    bool Settings::registerAndRetrieveParameter(unsigned& returnValue, const char* name, unsigned defaultValue, const char* description)
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
            returnValue = std::stoul(param.second);

        }

        return registerParameter<unsigned>(name, defaultValue, description);

    }

    
}