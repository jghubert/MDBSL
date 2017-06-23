/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   generateFitnessClass.cpp
 * Author: Julien Hubert
 *
 * Created on August 9, 2016, 10:52 AM
 */

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <list>
#include <string>

using namespace std;

bool isPoint(char a)
{
    return a == '.';
}

/*
 * 
 */
int main(int argc, char** argv) 
{
    // argv[1] = filename ; argv[2...] = classes to add
    if (argc < 3)
        return 1;

    std::list<std::string> fitnesses;
    std::string name;
    for (unsigned index = 0; argv[2][index] != '\0'; ++index) {
        if (argv[2][index] == ' ') {
            fitnesses.push_back(name);
            name.clear();
        }
        else
            name.push_back(argv[2][index]);
    }
    fitnesses.push_back(name);
    
    ofstream outfile(argv[1], ios_base::out | ios_base::trunc);
    if (!outfile.is_open())
        return 2;
    
    // Include section
    
    std::string tmp(argv[1]);
    size_t index = tmp.find_last_of('/');
    if (index != std::string::npos)
        tmp = tmp.substr(index+1);
    transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
    replace_if(tmp.begin(), tmp.end(), isPoint, '_');
    outfile << "#ifndef " << tmp << endl;
    outfile << "#define " << tmp << endl;
    outfile << endl;
    
    outfile << "#include <string>" << endl;
    outfile << "#include \"GeneticAlgorithm.hpp\"" << endl;
    for (auto it = fitnesses.begin(); it != fitnesses.end(); ++it)
        outfile << "#include \"../Experiments/" << *it << ".h\"" << endl; 
    outfile << endl;
    
    outfile << "namespace MDB_Social {" << endl << endl;
    outfile << "\t class FitnessLibrary {" << endl;
    outfile << "\t private:" << endl;
    for (auto it = fitnesses.begin(); it != fitnesses.end(); ++it) {
        outfile << "\t\t static GAFitness* get" << *it << "() {" << endl;
        outfile << "\t\t\t return new " << *it << "();" << endl;
        outfile << "\t\t}" << endl;
    }
    outfile << endl;

    auto it = fitnesses.begin();
    outfile << "\t public:" << endl;
    outfile << "\t\t static GAFitness* getFitness(std::string fit) {" << endl;
    outfile << "\t\t\t if (fit == \"" << *it << "\")" << endl;
    outfile << "\t\t\t\t return get" << *it << "();" << endl;
    for (it++; it != fitnesses.end(); ++it) {
        outfile << "\t\t\t else if (fit == \"" << *it << "\")" << endl;
        outfile << "\t\t\t\t return get" << *it << "();" << endl;
    }
    outfile << "\t\t\t else" << endl;
    outfile << "\t\t\t\t return NULL;" << endl;
    outfile << "\t\t }" << endl;
    
    outfile << "\t };" << endl;
    
    outfile << "}" << endl;
    outfile << "#endif" << endl;
    
    outfile << endl;
    outfile.close();
    return 0;
}

