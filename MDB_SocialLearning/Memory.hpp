
#ifndef MEMORY_HPP
#define MEMORY_HPP

#include <deque>
#include <mutex>
#include <iostream>
#include <fstream>
#include <tuple>
#include "RobotID.h"

namespace MDB_Social {

    class NeuralNetworkData {
    public:
        NeuralNetworkData();
        NeuralNetworkData(const NeuralNetworkData& v);
        virtual ~NeuralNetworkData();
        
        NeuralNetworkData& operator=(const NeuralNetworkData& v);
        
        unsigned from;
        unsigned to;
        double weight;

        friend std::ostream& operator<<(std::ostream& output, const NeuralNetworkData& T)
        {
            output << T.from << MDBSL_FIELD_SEPARATOR << T.to << MDBSL_FIELD_SEPARATOR << T.weight;
            return output;
        }
        
        friend std::istream& operator>>(std::istream& input, NeuralNetworkData& T)
        {
            unsigned u;
            double d;
            input >> u;
            if (input.good())
                T.from = u;
            input >> u;
            if (input.good())
                T.to = u;
            input >> d;
            if (input.good())
                T.weight = d;

            return input;
        }
    };
    
    template <class dtype>
    class Memory
    {
        private:
            std::deque<dtype> data;
            std::mutex mtx;
            
            unsigned maximumSize;
        public:
            Memory(size_t n=0) {
                if (n) {
                    mtx.lock();
                    data.resize(n);
                    mtx.unlock();
                }
            }
            
            virtual ~Memory() {}

            Memory<dtype>& operator=(const Memory<dtype>& m) {
                this->maximumSize = m.maximumSize;
                this->data = m.data;
                return *this;
            }
            
            virtual void push_front(dtype& d) {
                mtx.lock();
                data.push_front(d);
                mtx.unlock();
            }

            virtual void push_back(dtype& d) {
                mtx.lock();
                data.push_back(d);
                mtx.unlock();
            }
            
            virtual void pop_front() {
                mtx.lock();
                data.pop_front();
                mtx.unlock();
            }

            virtual void pop_back() {
                mtx.lock();
                data.pop_back();
                mtx.unlock();
            }
            
            virtual dtype front() {
                mtx.lock();
                dtype tmp = data.front();
                mtx.unlock();
                return tmp;
            }
            
            virtual dtype back() {
                mtx.lock();
                dtype& tmp = data.back();
                mtx.unlock();
                return tmp;
            }
            
            virtual dtype& operator[] (unsigned index) {
                mtx.lock();
                dtype& tmp = data[index];
                mtx.unlock();
                return tmp;
            }
                
            virtual void clear() {
                mtx.lock();
                data.clear();
                mtx.unlock();
            }
            
            virtual size_t size() const {
                return data.size();
            }

            typedef typename std::deque<dtype>::iterator iterator;
            typedef typename std::deque<dtype>::const_iterator const_iterator;
            typedef typename std::deque<dtype>::reverse_iterator reverse_iterator;
            typedef typename std::deque<dtype>::const_reverse_iterator const_reverse_iterator;
            
            virtual iterator begin()
            {
                return data.begin();
            }

            virtual const_iterator begin() const
            {
                return data.begin();
            }

            virtual reverse_iterator rbegin()
            {
                return data.rbegin();
            }

            virtual const_reverse_iterator rbegin() const
            {
                return data.rbegin();
            }
            
            virtual iterator end()
            {
                return data.end();
            }
            
            virtual const_iterator end() const
            {
                return data.end();
            }
            
            virtual reverse_iterator rend()
            {
                return data.rend();
            }

            virtual const_reverse_iterator rend() const
            {
                return data.rend();
            }
            
            virtual bool saveToStream(std::ostream& outstream) const {
                for (auto it = data.begin(); it != data.end(); ++it)
                    outstream << *it << std::endl;
                return true;
            }
            
            virtual bool saveToFile(const char* filename, bool truncate = false) const
            {
                std::ofstream outfile;
                if (truncate)
                    outfile.open(filename, std::ios_base::trunc | std::ios_base::out);
                else
                    outfile.open(filename, std::ios_base::app | std::ios_base::out);
                
                if (!outfile.is_open()) {
                    std::cerr << "Memory:: Impossible to open the file [" << filename << "] to save the memory." << std::endl;
                    return false;
                }
                
                bool ret = saveToStream(outfile);
                
                outfile.close();
                
                return ret;
                
            }
            
            virtual bool loadFromStream(std::istream& instream, bool clearBeforeLoading = false) {
                if (clearBeforeLoading)
                    this->clear();
                
//                std::cout << "loadFromStream : " << std::endl;
                dtype value;
                do {
                    instream >> value;
                    if (instream.good()) {
//                        std::cout << value << std::endl;
                        this->push_back(value);
                    }
                } while (instream.good());

                return true;
            }
            
            virtual bool loadFromFile(const char* filename, bool clearBeforeLoading = false) {
                std::ifstream infile(filename);
                if (!infile.is_open()) {
                    std::cerr << "Memory:: Impossible to open the file [" << filename << "] to load the memory." << std::endl;
                    return false;
                }

                bool ret = loadFromStream(infile, clearBeforeLoading);

                infile.close();
                return ret;
            }
            
            void setMaximumSize(unsigned m) {
                maximumSize = m;
            }
            
            unsigned getMaximumSize() const {
                return maximumSize;
            }
            
            void trimMemory() {
                if (maximumSize > 0) {  // 0 means unlimited memory
                    int surplus = data.size() - maximumSize;
                    if (surplus > 0)
                        data.erase(data.begin(), data.begin()+surplus);
                }
            }
            
    };


    
}


#endif
