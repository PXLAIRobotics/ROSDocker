/**
 * To compile run:
 * $ g++ -Wall -pedantic -std=c++14 ThreadedTask.cpp -o ThreadedTask -pthread
 *
 * Melodic has C++14 as a Targeted Languages!
 * Source: 
 *    https://www.ros.org/reps/rep-0003.html#melodic-morenia-may-2018-may-2023
 */
#include <iostream>
#include <thread>
 
class ThreadedTask
{
    public:
        void execute(std::string command) {
            for(int counter = 0; counter < 5; counter++) {
                std::cout<<command<<" :: "<<counter<<std::endl;
            }
        }
};
 
int main(void) {
	ThreadedTask * threadedTask = new ThreadedTask();
 
	std::thread thread(&ThreadedTask::execute, threadedTask, "Sample Task");
 
	thread.join();
 
	delete threadedTask;
	return 0;
}
