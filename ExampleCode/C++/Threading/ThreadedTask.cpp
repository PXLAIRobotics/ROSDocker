/**
 * To compile run:
 * $ g++ -Wall -pedantic -std=c++11 ThreadedTask.cpp -o ThreadedTask
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
