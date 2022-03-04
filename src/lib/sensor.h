#include <termios.h> // Contains POSIX terminal control definitions
#include <iostream>
#include <fstream>
#include <fcntl.h>    // For O_RDWR
#include <unistd.h>   // For open(), creat()
#include <list>
#include <nlohmann/json.hpp>

using json = nlohmann::json;
using namespace std;

list<json> getJsonList(unsigned int num);

json getJson();

void sendA2a();

void writeData(int num);