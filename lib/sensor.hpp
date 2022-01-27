#include <termios.h> // Contains POSIX terminal control definitions
#include <iostream>
#include <fcntl.h>    /* For O_RDWR */
#include <unistd.h>   /* For open(), creat() */
#include <list>
#include <nlohmann/json.hpp>

using json = nlohmann::json;
using namespace std;

list<json> getJsonList(unsigned int num=1);

json getJson();

void writeData(int num);