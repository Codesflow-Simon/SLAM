#include <iostream>
#include <array>
#include <list>
#include <vector>
#include <fstream>
#include <nlohmann/json.hpp>
#include <serial/serial.h>

using json = nlohmann::json;
using namespace std;

list<json> getJson() {
  list<json> output;

  int bufferSize = 1024;
  int bufferIndex = 0;
  char buffer[bufferSize];

  int device = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  cout << sensor.isOpen();

  while (output.size()<10) {
    char data = sensor.read(1)[0];
    buffer[bufferIndex] = data;
    bufferIndex++;

    if (data == '\n') {

      string jsonDump = "";
      for (int i=0; i<bufferIndex; i++) {
        jsonDump += buffer[i];
      }

      try {
        json json_obj = json::parse(jsonDump);
        cout << json_obj << '\n';
        output.push_back(json_obj);      
      } catch (const nlohmann::detail::parse_error) {
        
      }
    }
  }

  sensor.close();
  return output;
}

int main() {
  cout << getJson();
}
