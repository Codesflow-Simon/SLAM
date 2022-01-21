#include <iostream>
#include <nlohmann/json.hpp>
#include <list>
#include "lib/sensor.hpp"


using namespace std;

int main() {
    // Prints out mass sensor infomation
    while (true) {
        list<json> jsonObj = getJson(10);
        if (jsonObj == nlohmann::json::parse("{}")) {
            continue;
        } else {
            for (int i=0; i<10; i++) {
                cout << jsonObj.front() << endl;
                jsonObj.pop_front();
            }
        }
    }
    return 0;
}