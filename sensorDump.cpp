#include <iostream>
#include <nlohmann/json.hpp>
#include <list>
#include "lib/sensor.hpp"


using namespace std;

int main() {
    // Prints out mass sensor infomation
    while (true) {
        json json = getJson();
        if (json == nlohmann::json::parse("{}")) {
            continue;
        } else {
           cout << json << endl;
        }
    }
    return 0;
}