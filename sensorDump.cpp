#include <iostream>
#include <nlohmann/json.hpp>
#include <list>
#include "lib/sensor.hpp"


using namespace std;

int main() {
    while (true) {
        list<json> json_obj = getJson(10);
        if (json_obj == nlohmann::json::parse("{}")) {
            continue;
        } else {
            for (int i=0; i<10; i++) {
                cout << json_obj.front() << endl;
                json_obj.pop_front();
            }
        }
    }
    return 0;
}