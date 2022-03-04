#include <termios.h> // Contains POSIX terminal control definitions
#include <iostream>
#include <fstream>
#include <fcntl.h>    // For O_RDWR
#include <unistd.h>   // For open(), creat()
#include <list>
#include <nlohmann/json.hpp>

using json = nlohmann::json;
using namespace std;

list<json> getJsonList(unsigned int num) {
  list<json> output;

  int USB = open("/dev/ttyS4", O_RDWR| O_NOCTTY);

  // Settings for serial port
  struct termios tty;
  memset (&tty, 0, sizeof tty);

  // Error Handling
  if ( tcgetattr ( USB, &tty ) != 0 ) {
    std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
  }

  // Set Baud Rate 
  cfsetospeed (&tty, (speed_t)B1000000);
  cfsetispeed (&tty, (speed_t)B1000000);

  // Setting other Port Stuff
  tty.c_cflag     &=  ~PARENB;            // Make 8n1
  tty.c_cflag     &=  ~CSTOPB;
  tty.c_cflag     &=  ~CSIZE;
  tty.c_cflag     |=  CS8;

  tty.c_cflag     &=  ~CRTSCTS;           // no flow control
  tty.c_cc[VMIN]   =  1;                  // read doesn't block
  tty.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
  tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

  // Make raw
  cfmakeraw(&tty);

  // Flush Port, then applies attributes
  tcflush( USB, TCIFLUSH );
  if ( tcsetattr ( USB, TCSANOW, &tty ) != 0) {
    std::cout << "Error " << errno << " from tcsetattr" << std::endl;
  }

  // number of bytes read, index, char buffer respectively
  int n = 0,
    spot = 0;
  char buf = '\0';

  // Whole response
  char response[1024];
  memset(response, '\0', sizeof(response));

  // Read "num" many full jsons 
  do {
      n = read( USB, &buf, 1);
      // cout << buf;
      response[spot] = buf;
      spot += n;
      if (buf == '\n') {
        try {
          output.push_back(json::parse(response));
          fill_n(response, spot, '\0');
          spot=0;
        } catch(nlohmann::detail::parse_error const&) {
          fill_n(response, 1024, '\0');  // If error is json communication, clear response
          spot = 0;
          continue;
        }
      }
  } while( output.size()<num && n > 0);

  close(USB);
  return output;
}

json getJson() {
  return getJsonList(1).front();
}

void sendA2a() {
  int USB = open("/dev/ttyS4", O_RDWR| O_NOCTTY);
  write(USB, &"node a2a 10", 11);
}

void writeData(int num) {
  ofstream file;
  file.open ("sensors.csv");
  file << "acc,omega,mag,range\n";
  for (int i=0; i<num; i++) {
    auto data = getJson();
    file << data["acc"] << "," << data["gyro"] << "," << data["mag"] << "," << data["meas"]["d"] << "\n";
  }
  file.close();
}
