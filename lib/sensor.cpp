#include <termios.h> // Contains POSIX terminal control definitions
#include <iostream>
#include <fcntl.h>    /* For O_RDWR */
#include <unistd.h>   /* For open(), creat() */
#include <list>
#include <nlohmann/json.hpp>

using json = nlohmann::json;
using namespace std;

json parseJsonSafe(char* dump) {
  try {
    return json::parse(dump);
  } catch(nlohmann::detail::parse_error const&) {}
  return json::parse("{}");
}

list<json> getJson(unsigned int num=1) {
  list<json> output;

  int USB = open("/dev/ttyS4", O_RDWR| O_NOCTTY);

  struct termios tty;
  memset (&tty, 0, sizeof tty);

  /* Error Handling */
  if ( tcgetattr ( USB, &tty ) != 0 ) {
    std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
  }

  /* Set Baud Rate */
  cfsetospeed (&tty, (speed_t)B1000000);
  cfsetispeed (&tty, (speed_t)B1000000);

  /* Setting other Port Stuff */
  tty.c_cflag     &=  ~PARENB;            // Make 8n1
  tty.c_cflag     &=  ~CSTOPB;
  tty.c_cflag     &=  ~CSIZE;
  tty.c_cflag     |=  CS8;

  tty.c_cflag     &=  ~CRTSCTS;           // no flow control
  tty.c_cc[VMIN]   =  1;                  // read doesn't block
  tty.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
  tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

  /* Make raw */
  cfmakeraw(&tty);

  /* Flush Port, then applies attributes */
  tcflush( USB, TCIFLUSH );
  if ( tcsetattr ( USB, TCSANOW, &tty ) != 0) {
    std::cout << "Error " << errno << " from tcsetattr" << std::endl;
  }

  int n = 0,
    spot = 0;
  char buf = '\0';

  /* Whole response*/
  char response[1024];
  memset(response, '\0', sizeof(response));

  do {
      n = read( USB, &buf, 1);
      sprintf( &response[spot], "%c", buf );
      spot += n;
      if (buf == '\n') {
        output.push_back(parseJsonSafe(response));
        fill_n(response, spot, (char)0);
        spot=0;
      }
  } while( output.size()<num && n > 0);

  close(USB);
  return output;
}

// int main() {
//   cout << getJson();
//   return 0;
// }
