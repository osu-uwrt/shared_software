#include <ros/ros.h>
#include <geometry_msgs/Accel.h>
#include <termios.h>
#include <signal.h>
#include <stdio.h>

#define KEYCODE_W 0x77
#define KEYCODE_S 0x73
#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_Q 0x71
#define KEYCODE_R 0x72

int kfd = 0;
struct termios cooked, raw;
char c;
bool updated = false;

// ACCEL CLASS DEFINITIONS
class Accel
{
  private:
    ros::NodeHandle nh;
    ros::Publisher accels;
    geometry_msgs::Accel accel;


  public:
    Accel();
    void process_keys();
    void loop();
};

Accel::Accel()
{
  accels = nh.advertise<geometry_msgs::Accel>("command/accel", 1);
}

/**
* Sets acceleration based on keyboard input. Play with this method.
*/
void Accel::process_keys()
{
  // Get keyboard event
  if (read(kfd, &c, 1) < 0)
  {
    perror("read():");
    exit(-1);
  }

  // NOT IMPLEMENTED
  
}

void Accel::loop()
{
  ros::Rate rate(10);
  while(ros::ok())
  {
    Accel::process_keys();
    rate.sleep();
  }
}
// END ACCEL CLASS DEFINITIONS

void quit(int signal)
{
  (void)signal;
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "key_accel");
  Accel accel;

  // Resource management and setting up keyboard input stream
  signal(SIGINT, quit);

  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);

  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);
  ROS_INFO("Begin.");
  accel.loop();
  return(0);
}
