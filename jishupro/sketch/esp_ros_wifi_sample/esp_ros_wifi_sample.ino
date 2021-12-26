#include <ros.h>
#include <std_msgs/String.h>
#include <WiFi.h>

const char SSID[] = "aaaaa";
const char PASSWORD[] = "aaaaa";
IPAddress server(192,168,1,15);
const uint16_t serverPort = 11411;

WiFiClient client;

class WiFiHardware {

  public:
  WiFiHardware() {};

  void init() {
    // do your initialization here. this probably includes TCP server/client setup
    client.connect(server, serverPort);
  }

  // read a byte from the serial port. -1 = failure
  int read() {
    // implement this method so that it reads a byte from the TCP connection and returns it
    //  you may return -1 is there is an error; for example if the TCP connection is not open
    return client.read();         //will return -1 when it will works
  }

  // write data to the connection to ROS
  void write(uint8_t* data, int length) {
    // implement this so that it takes the arguments and writes or prints them to the TCP connection
    for(int i=0; i<length; i++)
      client.write(data[i]);
  }

  // returns milliseconds since start of program
  unsigned long time() {
     return millis(); // easy; did this one for you
  }
};


ros::NodeHandle_<WiFiHardware> nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[] = "hello world from ESP34!";

void setup()
{
  Serial.begin(57600);
  WiFi.begin(SSID,PASSWORD);
    Serial.print("WiFi connecting");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }

  Serial.println(" connected");
  nh.initNode();
  nh.advertise(chatter);
  delay(10);
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}
