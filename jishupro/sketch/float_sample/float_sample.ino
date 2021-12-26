#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

ros::NodeHandle  nh;
std_msgs::Float32MultiArray vec3_msg;
ros::Publisher pub_vec("vec3", &vec3_msg);

void setup()
{
    nh.initNode();
    nh.advertise(pub_vec);
    vec3_msg.data = (float*)malloc(sizeof(float) * 9);
    vec3_msg.data_length = 9;
}

void loop()
{
    for (int i = 0; i < 9; ++i)
    {
        vec3_msg.data[i] = 0.1 * i;
    }
    pub_vec.publish(&vec3_msg);
    nh.spinOnce();
    delay(100);
}
