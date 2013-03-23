#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <stdio.h>

using namespace sensor_msgs;

int main(int argc, char **argv)
{
    std::cout << "Starting..." << std::endl;

    ros::init(argc, argv, "camera_sim");
    ros::NodeHandle n("~");
    std::string name;
    n.getParam("topic_name", name);
    std::cout << "Topic is: " << name << std::endl;

    ros::Publisher pub = n.advertise<sensor_msgs::Image>(name, 1000);
    ros::Rate loop_rate(10);

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "camera";
    unsigned int seq = 0;

    while (ros::ok())
    {
        ImagePtr img(new Image);

        header.seq = seq++;
        img->header = header;
        img->width = 640;
        img->height = 320;
        img->encoding = image_encodings::RGB8;
        img->is_bigendian = false;
        img->step = 640*3;
        // This should resize the data vector to my image
        img->data.resize(img->step * img->height);

        pub.publish(img);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
