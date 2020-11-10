#include <pluginlib/class_list_macros.h>
 #include <robot_control/resized_publisher.h>
 #include <robot_control/resized_subscriber.h>

PLUGINLIB_REGISTER_CLASS(resized_pub, ResizedPublisher, image_transport::PublisherPlugin)

 PLUGINLIB_REGISTER_CLASS(resized_sub, ResizedSubscriber, image_transport::SubscriberPlugin)
