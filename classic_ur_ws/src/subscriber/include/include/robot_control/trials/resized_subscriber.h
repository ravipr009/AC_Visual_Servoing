#include <image_transport/simple_subscriber_plugin.h>
 #include <robot_control/ResizedImage.h>
 
 class ResizedSubscriber : public image_transport::SimpleSubscriberPlugin<robot_control::ResizedImage>
{
 public:
 virtual ~ResizedSubscriber() {}

   virtual std::string getTransportName() const
  {
     return "resized";
   }
  
protected:
   virtual void internalCallback(const typename robot_control::ResizedImage::ConstPtr& message,
                                const Callback& user_cb);
 };
