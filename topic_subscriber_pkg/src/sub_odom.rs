use std::sync::{Arc, Mutex};
use nav_msgs::msg::Odometry as Odom;

struct SubscriberNode { 
    node: rclrs::Node,
    _subscription: Arc<rclrs::Subscription<Odom>>,
}

impl SubscriberNode 
{
    fn new(context: &rclrs::Context) -> Result<Self, rclrs::RclrsError> 
    {
        let mut node = rclrs::Node::new(context, "simple_subscriber")?;
        let data = Arc::new(Mutex::new(None));  // (3)
        let data_cb = Arc::clone(&data);
        let _subscription = {
            node.create_subscription(
                "odom",
                rclrs::QOS_PROFILE_DEFAULT,
                move |msg: Odom| {
                    odom_callback(msg.clone());
                    *data_cb.lock().unwrap() = Some(msg);  
                },
            )?
        };
        fn odom_callback(msg:Odom)
        {
            println!("I heard: '{:?}'", msg.pose.pose.position.x);
        }
        Ok(Self {
            node,
            _subscription
        })

    }
}

fn main() -> Result<(), rclrs::RclrsError> {
    let context = rclrs::Context::new(std::env::args())?;
    let subscriber = SubscriberNode::new(&context)?;
    rclrs::spin(&subscriber.node)
}