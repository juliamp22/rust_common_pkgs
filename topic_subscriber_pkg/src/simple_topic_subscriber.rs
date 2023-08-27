use std::sync::{Arc, Mutex};
use std_msgs::msg::Int32 as Int32;

struct SubscriberNode { // Defining the class SubscriberNode
    node: rclrs::Node,
    _subscription: Arc<rclrs::Subscription<Int32>>,
}

impl SubscriberNode // Implementing the class SubscriberNode
{
    fn new(context: &rclrs::Context) -> Result<Self, rclrs::RclrsError> 
    {
        // Initiate a Node called 'simple_subscriber'
        let mut node = rclrs::Node::new(context, "simple_subscriber")?;
        let data = Arc::new(Mutex::new(None));  // (3)
        let data_cb = Arc::clone(&data);
        // Create a Subscriber object that will listen to the /counter topic and will execute
        // the callback function
        let _subscription = {
            // Create a new shared pointer instance that will be owned by the closure
            node.create_subscription(
                "counter",
                rclrs::QOS_PROFILE_DEFAULT,
                move |msg: Int32| {
                    // This subscription now owns the data_cb variable
                    topic_callback(msg.clone());
                    *data_cb.lock().unwrap() = Some(msg);  // (4)
                },
            )?
        };
        // Define a function called 'topic_callback' that receives a parameter named 'msg'
        fn topic_callback(msg:Int32)
        {
            println!("I heard: '{}'", msg.data);
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
    //Create a loop that will keep the program in execution
    rclrs::spin(&subscriber.node)
}