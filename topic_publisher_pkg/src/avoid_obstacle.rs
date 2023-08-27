use std::sync::{Arc, Mutex};
use sensor_msgs::msg::LaserScan as LaserScan;
use geometry_msgs::msg::Twist as Twist;
use std::time::Duration;

struct AvoidObstacle {
    node: rclrs::Node,
    publisher: rclrs::Publisher<Twist>,
    _subscription: Arc<rclrs::Subscription<LaserScan>>,
    data_pub: Arc<Mutex<Option<Twist>>>,
}

impl AvoidObstacle {
    fn new(context: &rclrs::Context) -> Result<Self, rclrs::RclrsError> {
        let mut node = rclrs::Node::new(context, "avoid_obstacle")?;
        let data_pub = Arc::new(Mutex::new(None)); 
        let data_cb = Arc::clone(&data_pub);
        let publisher = node.create_publisher("cmd_vel", rclrs::QOS_PROFILE_DEFAULT)?;
        let _subscription = {
            node.create_subscription(
                "scan",
                rclrs::QOS_PROFILE_DEFAULT,
                move |msg: LaserScan| {
                    let ranges = msg.ranges;
                    let mut twist_message = Twist::default();
                    if (ranges[ranges.len()*(3/4)] < 1.0) || (ranges[ranges.len()/2] < 1.0)
                    {
                        twist_message.angular.z = 1.0;
                        println!("Moving left to avoid the object located on the right {:}", ranges[ranges.len()/2]);
                    }
                    else if ranges[ranges.len()*(1/4)] < 1.0
                    {
                        twist_message.angular.z = -1.0;
                         println!("Moving right to avoid the object located on the left {:}", ranges[ranges.len()/2]);
                    }
                    else
                    {
                        twist_message.linear.x = 0.5;
                        twist_message.angular.z = 0.0;
                        println!("Moving forward {:}", ranges[ranges.len()/2]);
                    }
                    *data_cb.lock().unwrap() = Some(twist_message);  

                    println!("Processing Lidar data...");
                },
            )?
        };
        Ok(Self {
            node,
            publisher,
            _subscription,
            data_pub
        })
    }

    fn publish(&self) -> Result<(), rclrs::RclrsError> {  
        println!("hello");
        if let Some(s) = &*self.data_pub.lock().unwrap() {
            println!("The twist sent is {:?}", s);
            self.publisher.publish(s)?;
        }
        Ok(())
    }
}

fn main() -> Result<(), rclrs::RclrsError> {
    let context = rclrs::Context::new(std::env::args())?;
    let republisher = Arc::new(AvoidObstacle::new(&context)?);
    let republisher_other_thread = Arc::clone(&republisher);
    std::thread::spawn(move || -> Result<(), rclrs::RclrsError> {
        loop {
            std::thread::sleep(Duration::from_millis(1000));
            republisher_other_thread.publish()?;
        }
    });
    rclrs::spin(&republisher.node)
}