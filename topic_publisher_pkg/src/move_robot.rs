use std::sync::{Arc, Mutex};
use geometry_msgs::msg::Twist as Twist;
use std::time::Duration;

struct MoveRobot {
    publisher: rclrs::Publisher<Twist>,
    data: Arc<Mutex<Option<Twist>>>,
    sleep_duration_ms: u64
}

impl MoveRobot {
    fn new(context: &rclrs::Context) -> Result<Self, rclrs::RclrsError> {
        let node = rclrs::Node::new(context, "move_robot")?;
        let data = Arc::new(Mutex::new(None)); 
        let publisher = node.create_publisher("cmd_vel", rclrs::QOS_PROFILE_DEFAULT)?;
        let sleep_duration_ms = 1000; //ms
        Ok(Self {
            publisher,
            data,
            sleep_duration_ms
        })
    }
    
    fn update_msg(&self, x: f64, z: f64)
    {
        let mut twist_msg: Twist = Twist::default();
        twist_msg.linear.x = x;
        twist_msg.angular.z = z;
        let mut data = self.data.lock().unwrap();
        *data = Some(twist_msg);
    }
    
    fn publish(&self) -> Result<(), rclrs::RclrsError> {  
        if let Some(s) = &*self.data.lock().unwrap() {
            self.publisher.publish(s)?;
        }
        Ok(())
    }
}

fn main() -> Result<(), rclrs::RclrsError> {
    let context = rclrs::Context::new(std::env::args())?;
    let republisher = Arc::new(MoveRobot::new(&context)?);
    while context.ok() {
        republisher.update_msg(0.2,0.2);
        republisher.publish()?;
        std::thread::sleep(Duration::from_millis(republisher.sleep_duration_ms));
    }
    Ok(())
}