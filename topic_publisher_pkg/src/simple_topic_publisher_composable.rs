use std::sync::{Arc, Mutex};
use std_msgs::msg::Int32 as Int32;
use std::time::Duration;

struct PublisherNode {
    publisher: rclrs::Publisher<Int32>,
    data: Arc<Mutex<Option<Int32>>>,
    sleep_duration_ms: u64
}

impl PublisherNode {
    fn new(context: &rclrs::Context) -> Result<Self, rclrs::RclrsError> {
        let node = rclrs::Node::new(context, "republisher")?;
        let data = Arc::new(Mutex::new(None)); 
        let publisher = node.create_publisher("counter", rclrs::QOS_PROFILE_DEFAULT)?;
        let sleep_duration_ms = 1000; //ms
        Ok(Self {
            publisher,
            data,
            sleep_duration_ms
        })
    }
    
    fn update_msg(&self, counter:i32)
    {
        let mut u32_msg: Int32 = Int32::default();
        u32_msg.data = counter;
        let mut data = self.data.lock().unwrap();
        *data = Some(u32_msg);
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
    let republisher = Arc::new(PublisherNode::new(&context)?);
    let mut counter: i32 = 0;
    while context.ok() {
        counter +=1;
        republisher.update_msg(counter);
        republisher.publish()?;
        std::thread::sleep(Duration::from_millis(republisher.sleep_duration_ms));
    }
    Ok(())
}