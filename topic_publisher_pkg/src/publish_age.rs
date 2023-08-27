use std::sync::{Arc, Mutex};
use custom_interfaces::msg::Age as Age;
use std::time::Duration;

//This example creates a subclass of Node and uses a sleep of 1s between publishes.

struct PublishAge {
    publisher: rclrs::Publisher<Age>,
    data: Arc<Mutex<Option<Age>>>,
    sleep_duration_ms: u64
}

impl PublishAge {
    fn new(context: &rclrs::Context) -> Result<Self, rclrs::RclrsError> {
        let node = rclrs::Node::new(context, "move_robot")?;
        let data = Arc::new(Mutex::new(None)); 
        let publisher = node.create_publisher("age", rclrs::QOS_PROFILE_DEFAULT)?;
        let sleep_duration_ms = 1000; //ms
        Ok(Self {
            publisher,
            data,
            sleep_duration_ms
        })
    }
    
    fn update_msg(&self, years: f32, months: f32, days: f32)
    {
        let mut message: Age = Age::default();
        message.years = years;
        message.months = months;
        message.days = days;
        let mut data = self.data.lock().unwrap();
        *data = Some(message);
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
    let republisher = Arc::new(PublishAge::new(&context)?);
    while context.ok() {
        republisher.update_msg(4.0,11.0,21.0);
        republisher.publish()?;
        std::thread::sleep(Duration::from_millis(republisher.sleep_duration_ms));
    }
    Ok(())
}