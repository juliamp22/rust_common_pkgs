use std::env;
use anyhow::{Error, Result};

fn main() -> Result<(), Error> {
    let context = rclrs::Context::new(env::args())?;

    let node = rclrs::create_node(&context, "simple_publisher")?; 

    let publisher = node.create_publisher::<std_msgs::msg::UInt32>("counter", rclrs::QOS_PROFILE_DEFAULT)?;

    let mut message = std_msgs::msg::UInt32::default();

    while context.ok() {
        message.data += 1;
        publisher.publish(&message)?;
        //println!("Publishing: [{}]", message.data);
        std::thread::sleep(std::time::Duration::from_millis(500));
    }
    Ok(())
}