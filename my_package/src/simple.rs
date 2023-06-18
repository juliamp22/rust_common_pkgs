use std::env;

use anyhow::{Error, Result};

fn main() -> Result<(), Error> {
    //Initialize the ROS2 communication
    let context = rclrs::Context::new(env::args())?;

    //Create a ROS2 node named ObiWan
    let node = rclrs::create_node(&context, "ObiWan")?;

    //Endless loop until Ctrl + C

    while context.ok() 
    {
        //Print a message to the terminal
        println!("Help me {:} Kenobi, you're my only hope", node.name());
        //We sleep the needed time to maintain the Rate of 2Hz
        std::thread::sleep(std::time::Duration::from_millis(500));
    }

    //Shutdown the ROS2 communication
    Ok(())
}