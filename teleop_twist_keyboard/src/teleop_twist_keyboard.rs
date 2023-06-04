use std::sync::{Arc, Mutex};  // (1)
use geometry_msgs::msg::Twist as TwistMsg;
use std::collections::HashMap;
use anyhow::{Error, Result};
use std::io::{self, Read};
use std::io::Write;
use termios::{Termios, TCSANOW, ECHO, ICANON, tcsetattr};

const  OUT_STRING : &str = "Reading from the keyboard  and Publishing to Twist!
    ---------------------------
    Moving around:
    u    i    o
    j    k    l
    m    ,    .

    For Holonomic mode (strafing), hold down the shift key:
    ---------------------------
    U    I    O
    J    K    L
    M    <    >

    t : up (+z)
    b : down (-z)

    anything else : stop

    q/z : increase/decrease max speeds by 10%
    w/x : increase/decrease only linear speed by 10%
    e/c : increase/decrease only angular speed by 10%

    CTRL-C to quit";



    fn move_bindings() -> HashMap<&'static str, (f64, f64, f64, f64)> {
        let mut move_bindings = HashMap::new();
        move_bindings.insert("i", (1.0, 0.0, 0.0, 0.0));
        move_bindings.insert("o", (1.0, 0.0, 0.0, -1.0));
        move_bindings.insert("j", (0.0, 0.0, 0.0, 1.0));
        move_bindings.insert("l", (0.0, 0.0, 0.0, -1.0));
        move_bindings.insert("u", (1.0, 0.0, 0.0, 1.0));
        move_bindings.insert(",", (-1.0, 0.0, 0.0, 0.0));
        move_bindings.insert(".", (-1.0, 0.0, 0.0, 1.0));
        move_bindings.insert("m", (-1.0, 0.0, 0.0, -1.0));
        move_bindings.insert("O", (1.0, -1.0, 0.0, 0.0));
        move_bindings.insert("I", (1.0, 0.0, 0.0, 0.0));
        move_bindings.insert("J", (0.0, 1.0, 0.0, 0.0));
        move_bindings.insert("L", (0.0, -1.0, 0.0, 0.0));
        move_bindings.insert("U", (1.0, 1.0, 0.0, 0.0));
        move_bindings.insert("<", (-1.0, 0.0, 0.0, 0.0));
        move_bindings.insert(">", (-1.0, -1.0, 0.0, 0.0));
        move_bindings.insert("M", (-1.0, 1.0, 0.0, 0.0));
        move_bindings.insert("t", (0.0, 0.0, 1.0, 0.0));
        move_bindings.insert("b", (0.0, 0.0, -1.0, 0.0));
        move_bindings
    }
    

    fn speed_bindings() -> HashMap<&'static str, (f64, f64)> {
        let mut speed_bindings = HashMap::new();
        speed_bindings.insert("q", (1.1, 1.1));
        speed_bindings.insert("z", (0.9, 0.9));
        speed_bindings.insert("w", (1.1, 1.0));
        speed_bindings.insert("x", (0.9, 1.0));
        speed_bindings.insert("e", (1.0, 1.1));
        speed_bindings.insert("c", (1.0, 0.9));
        speed_bindings
    }

struct PublishTeleopByKey {
    publisher: rclrs::Publisher<TwistMsg>,
    data: Arc<Mutex<Option<TwistMsg>>>,  // (2)
}


impl PublishTeleopByKey 
{    
    fn get_key(&self) -> String  
    {
        let stdin = 0; 
        let termios = Termios::from_fd(stdin).unwrap();
        let mut new_termios = termios.clone();  
        new_termios.c_lflag &= !(ICANON | ECHO); 
        tcsetattr(stdin, TCSANOW, &mut new_termios).unwrap();
        let stdout = io::stdout();
        let mut reader = io::stdin();
        let mut buffer = [0;1];  // read exactly one byte
        stdout.lock().flush().unwrap();
        reader.read_exact(&mut buffer).unwrap();
        return std::str::from_utf8(&buffer).unwrap().to_string();        
    }

    fn new(context: &rclrs::Context) -> Result<Self, rclrs::RclrsError> 
    {
        let node = rclrs::Node::new(context, "teleop_twist_keyboard")?;
        let publisher = node.create_publisher("cmd_vel", rclrs::QOS_PROFILE_DEFAULT)?;
        let data = Arc::new(Mutex::new(None)); 
        Ok(Self {
            publisher,
            data,
        })
    }
    fn update(&self, x:f64, y:f64, z:f64, th:f64, speed:f64, turn:f64)
    {
        let mut data = self.data.lock().unwrap();
        let mut twist_msg = geometry_msgs::msg::Twist::default();
        twist_msg.linear.x = x*speed;
        twist_msg.linear.y = y*speed;
        twist_msg.linear.z = z*speed;
        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        twist_msg.angular.z = th*turn;
        *data = Some(twist_msg);
    }



    fn republish(&self) -> Result<(), rclrs::RclrsError> 
    {
        if let Some(s) = &*self.data.lock().unwrap() {
            self.publisher.publish(s)?;
        }
        Ok(())
    }

}



fn main() -> Result<(), Error> {
    println!("{}",OUT_STRING);
    let context = rclrs::Context::new(std::env::args())?;
    let republisher = Arc::new(PublishTeleopByKey::new(&context)?);
    let republisher_other_thread = Arc::clone(&republisher);
    let speed_limit = 1000.0;
    let turn_limit = 1000.0;
    let mut speed = 0.5;
    let mut turn = 1.0;

    while context.ok() {

        let mut x: f64 = 0.0;
        let mut y: f64 = 0.0;
        let mut z: f64 = 0.0;
        let mut th: f64 = 0.0;
    
        let key=republisher_other_thread.get_key(); 
        let move_bindings = move_bindings();
        let speed_bindings = speed_bindings();
        if let Some(teleop_value) = move_bindings.get(key.as_str()) {
            x = teleop_value.0;
            y = teleop_value.1;
            z = teleop_value.2;
            th =teleop_value.3;
        } 
        if let Some(teleop_value) = speed_bindings.get(key.as_str()) {
            speed = f64::min(speed_limit, speed*teleop_value.0);
            turn = f64::min(turn_limit, turn*teleop_value.1);
        } 
        republisher_other_thread.update(x, y, z, th, speed, turn);
        republisher_other_thread.republish()?;

    }
    Ok(())

}
