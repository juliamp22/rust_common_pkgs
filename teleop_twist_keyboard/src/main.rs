use std::sync::{Arc, Mutex};  // (1)
use geometry_msgs::msg::Twist as TwistMsg;
use termion::input::TermRead;
use std::collections::HashMap;
use std::io::prelude::*;
use anyhow::{Error, Result};
use std::thread;
use std::time::Duration;
use crossterm::event::{Event, KeyCode};
use crossterm::event::{poll, read};
use std::io::{self, Read};
use termion::async_stdin;
use termion::event::Key;
use ncurses::*;
use rustyline::Editor;
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



fn move_bindings() -> HashMap<&'static str, Vec<f64>>
{
    let mut move_bindings = HashMap::new();
    move_bindings.insert("i", vec![1.0,0.0,0.0,0.0]);
    move_bindings.insert("o", vec![1.0,0.0,0.0,-1.0]);
    move_bindings.insert("j", vec![0.0,0.0,0.0,1.0]);
    move_bindings.insert("l", vec![0.0,0.0,0.0,-1.0]);   
    move_bindings.insert("u", vec![1.0,0.0,0.0,1.0]);
    move_bindings.insert(",", vec![-1.0,0.0,0.0,0.0]);   
    move_bindings.insert(".", vec![-1.0,0.0,0.0,1.0]);   
    move_bindings.insert("m", vec![-1.0,0.0,0.0,-1.0]);   
    move_bindings.insert("O", vec![1.0,-1.0,0.0,0.0]);   
    move_bindings.insert("I", vec![1.0,0.0,0.0,0.0]);   
    move_bindings.insert("J", vec![0.0,1.0,0.0,0.0]);   
    move_bindings.insert("L", vec![0.0,-1.0,0.0,0.0]);    
    move_bindings.insert("U", vec![1.0,1.0,0.0,0.0]); 
    move_bindings.insert("<", vec![-1.0,0.0,0.0,0.0]);         
    move_bindings.insert(">", vec![-1.0,-1.0,0.0,0.0]);   
    move_bindings.insert("M", vec![-1.0,1.0,0.0,0.0]);   
    move_bindings.insert("t", vec![0.0,0.0,1.0,0.0]); 
    move_bindings.insert("b", vec![0.0,0.0,-1.0,0.0]); 
    return move_bindings;
}

fn speed_bindings() -> HashMap<&'static str, Vec<f64>>
{
    let mut speed_bindings = HashMap::new();
    speed_bindings.insert("q", vec![1.1,1.1]);
    speed_bindings.insert("z", vec![0.9,0.9]);
    speed_bindings.insert("w", vec![1.1,1.0]);
    speed_bindings.insert("x", vec![0.9,1.0]);   
    speed_bindings.insert("e", vec![1.0,1.1]);
    speed_bindings.insert("c", vec![1.0,0.9]);            
    return speed_bindings;
}

struct RepublisherNode {
    node: rclrs::Node,
    publisher: rclrs::Publisher<TwistMsg>,
    data: Arc<Mutex<Option<TwistMsg>>>,  // (2)
}


impl RepublisherNode 
{

    fn new(context: &rclrs::Context) -> Result<Self, rclrs::RclrsError> 
    {
        let node = rclrs::Node::new(context, "teleop_twist_keyboard")?;
        let publisher = node.create_publisher("cmd_vel", rclrs::QOS_PROFILE_DEFAULT)?;
        let data = Arc::new(Mutex::new(None));  // (3)
        Ok(Self {
            node,
            publisher,
            data,
        })
    }
    fn update(&self, x:f64, y:f64, z:f64, th:f64, speed:f64, turn:f64)
    {
        println!("holis");
        let mut data = self.data.lock().unwrap();
        let mut twist_msg = geometry_msgs::msg::Twist::default();
        twist_msg.linear.x = x*speed;
        twist_msg.linear.y = y*speed;
        twist_msg.linear.z = z*speed;
        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        twist_msg.angular.z = th*turn;
        println!("angular twist {:?}",twist_msg);
        *data = Some(twist_msg);
    }


    fn republish(&self) -> Result<(), rclrs::RclrsError> 
    {
        println!("republishing");
        if let Some(s) = &*self.data.lock().unwrap() {
            self.publisher.publish(s)?;
        }
        Ok(())
    }

}




fn get_single_key() -> char {
    initscr();
    cbreak();
    noecho();
    let ch = getch();
    endwin();
    ch as u8 as char
}
fn key_code_to_str(key_code: KeyCode) -> &'static str {
    match key_code {
        KeyCode::Char(c) => match c {
            '\n' => "Enter",
            '\t' => "Tab",
            _ => "Char",
        },
        KeyCode::Esc => "Esc",
        KeyCode::Backspace => "Backspace",
        KeyCode::Left => "Left",
        KeyCode::Right => "Right",
        KeyCode::Up => "Up",
        KeyCode::Down => "Down",
        // Add more cases for other key codes if needed
        _ => "Unknown",
    }
}
  

fn main() -> Result<(), Error> {
    println!("{}",OUT_STRING);
    let context = rclrs::Context::new(std::env::args())?;
    let republisher = Arc::new(RepublisherNode::new(&context)?);
    let republisher_other_thread = Arc::clone(&republisher);
    let mut publish_count: u32 = 1;

    while context.ok() {
        let mut teleop_vector: Vec<f64> = vec![];

        let stdin = 0; // couldn't get std::os::unix::io::FromRawFd to work 
                       // on /dev/stdin or /dev/tty
        let termios = Termios::from_fd(stdin).unwrap();
        let mut new_termios = termios.clone();  // make a mutable copy of termios 
                                                // that we will modify
        new_termios.c_lflag &= !(ICANON | ECHO); // no echo and canonical mode
        tcsetattr(stdin, TCSANOW, &mut new_termios).unwrap();
        let stdout = io::stdout();
        let mut reader = io::stdin();
        let mut buffer = [0;1];  // read exactly one byte
        print!("Hit a key! ");
        stdout.lock().flush().unwrap();
        reader.read_exact(&mut buffer).unwrap();
        println!("You have hit: {:?}", buffer);
        tcsetattr(stdin, TCSANOW, & termios).unwrap();  // reset the stdin to 
                                                        // original termios data
        println!("You have hit: {}", std::str::from_utf8(&buffer).unwrap());                       
        if let Some(teleop_value) = move_bindings().get(std::str::from_utf8(&buffer).unwrap()) {
            teleop_vector = teleop_value.clone();
            // Step 5: Retrieve the corresponding vector value
            println!("Teleop value: {:?}", teleop_value);
            println!("Value cloned: {:?}", teleop_vector);
        } 
        println!("Teleop cloned: {:?}", teleop_vector);

        republisher_other_thread.update(teleop_vector[0], teleop_vector[1], teleop_vector[2], teleop_vector[3], 1.0, 1.0);
        republisher_other_thread.republish()?;
        publish_count += 1;
        std::thread::sleep(std::time::Duration::from_millis(500));

    }
    Ok(())

}
