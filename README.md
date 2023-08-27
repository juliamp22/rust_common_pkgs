# rust_common_pkgs
The aim of this repository is to contribute to the ROS community with the basic nodes developed in Rust
You can find different packages:
1 - **custom_interfaces** : contains how to create a custom .msg which is used in [publish_age_node](https://github.com/juliamp22/rust_common_pkgs/blob/master/topic_publisher_pkg/src/publish_age.rs).
2 - **topic_publisher_pkg**: contains multiple nodes that publishes to topics: */cmd_vel*, */counter* and subscribes to topic */scan*.
3 - **topic_subscriber_pkg**: contains multiple nodes that subscribes to topics: */odom* and */counter*.
4 - **teleop_twist_keyboard**: contains a custom rust node of the common *teleop_twist_keyboard* node.
5 - **my_psackage**: contains a simple example which prints a message.
