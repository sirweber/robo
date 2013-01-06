// Connecting to ROS
// -----------------
var ros = new ROS();

// If there is an error on the backend, an 'error' emit will be emitted.
ros.on('error', function(error) {
  console.log(error);
});

// Create a connection to the rosbridge WebSocket server.
ros.connect('ws://localhost:9090');


// Subscribing to a Topic
// ----------------------

// Like when publishing a topic, we first create a Topic object with details of
// the topic's node, name, and message type. Note that we can call publish or
// subscribe on the same topic object.
var listener = new ros.Topic({
  name        : '/camera_info',
  messageType : 'sensor_msgs/CameraInfo'
});

// Then we add a callback to be called every time a message is published on this
// topic.
listener.subscribe(function(message) {
  console.log('Received message on ' + listener.name + ': ' + message);

  // If desired, we can unsubscribe from the topic as well.
  listener.unsubscribe();
});


