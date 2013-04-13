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

var info = new ros.Topic({
  name        : '/camera_info',
  messageType : 'sensor_msgs/CameraInfo'
});

info.subscribe(function(message) {
  console.log('Received message on ' + info.name + ': ' + message.width + 'x' + message.height);

  // If desired, we can unsubscribe from the topic as well.
  info.unsubscribe();
});

var video = new ros.Topic({
  name        : '/image_raw',
  messageType : 'sensor_msgs/Image'
});
video.subscribe(function(message) {
  console.log('Received message on ' + video.name + ': ' + message.width + 'x' + message.height + ' enc=' + message.encoding + ', step=' + message.step);

  //var data=$.base64.encode(message.data);
  var data = message.data;
  $("#image").attr("src","data:image/bmp,"+data);


  // If desired, we can unsubscribe from the topic as well.
  video.unsubscribe();
});
