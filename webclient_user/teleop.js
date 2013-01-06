window.teleop = (function() {
	
	function setupEvents() {
		
	}
	
	function setupConnection() {
		var ros = new ROS('ws://localhost:9090');
		ros.connect('ws://localhost:9090');
	}
	
	function callMove() {
		// ros.Topic provides publish and subscribe support for a ROS topic.
		// Creates a geometry_msgs/Twist topic named /cmd_vel.
		var cmdVel = new ros.Topic({
		  name        : '/cmd_vel',
		  messageType : 'geometry_msgs/Twist'
		});

		// ros.Message contains the data to publish.
		var twist = new ros.Message({
		  angular: {
		    x: 1,
		    y: 0,
		    z: 0
		  },
		  linear: {
		    x: 0,
		    y: 0,
		    z: 0
		  }
		});

		// The geometry_msgs/Twist message will be published in ROS.
		cmdVel.publish(twist);
	}
	
	function arrowAction(event) {
		switch(event) {
		case 37: //Arrow left
			
			break;
		case 38: //Arrow Right
			
			break;
		case 39: //Arrow Up
			
			break;
		case 40: //Arrow Down
			
			
			break;
		}
	}
	
	
	return {
		
		init : function() {
			
			
		}
	
	}
	
	
})();




$(document).ready(function() {
	init();
});