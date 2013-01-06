window.Teleop = (function() {
	
	var botIp = '192.168.1.101';
	var ros;
	
	var kb = {
			ARROW_LEFT : 37,
			ARROW_UP : 38,
			ARROW_RIGHT : 39,
			ARROW_DOWN :40
	};
	
	var pos = {
			x : 0,
			y : 0,
			z : 0
	}
	
	function setupEvents() {
		$(document).keydown(function(e) {
			arrowAction(e.which);
		})
	}
	
	function setupConnection() {
		ros = new ROS();
		ros.connect('ws://'+botIp+':9090');
		ros.on('error', function(err) {
			$('#errorLog').prepend($('<p>'+err+'</p>'));
		})
	}
	
	function getSysInfo() {
		window.setInterval(function() {
			var info = "hello world";
			var dom = $('#rosSysinfo');
			ros.getTopics(function(topics) {
				$('#rosSysinfo .topics').html('' + topics);
			});
			ros.getServices(function(services) {
				$('#rosSysinfo .services').html( '' + services);
			});
			ros.getParams(function(params) {
				$('#rosSysinfo .params').html('' + params);
			});
			
		}, 5000);
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
		    x: pos.x,
		    y: pos.y,
		    z: pos.z
		  },
		  linear: {
		    x: pos.x,
		    y: pos.y,
		    z: pos.z
		  }
		});

		// The geometry_msgs/Twist message will be published in ROS.
		cmdVel.publish(twist);
	}
	
	function arrowAction(event) {
		switch(event) {
		case kb.ARROW_LEFT: //Arrow left
			$('#keyboardLog').prepend($("<p>Send Call: Turn Left.</p>"));
			pos.z += 1;
			break;
		case kb.ARROW_RIGHT: //Arrow Right
			$('#keyboardLog').prepend($("<p>Send Call: Turn Right.</p>"));
			pos.z -= 1;
			
			break;
		case kb.ARROW_UP: //Arrow Up
			$('#keyboardLog').prepend($("<p>Send Call: Drive forward.</p>"));
			pos.x += 0.5;
			
			break;
		case kb.ARROW_DOWN: //Arrow Down
			$('#keyboardLog').prepend($("<p>Send Call: Drive backward.</p>"));
			
			pos.x -= 0.5;
			
			break;
		}
		callMove();
	}
	
	
	return {
		
		init : function() {
			setupConnection();
			getSysInfo();
			setupEvents();
			
		}
	
	}
	
	
})();




$(document).ready(function() {
	Teleop.init();
});