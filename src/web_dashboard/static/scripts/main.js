// Connecting to ROS
  // -----------------

  var ros = new ROSLIB.Ros({
    url : 'ws://' + location.host.replace(/:.*/, ":9090")
  });

  ros.on('connection', function() {
    console.log('Connected to websocket server.');
  });

  ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
  });

  ros.on('close', function() {
    console.log('Connection to websocket server closed.');
  });

  /*
   * Topics
   */
  var cmdVel = new ROSLIB.Topic({
    ros: ros,
    name : '/cmd_vel',
    messageType: 'geometry_msgs/Twist'
  });

  var twist = new ROSLIB.Message({
    linear: {
	x : 0.0,
	y : 0.0,
        z : 0.0
    },
    angular: {
        x : 0.0,
        y : 0.0,
        z : 0.0
    }
  });

  /*
   * Services
   */
  var startRecordingClient = new ROSLIB.Service({
    ros: ros,
    name: '/data_recorder/start_recording',
    serviceType : 'std_srvs/Empty'
  });

  var stopRecordingClient = new ROSLIB.Service({
    ros: ros,
    name: '/data_recorder/stop_recording',
    serviceType : 'std_srvs/Emtpy'
  });

  var saveImageClient = new ROSLIB.Service({
    ros: ros,
    name: '/data_recorder/save_image',
    serviceType : 'std_srvs/Empty'
  });
$("#stop-button").click(function() {
  twist.linear.x = 0.0;
  twist.angular.z = 0.0;
  cmdVel.publish(twist);
  console.log(twist);
});

$("#forward-button").click(function() {
  twist.linear.x = -Number($("#speed").val()) / 100;
  twist.angular.z = 0.0;
  cmdVel.publish(twist);
  window.setTimeout(function() {
    twist.linear.x = 0.0;
    cmdVel.publish(twist);
  }, $("#duration").val());
});

$("#reverse-button").click(function() {
  twist.linear.x = Number($("#speed").val()) / 100;
  twist.angular.z = 0.0;
  cmdVel.publish(twist);
  window.setTimeout(function() {
    twist.linear.x = 0.0;
    cmdVel.publish(twist);
  }, $("#duration").val());
});

$("#left-button").click(function() {
  twist.linear.x = 0.0;
  twist.angular.z = Number($("#speed").val()) / 100;
  cmdVel.publish(twist);
  window.setTimeout(function() {
    twist.angular.z = 0.0;
    cmdVel.publish(twist);
  }, $("#duration").val());
});

$("#right-button").click(function() {
  twist.linear.x = 0.0;
  twist.angular.z = -Number($("#speed").val()) / 100;
  cmdVel.publish(twist);
  window.setTimeout(function() {
    twist.angular.z = 0.0;
    cmdVel.publish(twist);
  }, $("#duration").val());
});

$("#vid-source").change(function() {
  $("#videofeed").remove();
  $("#capture-row").hide();
  $("#record-button").data("recording", false);
  $("#record-button").css("color", "green");
  if($("#vid-source").val() != 'none') {
    $("#feedholder").append("<img id='videofeed'></img>")
    $("#videofeed").attr("src", "http://" + location.host.replace(/:.*/, ":8080") + "/stream?topic=" + $("#vid-source").val());
    $("#capture-row").show();
  }
});


$("#record-button").data("recording", false);
$("#record-button").click(
  function() {
    if($("#record-button").data("recording") === false) {
      $("#record-time").data("start", new Date());
      $("#record-button").data("recording", true);
      $("#record-button").css("color", "red");
      $("#record-time").html("");
      startRecordingClient.callService(new ROSLIB.ServiceRequest({}), function(result) {});
    }
    else {
      stopRecordingClient.callService(new ROSLIB.ServiceRequest({}), function(result) {});
      $("#record-time").data("start", new Date());
      $("#record-button").data("recording", false);
      $("#record-button").css("color", "green");
    }
  });

setInterval(function(){
  if($("#record-button").data("recording")) {
    var elapsed = new Date() - $("#record-time").data("start");
    $("#record-time").html(moment(elapsed,"x").format("mm:ss"));
  }
  else {
    $("#record-time").html("");
  }
}, 200);


$("#photo-button").click(function() {
  var request = new ROSLIB.ServiceRequest({});
  saveImageClient.callService(request, function(result) {});
});



