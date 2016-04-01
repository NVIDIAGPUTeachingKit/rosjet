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
  var left_motor = new ROSLIB.Topic({
    ros : ros,
    name : '/arduino/motor_left_speed',
    messageType : 'std_msgs/Int16'
  });

  var right_motor = new ROSLIB.Topic({
    ros : ros,
    name : '/arduino/motor_right_speed',
    messageType : 'std_msgs/Int16'
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


$("#forward-button").click(function() {
  var motor_control = new ROSLIB.Message({
      data: -1 * Number($("#speed").val())
  });
  var stop_motor = new ROSLIB.Message({
      data: 0
  });
  console.log($("#speed").val());
  left_motor.publish(motor_control);
  right_motor.publish(motor_control);
  window.setTimeout(function() {
    left_motor.publish(stop_motor);
    right_motor.publish(stop_motor);
  }, $("#duration").val());
});

$("#reverse-button").click(function() {
  var motor_control = new ROSLIB.Message({
      data: Number($("#speed").val())
  });
  var stop_motor = new ROSLIB.Message({
      data: 0
  });
  left_motor.publish(motor_control);
  right_motor.publish(motor_control);
  window.setTimeout(function() {
    left_motor.publish(stop_motor);
    right_motor.publish(stop_motor);
  }, $("#duration").val());
});

$("#left-button").click(function() {
  var motor_left = new ROSLIB.Message({
      data: Math.floor(0.75 * Number($("#speed").val()))
  });
  var motor_right = new ROSLIB.Message({
      data: Math.floor(-0.75 * Number($("#speed").val()))
  });
  var stop_motor = new ROSLIB.Message({
      data: 0
  });
  left_motor.publish(motor_left);
  right_motor.publish(motor_right);
  window.setTimeout(function() {
    left_motor.publish(stop_motor);
    right_motor.publish(stop_motor);
  }, $("#duration").val());
});

$("#right-button").click(function() {
  var motor_left = new ROSLIB.Message({
      data: Math.floor(-0.75 * Number($("#speed").val()))
  });
  var motor_right = new ROSLIB.Message({
      data: Math.floor(0.75 * Number($("#speed").val()))
  });
  var stop_motor = new ROSLIB.Message({
      data: 0
  });
  left_motor.publish(motor_left);
  right_motor.publish(motor_right);
  window.setTimeout(function() {
    left_motor.publish(stop_motor);
    right_motor.publish(stop_motor);
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



