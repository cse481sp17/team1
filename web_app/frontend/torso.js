Torso = function(ros) {
  // HTML elements
  var torsoHeight = document.querySelector('#torsoHeight');
  var desiredTorsoHeight = document.querySelector('#desiredTorsoHeight');
  var torsoSlider = document.querySelector('#torsoSlider');
  var torsoButton = document.querySelector('#torsoButton');

  var that = this;

  var setTorsoClient = new ROSLIB.Service({
    ros: ros,
    name: '/web_teleop/set_torso',
    serviceType: 'web_teleop/SetTorso'
  });

  // Listen to torso height from the joint_state_republisher.
  var listener = new ROSLIB.Topic({
    ros: ros,
    name: 'joint_state_republisher/torso_lift_joint',
    messageType: 'std_msgs/Float64'
  });

  listener.subscribe(function(message) {
    // Whenever we get a message with a new torso height, update
    // the torso height display on the webpage.
    var height = message.data;

    // Note the noise in the data. You can smooth it out using this line of code.
    // height = Math.round(height*1000) / 1000
    torsoHeight.textContent = height;
  });
  
  // Initialize slider.
  var desiredHeight = 0.1;
  desiredTorsoHeight.textContent = desiredHeight;
  // For most input elements, the .value field is both a getter and a setter.
  // Here we can set its value to the default (0.1).
  torsoSlider.value = desiredHeight;

  // Update desiredHeight when slider moves.
  torsoSlider.addEventListener('input', function() {
    // Read where the slider is now.
    desiredHeight = torsoSlider.value;
    // Update the desired torso height display.
    desiredTorsoHeight.textContent = desiredHeight;
  });

  // Method to set the height.
  this.setHeight = function(height) {
    var height = Math.min(Math.max(0.0, height), 0.4);
    var request = new ROSLIB.ServiceRequest({
      height: height
    });
    setTorsoClient.callService(request);
  };

  // Set the height when the button is clicked.
  torsoButton.addEventListener('click', function() {
    that.setHeight(desiredHeight);
  });
}

