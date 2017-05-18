Arm = function(ros) {
  // HTML elements
  var strikePoseButton = document.querySelector('#armStrikePoseButton');

  var that = this;

  var armStrikePoseClient = new ROSLIB.Service({
    ros: ros,
    name: '/web_teleop/set_arm_strike_pose',
    serviceType: 'web_teleop/SetArmStrikePose'
  });

  this.setArmStrikePose = function() {
    var request = new ROSLIB.ServiceRequest({});
    armStrikePoseClient.callService(request);
  };

  strikePoseButton.addEventListener('click', function() {
    that.setArmStrikePose();
  });
}

