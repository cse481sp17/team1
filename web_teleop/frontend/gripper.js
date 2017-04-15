Gripper = function(ros) {
  // HTML elements
  var gripOpenButton = document.querySelector('#gripOpenButton');
  var gripCloseButton = document.querySelector('#gripCloseButton');

  var that = this;

  var setGripperClient = new ROSLIB.Service({
    ros: ros,
    name: '/web_teleop/set_gripper',
    serviceType: 'web_teleop/SetGripper'
  });

  // Method to set the gripper
  this.setGripper = function(open) {
    var request = new ROSLIB.ServiceRequest({
        open: open
    });
    setGripperClient.callService(request);
  };

  // Set gripper to open or losed when button is clicked
  gripOpenButton.addEventListener('click', function() {
    that.setGripper(true);
  });
  gripCloseButton.addEventListener('click', function() {
    that.setGripper(false);
  });
}

