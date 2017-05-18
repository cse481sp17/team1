Head = function(ros) {
  // HTML elements
  var headX = document.querySelector('#headLookAtX');
  var headY = document.querySelector('#headLookAtY');
  var headZ = document.querySelector('#headLookAtZ');
  var headFrame = document.querySelector('#headLookAtFrame');
  var headLookAtButton = document.querySelector('#headLookAtButton');
  var headPan = document.querySelector('#headPan');
  var headTilt = document.querySelector('#headTilt');
  var headPanTiltButton = document.querySelector('#headPanTiltButton');

  var that = this;

  var headLookAtClient = new ROSLIB.Service({
    ros: ros,
    name: '/web_teleop/set_head_look_at',
    serviceType: 'web_teleop/SetHeadLookAt'
  });

  var headPanTiltClient = new ROSLIB.Service({
    ros: ros,
    name: '/web_teleop/set_head_pan_tilt',
    serviceType: 'web_teleop/SetHeadPanTilt'
  });

  this.setHeadLookAt = function(frame, x, y, z) {
    var request = new ROSLIB.ServiceRequest({
        frame_id: frame,
        x: x,
        y: y,
        z: z
    });
    headLookAtClient.callService(request);
  };

  this.setHeadPanTilt = function(pan, tilt) {
    var request = new ROSLIB.ServiceRequest({
        pan: pan,
        tilt: tilt
    });
    headPanTiltClient.callService(request);
  };

  headLookAtButton.addEventListener('click', function() {
    that.setHeadLookAt(headFrame.value,
                       parseFloat(headX.value),
                       parseFloat(headY.value),
                       parseFloat(headZ.value));
  });

  headPanTiltButton.addEventListener('click', function() {
    that.setHeadPanTilt(parseFloat(headPan.value),
                        parseFloat(headTilt.value));
  });
}

