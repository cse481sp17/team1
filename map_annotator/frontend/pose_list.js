PoseList = function(ros) {
  // HTML elements
  var poseListDiv = document.querySelector('#poseList');
  var createButton = document.querySelector('#createButton');

  var that = this;

  var pub = new ROSLIB.Topic({
    ros: ros,
    name: '/user_actions',
    messageType: 'map_annotator/UserAction'
  });

  var sub = new ROSLIB.Topic({
    ros: ros,
    name: '/pose_names',
    messageType: 'map_annotator/PoseNames'
  });

  var render = function(poseList) {
    if (poseList.length == 0) {
      poseListDiv.textContent = "No poses."
    } else {
      poseListDiv.innerHTML = '';
      for (var i=0; i<poseList.length; ++i) {
        var pose = new Pose(ros, poseList[i]);
        var poseDiv = pose.render();
        poseListDiv.appendChild(poseDiv);
      }
    }
  }

  sub.subscribe(function(message) {
    console.log("message", message);
    render(message.names);
  });
  render([]);

  function handleSave(name) {
    console.log('Creating pose with name' + name);
    var msg = new ROSLIB.Message({
      command: "save",
      pose_name: name,
      pose_name_new: ""
    });
    pub.publish(msg);
  }

  createButton.addEventListener('click', function() {
    var name = prompt('Enter a name for this pose:');
    if (!name) {
      return;
    }
    handleSave(name)
  })
}
