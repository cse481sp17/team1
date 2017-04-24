Pose = function(ros, name) {
  var that = this;
  this.name = name;

  var pub = new ROSLIB.Topic({
    ros: ros,
    name: '/user_actions',
    messageType: 'map_annotator/UserAction'
  });

  function handleGoTo() {
    console.log('Go to ' + name + ' clicked.');
    var msg = new ROSLIB.Message({
      command: "goto",
      pose_name: name,
      pose_name_new: "None"
    });
    pub.publish(msg);
  }

  function handleDelete() {
    console.log('Delete ' + name + ' clicked.');
    var msg = new ROSLIB.Message({
      command: "delete",
      pose_name: name,
      pose_name_new: "None"
    });
    pub.publish(msg);
  }

  function handleRename(newName) {
    console.log('Rename ' + name + ' clicked.');
    var msg = new ROSLIB.Message({
      command: "rename",
      pose_name: name,
      pose_name_new: newName
    });
    pub.publish(msg);

  }

  this.render = function() {
    var node = document.createElement('div');
    var nameNode = document.createTextNode(name);
    node.appendChild(nameNode)

    var sendNode = document.createElement('input');
    sendNode.type = 'button';
    sendNode.value = 'Go to';
    sendNode.addEventListener('click', handleGoTo);
    node.appendChild(sendNode);

    var deleteNode = document.createElement('input');
    deleteNode.type = 'button';
    deleteNode.value = 'Delete';
    deleteNode.addEventListener('click', handleDelete);
    node.appendChild(deleteNode);

    var renameNode = document.createElement('input');
    renameNode.type = 'button';
    renameNode.value = 'Rename';
    renameNode.addEventListener('click', function() {
        var newName = prompt('Enter a new name for this pose:');
        if (!newName) {
          return;
        }
        handleRename(newName)
    });
    node.appendChild(renameNode);

    return node;
  }
}
