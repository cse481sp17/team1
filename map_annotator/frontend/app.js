App = function() {
  // HTML elements
  var websocketStatus = document.querySelector('#websocketStatus');
  var websocketUrl = document.querySelector('#websocketUrl');
  var websocketButton = document.querySelector('#websocketButton');

  // Compute websocket URL.
  var url = (function() {
    var hostname = window.location.hostname;
    var protocol = 'ws:';
    if (window.location.protocol == 'https:') {
      protocol = 'wss:'
    }
    return protocol + '//' + hostname + ':9090';
  })();
  websocketUrl.value = url;

  // This is a common technique in JavaScript callbacks.
  // If you are not sure what 'this' refers to (and the rules are often unclear,
  // just assign a local variable (named 'that') to 'this' outside the callback
  // and use that variable instead.
  var that = this;

  // Connects to the websocket URL and sets this.ros.
  this.connect = function(url) {
    this.ros = new ROSLIB.Ros({url: url});
    this.ros.on('connection', function() {
      websocketStatus.textContent = 'Connected to websocket server.';
      that.poseList = new PoseList(that.ros);
    });
    this.ros.on('error', function(error) {
      websocketStatus.textContent = 'Error connecting to websocket server.';
    });
    this.ros.on('close', function() {
      websocketStatus.textContent = 'Disconnected from websocket server.';
    });
  }

  // Set up the "Reconnect" button.
  var connectFromButton = function() { that.connect(websocketUrl.value); };
  websocketButton.addEventListener('click', connectFromButton);

  // Initialize app.
  this.connect(url);
};

function init() {
  var app = new App();
}
