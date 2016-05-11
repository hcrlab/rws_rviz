(function(document) {
  'use strict';

  var app = document.querySelector('#app');

  var websocketUrl = (function() {
    var hostname = window.location.hostname;
    var protocol = 'ws:';
    if (window.location.protocol === 'https:') {
      protocol = 'wss:';
    }
    return protocol + '//' + hostname + ':9090';
  })();

  app.addEventListener('dom-change', function() {
    app.$.ros.connect(websocketUrl);
    var rosElement = app.$.ros;
    rosElement.addEventListener('connection', function() {
      console.log('Connected to the websocket server.');
    });
    rosElement.addEventListener('error', function(error) {
      console.log('Error connecting to the websocket server:', error);
    });
    rosElement.addEventListener('close', function() {
      console.log('Connection to websocket server closed.');
    });
  });
  
  // See https://github.com/Polymer/polymer/issues/1381
  window.addEventListener('WebComponentsReady', function() {
    // imports are loaded and elements have been registered
  });
})(document);
