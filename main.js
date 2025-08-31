var app = new Vue({
  el: '#app',

  computed: {
    ws_address() { return `${this.rosbridge_address}`; },
  },

  data: {
    connected: false,
    ros: null,
    logs: [],
    loading: false,
    topic: null,
    message: null,
    rosbridge_address: 'wss://i-0a6c68ffe37aa397b.robotigniteacademy.com/7c1b40e8-0db2-4023-be99-bc4666011555/rosbridge/',
    port: '9090',

    // Robot Status (shown in sidebar)
    robotStatus: {
      speed: 0.0,
      speedAngular: 0.0,
      position: { x: 0.0, y: 0.0 },
      orientation: 0.0,
      battery: 100,
      lastTwistAt: 0,
      lastOdomAt: 0,
    },

    // ROS topic handles (top-level so we can unsubscribe cleanly)
    twistSub: null,
    odomSub:  null,
    cmdVelTopic: null,
    navGoalTopic: null,
    cmdVelTopicName: '/fastbot_1/cmd_vel',

    // 2D stuff
    mapRotated: false,
    mapViewer: null,
    mapGridClient: null,
    interval: null,

    // 3D stuff
    viewer: null,
    tfClient: null,
    urdfClient: null,

    // page content
    menu_title: 'Connection',

    // joystick
    dragging: false,
    x: 'no',
    y: 'no',
    dragCircleStyle: { margin: '0px', top: '0px', left: '0px', display: 'none', width: '75px', height: '75px' },
    joystick: { vertical: 0, horizontal: 0 },

    // manual publisher
    pubInterval: null,
    buttonsOverride: false,
    manualLinear: 0,
    manualAngular: 0,

    // Navigation
    isNavigating: false,
    estopActive: false,

    // Waypoints
    waypoints: {
      sofa:         { x: -2.63, y: -0.91, theta: 1.0, name: 'Sofa' },
      living_room:  { x:  1.41, y: -1.93, theta: 1.0, name: 'Living Room' },
      kitchen:      { x:  0.732, y:  2.53, theta: 1.0, name: 'Kitchen' }
    }
  },

  methods: {
    connect() {
      this.loading = true;
      this.ros = new ROSLIB.Ros({ url: this.rosbridge_address, groovyCompatibility: false });

      this.ros.on('connection', () => {
        this.logs.unshift(new Date().toTimeString() + ' - Connected!');
        this.connected = true;
        this.loading = false;

        this.setupROSCommunication();
        this.autoDetectCmdVelTopic().then(name => this.subscribeTwist(name));

        this.setup3DViewer();
        this.setCamera();

        this.mapViewer = new ROS2D.Viewer({ divID: 'map', width: 380, height: 360 });
        this.mapGridClient = new ROS2D.OccupancyGridClient({
          ros: this.ros, rootObject: this.mapViewer.scene, continuous: true,
        });
        this.mapGridClient.on('change', () => {
          this.mapViewer.scaleToDimensions(this.mapGridClient.currentGrid.width,
                                           this.mapGridClient.currentGrid.height);
          this.mapViewer.shift(this.mapGridClient.currentGrid.pose.position.x,
                               this.mapGridClient.currentGrid.pose.position.y);
          if (!this.mapRotated) {
            const canvas = document.querySelector('#map canvas');
            if (canvas) {
              canvas.style.transform = 'rotate(90deg)';
              canvas.style.transformOrigin = 'center center';
              const mapDiv = document.getElementById('map');
              if (mapDiv) mapDiv.style.overflow = 'visible';
              this.mapRotated = true;
            }
          }
        });

        // start manual publish loop AFTER topics are set up
        this.pubInterval = setInterval(this.publish, 100); // ~10 Hz
      });

      this.ros.on('error', (error) => {
        this.logs.unshift(new Date().toTimeString() + ` - Error: ${error}`);
      });

      this.ros.on('close', () => {
        this.logs.unshift(new Date().toTimeString() + ' - Disconnected!');
        this.connected = false;
        this.loading = false;

        this.unset3DViewer();
        if (this.pubInterval) { clearInterval(this.pubInterval); this.pubInterval = null; }
        const map = document.getElementById('map'); if (map) map.innerHTML = '';

        try { if (this.twistSub) this.twistSub.unsubscribe(); } catch(e) {}
        try { if (this.odomSub)  this.odomSub.unsubscribe(); }  catch(e) {}
        this.twistSub = null;
        this.odomSub  = null;
        this.cmdVelTopic = null;
        this.navGoalTopic = null;
      });
    },

    // SINGLE publish() (removed the duplicate)
    publish() {
      if (!this.connected || !this.cmdVelTopic) return;

      const useButtons = this.buttonsOverride && !this.estopActive;
      const joyActive =
        Math.abs(this.joystick.vertical)   > 0.01 ||
        Math.abs(this.joystick.horizontal) > 0.01;

      // Let Nav2 drive unless user explicitly overrides
      if (this.isNavigating && !useButtons && !joyActive) return;

      // Don't spam zeros
      if (!useButtons && !joyActive) return;

      const lin = useButtons ? this.manualLinear  : this.joystick.vertical;
      const ang = useButtons ? this.manualAngular : this.joystick.horizontal;

      const msg = new ROSLIB.Message({
        linear:  { x: lin, y: 0, z: 0 },
        angular: { x: 0,  y: 0, z: -ang } // right = +
      });
      this.cmdVelTopic.publish(msg);
    },

    disconnect() { this.ros && this.ros.close(); },

    setTopic() {
      this.topic = new ROSLIB.Topic({
        ros: this.ros,
        name: '/fastbot_1/cmd_vel',
        messageType: 'geometry_msgs/msg/Twist' // ROS 2
      });
    },

    forward()  { this.buttonsOverride = true; this.manualLinear =  0.2; this.manualAngular =  0.0; },
    backward() { this.buttonsOverride = true; this.manualLinear = -0.2; this.manualAngular =  0.0; },
    turnLeft() { this.buttonsOverride = true; this.manualLinear =  0.0; this.manualAngular = -0.5; },
    turnRight(){ this.buttonsOverride = true; this.manualLinear =  0.0; this.manualAngular = +0.5; },
    stop()     { this.buttonsOverride = false; this.manualLinear = 0.0;  this.manualAngular = 0.0; },

    setCamera() {
      const without_wss = this.rosbridge_address.split('wss://')[1];
      const domain = without_wss.split('/')[0] + '/' + without_wss.split('/')[1];
      const host = domain + '/cameras';
      new MJPEGCANVAS.Viewer({
        divID: 'divCamera', host, width: 500, height: 360,
        topic: '/fastbot_1/camera/image_raw', ssl: true,
      });
    },

    // joystick handlers
    sendCommand() {
      const topic = new ROSLIB.Topic({
        ros: this.ros,
        name: '/fastbot_1/cmd_vel',
        messageType: 'geometry_msgs/msg/Twist'
      });
      topic.publish(new ROSLIB.Message({
        linear: { x: 0.2, y: 0, z: 0 },
        angular:{ x: 0,   y: 0, z: 0.5 }
      }));
    },
    startDrag() { this.dragging = true; this.x = this.y = 0; },
    stopDrag()  { this.dragging = false; this.x = this.y = 'no'; this.dragCircleStyle.display = 'none'; this.resetJoystickVals(); },
    doDrag(event) {
      if (!this.dragging) return;
      this.x = event.offsetX; this.y = event.offsetY;
      const ref = document.getElementById('dragstartzone');
      this.dragCircleStyle.display = 'inline-block';
      const minTop  = ref.offsetTop  - parseInt(this.dragCircleStyle.height) / 2;
      const top     = this.y + minTop;
      this.dragCircleStyle.top  = `${top}px`;
      const minLeft = ref.offsetLeft - parseInt(this.dragCircleStyle.width) / 2;
      const left    = this.x + minLeft;
      this.dragCircleStyle.left = `${left}px`;
      this.setJoystickVals();
    },
    setJoystickVals() {
      this.joystick.vertical   = -((this.y / 200) - 0.5);
      this.joystick.horizontal =  +((this.x / 200) - 0.5);
    },
    resetJoystickVals() { this.joystick.vertical = 0; this.joystick.horizontal = 0; },

    setup3DViewer() {
      this.viewer = new ROS3D.Viewer({
        background: '#cccccc', divID: 'div3DViewer', width: 340, height: 280,
        antialias: true, fixedFrame: 'fastbot_1/odom'
      });
      this.viewer.addObject(new ROS3D.Grid({ color:'#0181c4', cellSize: 0.5, num_cells: 20 }));
      this.tfClient = new ROSLIB.TFClient({
        ros: this.ros, angularThres: 0.01, transThres: 0.01, rate: 10.0, fixedFrame: 'fastbot_1_base_link'
      });
      this.urdfClient = new ROS3D.UrdfClient({
        ros: this.ros, param: '/fastbot_1_robot_state_publisher:robot_description',
        tfClient: this.tfClient, path: location.origin + location.pathname,
        rootObject: this.viewer.scene, loader: ROS3D.COLLADA_LOADER_2
      });
    },
    unset3DViewer() { const d = document.getElementById('div3DViewer'); if (d) d.innerHTML = ''; },

    setupROSCommunication() {
      // subs & pubs used elsewhere
      this.twistSub = new ROSLIB.Topic({
        ros: this.ros, name: '/fastbot_1/cmd_vel', messageType: 'geometry_msgs/msg/Twist'
      });
      this.cmdVelTopic = new ROSLIB.Topic({
        ros: this.ros, name: '/fastbot_1/cmd_vel', messageType: 'geometry_msgs/msg/Twist'
      });
      this.twistSub.subscribe((msg) => {
        this.robotStatus.speed        = Number((msg.linear  && msg.linear.x)  || 0);
        this.robotStatus.speedAngular = Number((msg.angular && msg.angular.z) || 0);
        this.robotStatus.lastTwistAt  = Date.now();
      });

      this.odomSub = new ROSLIB.Topic({
        ros: this.ros, name: '/fastbot_1/odom', messageType: 'nav_msgs/msg/Odometry'
      });
      this.odomSub.subscribe((odom) => {
        const p = odom.pose.pose.position;
        const q = odom.pose.pose.orientation;
        this.robotStatus.position = { x: Number((p && p.x) || 0), y: Number((p && p.y) || 0) };
        const siny_cosp = 2 * ((q.w||0) * (q.z||0) + (q.x||0) * (q.y||0));
        const cosy_cosp = 1 - 2 * ((q.y||0) * (q.y||0) + (q.z||0) * (q.z||0));
        const yaw = Math.atan2(siny_cosp, cosy_cosp);
        this.robotStatus.orientation = (yaw * 180) / Math.PI;
        this.robotStatus.lastOdomAt  = Date.now();
      });

      this.navGoalTopic = new ROSLIB.Topic({
        ros: this.ros, name: '/goal_pose', messageType: 'geometry_msgs/msg/PoseStamped'
      });
    },

    goToWaypoint(waypointKey) {
      if (!this.connected || !this.navGoalTopic) return;
      const wp = this.waypoints[waypointKey];
      this.isNavigating = true;
      this.controlMode = 'Navigation to ' + wp.name;

      const now = Date.now();
      const sec = Math.floor(now / 1000);
      const nsec = (now % 1000) * 1e6;

      const half = wp.theta / 2.0;
      const qz = Math.sin(half);
      const qw = Math.cos(half);

      const goal = new ROSLIB.Message({
        header: { frame_id: 'map', stamp: { sec, nanosec: nsec } },
        pose: { position: { x: wp.x, y: wp.y, z: 0.0 }, orientation: { x: 0.0, y: 0.0, z: qz, w: qw } }
      });
      this.navGoalTopic.publish(goal);

      setTimeout(() => { this.isNavigating = false; this.controlMode = 'Manual'; }, 5000);
    },

    emergencyStop() {
      if (!this.connected || !this.cmdVelTopic) return;
      this.estopActive = true;
      this.isNavigating = false;
      this.controlMode = 'EMERGENCY STOP';

      const zero = new ROSLIB.Message({ linear: { x:0, y:0, z:0 }, angular: { x:0, y:0, z:0 } });
      let count = 0;
      const t = setInterval(() => {
        this.cmdVelTopic.publish(zero);
        if (++count >= 20) { clearInterval(t); this.estopActive = false; this.controlMode = 'Manual'; }
      }, 50);
    },

    autoDetectCmdVelTopic() {
      return new Promise((resolve) => {
        this.ros.getTopics((res) => {
          const list = (res && res.topics) || [];
          const prefer = [
            '/fastbot_1/cmd_vel_smoothed','/cmd_vel_smoothed','/fastbot_1/cmd_vel','/cmd_vel','/nav2_controller/cmd_vel'
          ];
          const found = prefer.find(t => list.includes(t)) || list.find(t => /cmd_vel/.test(t));
          resolve(found || this.cmdVelTopicName);
        }, () => resolve(this.cmdVelTopicName));
      });
    },

    subscribeTwist(name) {
      if (this.twistSub) { try { this.twistSub.unsubscribe(); } catch(e) {} this.twistSub = null; }
      this.cmdVelTopicName = name;
      this.twistSub = new ROSLIB.Topic({
        ros: this.ros, name, messageType: 'geometry_msgs/msg/Twist'
      });
      this.twistSub.subscribe((msg) => {
        const lin = Number((msg.linear  && msg.linear.x) || 0);
        const ang = Number((msg.angular && msg.angular.z) || 0);
        this.robotStatus.speed        = lin;
        this.robotStatus.speedAngular = ang;
        this.robotStatus.lastTwistAt  = Date.now();
      });
    },
  },

mounted() {
// your existing mouseup listener
window.addEventListener('mouseup', this.stopDrag);

// existing connection check every 10s
this.interval = setInterval(() => {
    if (this.ros && this.ros.isConnected) {
    this.ros.getNodes(() => {}, () => {});
    }
}, 10000);

// --- Simulated battery drain every 5s ---
this.batteryInterval = setInterval(() => {
    // subtract 1–3% randomly
    this.robotStatus.battery -= Math.random() * 3;

    // clamp between 0 and 100
    if (this.robotStatus.battery < 0) this.robotStatus.battery = 0;
    if (this.robotStatus.battery > 100) this.robotStatus.battery = 100;
}, 5000);
},
beforeDestroy() {
// clean up timers
if (this.interval) clearInterval(this.interval);
if (this.batteryInterval) clearInterval(this.batteryInterval);
window.removeEventListener('mouseup', this.stopDrag);
},

});
