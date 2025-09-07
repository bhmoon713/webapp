var app = new Vue({
  el: '#app',

  computed: {
    ws_address() { return `${this.rosbridge_address}`; }
  },

  data: {
    connected: false,
    loading: false,
    ros: null,
    logs: [],
    topic: null,
    message: null,
    rosbridge_address: 'wss://i-03f2c71ab6b014cd6.robotigniteacademy.com/881af87e-7a3f-47ea-b17a-6449a81dfb6a/rosbridge/',
    port: '9090',

    // Robot Status
    robotStatus: {
      speed: 0.0,
      speedAngular: 0.0,
      position: { x: 0.0, y: 0.0 },
      orientation: 0.0,
      battery: 100,
      lastTwistAt: 0,
      lastOdomAt: 0,
    },

    // Joystick (single source of truth)
    dragging: false,
    handleSize: 44,
    dragCircleStyle: {
      display: 'none',
      left: '50%',
      top: '50%',
      width: '44px',
      height: '44px',
      transform: 'translate(-50%, -50%)'
    },
    joystick: { vertical: 0, horizontal: 0 },

    // ROS topic handles
    twistSub: null,
    odomSub:  null,
    cmdVelTopic: null,
    navGoalTopic: null,
    cmdVelTopicName: '/fastbot_1/cmd_vel',

    // 2D Map
    mapRotated: false,
    mapViewer: null,
    mapGridClient: null,

    // 3D
    viewer: null,
    tfClient: null,
    urdfClient: null,

    // UI / control
    menu_title: 'Connection',
    pubInterval: null,
    buttonsOverride: false,
    manualLinear: 0,
    manualAngular: 0,

    // Navigation
    isNavigating: false,
    estopActive: false,
    controlMode: 'Manual',

    // Waypoints
    waypoints: {
      sofa:         { x: -2.63, y: -0.91, theta: 1.0, name: 'Sofa' },
      living_room:  { x:  1.41, y: -1.93, theta: 1.0, name: 'Living Room' },
      kitchen:      { x:  0.732, y:  2.53, theta: 1.0, name: 'Kitchen' }
    },

    // timers
    interval: null,
    batteryInterval: null
  },

  methods: {
    connect() {
      if (this.loading || this.connected) return;
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
        this.initMap();

        // start manual publish loop AFTER topics are set up
        this.pubInterval = setInterval(this.publish, 100); // ~10 Hz
      });

      this.ros.on('error', (error) => {
        this.logs.unshift(new Date().toTimeString() + ` - Error: ${error}`);
        this.loading = false;
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

    disconnect() { if (this.ros) this.ros.close(); },

    // Publish cmd_vel from joystick or manual buttons
    publish() {
      if (!this.connected || !this.cmdVelTopic) return;

      const useButtons = this.buttonsOverride && !this.estopActive;
      const joyActive =
        Math.abs(this.joystick.vertical)   > 0.01 ||
        Math.abs(this.joystick.horizontal) > 0.01;

      if (this.isNavigating && !useButtons && !joyActive) return;  // let nav drive
      if (!useButtons && !joyActive) return; // don't spam zeros

      const lin = useButtons ? this.manualLinear  : this.joystick.vertical;
      const ang = useButtons ? this.manualAngular : this.joystick.horizontal;

      const msg = new ROSLIB.Message({
        linear:  { x: lin, y: 0, z: 0 },
        angular: { x: 0,  y: 0, z: -ang } // right = +
      });
      this.cmdVelTopic.publish(msg);
    },

    // Optional manual controls (kept for completeness)
    forward()  { this.buttonsOverride = true; this.manualLinear =  0.2; this.manualAngular =  0.0; },
    backward() { this.buttonsOverride = true; this.manualLinear = -0.2; this.manualAngular =  0.0; },
    turnLeft() { this.buttonsOverride = true; this.manualLinear =  0.0; this.manualAngular = -0.5; },
    turnRight(){ this.buttonsOverride = true; this.manualLinear =  0.0; this.manualAngular = +0.5; },
    stop()     { this.buttonsOverride = false; this.manualLinear = 0.0;  this.manualAngular = 0.0; },

    setCamera() {
      const without_wss = (this.rosbridge_address || '').split('wss://')[1] || '';
      const parts = without_wss.split('/');
      const domain = (parts[0] || '') + '/' + (parts[1] || '');
      const host = domain + '/cameras';
      new MJPEGCANVAS.Viewer({
        divID: 'divCamera',
        host,
        width: 320,
        height: 240,
        topic: '/fastbot_1/camera/image_raw',
        ssl: true
      });
    },

    /* ---------- Joystick: centered math & clamping ---------- */
    startDrag(e) {
      this.dragging = true;
      this.updateJoystick(e);
    },

    stopDrag() {
      this.dragging = false;
      this.dragCircleStyle.display = 'none';
      this.resetJoystickVals();
      this.buttonsOverride = false;

      // Publish a short burst of zero for safety
      if (this.connected && this.cmdVelTopic) {
        const zero = new ROSLIB.Message({ linear: { x:0, y:0, z:0 }, angular: { x:0, y:0, z:0 } });
        if (this._zeroTimer) { clearInterval(this._zeroTimer); this._zeroTimer = null; }
        this.cmdVelTopic.publish(zero);
        let count = 0;
        this._zeroTimer = setInterval(() => {
          this.cmdVelTopic.publish(zero);
          if (++count >= 5) { clearInterval(this._zeroTimer); this._zeroTimer = null; }
        }, 50);
      }
    },

    doDrag(e) {
      if (!this.dragging) return;
      this.updateJoystick(e);
    },

    setJoystickVals(dx, dy, max) {
      // normalized [-1,1]; invert Y so up is positive
      this.joystick.horizontal = +(dx / max).toFixed(3);
      this.joystick.vertical   = +(-dy / max).toFixed(3);
    },
    resetJoystickVals() { this.joystick.vertical = 0; this.joystick.horizontal = 0; },

    updateJoystick(e) {
      const pad  = this.$refs.pad || document.getElementById('dragstartzone');
      const rect = pad.getBoundingClientRect();

      const p  = e && e.touches ? e.touches[0] : e;
      const px = p.clientX;
      const py = p.clientY;

      // center of pad in viewport coords
      const cx = rect.left + rect.width  / 2;
      const cy = rect.top  + rect.height / 2;

      // vector from center to pointer
      let dx = px - cx;
      let dy = py - cy;

      // clamp to radius minus handle radius
      const r   = rect.width / 2;                // pad radius (assumes circle)
      const max = r - this.handleSize / 2;       // max travel for handle center
      const len = Math.hypot(dx, dy);
      if (len > max) { dx *= max / len; dy *= max / len; }

      // place handle (centered) in pad-local coords
      const handleX = r + dx;
      const handleY = r + dy;

      this.dragCircleStyle.display = 'inline-block';
      this.dragCircleStyle.left    = handleX + 'px';
      this.dragCircleStyle.top     = handleY + 'px';

      // normalized outputs
      this.setJoystickVals(dx, dy, max);
    },
    /* -------------------------------------------------------- */

    /* ---------- 3D Viewer ---------- */
    setup3DViewer() {
      this.viewer = new ROS3D.Viewer({
        background: '#cccccc',
        divID: 'div3DViewer',
        width: 325,
        height: 280,
        antialias: true,
        fixedFrame: 'fastbot_1/odom'
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

    /* ---------- ROS topics ---------- */
    setupROSCommunication() {
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

    /* ---------- Map ---------- */
    initMap() {
      const el = document.getElementById('map');
      const w = el.clientWidth || 320;
      const h = el.clientHeight || 240;

      this.mapViewer = new ROS2D.Viewer({ divID: 'map', width: w, height: h });
      this.mapGridClient = new ROS2D.OccupancyGridClient({
        ros: this.ros, rootObject: this.mapViewer.scene, continuous: true
      });

      this.mapGridClient.on('change', () => {
        const grid = this.mapGridClient.currentGrid;
        this.mapViewer.scaleToDimensions(grid.width, grid.height);
        this.mapViewer.shift(grid.pose.position.x, grid.pose.position.y);

        // Optional rotate if your map is 90° off
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

      // Keep viewer sized if layout changes
      window.addEventListener('resize', () => {
        const nw = el.clientWidth || w;
        const nh = el.clientHeight || h;
        if (this.mapViewer && this.mapViewer.stage && this.mapViewer.stage.canvas) {
          this.mapViewer.stage.canvas.width  = nw;
          this.mapViewer.stage.canvas.height = nh;
          this.mapViewer.stage.canvas.style.width  = nw + 'px';
          this.mapViewer.stage.canvas.style.height = nh + 'px';
          this.mapViewer.stage.update();
        }
      });
    },

    /* ---------- Waypoints ---------- */
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
  },

  mounted() {
    // mouseup listener to stop dragging if released outside pad
    window.addEventListener('mouseup', this.stopDrag);

    // connection keepalive every 10s
    this.interval = setInterval(() => {
      if (this.ros && this.ros.isConnected) {
        this.ros.getNodes(() => {}, () => {});
      }
    }, 10000);

    // Simulated battery drain
    this.batteryInterval = setInterval(() => {
      this.robotStatus.battery -= Math.random() * 3;
      if (this.robotStatus.battery < 0) this.robotStatus.battery = 0;
      if (this.robotStatus.battery > 100) this.robotStatus.battery = 100;
    }, 5000);
  },

  beforeDestroy() {
    if (this.interval) clearInterval(this.interval);
    if (this.batteryInterval) clearInterval(this.batteryInterval);
    window.removeEventListener('mouseup', this.stopDrag);
  }
});
