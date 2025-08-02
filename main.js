var app = new Vue({
    el: '#app',
    // storing the state of the page
    data: {
        connected: false,
        ros: null,
        logs: [],
        loading: false,
        rosbridge_address: 'wss://i-085b4acb4e840851d.robotigniteacademy.com/168606c0-8f5d-40f1-bb20-86ceb48094d8/rosbridge/',
        port: '9090',
        // Robot Status for Task 2
        robotStatus: {
            speed: 0.0,
            position: { x: 0.0, y: 0.0 },
            orientation: 0.0,
            battery: 100
        },
        controlMode: 'Manual',
        isNavigating: false,
        
        // Visualization loading states
        mapLoaded: false,
        robot3DLoaded: false,
        cameraLoaded: false,
        cameraStatus: 'Connecting to camera...',
        // 3D stuff
        viewer: null,
        tfClient: null,
        urdfClient: null,
        // 2D map
        map2D: null,
        // Camera
        cameraViewer: null,
        // Joystick
        joystick: null,
        
        // speed
        lastPose: { x: null, y: null, t: null },

        
        // ROS Topics
        cmdVelTopic: null,
        navGoalTopic: null,
        odomListener: null,
        // Waypoints
        waypoints: {
            'sofa': { x: -2.65, y: -1.36, theta: -0.80, name: 'Sofa' },
            'living_room': { x: 0.81, y: -1.29, theta: 0.99, name: 'Living Room' },
            'kitchen': { x: 0.49, y: 2.97, theta: 0.46, name: 'Kitchen' }
        }
    },
    // helper methods to connect to ROS
    methods: {
        connect: function() {
            this.loading = true
            this.ros = new ROSLIB.Ros({
                url: this.rosbridge_address,
                groovyCompatibility: false
            })
            this.ros.on('connection', () => {
                this.logs.unshift((new Date()).toTimeString() + ' - Connected!')
                this.connected = true
                this.loading = false
                this.setupROSCommunication()
                this.setup3DViewer()
                this.setup2DMap()
                this.setupCamera()
                this.setupJoystick()
            })
            this.ros.on('error', (error) => {
                this.logs.unshift((new Date()).toTimeString() + ` - Error: ${error}`)
            })
            this.ros.on('close', () => {
                this.logs.unshift((new Date()).toTimeString() + ' - Disconnected!')
                this.connected = false
                this.loading = false
                this.unset3DViewer()
                this.unsetCamera()
                this.unsetMap()
                this.unsetJoystick()
            })
        },
        disconnect: function() {
            this.ros.close()
        },

        setupROSCommunication: function() {
            // Command velocity publisher for joystick
            this.cmdVelTopic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/fastbot_1/cmd_vel',
                messageType: 'geometry_msgs/Twist'
            });
            
            // Navigation goal publisher for waypoints
            this.navGoalTopic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/move_base_simple/goal',
                messageType: 'geometry_msgs/PoseStamped'
            });
            
            // Odometry listener for robot status
            this.odomListener = new ROSLIB.Topic({
                ros: this.ros,
                name: '/fastbot_1/odom',
                messageType: 'nav_msgs/Odometry'
            });
            
            this.odomListener.subscribe((message) => {
                this.updateRobotStatus(message);
            });
        },

        setup3DViewer() {
            this.viewer = new ROS3D.Viewer({
                background: '#cccccc',
                divID: 'div3DViewer',
                width: 400,
                height: 300,
                antialias: true,
                fixedFrame: 'fastbot_1_odom'
            })

            // Add a grid.
            this.viewer.addObject(new ROS3D.Grid({
                color:'#0181c4',
                cellSize: 0.5,
                num_cells: 20
            }))

            // Setup a client to listen to TFs.
            this.tfClient = new ROSLIB.TFClient({
                ros: this.ros,
                angularThres: 0.01,
                transThres: 0.01,
                rate: 10.0,
                fixedFrame: 'fastbot_1_base_link'
            })

            // Setup the URDF client.
            this.urdfClient = new ROS3D.UrdfClient({
                ros: this.ros,
                param: '/fastbot_1_robot_state_publisher:robot_description',
                tfClient: this.tfClient,
                // We use "path: location.origin + location.pathname"
                // instead of "path: window.location.href" to remove query params,
                // otherwise the assets fail to load
                path: location.origin + location.pathname,
                rootObject: this.viewer.scene,
                loader: ROS3D.COLLADA_LOADER_2
            })
        },
        unset3DViewer() {
            document.getElementById('div3DViewer').innerHTML = ''
            this.robot3DLoaded = false;
        },

        setup2DMap: function() {
            this.map2D = new ROS2D.Viewer({
                divID: 'mapViewer',
                width: 400,
                height: 300
            });
            
            // Add occupancy grid
            var gridClient = new ROS2D.OccupancyGridClient({
                ros: this.ros,
                rootObject: this.map2D.scene,
                topic: '/map'
            });
            
            gridClient.on('change', () => {
                this.mapLoaded = true;
            });
            
            // Add robot marker
            var robotMarker = new ROS2D.NavigationArrow({
                size: 20,
                strokeSize: 2,
                fillColor: createjs.Graphics.getRGB(255, 128, 0),
                pulse: true
            });
            
            this.map2D.scene.addChild(robotMarker);
            
            // Update robot position on map
            if (this.tfClient) {
                this.tfClient.subscribe('fastbot_1_base_link', (tf) => {
                    if (this.map2D && this.map2D.scene) {
                        robotMarker.x = tf.translation.x * this.map2D.scene.scaleX;
                        robotMarker.y = tf.translation.y * this.map2D.scene.scaleY;
                        robotMarker.rotation = tf.rotation.z * 180 / Math.PI;
                    }
                });
            }
        },

        unsetMap: function() {
            document.getElementById('mapViewer').innerHTML = '';
        },

        setupCamera: function() {
            try {
                var without_wss = this.rosbridge_address.split('wss://')[1];
                var domain = without_wss.split('/')[0] + '/' + without_wss.split('/')[1];
                var host = domain + '/cameras';
                
                this.cameraViewer = new MJPEGCANVAS.Viewer({
                    divID: 'cameraViewer',
                    host: host,
                    width: 400,
                    height: 300,
                    topic: '/fastbot_1/camera/image_raw',
                    ssl: true,
                });
                
                this.cameraLoaded = true;
                this.cameraStatus = 'Camera connected';
            } catch (error) {
                console.error('Camera setup error:', error);
                this.cameraStatus = 'Camera unavailable';
            }
        },

        unsetCamera: function() {
            document.getElementById('cameraViewer').innerHTML = '';
        },

        setupJoystick: function() {
            this.joystick = nipplejs.create({
                zone: document.getElementById('joystickContainer'),
                mode: 'static',
                position: { left: '50%', top: '50%' },
                color: 'grey',
                size: 120
            });
            
            this.joystick.on('move', (evt, data) => {
                if (!this.connected || !this.cmdVelTopic) return;
                
                var maxDistance = 60;
                var distance = Math.min(data.distance, maxDistance);
                const angle = data.angle.radian;

                // Up (90°) → +linear.x ; Down (270°) → -linear.x
                const linear  = (distance / maxDistance) * 0.5 * Math.sin(angle);

                // Right (0°) → negative angular.z (turn right); Left (180°) → positive (turn left)
                const angular = -(distance / maxDistance) * 1.0 * Math.cos(angle);
                                
                var message = new ROSLIB.Message({
                    linear: { x: linear, y: 0, z: 0 },
                    angular: { x: 0, y: 0, z: angular }
                });
                
                this.cmdVelTopic.publish(message);
                this.controlMode = 'Manual';
            });
            
            this.joystick.on('end', () => {
                if (this.connected && this.cmdVelTopic) {
                    var message = new ROSLIB.Message({
                        linear: { x: 0, y: 0, z: 0 },
                        angular: { x: 0, y: 0, z: 0 }
                    });
                    this.cmdVelTopic.publish(message);
                }
            });
        },

        unsetJoystick: function() {
            if (this.joystick) {
                this.joystick.destroy();
            }
        },

        updateRobotStatus: function(odomMessage) {
            var pose = odomMessage.pose.pose;
            var twist = odomMessage.twist.twist;
            
            // Update position
            this.robotStatus.position.x = pose.position.x;
            this.robotStatus.position.y = pose.position.y;
            
            // Update orientation (quaternion to euler)
            var quat = pose.orientation;
            var siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y);
            var cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z);
            var theta = Math.atan2(siny_cosp, cosy_cosp);
            this.robotStatus.orientation = theta * 180 / Math.PI;
            
            // Update speed (prefer twist, fallback to pose delta)
            let speed = 0;
            if (odomMessage.twist && odomMessage.twist.twist) {
            const lin = odomMessage.twist.twist.linear || {x:0,y:0};
            speed = Math.hypot(lin.x || 0, lin.y || 0);
            }

            // Fallback: compute from pose if twist is missing/zero
            if (!speed || speed < 1e-4) {
            // ROS 2 stamp: sec + nanosec
            const stamp =
                odomMessage.header && odomMessage.header.stamp
                ? odomMessage.header.stamp.sec + odomMessage.header.stamp.nanosec * 1e-9
                : null;

            if (stamp != null && this.lastPose.t != null) {
                const dt = Math.max(1e-3, stamp - this.lastPose.t);
                const dx = pose.position.x - this.lastPose.x;
                const dy = pose.position.y - this.lastPose.y;
                speed = Math.hypot(dx, dy) / dt;
            }

            if (stamp != null) {
                this.lastPose = { x: pose.position.x, y: pose.position.y, t: stamp };
            }
            }

            this.robotStatus.speed = speed;
        },

        goToWaypoint: function(waypointKey) {
            if (!this.connected || !this.navGoalTopic) return;
            
            var waypoint = this.waypoints[waypointKey];
            this.isNavigating = true;
            this.controlMode = 'Navigation to ' + waypoint.name;
            
            var goal = new ROSLIB.Message({
                header: {
                    frame_id: 'map',
                    stamp: {
                        sec: Math.floor(Date.now() / 1000),
                        nanosec: (Date.now() % 1000) * 1000000
                    }
                },
                pose: {
                    position: { x: waypoint.x, y: waypoint.y, z: 0.0 },
                    orientation: {
                        x: 0.0, y: 0.0,
                        z: Math.sin(waypoint.theta / 2),
                        w: Math.cos(waypoint.theta / 2)
                    }
                }
            });
            
            this.navGoalTopic.publish(goal);
            
            // Reset navigation state after 5 seconds
            setTimeout(() => {
                this.isNavigating = false;
                this.controlMode = 'Manual';
            }, 5000);
        },

        emergencyStop: function() {
            if (!this.connected || !this.cmdVelTopic) return;
            
            var message = new ROSLIB.Message({
                linear: { x: 0, y: 0, z: 0 },
                angular: { x: 0, y: 0, z: 0 }
            });
            
            this.cmdVelTopic.publish(message);
            this.controlMode = 'EMERGENCY STOP';
            this.isNavigating = false;
            
            setTimeout(() => {
                this.controlMode = 'Manual';
            }, 3000);
        }
    },
    
    mounted: function() {
        // Simulate battery level updates
        setInterval(() => {
            this.robotStatus.battery = Math.max(20, 100 - Math.random() * 3);
        }, 5000);
    }
})