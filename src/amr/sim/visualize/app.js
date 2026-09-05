class AMRViewer {
  constructor() {
    const host = window.location.hostname || 'localhost';
    const scheme = window.location.protocol === 'https:' ? 'wss' : 'ws';

    this.world = 'amr_world';
    this.gzUrl = `${scheme}://${host}:9003`;
    this.gz = null;
    this.objects = new Map();
    this.poseTargets = new Map();
    this.pressedKeys = new Set();
    this.command = {linear: 0, angular: 0};
    this.commandHeartbeat = null;
    this.reconnectTimer = null;
    this.sceneLoaded = false;
    this.lastAnimationTime = performance.now();

    this.connectionStatus = document.getElementById('connection-status');
    this.sensorStatus = document.getElementById('sensor-status');
    this.poseStatus = document.getElementById('pose-status');
    this.commandStatus = document.getElementById('command-status');
    this.viewer = document.getElementById('viewer');

    this.scene = new GZ3D.Scene(new GZ3D.Shaders());
    this.scene.ambient.color.setHex(0x777777);
    this.scene.renderer.setClearColor(0xdde3e8, 1);
    this.scene.renderer.shadowMap.enabled = true;
    this.scene.renderer.shadowMap.type = THREE.PCFSoftShadowMap;

    const light = new THREE.DirectionalLight(0xffffff, 0.8);
    light.position.set(-7, -9, 12);
    light.castShadow = true;
    this.scene.add(light);
    this.scene.grid.visible = true;
  }

  start() {
    this.viewer.appendChild(this.scene.renderer.domElement);
    this.resize();
    this.setCamera();
    window.addEventListener('resize', () => this.resize());
    window.addEventListener('blur', () => this.stop());
    window.addEventListener('beforeunload', () => this.sendCommand(0, 0));

    this.bindControls();
    this.connect(true);
    this.animate();
    this.pollTelemetry();
    window.setInterval(() => this.pollTelemetry(), 500);
  }

  bindControls() {
    document.querySelectorAll('[data-linear]').forEach((button) => {
      const start = (event) => {
        event.preventDefault();
        this.setCommand(
          Number(button.dataset.linear),
          Number(button.dataset.angular)
        );
      };
      button.addEventListener('pointerdown', start);
      button.addEventListener('pointerup', () => this.stop());
      button.addEventListener('pointercancel', () => this.stop());
      button.addEventListener('pointerleave', () => this.stop());
    });

    document.getElementById('reset-button').addEventListener(
      'click', () => this.resetSimulation()
    );

    window.addEventListener('keydown', (event) => {
      if (!['KeyW', 'KeyA', 'KeyS', 'KeyD', 'Space'].includes(event.code)) {
        return;
      }
      event.preventDefault();
      if (event.code === 'Space') {
        this.pressedKeys.clear();
        this.stop();
        return;
      }
      this.pressedKeys.add(event.code);
      this.updateKeyboardCommand();
    });

    window.addEventListener('keyup', (event) => {
      if (!['KeyW', 'KeyA', 'KeyS', 'KeyD'].includes(event.code)) return;
      event.preventDefault();
      this.pressedKeys.delete(event.code);
      this.updateKeyboardCommand();
    });
  }

  updateKeyboardCommand() {
    const linear = (this.pressedKeys.has('KeyW') ? 0.18 : 0)
      + (this.pressedKeys.has('KeyS') ? -0.18 : 0);
    const angular = (this.pressedKeys.has('KeyA') ? 1.5 : 0)
      + (this.pressedKeys.has('KeyD') ? -1.5 : 0);
    this.setCommand(linear, angular);
  }

  setCommand(linear, angular) {
    this.command = {linear, angular};
    this.commandStatus.textContent =
      `linear ${linear.toFixed(2)} · angular ${angular.toFixed(2)}`;
    this.sendCommand(linear, angular);

    if (this.commandHeartbeat) window.clearInterval(this.commandHeartbeat);
    this.commandHeartbeat = null;
    if (linear !== 0 || angular !== 0) {
      this.commandHeartbeat = window.setInterval(
        () => this.sendCommand(this.command.linear, this.command.angular),
        100
      );
    }
  }

  stop() {
    this.pressedKeys.clear();
    this.setCommand(0, 0);
  }

  sendCommand(linear, angular) {
    fetch('/api/amr/command', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({linear, angular}),
      keepalive: true
    }).catch(() => this.setConnectionState('Control offline', 'error'));
  }

  async resetSimulation() {
    this.stop();
    try {
      const response = await fetch('/api/amr/reset', {method: 'POST'});
      const result = await response.json();
      if (!response.ok || !result.ok) {
        throw new Error(result.error || 'reset failed');
      }
      this.poseTargets.clear();
    } catch (error) {
      this.setConnectionState(`Reset failed: ${error.message}`, 'error');
    }
  }

  async pollTelemetry() {
    try {
      const response = await fetch('/api/amr/telemetry', {cache: 'no-store'});
      if (!response.ok) throw new Error('telemetry unavailable');
      const telemetry = await response.json();
      const freshness = telemetry.freshness || {};
      const freshCount = Object.values(freshness)
        .filter((item) => item.fresh).length;
      this.sensorStatus.textContent = `Sensors ${freshCount}/4`;

      const odometry = telemetry.sensors && telemetry.sensors.odometry;
      if (odometry) {
        const x = Number(odometry.position.x || 0);
        const y = Number(odometry.position.y || 0);
        const yaw = Number(odometry.yaw || 0) * 180 / Math.PI;
        this.poseStatus.textContent =
          `x ${x.toFixed(2)} · y ${y.toFixed(2)} · yaw ${yaw.toFixed(1)}°`;
      }
    } catch (error) {
      this.sensorStatus.textContent = error.message;
    }
  }

  connect(loadScene) {
    const gz = new Gazebo({url: this.gzUrl, key: ''});
    this.gz = gz;

    gz.on('connection', () => {
      if (this.gz !== gz) return;
      if (this.reconnectTimer) window.clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
      this.setConnectionState('Connected', 'connected');
    });
    gz.on('error', () => {
      if (this.gz !== gz) return;
      this.setConnectionState('WebSocket error', 'error');
      this.scheduleReconnect();
    });
    gz.on('close', () => {
      if (this.gz !== gz) return;
      this.setConnectionState('Reconnecting', 'pending');
      this.scheduleReconnect();
    });

    if (loadScene) {
      gz.on('worlds', () => {
        if (this.gz !== gz) return;
        gz.once('scene', (scene) => this.loadScene(scene));
        gz.socket.send(buildMsg(['scene', this.world, '', '']));
      });
    }

    new Topic({
      gz,
      name: `/world/${this.world}/dynamic_pose/info`,
      messageType: 'gz.msgs.Pose_V',
      callback: (message) => {
        if (this.gz === gz) this.updatePoses(message);
      }
    });
  }

  scheduleReconnect() {
    if (this.reconnectTimer) return;
    this.reconnectTimer = window.setTimeout(() => {
      this.reconnectTimer = null;
      this.connect(!this.sceneLoaded);
    }, 1000);
  }

  loadScene(scene) {
    (scene.model || []).forEach((model) => {
      this.scene.add(this.createModel(model));
    });
    this.sceneLoaded = true;
    this.setConnectionState(
      `Connected · ${(scene.model || []).length} models`,
      'connected'
    );
  }

  createModel(model) {
    const object = new THREE.Object3D();
    object.name = model.name;
    this.remember(model.name, object);
    this.setPose(object, model.pose);

    (model.link || []).forEach((link) => {
      const linkObject = new THREE.Object3D();
      linkObject.name = link.name;
      this.remember(link.name, linkObject);
      this.remember(`${model.name}::${link.name}`, linkObject);
      this.setPose(linkObject, link.pose);
      (link.visual || []).forEach((visual) => {
        const visualObject = this.createVisual(visual);
        if (visualObject) linkObject.add(visualObject);
      });
      object.add(linkObject);
    });

    (model.model || []).forEach((nested) => {
      object.add(this.createModel(nested));
    });
    return object;
  }

  createVisual(visual) {
    const object = new THREE.Object3D();
    this.setPose(object, visual.pose);
    const geometry = visual.geometry || {};
    let mesh = null;

    if (geometry.box) {
      const size = geometry.box.size;
      mesh = this.scene.createBox(size.x, size.y, size.z);
    } else if (geometry.sphere) {
      mesh = this.scene.createSphere(geometry.sphere.radius);
    } else if (geometry.cylinder) {
      mesh = this.scene.createCylinder(
        geometry.cylinder.radius,
        geometry.cylinder.length
      );
    } else if (geometry.plane) {
      const plane = geometry.plane;
      mesh = this.scene.createPlane(
        plane.normal.x,
        plane.normal.y,
        plane.normal.z,
        plane.size.x,
        plane.size.y
      );
    } else if (geometry.mesh) {
      const meshInfo = geometry.mesh;
      const uri = meshInfo.filename || meshInfo.uri || '';
      const assetUrl = this.resolveMeshUri(uri);
      if (!assetUrl) return null;
      this.scene.loadMeshFromUri(
        assetUrl,
        meshInfo.submesh,
        meshInfo.center_submesh,
        (loaded) => {
          if (!loaded) return;
          const scale = meshInfo.scale;
          if (scale) loaded.scale.set(scale.x, scale.y, scale.z);
          this.applyMaterialTree(loaded, visual.material);
          object.add(loaded);
        }
      );
      return object;
    }

    if (!mesh) return null;
    this.applyMaterial(mesh, visual.material);
    mesh.castShadow = !geometry.plane;
    mesh.receiveShadow = true;
    object.add(mesh);
    return object;
  }

  resolveMeshUri(uri) {
    if (!uri) return null;
    if (/^https?:\/\//.test(uri)) return uri;
    const marker = '/models/';
    if (uri.startsWith('model://')) {
      return `/assets/models/${uri.slice('model://'.length)}`;
    }
    const markerIndex = uri.indexOf(marker);
    if (markerIndex >= 0) {
      return `/assets/models/${uri.slice(markerIndex + marker.length)}`;
    }
    return `/assets/models/${uri.replace(/^file:\/\//, '').replace(/^\/+/, '')}`;
  }

  applyMaterialTree(object, material) {
    if (object instanceof THREE.Mesh) this.applyMaterial(object, material);
    const descendants = [];
    if (object.getDescendants) object.getDescendants(descendants);
    descendants.forEach((child) => {
      if (child instanceof THREE.Mesh) this.applyMaterial(child, material);
    });
  }

  applyMaterial(mesh, material) {
    const color = (value) => {
      if (!value) return null;
      if (Array.isArray(value)) return value;
      if (typeof value.r === 'number') {
        return [value.r, value.g, value.b, value.a ?? 1];
      }
      return null;
    };
    const diffuse = color(material && material.diffuse);
    if (!diffuse) return;
    this.scene.setMaterial(mesh, {
      ambient: color(material.ambient),
      diffuse,
      specular: color(material.specular),
      opacity: diffuse[3]
    });
  }

  updatePoses(message) {
    (message.pose || []).forEach((pose) => {
      const object = this.objects.get(pose.name)
        || this.objects.get(this.rootName(pose.name));
      if (!object || !pose.position || !pose.orientation) return;
      this.poseTargets.set(object, {
        position: new THREE.Vector3(
          pose.position.x,
          pose.position.y,
          pose.position.z
        ),
        orientation: new THREE.Quaternion(
          pose.orientation.x,
          pose.orientation.y,
          pose.orientation.z,
          pose.orientation.w
        )
      });
    });
  }

  remember(name, object) {
    if (name) this.objects.set(name, object);
  }

  rootName(name) {
    return String(name || '').split('::')[0];
  }

  setPose(object, pose) {
    if (pose) this.scene.setPose(object, pose.position, pose.orientation);
  }

  setCamera() {
    this.scene.camera.position.set(-5.5, -7.5, 5.5);
    this.scene.camera.lookAt(new THREE.Vector3(0, 0, 0.4));
  }

  resize() {
    this.scene.setSize(this.viewer.clientWidth, this.viewer.clientHeight);
  }

  animate() {
    requestAnimationFrame(() => this.animate());
    const now = performance.now();
    const elapsed = Math.min(100, now - this.lastAnimationTime);
    this.lastAnimationTime = now;
    const alpha = 1 - Math.exp(-elapsed / 40);
    this.poseTargets.forEach((target, object) => {
      object.position.lerp(target.position, alpha);
      object.quaternion.slerp(target.orientation, alpha);
    });
    this.scene.render();
  }

  setConnectionState(message, state) {
    this.connectionStatus.textContent = message;
    this.connectionStatus.className = `status ${state}`;
  }
}

window.addEventListener('load', () => new AMRViewer().start());
