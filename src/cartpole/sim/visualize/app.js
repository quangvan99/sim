class CartPoleViewer {
  constructor() {
    const host = window.location.hostname || '168.144.101.221';
    const scheme = window.location.protocol === 'https:' ? 'wss' : 'ws';

    this.world = 'cart_pole';
    this.objects = new Map();
    this.pressedKeys = new Set();
    this.currentForce = 0;
    this.controlHeartbeat = null;
    this.mode = 'manual';
    this.poseTargets = new Map();
    this.snapNextPoseUpdate = false;
    this.lastAnimationTime = performance.now();
    this.status = document.getElementById('status');
    this.controlState = document.getElementById('control-state');
    this.rlBar = document.getElementById('rl-bar');
    this.rlPhase = document.getElementById('rl-phase');
    this.rlStatusDot = document.getElementById('rl-status-dot');
    this.rlLiveData = document.getElementById('rl-live-data');
    this.rlResult = document.getElementById('rl-result');
    this.rlPoleAngle = document.getElementById('rl-pole-angle');
    this.rlAction = document.getElementById('rl-action');
    this.rlTime = document.getElementById('rl-time');
    this.rlButtons = {
      start: document.getElementById('rl-start'),
      reset: document.getElementById('rl-reset'),
      pushLeft: document.getElementById('rl-push-left'),
      pushRight: document.getElementById('rl-push-right')
    };
    this.viewer = document.getElementById('viewer');
    this.scene = new GZ3D.Scene(new GZ3D.Shaders());
    this.scene.ambient.color.setHex(0x888888);
    this.scene.renderer.setClearColor(0xe8edf2, 1);
    this.scene.renderer.shadowMap.enabled = true;
    this.scene.renderer.shadowMap.type = THREE.PCFSoftShadowMap;

    const keyLight = new THREE.DirectionalLight(0xffffff, 0.8);
    keyLight.position.set(-6, -8, 10);
    keyLight.target.position.set(0, 0, 0);
    keyLight.castShadow = true;
    keyLight.shadow.mapSize.width = 1024;
    keyLight.shadow.mapSize.height = 1024;
    keyLight.shadow.camera.left = -12;
    keyLight.shadow.camera.right = 12;
    keyLight.shadow.camera.top = 12;
    keyLight.shadow.camera.bottom = -12;
    keyLight.shadow.bias = -0.0005;
    this.scene.add(keyLight);
    this.scene.add(keyLight.target);

    this.scene.grid.material.vertexColors = THREE.NoColors;
    this.scene.grid.material.color.setHex(0xd9d9d9);
    this.scene.grid.material.opacity = 0.7;
    this.scene.grid.material.needsUpdate = true;
    this.scene.grid.visible = true;
    this.gzUrl = `${scheme}://${host}:9002`;
    this.gz = null;
  }

  async start() {
    this.viewer.appendChild(this.scene.renderer.domElement);
    this.resize();
    this.setCamera();
    window.addEventListener('resize', () => this.resize());

    this.connect(true);
    this.animate();
    await this.detectMode();
    if (this.mode === 'rl') {
      this.initRLControls();
    } else {
      this.initManualControls();
    }
  }

  async detectMode() {
    try {
      const response = await fetch('/api/health', {cache: 'no-store'});
      const health = await response.json();
      if (health.mode === 'rl') {
        this.mode = 'rl';
        document.body.dataset.mode = 'rl';
        document.getElementById('controls').hidden = true;
        this.status.hidden = true;
        this.rlBar.hidden = false;
        this.renderRLStatus({
          phase: health.phase,
          ready: health.ready,
          error: health.error
        });
      }
    } catch (error) {
      console.warn('Control bridge health unavailable', error);
    }
  }

  initManualControls() {
    document.getElementById('controls').hidden = false;
    const buttons = document.querySelectorAll('[data-force]');
    buttons.forEach((button) => {
      const force = Number(button.dataset.force);
      button.addEventListener('pointerdown', (event) => {
        event.preventDefault();
        this.setForce(force);
      });
      button.addEventListener('pointerup', () => this.setForce(0));
      button.addEventListener('pointercancel', () => this.setForce(0));
      button.addEventListener('pointerleave', () => this.setForce(0));
    });

    document.getElementById('reset-button').addEventListener(
      'click', () => this.resetSimulation());

    window.addEventListener('keydown', (event) => {
      if (!['ArrowLeft', 'ArrowRight', 'Space'].includes(event.code)) {
        return;
      }
      event.preventDefault();
      this.pressedKeys.add(event.code);
      this.updateKeyboardForce();
    });

    window.addEventListener('keyup', (event) => {
      if (!['ArrowLeft', 'ArrowRight', 'Space'].includes(event.code)) {
        return;
      }
      event.preventDefault();
      this.pressedKeys.delete(event.code);
      this.updateKeyboardForce();
    });

    window.addEventListener('blur', () => {
      this.pressedKeys.clear();
      this.setForce(0);
    });

    this.setForce(0);
  }

  initRLControls() {
    this.rlButtons.start.addEventListener(
      'click', () => this.sendRLCommand('/api/rl/start'));
    this.rlButtons.reset.addEventListener(
      'click', () => this.sendRLCommand('/api/cartpole/reset'));
    this.rlButtons.pushLeft.addEventListener(
      'click', () => this.sendRLCommand(
        '/api/rl/disturbance', {direction: 'left'}));
    this.rlButtons.pushRight.addEventListener(
      'click', () => this.sendRLCommand(
        '/api/rl/disturbance', {direction: 'right'}));

    this.pollRLStatus();
    window.setInterval(
      () => this.pollRLStatus(), 100);
  }

  async sendRLCommand(path, payload) {
    const options = {method: 'POST'};
    if (payload) {
      options.headers = {'Content-Type': 'application/json'};
      options.body = JSON.stringify(payload);
    }

    try {
      if (path === '/api/rl/start' || path === '/api/cartpole/reset') {
        this.snapNextPoseUpdate = true;
      }
      const response = await fetch(path, options);
      const message = await response.json();
      if (!response.ok || !message.ok) {
        throw new Error(message.error || 'command failed');
      }
      if (path === '/api/rl/start') {
        const balancing = await this.waitForRLPhase('BALANCING');
        if (!balancing) throw new Error('episode did not start');
        this.reconnectPoseStream();
      } else {
        await this.pollRLStatus();
      }
    } catch (error) {
      this.snapNextPoseUpdate = false;
      this.renderRLConnectionError(error.message);
    }
  }

  async waitForRLPhase(phase, timeoutMs = 2000) {
    const deadline = performance.now() + timeoutMs;
    while (performance.now() < deadline) {
      const response = await fetch('/api/rl/status', {cache: 'no-store'});
      if (!response.ok) throw new Error('status unavailable');
      const snapshot = await response.json();
      this.renderRLStatus(snapshot);
      if (snapshot.phase === phase) return true;
      await new Promise((resolve) => window.setTimeout(resolve, 20));
    }
    return false;
  }

  async pollRLStatus() {
    try {
      const response = await fetch('/api/rl/status', {cache: 'no-store'});
      if (!response.ok) throw new Error('status unavailable');
      this.renderRLStatus(await response.json());
    } catch (error) {
      this.renderRLConnectionError(error.message);
    }
  }

  renderRLStatus(snapshot) {
    const phase = snapshot.phase || 'LOADING';
    const terminal = phase === 'SUCCESS' || phase === 'FAILED';
    const balancing = phase === 'BALANCING';
    const ready = Boolean(snapshot.ready);
    const failed = phase === 'FAILED';
    const error = phase === 'ERROR';

    this.rlPhase.textContent = phase;
    this.rlStatusDot.className = error
      ? 'error'
      : failed
        ? 'failed'
        : ready ? 'ready' : '';
    this.rlStatusDot.title = snapshot.error || phase;

    this.rlLiveData.hidden = terminal || error;
    this.rlResult.hidden = !terminal && !error;
    if (terminal || error) {
      this.rlResult.textContent = snapshot.result
        || snapshot.error
        || 'Runtime unavailable';
    }

    const angle = Number(snapshot.pole_angle_deg || 0);
    this.rlPoleAngle.textContent = `Pole ${angle >= 0 ? '+' : ''}${angle.toFixed(1)}°`;
    const action = snapshot.action;
    const force = Number(snapshot.force || 0);
    if (action === 0) {
      this.rlAction.textContent = `◀ ${force.toFixed(0)} N`;
    } else if (action === 1) {
      this.rlAction.textContent = `▶ +${force.toFixed(0)} N`;
    } else {
      this.rlAction.textContent = '— 0 N';
    }
    this.rlTime.textContent = `${Number(snapshot.episode_time || 0)
      .toFixed(1).padStart(4, '0')}s`;

    this.rlButtons.start.disabled = !['IDLE', 'SUCCESS', 'FAILED']
      .includes(phase);
    this.rlButtons.reset.disabled = !ready;
    this.rlButtons.pushLeft.disabled = !balancing;
    this.rlButtons.pushRight.disabled = !balancing;
  }

  renderRLConnectionError(message) {
    this.rlPhase.textContent = 'ERROR';
    this.rlStatusDot.className = 'error';
    this.rlStatusDot.title = message;
    this.rlLiveData.hidden = true;
    this.rlResult.hidden = false;
    this.rlResult.textContent = 'Control bridge unavailable';
    Object.values(this.rlButtons).forEach((button) => {
      button.disabled = true;
    });
  }

  resetSimulation() {
    this.pressedKeys.clear();
    this.setForce(0);
    fetch('/api/cartpole/reset', {method: 'POST'})
      .then((response) => {
        if (!response.ok) throw new Error('reset failed');
        return response.json();
      })
      .then((message) => {
        if (!message.ok) throw new Error('reset failed');
        this.controlState.textContent = 'Reset complete';
      })
      .catch(() => {
        this.controlState.textContent = 'Reset unavailable';
      });
  }

  updateKeyboardForce() {
    if (this.pressedKeys.has('Space')) {
      this.setForce(0);
    } else if (this.pressedKeys.has('ArrowLeft')) {
      this.setForce(-2000);
    } else if (this.pressedKeys.has('ArrowRight')) {
      this.setForce(2000);
    } else {
      this.setForce(0);
    }
  }

  setForce(force) {
    this.currentForce = force;
    const sign = force > 0 ? '+' : '';
    this.controlState.textContent = `Force: ${sign}${force} N`;

    if (force === 0) {
      window.clearInterval(this.controlHeartbeat);
      this.controlHeartbeat = null;
    } else if (!this.controlHeartbeat) {
      this.controlHeartbeat = window.setInterval(
        () => this.sendForce(this.currentForce), 100);
    }

    this.sendForce(force);
  }

  sendForce(force) {

    fetch('/api/cartpole/command', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({force})
    })
      .then((response) => {
        if (!response.ok) throw new Error('control bridge unavailable');
        return response.json();
      })
      .then((message) => {
        if (!message.ok) throw new Error(message.error || 'command failed');
      })
      .catch(() => {
        this.controlState.textContent = 'Control bridge offline';
      });
  }

  connect(loadScene = false) {
    const gz = new Gazebo({url: this.gzUrl, key: ''});
    this.gz = gz;

    gz.on('connection', () => {
      if (this.gz === gz) this.setStatus('Connected');
    });
    gz.on('error', () => {
      if (this.gz === gz) this.setStatus('Connection error');
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

  reconnectPoseStream() {
    const previous = this.gz;
    this.gz = null;
    if (previous && previous.socket) previous.socket.close();
    this.connect(false);
  }

  loadScene(scene) {
    (scene.model || []).forEach((model) => {
      this.scene.add(this.createModel(model));
    });

    this.setStatus(`Connected · ${(scene.model || []).length} model`);
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
    }

    if (!mesh) return null;
    this.applyMaterial(mesh, visual.material);
    mesh.castShadow = !geometry.plane;
    mesh.receiveShadow = true;
    object.add(mesh);
    return object;
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

      if (this.snapNextPoseUpdate) {
        this.setPose(object, pose);
        this.poseTargets.delete(object);
        return;
      }

      this.poseTargets.set(object, {
        position: new THREE.Vector3(
          pose.position.x, pose.position.y, pose.position.z),
        orientation: new THREE.Quaternion(
          pose.orientation.x,
          pose.orientation.y,
          pose.orientation.z,
          pose.orientation.w)
      });
    });
    this.snapNextPoseUpdate = false;
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
    const camera = this.scene.camera;
    camera.position.set(0, -11, 5);
    camera.lookAt(new THREE.Vector3(0, 0, 1));
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

  setStatus(message) {
    this.status.textContent = message;
  }
}

window.addEventListener('load', () => new CartPoleViewer().start());
