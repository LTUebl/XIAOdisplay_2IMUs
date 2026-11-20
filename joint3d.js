/**
 * joint3d.js - Realistic 3D Anatomical Knee / Limb & Dual-IMU Prototype Visualizer
 * =================================================================================
 * Faithfully models the physical Seeed XIAO 1.14" IPS Display (nRF52840) + Dual
 * Adafruit ICM-20948 9-DOF IMU prototype across:
 *   - KNEE / ANKLE MODE (Default):
 *       * Standing anatomical Femur (shaft, condyles, trochanter, femoral head)
 *       * Patella (kneecap) & patellar tendon
 *       * Anatomical Tibia + lateral Fibula + articulated Athletic Shoe / Foot
 *       * Form-fitting charcoal Athletic Compression Knee Sleeve + translucent tissue
 *       * Mid-femur strapped Seeed XIAO 1.14" IPS Display with live LCD canvas
 *       * Distal lateral femur Sensor #1 (0x69) & proximal lateral tibia Sensor #2 (0x68)
 *       * Dynamic 4-pin STEMMA QT cable harness flexing across the lateral knee joint
 *       * Dynamic 3D sagittal flexion angle sector & arc
 *   - WRIST / ELBOW MODE:
 *       * Anatomical Radius & Ulna forearm + articulated Hand & Fingers with dorsal S2
 *
 * Developed for LTU Experimental Biomechanics Laboratory (EBL)
 */

class DualIMUJoint3D {
  constructor(containerId) {
    this.container = document.getElementById(containerId);
    if (!this.container) {
      console.error(`Container #${containerId} not found.`);
      return;
    }

    this.scene = null;
    this.camera = null;
    this.renderer = null;
    this.controls = null;

    // Scene Hierarchy
    this.rootGroup = null;
    this.proximalGroup = null;     // Femur / Forearm + XIAO Display + Sensor #1 (0x69)
    this.jointPivot = null;        // Knee / Wrist Joint Center at (0, 0, 0)
    this.distalGroup = null;       // Tibia + Fibula + Foot / Hand + Sensor #2 (0x68)
    this.kneeModelGroup = null;
    this.wristModelGroup = null;

    // Cable anchors & dynamic meshes
    this.xiaoAnchor = null;
    this.s1TopAnchor = null;
    this.s1BotAnchor = null;
    this.s2TopAnchor = null;
    this.axesS1 = null;
    this.axesS2 = null;
    this.upperCableLine = null;
    this.lowerCableTube = null;
    this.flexionSectorMesh = null;
    this.patellaGroup = null;

    // Dynamic LCD canvas texture on the 3D XIAO 1.14" Display model
    this.lcdCanvas = null;
    this.lcdCtx = null;
    this.lcdTexture = null;

    // Quaternions for Sensor #1 (Proximal) and Sensor #2 (Distal)
    this.q1Raw = new THREE.Quaternion();
    this.q2Raw = new THREE.Quaternion();
    this.q1Tare = new THREE.Quaternion();
    this.q1Smooth = new THREE.Quaternion();
    this.q2Smooth = new THREE.Quaternion();
    this.currentFeDeg = 0;

    this.showAxes = true;
    this.jointPreset = 'KNEE';

    this.initScene();
    this.buildLightingAndEnvironment();
    this.buildArticulatedModel();
    this.animate();

    window.addEventListener('resize', () => this.onWindowResize());
  }

  initScene() {
    const w = this.container.clientWidth || 640;
    const h = this.container.clientHeight || 420;

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0b1221);
    this.scene.fog = new THREE.FogExp2(0x0b1221, 0.032);

    this.camera = new THREE.PerspectiveCamera(40, w / h, 0.1, 100);
    // Lateral-oblique view looking at the lateral right knee (matching prototype photos)
    this.camera.position.set(3.2, 0.9, 6.2);

    this.renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
    this.renderer.setSize(w, h);
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;

    this.container.innerHTML = '';
    this.container.appendChild(this.renderer.domElement);

    if (typeof THREE.OrbitControls !== 'undefined') {
      this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
      this.controls.enableDamping = true;
      this.controls.dampingFactor = 0.08;
      this.controls.target.set(0.15, -0.25, 0.0);
      this.controls.minDistance = 2.4;
      this.controls.maxDistance = 15.0;
      this.controls.update();
    }
  }

  buildLightingAndEnvironment() {
    const hemiLight = new THREE.HemisphereLight(0xf8fafc, 0x1e293b, 0.9);
    hemiLight.position.set(0, 10, 0);
    this.scene.add(hemiLight);

    const dirLight = new THREE.DirectionalLight(0xffffff, 1.15);
    dirLight.position.set(5, 8, 7);
    dirLight.castShadow = true;
    dirLight.shadow.mapSize.width = 1024;
    dirLight.shadow.mapSize.height = 1024;
    this.scene.add(dirLight);

    const rimLight = new THREE.DirectionalLight(0x38bdf8, 0.55);
    rimLight.position.set(-6, 3, -5);
    this.scene.add(rimLight);

    const warmFill = new THREE.PointLight(0xfde68a, 0.35, 14);
    warmFill.position.set(3, -2, 4);
    this.scene.add(warmFill);

    // Subtle Lab Floor Grid below standing foot (y = -3.85)
    const grid = new THREE.GridHelper(14, 28, 0x1e3a5f, 0x152238);
    grid.position.y = -3.85;
    this.scene.add(grid);

    // Anatomical Joint Protractor Rings at Origin (0,0,0)
    const ringGroup = new THREE.Group();
    // Sagittal Flexion/Extension Plane (X-Y plane, Z normal)
    const sagGeo = new THREE.RingGeometry(1.42, 1.47, 64);
    const sagMat = new THREE.MeshBasicMaterial({
      color: 0x38bdf8,
      side: THREE.DoubleSide,
      transparent: true,
      opacity: 0.35
    });
    const sagRing = new THREE.Mesh(sagGeo, sagMat);
    ringGroup.add(sagRing);

    // Frontal Varus/Valgus Plane (Y-Z plane, X normal)
    const frontGeo = new THREE.RingGeometry(1.22, 1.26, 64);
    const frontMat = new THREE.MeshBasicMaterial({
      color: 0x34d399,
      side: THREE.DoubleSide,
      transparent: true,
      opacity: 0.22
    });
    const frontRing = new THREE.Mesh(frontGeo, frontMat);
    frontRing.rotation.y = Math.PI / 2;
    ringGroup.add(frontRing);

    // Dynamic 3D Flexion Sector Mesh (shows exact knee flexion angle between femur & tibia)
    const sectorGeo = new THREE.CircleGeometry(1.42, 40, -Math.PI / 2, 0.001);
    const sectorMat = new THREE.MeshBasicMaterial({
      color: 0x0ea5e9,
      side: THREE.DoubleSide,
      transparent: true,
      opacity: 0.24
    });
    this.flexionSectorMesh = new THREE.Mesh(sectorGeo, sectorMat);
    this.flexionSectorMesh.position.z = 0.02;
    ringGroup.add(this.flexionSectorMesh);

    this.scene.add(ringGroup);
  }

  createXIAOLcdTexture() {
    this.lcdCanvas = document.createElement('canvas');
    this.lcdCanvas.width = 240;
    this.lcdCanvas.height = 135;
    this.lcdCtx = this.lcdCanvas.getContext('2d');
    this.lcdTexture = new THREE.CanvasTexture(this.lcdCanvas);
    this.lcdTexture.minFilter = THREE.LinearFilter;
    this.updateXIAOLcdTexture(0, 0, 0, 96, 'KNEE', true);
    return this.lcdTexture;
  }

  updateXIAOLcdTexture(fe, ru, ps, battPct, jointName = 'KNEE', isCal = true) {
    if (!this.lcdCtx) return;
    const ctx = this.lcdCtx;
    ctx.fillStyle = '#090e1a';
    ctx.fillRect(0, 0, 240, 135);

    // Header Bar
    ctx.fillStyle = '#0284c7';
    ctx.fillRect(0, 0, 240, 24);
    ctx.fillStyle = '#ffffff';
    ctx.font = 'bold 12px monospace';
    ctx.fillText(`LTU EBL | ${jointName} [${isCal ? 'CAL' : 'RAW'}]`, 6, 16);

    const isKnee = String(jointName).toUpperCase().includes('KNEE');
    const l1 = isKnee ? 'Flx/Ext' : 'Flx/Ext';
    const l2 = isKnee ? 'Var/Val' : 'Rad/Uln';
    const l3 = isKnee ? 'Int/Ext' : 'Pro/Sup';

    ctx.font = 'bold 18px monospace';
    ctx.fillStyle = '#38bdf8';
    ctx.fillText(`${l1}: ${(fe >= 0 ? '+' : '') + fe.toFixed(1)}°`, 8, 50);
    ctx.fillStyle = '#34d399';
    ctx.fillText(`${l2}: ${(ru >= 0 ? '+' : '') + ru.toFixed(1)}°`, 8, 78);
    ctx.fillStyle = '#fbbf24';
    ctx.fillText(`${l3}: ${(ps >= 0 ? '+' : '') + ps.toFixed(1)}°`, 8, 106);

    ctx.fillStyle = '#94a3b8';
    ctx.font = '11px monospace';
    ctx.fillText(`BLE:XIAO-IMU | BAT:${battPct}%`, 6, 128);

    if (this.lcdTexture) {
      this.lcdTexture.needsUpdate = true;
    }
  }

  /**
   * Builds a realistic Adafruit ICM-20948 9-DOF IMU PCB breakout mesh
   * with two white STEMMA QT 4-pin JST-SH connectors on its ends, center QFN IC,
   * mounting holes, and silkscreen header pads.
   */
  createICM20948BreakoutMesh(isSensor2 = false) {
    const pod = new THREE.Group();

    const pcbMat = new THREE.MeshStandardMaterial({ color: 0x111827, roughness: 0.35, metalness: 0.45 });
    const qfnMat = new THREE.MeshStandardMaterial({ color: 0x090d16, roughness: 0.25, metalness: 0.8 });
    const jstMat = new THREE.MeshStandardMaterial({ color: 0xf1f5f9, roughness: 0.3, metalness: 0.1 });
    const goldMat = new THREE.MeshStandardMaterial({ color: 0xfbbf24, roughness: 0.25, metalness: 0.9 });
    const accentMat = new THREE.MeshStandardMaterial({
      color: isSensor2 ? 0x10b981 : 0x0284c7,
      roughness: 0.3,
      metalness: 0.2
    });

    // Main Adafruit rectangular PCB (aligned along local Y shaft axis, Z facing laterally outward)
    const pcb = new THREE.Mesh(new THREE.BoxGeometry(0.42, 0.64, 0.06), pcbMat);
    pcb.castShadow = true;
    pod.add(pcb);

    // Colored sensor identifier border strip
    const strip = new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.58, 0.065), accentMat);
    strip.position.set(0.17, 0, 0);
    pod.add(strip);

    // Center ICM-20948 3x3mm QFN chip package
    const ic = new THREE.Mesh(new THREE.BoxGeometry(0.14, 0.14, 0.03), qfnMat);
    ic.position.set(0, 0, 0.04);
    pod.add(ic);

    // Top & Bottom White STEMMA QT / Qwiic 4-pin JST-SH Connectors
    const jstTop = new THREE.Mesh(new THREE.BoxGeometry(0.18, 0.11, 0.08), jstMat);
    jstTop.position.set(0, 0.26, 0.05);
    pod.add(jstTop);

    const jstBot = new THREE.Mesh(new THREE.BoxGeometry(0.18, 0.11, 0.08), jstMat);
    jstBot.position.set(0, -0.26, 0.05);
    pod.add(jstBot);

    // Row of 6 Gold Header Pads along posterior edge
    const padGeo = new THREE.CylinderGeometry(0.022, 0.022, 0.065, 12);
    padGeo.rotateX(Math.PI / 2);
    for (let i = -2.5; i <= 2.5; i += 1.0) {
      const pad = new THREE.Mesh(padGeo, goldMat);
      pad.position.set(-0.15, i * 0.085, 0);
      pod.add(pad);
    }

    // If Sensor #2 (0x68), show gold AD0 soldered jumper badge
    if (isSensor2) {
      const jmp = new THREE.Mesh(new THREE.BoxGeometry(0.08, 0.06, 0.025), goldMat);
      jmp.position.set(0.09, -0.12, 0.04);
      pod.add(jmp);
    }

    return pod;
  }

  buildArticulatedModel() {
    this.rootGroup = new THREE.Group();
    this.scene.add(this.rootGroup);

    //Coordinate System in Standing Posture (KNEE default):
    //   +Y = Superior (Upward toward Hip)
    //   -Y = Inferior (Downward toward Floor)
    //   +X = Anterior (Forward / Patella & Toes direction)
    //   -X = Posterior (Backward / Hamstring & Knee Flexion swing direction)
    //   +Z = Lateral (Outward side of right leg where XIAO, S1, and S2 are mounted)
    this.proximalGroup = new THREE.Group();
    this.rootGroup.add(this.proximalGroup);

    this.jointPivot = new THREE.Group();
    this.jointPivot.position.set(0, 0, 0);
    this.proximalGroup.add(this.jointPivot);

    this.distalGroup = new THREE.Group();
    this.jointPivot.add(this.distalGroup);

    // Shared Realistic Materials
    const boneMat = new THREE.MeshStandardMaterial({
      color: 0xf3ead2,
      roughness: 0.46,
      metalness: 0.05
    });
    const cartilageMat = new THREE.MeshStandardMaterial({
      color: 0xe0f2fe,
      roughness: 0.25,
      metalness: 0.08
    });
    const sleeveMat = new THREE.MeshStandardMaterial({
      color: 0x172033,
      roughness: 0.65,
      metalness: 0.1,
      transparent: true,
      opacity: 0.52
    });
    const limbSkinMat = new THREE.MeshStandardMaterial({
      color: 0x64748b,
      roughness: 0.55,
      metalness: 0.05,
      transparent: true,
      opacity: 0.16
    });
    const strapMat = new THREE.MeshStandardMaterial({
      color: 0x0f172a,
      roughness: 0.75,
      metalness: 0.1
    });

    // ========================================================================
    // 1. PROXIMAL SEGMENT: ANATOMICAL FEMUR + KNEE SLEEVE + XIAO + SENSOR #1
    // ========================================================================
    // Femoral Shaft (extending from y = +0.25 up to y = +3.1)
    const femurShaftGeo = new THREE.CylinderGeometry(0.23, 0.20, 2.75, 24);
    const femurShaft = new THREE.Mesh(femurShaftGeo, boneMat);
    femurShaft.position.set(0, 1.62, 0);
    femurShaft.castShadow = true;
    this.proximalGroup.add(femurShaft);

    // Distal Femoral Condyles (Medial & Lateral Condyles at the Knee Joint)
    const latCondyle = new THREE.Mesh(new THREE.SphereGeometry(0.32, 24, 18), boneMat);
    latCondyle.scale.set(1.22, 0.95, 0.75);
    latCondyle.position.set(-0.04, 0.22, 0.21);
    this.proximalGroup.add(latCondyle);

    const medCondyle = new THREE.Mesh(new THREE.SphereGeometry(0.32, 24, 18), boneMat);
    medCondyle.scale.set(1.22, 0.95, 0.75);
    medCondyle.position.set(-0.04, 0.22, -0.21);
    this.proximalGroup.add(medCondyle);

    // Distal Femoral Metaphysis Flare
    const femurFlare = new THREE.Mesh(new THREE.CylinderGeometry(0.21, 0.38, 0.65, 20), boneMat);
    femurFlare.position.set(0, 0.55, 0);
    this.proximalGroup.add(femurFlare);

    // Articular Cartilage Cap on Femoral Condyles
    const cartCap = new THREE.Mesh(new THREE.SphereGeometry(0.31, 20, 16), cartilageMat);
    cartCap.scale.set(1.18, 0.72, 1.35);
    cartCap.position.set(0.02, 0.14, 0);
    this.proximalGroup.add(cartCap);

    // Proximal Femur: Greater Trochanter, Femoral Neck & Spherical Femoral Head
    const trochanter = new THREE.Mesh(new THREE.SphereGeometry(0.28, 18, 16), boneMat);
    trochanter.scale.set(0.9, 1.2, 0.85);
    trochanter.position.set(0, 2.98, 0.16);
    this.proximalGroup.add(trochanter);

    const femNeck = new THREE.Mesh(new THREE.CylinderGeometry(0.15, 0.18, 0.62, 16), boneMat);
    femNeck.position.set(0, 3.12, -0.22);
    femNeck.rotation.x = -0.82;
    this.proximalGroup.add(femNeck);

    const femHead = new THREE.Mesh(new THREE.SphereGeometry(0.29, 24, 20), boneMat);
    femHead.position.set(0, 3.34, -0.46);
    this.proximalGroup.add(femHead);

    // Upper Thigh Soft-Tissue Silhouette (translucent so bone + hardware remain crisp)
    const thighSilhouette = new THREE.Mesh(new THREE.CylinderGeometry(0.66, 0.52, 2.5, 28, 1, true), limbSkinMat);
    thighSilhouette.position.set(0, 1.75, 0);
    this.proximalGroup.add(thighSilhouette);

    // Upper Cuff of Athletic Compression Knee Sleeve (matching Photo 1)
    const upperSleeve = new THREE.Mesh(new THREE.CylinderGeometry(0.54, 0.48, 1.25, 28), sleeveMat);
    upperSleeve.position.set(0, 0.72, 0);
    this.proximalGroup.add(upperSleeve);

    // Patella (Kneecap) on Anterior (+X) aspect of the Knee Joint
    this.patellaGroup = new THREE.Group();
    const patellaMesh = new THREE.Mesh(new THREE.SphereGeometry(0.22, 18, 16), boneMat);
    patellaMesh.scale.set(0.55, 1.05, 0.95);
    patellaMesh.position.set(0.42, 0.16, 0);
    this.patellaGroup.add(patellaMesh);
    this.proximalGroup.add(this.patellaGroup);

    // --- Seeed XIAO 1.14" IPS Display Module Strapped to Mid-Femur (matching Photo 2) ---
    const xiaoMount = new THREE.Group();
    xiaoMount.position.set(0.06, 1.85, 0.42); // Lateral-anterior mid-femur shaft
    this.proximalGroup.add(xiaoMount);

    // Black Velcro Strap wrapping the Femur shaft
    const femurStrap = new THREE.Mesh(new THREE.CylinderGeometry(0.32, 0.30, 0.55, 24), strapMat);
    femurStrap.position.set(-0.06, 1.85, 0);
    this.proximalGroup.add(femurStrap);

    // Black Enclosure Base & LiPo Battery under XIAO Display
    const xiaoCase = new THREE.Mesh(
      new THREE.BoxGeometry(0.56, 0.96, 0.22),
      new THREE.MeshStandardMaterial({ color: 0x0f172a, roughness: 0.3, metalness: 0.5 })
    );
    xiaoCase.castShadow = true;
    xiaoMount.add(xiaoCase);

    // 1.14" IPS Landscape Screen Texture on XIAO
    const lcdTex = this.createXIAOLcdTexture();
    const lcdPlane = new THREE.Mesh(
      new THREE.PlaneGeometry(0.46, 0.80),
      new THREE.MeshBasicMaterial({ map: lcdTex })
    );
    lcdPlane.position.set(0, 0.0, 0.115);
    xiaoMount.add(lcdPlane);

    // Silver USB-C Connector & Green Status LED on XIAO
    const usbPort = new THREE.Mesh(
      new THREE.BoxGeometry(0.20, 0.08, 0.08),
      new THREE.MeshStandardMaterial({ color: 0x94a3b8, roughness: 0.2, metalness: 0.9 })
    );
    usbPort.position.set(0, 0.50, 0.02);
    xiaoMount.add(usbPort);

    const ledDot = new THREE.Mesh(
      new THREE.SphereGeometry(0.032, 10, 10),
      new THREE.MeshBasicMaterial({ color: 0x22c55e })
    );
    ledDot.position.set(0.20, 0.42, 0.12);
    xiaoMount.add(ledDot);

    this.xiaoAnchor = new THREE.Object3D();
    this.xiaoAnchor.position.set(0, -0.48, 0.05);
    xiaoMount.add(this.xiaoAnchor);

    // --- Sensor #1 (0x69) Adafruit ICM-20948 on Distal Lateral Femur (matching Photo 1 & 2) ---
    const s1Pod = this.createICM20948BreakoutMesh(false);
    s1Pod.position.set(0.0, 0.72, 0.50); // Distal lateral femur above knee joint
    this.proximalGroup.add(s1Pod);

    this.s1TopAnchor = new THREE.Object3D();
    this.s1TopAnchor.position.set(0, 0.30, 0.05);
    s1Pod.add(this.s1TopAnchor);

    this.s1BotAnchor = new THREE.Object3D();
    this.s1BotAnchor.position.set(0, -0.30, 0.05);
    s1Pod.add(this.s1BotAnchor);

    this.axesS1 = new THREE.AxesHelper(0.65);
    this.axesS1.position.set(0, 0, 0.08);
    s1Pod.add(this.axesS1);

    // ========================================================================
    // 2. DISTAL SEGMENT: ANATOMICAL TIBIA + FIBULA + FOOT/SHOE + SENSOR #2
    // ========================================================================
    // Tibial Plateau (Proximal Articular Surface at y = -0.18)
    const tibPlateau = new THREE.Mesh(new THREE.CylinderGeometry(0.38, 0.24, 0.45, 24), boneMat);
    tibPlateau.position.set(0, -0.26, 0);
    tibPlateau.castShadow = true;
    this.distalGroup.add(tibPlateau);

    // Anterior Tibial Tuberosity (where patellar ligament attaches)
    const tibTuberosity = new THREE.Mesh(new THREE.SphereGeometry(0.14, 16, 14), boneMat);
    tibTuberosity.scale.set(1.1, 1.4, 0.8);
    tibTuberosity.position.set(0.22, -0.45, 0);
    this.distalGroup.add(tibTuberosity);

    // Main Tibia Shaft (from y = -0.45 down to Ankle at y = -3.15)
    const tibiaShaft = new THREE.Mesh(new THREE.CylinderGeometry(0.21, 0.16, 2.70, 22), boneMat);
    tibiaShaft.position.set(0, -1.75, -0.03);
    tibiaShaft.castShadow = true;
    this.distalGroup.add(tibiaShaft);

    // Lateral Fibula Head & Slender Fibula Shaft (anatomical landmark on lateral +Z side)
    const fibulaHead = new THREE.Mesh(new THREE.SphereGeometry(0.13, 16, 14), boneMat);
    fibulaHead.position.set(-0.08, -0.42, 0.28);
    this.distalGroup.add(fibulaHead);

    const fibulaShaft = new THREE.Mesh(new THREE.CylinderGeometry(0.075, 0.065, 2.65, 16), boneMat);
    fibulaShaft.position.set(-0.08, -1.75, 0.26);
    fibulaShaft.castShadow = true;
    this.distalGroup.add(fibulaShaft);

    // Distal Malleoli at the Ankle (y = -3.10)
    const latMalleolus = new THREE.Mesh(new THREE.SphereGeometry(0.12, 14, 12), boneMat);
    latMalleolus.position.set(-0.06, -3.08, 0.25);
    this.distalGroup.add(latMalleolus);

    const medMalleolus = new THREE.Mesh(new THREE.SphereGeometry(0.13, 14, 12), boneMat);
    medMalleolus.position.set(0.02, -3.05, -0.20);
    this.distalGroup.add(medMalleolus);

    // Lower Cuff of Athletic Compression Knee Sleeve (wrapping proximal Tibia)
    const lowerSleeve = new THREE.Mesh(new THREE.CylinderGeometry(0.47, 0.42, 1.25, 28), sleeveMat);
    lowerSleeve.position.set(0, -0.72, 0);
    this.distalGroup.add(lowerSleeve);

    // Lower Leg / Calf Soft-Tissue Silhouette
    const calfSilhouette = new THREE.Mesh(new THREE.CylinderGeometry(0.48, 0.32, 2.55, 26, 1, true), limbSkinMat);
    calfSilhouette.position.set(-0.04, -1.75, 0);
    this.distalGroup.add(calfSilhouette);

    // --- Articulated Foot & Athletic Shoe at Distal End (pointing anteriorly +X) ---
    const footGroup = new THREE.Group();
    footGroup.position.set(0, -3.28, 0);
    this.distalGroup.add(footGroup);

    // Talus & Calcaneus (Heel Bone)
    const heelBone = new THREE.Mesh(new THREE.BoxGeometry(0.48, 0.28, 0.34), boneMat);
    heelBone.position.set(-0.12, 0.04, 0);
    footGroup.add(heelBone);

    // Metatarsals / Forefoot Bone Arch pointing forward (+X)
    const forefootBone = new THREE.Mesh(new THREE.CylinderGeometry(0.14, 0.18, 0.85, 16), boneMat);
    forefootBone.rotation.z = Math.PI / 2;
    forefootBone.position.set(0.36, -0.02, 0);
    footGroup.add(forefootBone);

    // Sleek Athletic Running Shoe Overlay (matching Photo 1 athlete shoe)
    const shoeUpperMat = new THREE.MeshStandardMaterial({
      color: 0x1e293b,
      roughness: 0.55,
      metalness: 0.15,
      transparent: true,
      opacity: 0.72
    });
    const shoeSoleMat = new THREE.MeshStandardMaterial({
      color: 0xe2e8f0,
      roughness: 0.4,
      metalness: 0.05
    });

    const shoeUpper = new THREE.Mesh(new THREE.CylinderGeometry(0.22, 0.27, 1.22, 20), shoeUpperMat);
    shoeUpper.rotation.z = Math.PI / 2;
    shoeUpper.scale.set(1.0, 1.0, 0.85);
    shoeUpper.position.set(0.26, -0.02, 0);
    footGroup.add(shoeUpper);

    const shoeSole = new THREE.Mesh(new THREE.BoxGeometry(1.30, 0.12, 0.46), shoeSoleMat);
    shoeSole.position.set(0.26, -0.22, 0);
    footGroup.add(shoeSole);

    // --- Sensor #2 (0x68) Adafruit ICM-20948 on Proximal Lateral Tibia (matching Photo 1 & 2) ---
    const s2Pod = this.createICM20948BreakoutMesh(true);
    s2Pod.position.set(0.0, -0.75, 0.49); // Proximal lateral tibia below knee joint
    this.distalGroup.add(s2Pod);

    this.s2TopAnchor = new THREE.Object3D();
    this.s2TopAnchor.position.set(0, 0.30, 0.05);
    s2Pod.add(this.s2TopAnchor);

    this.axesS2 = new THREE.AxesHelper(0.65);
    this.axesS2.position.set(0, 0, 0.08);
    s2Pod.add(this.axesS2);

    // ========================================================================
    // 3. DYNAMIC 4-PIN STEMMA QT WIRE HARNESS ACROSS THE LATERAL KNEE JOINT
    // ========================================================================
    // Upper harness: XIAO Display -> Sensor #1 (0x69) on Femur
    const upperCableMat = new THREE.LineBasicMaterial({ color: 0xf59e0b, linewidth: 3 });
    const upperPts = [new THREE.Vector3(), new THREE.Vector3(), new THREE.Vector3()];
    this.upperCableGeo = new THREE.BufferGeometry().setFromPoints(upperPts);
    this.upperCableLine = new THREE.Line(this.upperCableGeo, upperCableMat);
    this.scene.add(this.upperCableLine);

    // Lower harness across Knee Joint: Sensor #1 (0x69) -> Sensor #2 (0x68)
    const lowerCableMat = new THREE.LineBasicMaterial({ color: 0x38bdf8, linewidth: 3 });
    const lowerPts = [];
    for (let i = 0; i <= 24; i++) lowerPts.push(new THREE.Vector3());
    this.cableGeometry = new THREE.BufferGeometry().setFromPoints(lowerPts);
    this.cableLine = new THREE.Line(this.cableGeometry, lowerCableMat);
    this.scene.add(this.cableLine);
  }

  /**
   * Updates both the femur cable (XIAO -> S1) and the joint-crossing STEMMA QT cable (S1 -> S2)
   * plus the dynamic 3D sagittal flexion sector.
   */
  updateCableCurve() {
    if (!this.s1BotAnchor || !this.s2TopAnchor || !this.cableGeometry) return;

    // 1. Upper cable (XIAO Display -> S1 top connector)
    if (this.xiaoAnchor && this.s1TopAnchor && this.upperCableGeo) {
      const p0 = new THREE.Vector3();
      const p1 = new THREE.Vector3();
      this.xiaoAnchor.getWorldPosition(p0);
      this.s1TopAnchor.getWorldPosition(p1);
      const mid = p0.clone().add(p1).multiplyScalar(0.5);
      mid.z += 0.14;
      mid.x -= 0.08;
      const curveUp = new THREE.QuadraticBezierCurve3(p0, mid, p1);
      this.upperCableGeo.setFromPoints(curveUp.getPoints(12));
    }

    // 2. Joint-crossing cable (S1 bottom connector -> S2 top connector across lateral knee)
    const pStart = new THREE.Vector3();
    const pEnd = new THREE.Vector3();
    this.s1BotAnchor.getWorldPosition(pStart);
    this.s2TopAnchor.getWorldPosition(pEnd);

    const pMid = pStart.clone().add(pEnd).multiplyScalar(0.5);
    pMid.z += 0.18; // bows slightly outward laterally across the knee condyle
    pMid.x -= 0.06;

    const curve = new THREE.QuadraticBezierCurve3(pStart, pMid, pEnd);
    const pts = curve.getPoints(24);
    this.cableGeometry.setFromPoints(pts);
  }

  setJointPreset(presetKey) {
    this.jointPreset = String(presetKey || 'KNEE').toUpperCase();
    if (!this.rootGroup) return;

    // For Wrist/Elbow mode, tilt the root limb horizontally; for Knee/Ankle keep vertical standing posture
    if (this.jointPreset === 'WRIST' || this.jointPreset === 'ELBOW') {
      this.rootGroup.rotation.z = Math.PI / 2;
    } else {
      this.rootGroup.rotation.z = 0;
    }
  }

  /**
   * Updates the 3D articulated joint model from real-time dual-IMU telemetry.
   *
   * In standing calibration (0°, 0°, 0°):
   *   - Femur (proximalGroup) is upright along +Y.
   *   - Tibia & Foot (distalGroup) extend vertically down along -Y.
   *   - Flexion (+fe) swings the lower leg (Tibia & Foot) posteriorly (-X) in the sagittal plane,
   *     matching an athlete flexing their knee in standing posture.
   *   - Varus/Valgus (ru) abducts/adducts the tibia in the frontal (Y-Z) plane.
   *   - Internal/External Rotation (ps) twists the tibia axially around its longitudinal Y shaft.
   */
  updateKinematics(s1, s2, flexExtDeg, radUlnDeg, proSupDeg, battPct = 95, jointName = 'KNEE', isCal = true) {
    this.jointPreset = String(jointName || 'KNEE').toUpperCase();
    this.currentFeDeg = flexExtDeg;

    // Keep proximal femur steady in the standing anatomical reference frame (with subtle thigh sway if uncalibrated)
    const qProxTarget = new THREE.Quaternion();
    this.q1Smooth.slerp(qProxTarget, 0.25);

    // Map anatomical 3D angles to distal Tibia / Hand segment:
    // Knee Flexion swings the lower leg backward (-X), which is a negative Z rotation in right-handed (X=Anterior, Y=Up, Z=Lateral)
    const feAbsOrSigned = this.jointPreset === 'KNEE' ? Math.abs(flexExtDeg) : flexExtDeg;
    const feRad = THREE.MathUtils.degToRad(feAbsOrSigned);
    const vvRad = THREE.MathUtils.degToRad(radUlnDeg);
    const ieRad = THREE.MathUtils.degToRad(proSupDeg);

    // Intrinsic Z (Flexion/Extension) -> X (Varus/Valgus) -> Y (Internal/External Axial Tibial Rotation)
    const relEuler = new THREE.Euler(vvRad, ieRad, -feRad, 'ZXY');
    const qRelTarget = new THREE.Quaternion().setFromEuler(relEuler);
    this.q2Smooth.slerp(qRelTarget, 0.35);

    // Update Patella tracking (patella glides slightly inferiorly along the condyles during knee flexion)
    if (this.patellaGroup) {
      const glide = Math.min(1.0, Math.abs(flexExtDeg) / 110.0);
      this.patellaGroup.rotation.z = -feRad * 0.38;
    }

    // Update dynamic 3D Flexion Sector wedge at the joint center
    if (this.flexionSectorMesh) {
      const spanRad = Math.max(0.01, Math.min(Math.PI * 0.9, Math.abs(feRad)));
      this.flexionSectorMesh.geometry.dispose();
      this.flexionSectorMesh.geometry = new THREE.CircleGeometry(1.42, 36, -Math.PI / 2 - spanRad, spanRad);
      // Color progression: Emerald (<45°) -> Sky Blue (45°-90°) -> Amber (>90°)
      const absDeg = Math.abs(flexExtDeg);
      const col = absDeg > 90 ? 0xf59e0b : (absDeg > 45 ? 0x0ea5e9 : 0x10b981);
      this.flexionSectorMesh.material.color.setHex(col);
    }

    // Update LCD canvas texture on virtual XIAO 1.14" display
    this.updateXIAOLcdTexture(flexExtDeg, radUlnDeg, proSupDeg, battPct, jointName, isCal);
  }

  tare() {
    this.q1Tare.copy(this.q1Raw);
  }

  resetView() {
    if (!this.controls || !this.camera) return;
    this.camera.position.set(3.2, 0.9, 6.2);
    this.controls.target.set(0.15, -0.25, 0.0);
    this.controls.update();
  }

  toggleAxes() {
    this.showAxes = !this.showAxes;
    if (this.axesS1) this.axesS1.visible = this.showAxes;
    if (this.axesS2) this.axesS2.visible = this.showAxes;
    return this.showAxes;
  }

  onWindowResize() {
    if (!this.container || !this.renderer || !this.camera) return;
    const w = this.container.clientWidth;
    const h = this.container.clientHeight;
    if (w === 0 || h === 0) return;
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(w, h);
  }

  onResize() {
    this.onWindowResize();
  }

  animate() {
    requestAnimationFrame(() => this.animate());
    if (this.proximalGroup) {
      this.proximalGroup.quaternion.copy(this.q1Smooth);
    }
    if (this.distalGroup) {
      this.distalGroup.quaternion.copy(this.q2Smooth);
    }
    this.updateCableCurve();
    if (this.controls) this.controls.update();
    if (this.renderer && this.scene && this.camera) {
      this.renderer.render(this.scene, this.camera);
    }
  }
}

window.DualIMUJoint3D = DualIMUJoint3D;
window.JointVisualizer3D = DualIMUJoint3D;
