/**
 * Three.js 3D viewport panel.
 * Handles NED->Three.js coordinate transform.
 *
 * NED -> Three.js mapping (Three.js is Y-up):
 *   x_NED (North)  -> z_three
 *   y_NED (East)   -> x_three
 *   z_NED (Down)   -> -y_three
 */

import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import type { DroneSnapshot } from '@worker/worker-protocol';

export class ViewportPanel {
  private container: HTMLDivElement;
  private renderer: THREE.WebGLRenderer;
  private scene: THREE.Scene;
  private camera: THREE.PerspectiveCamera;
  private controls: OrbitControls;
  private droneMeshes: Map<number, THREE.Group> = new Map();
  private commLines: THREE.LineSegments | null = null;
  private formationLines: THREE.LineSegments | null = null;
  private selectedDroneId = 0;
  private animId = 0;
  private raycaster = new THREE.Raycaster();
  private mouse = new THREE.Vector2();
  private pointerDownPos = { x: 0, y: 0 };
  onDroneSelected: ((droneId: number) => void) | null = null;

  // Trail system
  private static readonly TRAIL_MAX_POINTS = 600; // ~10s at 60fps
  private static readonly TRAIL_COLORS = [0x4488ff, 0xff4466, 0x44cc66, 0xffaa22, 0xcc44ff, 0x44dddd, 0xff88cc, 0xaacc44];
  private trailBuffers: Map<number, { positions: Float32Array; head: number; count: number }> = new Map();
  private trailLines: Map<number, THREE.Line> = new Map();

  constructor(parent: HTMLElement) {
    this.container = document.createElement('div');
    this.container.style.cssText = 'width:100%;height:100%;position:relative;';
    parent.appendChild(this.container);

    // Renderer
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.setClearColor(0xe8e8f0);
    this.container.appendChild(this.renderer.domElement);

    // Scene
    this.scene = new THREE.Scene();

    // Camera
    this.camera = new THREE.PerspectiveCamera(60, 1, 0.1, 500);
    // Default: will be repositioned on first drone update
    this.camera.position.set(10, 5, 10);
    this.camera.lookAt(5, 2, 5);

    // Controls
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.1;
    this.controls.target.set(5, 2, 5); // NED (5,5,-2) → Three.js (5,2,5)

    // Lights
    this.scene.add(new THREE.AmbientLight(0x404060, 1.5));
    const dirLight = new THREE.DirectionalLight(0xffffff, 1);
    dirLight.position.set(5, 10, 5);
    this.scene.add(dirLight);

    // Ground grid (centered on drone area)
    const grid = new THREE.GridHelper(30, 60, 0xbbbbcc, 0xd0d0dd);
    grid.position.set(5, 0, 5); // center at NED x=5, y=5
    this.scene.add(grid);

    // World axes (Three.js coordinates)
    const worldAxes = new THREE.AxesHelper(1.5);
    this.scene.add(worldAxes);

    // Drone meshes added dynamically in updateSwarm()

    // Click to select drone (pointerdown+pointerup to distinguish from orbit drag)
    this.renderer.domElement.addEventListener('pointerdown', (e) => {
      this.pointerDownPos.x = e.clientX;
      this.pointerDownPos.y = e.clientY;
    });
    this.renderer.domElement.addEventListener('pointerup', (e) => {
      const dx = e.clientX - this.pointerDownPos.x;
      const dy = e.clientY - this.pointerDownPos.y;
      if (dx * dx + dy * dy < 25) this.onClickSelect(e); // <5px movement = click
    });

    // Handle resize
    this.onResize();
    window.addEventListener('resize', () => this.onResize());

    // Start render loop
    this.render();
  }

  private buildDroneMesh(droneId: number, selected = false): THREE.Group {
    const group = new THREE.Group();
    const droneColor = ViewportPanel.TRAIL_COLORS[droneId % ViewportPanel.TRAIL_COLORS.length];
    const bodyMat = new THREE.MeshStandardMaterial({
      color: selected ? 0xffffff : droneColor,
      emissive: selected ? droneColor : 0x000000,
      emissiveIntensity: selected ? 0.4 : 0,
    });
    const armMat = new THREE.MeshStandardMaterial({ color: 0x666688 });
    const rotorMatCCW = new THREE.MeshStandardMaterial({ color: 0x44aa66, transparent: true, opacity: 0.6 });
    const rotorMatCW = new THREE.MeshStandardMaterial({ color: 0xaa6644, transparent: true, opacity: 0.6 });

    // Body
    const body = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.03, 0.06), bodyMat);
    group.add(body);

    // Arms and rotors (X-config at 45-degree angles)
    const armLength = 0.17;
    const d = armLength / Math.SQRT2;
    const positions = [
      [d, -d], [d, d], [-d, d], [-d, -d],  // NED: [x_front, y_right]
    ];
    const dirs = [1, -1, 1, -1]; // CCW, CW, CCW, CW

    for (let i = 0; i < 4; i++) {
      const [nx, ny] = positions[i];
      // Convert NED position to Three.js: [ny, 0, nx]
      const tx = ny;
      const tz = nx;

      // Arm
      const armLen = Math.sqrt(nx * nx + ny * ny);
      const arm = new THREE.Mesh(
        new THREE.CylinderGeometry(0.005, 0.005, armLen, 6),
        armMat,
      );
      arm.position.set(tx / 2, 0, tz / 2);
      arm.rotation.z = Math.atan2(tz, tx);
      arm.rotation.x = Math.PI / 2;
      group.add(arm);

      // Rotor disc
      const rotor = new THREE.Mesh(
        new THREE.CylinderGeometry(0.06, 0.06, 0.004, 16),
        dirs[i] > 0 ? rotorMatCCW : rotorMatCW,
      );
      rotor.position.set(tx, 0.01, tz);
      group.add(rotor);
    }

    // Body axes helper (small, attached to drone)
    const axesHelper = new THREE.AxesHelper(0.15);
    group.add(axesHelper);

    // Invisible hit sphere for click selection (much larger than visual mesh)
    const hitSphere = new THREE.Mesh(
      new THREE.SphereGeometry(0.3, 8, 8),
      new THREE.MeshBasicMaterial({ visible: false }),
    );
    group.add(hitSphere);

    return group;
  }

  /** Single-drone compat: update one drone. */
  updateState(snapshot: DroneSnapshot): void {
    this.updateSwarm({ simTime: 0, stepCount: 0, selectedDroneId: snapshot.id,
      drones: [snapshot], commLinks: [],
      formation: { enabled: false, mode: 'leader-follower', topology: 'line', spacing: 2, leaderDroneId: 0, consensusGain: 0.5, leaderLossFallback: 'hover', offsetFrame: 'world' },
      safetyMetrics: { minSeparation: Infinity, collisionCount: 0, avgSeparation: Infinity, safetyOverrideCount: 0, emergencyStopCount: 0 },
      safety: { enabled: true, minSeparation: 0.5, orcaRadius: 0.3, orcaTimeHorizon: 3, maxSpeed: 5, minAltitude: 0.3, emergencyStopDistance: 0.3 },
      safetyEvents: [] });
  }

  /** Multi-drone: update all drones + comm links. */
  updateSwarm(swarm: import('@worker/worker-protocol').SwarmSnapshot): void {
    this.selectedDroneId = swarm.selectedDroneId;

    // Track which drones we've seen this frame
    const seen = new Set<number>();

    for (const drone of swarm.drones) {
      seen.add(drone.id);
      let mesh = this.droneMeshes.get(drone.id);
      if (!mesh) {
        mesh = this.buildDroneMesh(drone.id, drone.id === swarm.selectedDroneId);
        this.scene.add(mesh);
        this.droneMeshes.set(drone.id, mesh);
      }

      // NED -> Three.js
      const [nx, ny, nz] = drone.position;
      mesh.position.set(ny, -nz, nx);
      const [qw, qx, qy, qz] = drone.quaternion;
      mesh.quaternion.set(qy, -qz, qx, qw);

      // Highlight selected
      mesh.scale.setScalar(drone.id === swarm.selectedDroneId ? 1.2 : 1.0);
    }

    // Remove meshes for drones no longer in snapshot
    for (const [id, mesh] of this.droneMeshes) {
      if (!seen.has(id)) {
        this.scene.remove(mesh);
        this.droneMeshes.delete(id);
      }
    }

    // Remove trails for removed drones
    for (const [id, line] of this.trailLines) {
      if (!seen.has(id)) {
        this.scene.remove(line);
        this.trailLines.delete(id);
        this.trailBuffers.delete(id);
      }
    }

    // Update trails
    for (const drone of swarm.drones) {
      const maxPts = ViewportPanel.TRAIL_MAX_POINTS;
      let buf = this.trailBuffers.get(drone.id);
      if (!buf) {
        buf = { positions: new Float32Array(maxPts * 3), head: 0, count: 0 };
        this.trailBuffers.set(drone.id, buf);
      }

      // NED -> Three.js
      const [nx, ny, nz] = drone.position;
      const tx = ny, ty = -nz, tz = nx;
      const i3 = buf.head * 3;
      buf.positions[i3] = tx; buf.positions[i3 + 1] = ty; buf.positions[i3 + 2] = tz;
      buf.head = (buf.head + 1) % maxPts;
      if (buf.count < maxPts) buf.count++;

      // Build/update line
      let line = this.trailLines.get(drone.id);
      if (!line) {
        const geom = new THREE.BufferGeometry();
        const color = ViewportPanel.TRAIL_COLORS[drone.id % ViewportPanel.TRAIL_COLORS.length];
        const mat = new THREE.LineBasicMaterial({ color, opacity: 0.6, transparent: true });
        line = new THREE.Line(geom, mat);
        line.frustumCulled = false;
        this.scene.add(line);
        this.trailLines.set(drone.id, line);
      }

      // Rebuild ordered array from ring buffer
      const ordered = new Float32Array(buf.count * 3);
      const start = buf.count < maxPts ? 0 : buf.head;
      for (let j = 0; j < buf.count; j++) {
        const src = ((start + j) % maxPts) * 3;
        const dst = j * 3;
        ordered[dst] = buf.positions[src];
        ordered[dst + 1] = buf.positions[src + 1];
        ordered[dst + 2] = buf.positions[src + 2];
      }
      const geom = line.geometry as THREE.BufferGeometry;
      geom.setAttribute('position', new THREE.Float32BufferAttribute(ordered, 3));
      geom.setDrawRange(0, buf.count);
    }

    // Comm links overlay
    if (this.commLines) this.scene.remove(this.commLines);
    if (swarm.commLinks.length > 0 && swarm.drones.length > 1) {
      const points: number[] = [];
      for (const link of swarm.commLinks) {
        const a = swarm.drones.find(d => d.id === link.from);
        const b = swarm.drones.find(d => d.id === link.to);
        if (a && b) {
          points.push(a.position[1], -a.position[2], a.position[0]);
          points.push(b.position[1], -b.position[2], b.position[0]);
        }
      }
      const geom = new THREE.BufferGeometry();
      geom.setAttribute('position', new THREE.Float32BufferAttribute(points, 3));
      this.commLines = new THREE.LineSegments(geom,
        new THREE.LineBasicMaterial({ color: 0x44ff88, opacity: 0.4, transparent: true }));
      this.scene.add(this.commLines);
    }

    // Formation overlay: lines between drones in formation
    if (this.formationLines) this.scene.remove(this.formationLines);
    if (swarm.formation.enabled && swarm.drones.length > 1) {
      const fPoints: number[] = [];
      const activeDrones = swarm.drones.filter(d => d.formationState?.active);
      const leader = swarm.drones.find(d =>
        d.formationState?.role === 'leader' || d.id === swarm.formation.leaderDroneId);
      for (const d of activeDrones) {
        if (d === leader) continue;
        const ref = swarm.formation.mode === 'leader-follower' ? leader : activeDrones[0];
        if (ref && ref !== d) {
          fPoints.push(d.position[1], -d.position[2], d.position[0]);
          fPoints.push(ref.position[1], -ref.position[2], ref.position[0]);
        }
      }
      if (fPoints.length > 0) {
        const fGeom = new THREE.BufferGeometry();
        fGeom.setAttribute('position', new THREE.Float32BufferAttribute(fPoints, 3));
        this.formationLines = new THREE.LineSegments(fGeom,
          new THREE.LineBasicMaterial({ color: 0xff8844, opacity: 0.6, transparent: true }));
        this.scene.add(this.formationLines);
      }
    }
  }

  private onResize(): void {
    const w = this.container.clientWidth;
    const h = this.container.clientHeight;
    if (w === 0 || h === 0) return;
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(w, h);
  }

  private render = (): void => {
    this.animId = requestAnimationFrame(this.render);
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  };

  private onClickSelect(event: MouseEvent): void {
    const rect = this.renderer.domElement.getBoundingClientRect();
    this.mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    this.mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
    this.raycaster.setFromCamera(this.mouse, this.camera);

    // Find closest drone mesh
    let closestId = -1;
    let closestDist = Infinity;
    for (const [id, group] of this.droneMeshes) {
      const intersects = this.raycaster.intersectObject(group, true);
      if (intersects.length > 0 && intersects[0].distance < closestDist) {
        closestDist = intersects[0].distance;
        closestId = id;
      }
    }
    if (closestId >= 0 && this.onDroneSelected) {
      this.onDroneSelected(closestId);
    }
  }

  /** Clear all trail lines (call on reset/reform). */
  clearTrails(): void {
    for (const [, line] of this.trailLines) this.scene.remove(line);
    this.trailLines.clear();
    this.trailBuffers.clear();
  }

  dispose(): void {
    cancelAnimationFrame(this.animId);
    this.renderer.dispose();
  }
}
