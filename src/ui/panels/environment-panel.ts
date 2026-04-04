/**
 * Environment configuration panel.
 * Wind speed/direction, turbulence, scene selector, ground effect.
 */

import type { WorkerCommand, DroneSnapshot } from '@worker/worker-protocol';
import { getScenePreset } from '@sim/environment/scene-presets';

export class EnvironmentPanel {
  private container: HTMLDivElement;
  private sendCommand: (cmd: WorkerCommand) => void;
  private envReadout: HTMLDivElement;

  constructor(parent: HTMLElement, sendCommand: (cmd: WorkerCommand) => void) {
    this.sendCommand = sendCommand;
    this.container = document.createElement('div');
    this.container.className = 'panel';
    parent.appendChild(this.container);

    const title = document.createElement('div');
    title.className = 'panel-title';
    title.textContent = 'Environment';
    this.container.appendChild(title);

    // Wind speed
    this.addSlider('Wind speed', 0, 20, 0.5, 0, 'm/s', (v) => {
      this.sendCommand({ type: 'set-environment-config', config: {
        wind: { meanSpeed: v, meanDirection: new Float64Array([1,0,0]), shearEnabled: false,
          shearExponent: 0.14, gustEvents: [] },
      }});
    });

    // Turbulence
    this.addToggle('Turbulence', false, (on) => {
      this.sendCommand({ type: 'set-environment-config', config: {
        turbulence: { enabled: on, intensity: 1.0, correlationTime: 2.0, spatialCorrelation: false, integralLengthScale: 5 },
      }});
    });

    this.addSlider('Turb. intensity', 0, 5, 0.1, 1.0, 'm/s', (v) => {
      this.sendCommand({ type: 'set-environment-config', config: {
        turbulence: { enabled: true, intensity: v, correlationTime: 2.0, spatialCorrelation: false, integralLengthScale: 5 },
      }});
    });

    // Spatial turbulence (Phase 11)
    this.addToggle('Spatial turb.', false, (on) => {
      this.sendCommand({ type: 'set-environment-config', config: {
        turbulence: { enabled: true, intensity: 1.0, correlationTime: 2.0, spatialCorrelation: on, integralLengthScale: 5 },
      }});
    });

    // Ground effect toggle
    this.addToggle('Ground effect', true, (on) => {
      this.sendCommand({ type: 'set-environment-config', config: {
        groundEffect: { enabled: on, propRadiusFactor: 0.7, wallCeilingEffectEnabled: false },
      }});
    });

    // Wall/ceiling effect (Phase 11)
    this.addToggle('Wall/ceiling fx', false, (on) => {
      this.sendCommand({ type: 'set-environment-config', config: {
        groundEffect: { enabled: true, propRadiusFactor: 0.7, wallCeilingEffectEnabled: on },
      }});
    });

    // WMM magnetic field (Phase 11) with lat/lon/date inputs
    let wmmLat = 55.75, wmmLon = 37.62, wmmDate = 2025.0, wmmOn = false;
    const sendWMM = () => {
      this.sendCommand({ type: 'set-environment-config', config: {
        magneticField: {
          fieldStrength: 50e-6, inclination: 1.15, declination: 0.05, anomalies: [],
          wmmEnabled: wmmOn, latitude: wmmLat, longitude: wmmLon, wmmDate,
        },
      }});
    };
    this.addToggle('WMM field', false, (on) => { wmmOn = on; sendWMM(); });
    this.addSlider('WMM lat', -90, 90, 1, 55.75, '°N', (v) => { wmmLat = v; sendWMM(); });
    this.addSlider('WMM lon', -180, 180, 1, 37.62, '°E', (v) => { wmmLon = v; sendWMM(); });
    this.addSlider('WMM date', 2020, 2030, 0.5, 2025, 'yr', (v) => { wmmDate = v; sendWMM(); });

    // Scene selector — loads full preset geometry from scene-presets.ts
    this.addSelect('Scene', ['open_field', 'indoor_basic', 'warehouse'], 'open_field', (v) => {
      const sceneConfig = getScenePreset(v);
      this.sendCommand({ type: 'set-environment-config', config: { scene: sceneConfig } });
    });

    // Live readout
    this.envReadout = document.createElement('div');
    this.envReadout.style.cssText = 'margin-top:8px;font-size:var(--font-size-sm);';
    this.container.appendChild(this.envReadout);
  }

  update(snap: DroneSnapshot): void {
    const e = snap.environment;
    const lines = [
      `Wind: ${e.windSpeed.toFixed(1)} m/s`,
      `Height AGL: ${e.heightAboveGround.toFixed(2)} m`,
      `GE mult: ${e.groundEffectMultiplier.toFixed(3)}`,
      `Texture: ${e.surfaceTextureQuality.toFixed(2)}`,
    ];
    this.envReadout.textContent = lines.join(' | ');
  }

  private addSlider(label: string, min: number, max: number, step: number,
    initial: number, unit: string, onChange: (v: number) => void): void {
    const row = document.createElement('div');
    row.className = 'slider-container';
    row.style.marginBottom = '4px';

    const lbl = document.createElement('span');
    lbl.className = 'param-label';
    lbl.textContent = label;
    row.appendChild(lbl);

    const slider = document.createElement('input');
    slider.type = 'range';
    slider.min = String(min); slider.max = String(max);
    slider.step = String(step); slider.value = String(initial);
    row.appendChild(slider);

    const val = document.createElement('span');
    val.className = 'param-value';
    val.style.minWidth = '40px';
    val.textContent = `${initial}${unit}`;
    row.appendChild(val);

    slider.addEventListener('input', () => {
      const v = parseFloat(slider.value);
      val.textContent = `${v.toFixed(1)}${unit}`;
      onChange(v);
    });

    this.container.appendChild(row);
  }

  private addToggle(label: string, initial: boolean, onChange: (on: boolean) => void): void {
    const row = document.createElement('div');
    row.className = 'param-row';

    const lbl = document.createElement('span');
    lbl.className = 'param-label';
    lbl.textContent = label;
    row.appendChild(lbl);

    const btn = document.createElement('button');
    btn.className = 'btn';
    btn.textContent = initial ? 'ON' : 'OFF';
    btn.classList.toggle('active', initial);
    let state = initial;
    btn.addEventListener('click', () => {
      state = !state;
      btn.textContent = state ? 'ON' : 'OFF';
      btn.classList.toggle('active', state);
      onChange(state);
    });
    row.appendChild(btn);

    this.container.appendChild(row);
  }

  private addSelect(label: string, options: string[], initial: string,
    onChange: (v: string) => void): void {
    const row = document.createElement('div');
    row.className = 'param-row';

    const lbl = document.createElement('span');
    lbl.className = 'param-label';
    lbl.textContent = label;
    row.appendChild(lbl);

    const select = document.createElement('select');
    select.style.cssText = 'background:var(--bg-tertiary);border:1px solid var(--border);color:var(--text-primary);font-family:var(--font-ui);font-size:var(--font-size-sm);padding:2px 4px;border-radius:var(--radius);';
    for (const opt of options) {
      const o = document.createElement('option');
      o.value = opt; o.textContent = opt;
      if (opt === initial) o.selected = true;
      select.appendChild(o);
    }
    select.addEventListener('change', () => onChange(select.value));
    row.appendChild(select);

    this.container.appendChild(row);
  }
}
