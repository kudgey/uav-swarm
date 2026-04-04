/**
 * Canvas-2D time-series chart for telemetry data.
 * Lightweight, no external charting library.
 */

export interface ChartSeries {
  label: string;
  color: string;
  data: number[];
}

export class TelemetryChart {
  private canvas: HTMLCanvasElement;
  private ctx: CanvasRenderingContext2D;
  private series: ChartSeries[] = [];
  private times: number[] = [];
  private maxPoints = 600;
  constructor(parent: HTMLElement, title: string, width = 300, height = 120) {

    const wrapper = document.createElement('div');
    wrapper.style.cssText = 'margin-bottom:6px;';
    parent.appendChild(wrapper);

    const label = document.createElement('div');
    label.textContent = title;
    label.style.cssText = 'color:var(--text-secondary);font-size:10px;margin-bottom:2px;';
    wrapper.appendChild(label);

    this.canvas = document.createElement('canvas');
    this.canvas.width = width;
    this.canvas.height = height;
    this.canvas.style.cssText = `width:${width}px;height:${height}px;background:var(--bg-tertiary);border-radius:var(--radius);`;
    wrapper.appendChild(this.canvas);

    this.ctx = this.canvas.getContext('2d')!;
  }

  addSeries(label: string, color: string): number {
    this.series.push({ label, color, data: [] });
    return this.series.length - 1;
  }

  pushData(time: number, values: number[]): void {
    this.times.push(time);
    for (let i = 0; i < this.series.length; i++) {
      this.series[i].data.push(values[i] ?? 0);
    }
    // Trim to max points
    if (this.times.length > this.maxPoints) {
      const excess = this.times.length - this.maxPoints;
      this.times.splice(0, excess);
      for (const s of this.series) {
        s.data.splice(0, excess);
      }
    }
  }

  render(): void {
    const { ctx, canvas } = this;
    const w = canvas.width;
    const h = canvas.height;
    const pad = { top: 4, bottom: 14, left: 40, right: 8 };
    const plotW = w - pad.left - pad.right;
    const plotH = h - pad.top - pad.bottom;

    // Clear
    ctx.fillStyle = '#12121a';
    ctx.fillRect(0, 0, w, h);

    if (this.times.length < 2) return;

    // Compute Y range
    let yMin = Infinity, yMax = -Infinity;
    for (const s of this.series) {
      for (const v of s.data) {
        if (v < yMin) yMin = v;
        if (v > yMax) yMax = v;
      }
    }
    if (yMin === yMax) { yMin -= 1; yMax += 1; }
    const yRange = yMax - yMin;
    yMin -= yRange * 0.05;
    yMax += yRange * 0.05;

    const tMin = this.times[0];
    const tMax = this.times[this.times.length - 1];
    const tRange = tMax - tMin || 1;

    // Draw grid
    ctx.strokeStyle = '#222230';
    ctx.lineWidth = 0.5;
    ctx.beginPath();
    for (let i = 0; i <= 4; i++) {
      const y = pad.top + (i / 4) * plotH;
      ctx.moveTo(pad.left, y);
      ctx.lineTo(pad.left + plotW, y);
    }
    ctx.stroke();

    // Y axis labels
    ctx.fillStyle = '#555568';
    ctx.font = '9px monospace';
    ctx.textAlign = 'right';
    for (let i = 0; i <= 4; i++) {
      const val = yMax - (i / 4) * (yMax - yMin);
      const y = pad.top + (i / 4) * plotH;
      ctx.fillText(val.toFixed(1), pad.left - 3, y + 3);
    }

    // Draw series
    for (const s of this.series) {
      ctx.strokeStyle = s.color;
      ctx.lineWidth = 1;
      ctx.beginPath();
      for (let i = 0; i < s.data.length; i++) {
        const x = pad.left + ((this.times[i] - tMin) / tRange) * plotW;
        const y = pad.top + ((yMax - s.data[i]) / (yMax - yMin)) * plotH;
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.stroke();
    }

    // Legend
    ctx.font = '9px monospace';
    ctx.textAlign = 'left';
    let lx = pad.left + 4;
    for (const s of this.series) {
      ctx.fillStyle = s.color;
      ctx.fillRect(lx, h - 10, 8, 3);
      ctx.fillText(s.label, lx + 10, h - 6);
      lx += ctx.measureText(s.label).width + 18;
    }
  }

  clear(): void {
    this.times = [];
    for (const s of this.series) {
      s.data = [];
    }
  }
}
