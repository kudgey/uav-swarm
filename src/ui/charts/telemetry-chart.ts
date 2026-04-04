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
  private wrapper: HTMLDivElement;
  private series: ChartSeries[] = [];
  private times: number[] = [];
  private maxPoints = 600;
  constructor(parent: HTMLElement, title: string, _width = 300, _height = 120) {

    this.wrapper = document.createElement('div');
    this.wrapper.style.cssText = 'flex:1;min-width:200px;display:flex;flex-direction:column;';
    parent.appendChild(this.wrapper);

    const label = document.createElement('div');
    label.textContent = title;
    label.style.cssText = 'color:var(--text-secondary);font-family:var(--font-ui);font-size:var(--font-size-xs);margin-bottom:2px;font-weight:500;';
    this.wrapper.appendChild(label);

    this.canvas = document.createElement('canvas');
    this.canvas.style.cssText = 'width:100%;flex:1;background:var(--bg-tertiary);border-radius:var(--radius);';
    this.wrapper.appendChild(this.canvas);

    this.ctx = this.canvas.getContext('2d')!;

    // Resize canvas to actual pixel size
    const ro = new ResizeObserver(() => this.resizeCanvas());
    ro.observe(this.canvas);
  }

  private resizeCanvas(): void {
    const dpr = Math.min(window.devicePixelRatio, 2);
    const rect = this.canvas.getBoundingClientRect();
    if (rect.width === 0 || rect.height === 0) return;
    this.canvas.width = Math.round(rect.width * dpr);
    this.canvas.height = Math.round(rect.height * dpr);
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
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
    const rect = canvas.getBoundingClientRect();
    const w = rect.width;
    const h = rect.height;
    if (w === 0 || h === 0) return;
    const pad = { top: 4, bottom: 14, left: 40, right: 8 };
    const plotW = w - pad.left - pad.right;
    const plotH = h - pad.top - pad.bottom;

    // Clear
    ctx.fillStyle = '#1a1a28';
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
