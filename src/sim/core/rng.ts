/**
 * Seedable deterministic PRNG: xoshiro128** with splitmix64 seeding.
 *
 * Produces reproducible sequences for a given seed. All simulation randomness
 * MUST go through this RNG to preserve determinism.
 */

/**
 * splitmix64 used to expand a single seed into 128 bits of state.
 */
function splitmix64(seed: number): { lo: number; hi: number } {
  // Work with 32-bit halves since JS doesn't have native u64
  let s0 = seed >>> 0;
  let s1 = (seed / 0x100000000) >>> 0;

  // First output
  s0 = (s0 + 0x9E3779B9) >>> 0;
  s1 = (s1 + (s0 < 0x9E3779B9 ? 1 : 0)) >>> 0;
  let z0 = s0;
  z0 = ((z0 ^ (z0 >>> 30)) >>> 0);
  z0 = Math.imul(z0, 0xBF58476D) >>> 0;
  z0 = ((z0 ^ (z0 >>> 27)) >>> 0);
  z0 = Math.imul(z0, 0x94D049BB) >>> 0;
  z0 = ((z0 ^ (z0 >>> 31)) >>> 0);

  // Second output for more state bits
  s0 = (s0 + 0x9E3779B9) >>> 0;
  s1 = (s1 + (s0 < 0x9E3779B9 ? 1 : 0)) >>> 0;
  let z2 = s0;
  z2 = ((z2 ^ (z2 >>> 30)) >>> 0);
  z2 = Math.imul(z2, 0xBF58476D) >>> 0;
  z2 = ((z2 ^ (z2 >>> 27)) >>> 0);
  z2 = Math.imul(z2, 0x94D049BB) >>> 0;
  z2 = ((z2 ^ (z2 >>> 31)) >>> 0);

  return { lo: z0 >>> 0, hi: z2 >>> 0 };
}

export class DeterministicRNG {
  private s: Uint32Array;

  constructor(seed: number) {
    this.s = new Uint32Array(4);
    this.seed(seed);
  }

  seed(seed: number): void {
    const a = splitmix64(seed);
    const b = splitmix64(seed + 1);
    this.s[0] = a.lo || 1; // Ensure non-zero state
    this.s[1] = a.hi || 2;
    this.s[2] = b.lo || 3;
    this.s[3] = b.hi || 4;
  }

  /**
   * xoshiro128**: returns a 32-bit unsigned integer.
   */
  nextU32(): number {
    const s = this.s;
    const result = Math.imul(rotl(Math.imul(s[1], 5), 7), 9) >>> 0;

    const t = (s[1] << 9) >>> 0;
    s[2] ^= s[0];
    s[3] ^= s[1];
    s[1] ^= s[2];
    s[0] ^= s[3];
    s[2] ^= t;
    s[3] = rotl(s[3], 11);

    return result;
  }

  /** Returns a float in [0, 1). */
  next(): number {
    return (this.nextU32() >>> 0) / 0x100000000;
  }

  /** Returns a float in [lo, hi). */
  range(lo: number, hi: number): number {
    return lo + this.next() * (hi - lo);
  }

  /** Standard normal via Box-Muller. */
  gaussian(mean = 0, stddev = 1): number {
    const u1 = this.next() || 1e-15; // Avoid log(0)
    const u2 = this.next();
    const z = Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
    return mean + z * stddev;
  }

  /** Clone current state for forking. */
  clone(): DeterministicRNG {
    const copy = new DeterministicRNG(0);
    copy.s.set(this.s);
    return copy;
  }
}

function rotl(x: number, k: number): number {
  return ((x << k) | (x >>> (32 - k))) >>> 0;
}
