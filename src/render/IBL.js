/**
 * The irradiance environment the shells reflect.
 *
 * Built from the scene's own physics rather than picked by eye: on an airless
 * body the sky contributes nothing and every photon that is not direct sunlight
 * has bounced off the ground, so the lower hemisphere carries
 * groundAlbedo x sunColor x sunIntensity x sin(elev) and the upper hemisphere
 * carries only the sky term. That asymmetry — lit from BELOW — is what makes a
 * white robot read against a black sky.
 */
import { DataTexture, EquirectangularReflectionMapping, FloatType, RGBAFormat,
         PMREMGenerator, Color, MathUtils } from 'three';

const W = 64, H = 32;

export class IBL {
  constructor(renderer) {
    this.pmrem = new PMREMGenerator(renderer);
    this.pmrem.compileEquirectangularShader();
    this.data = new Float32Array(W * H * 4);
    this.source = new DataTexture(this.data, W, H, RGBAFormat, FloatType);
    this.source.mapping = EquirectangularReflectionMapping;
    this.target = null;
    this._g = new Color(); this._s = new Color(); this._key = '';
  }

  update(env) {
    const key = `${env.id}`;
    if (key === this._key) return this.target?.texture || null;
    this._key = key;

    const cosine = Math.max(0.06, Math.sin(MathUtils.degToRad(env.sunElev)));
    const ground = this._g.copy(env.groundAlbedo).multiply(env.sunColor)
      .multiplyScalar(env.sunIntensity * cosine);
    const sky = this._s.copy(env.skyColor).multiplyScalar(env.ambientIntensity);

    for (let y = 0; y < H; y++) {
      const up = Math.cos((y + 0.5) / H * Math.PI);          // +1 zenith, -1 nadir
      // Smooth the horizon; a hard seam here rules a line across the shoulders.
      const t = MathUtils.smoothstep(up, -0.35, 0.35);
      const r = sky.r * t + ground.r * (1 - t);
      const g = sky.g * t + ground.g * (1 - t);
      const b = sky.b * t + ground.b * (1 - t);
      for (let x = 0; x < W; x++) {
        const i = (y * W + x) * 4;
        this.data[i] = r; this.data[i + 1] = g; this.data[i + 2] = b; this.data[i + 3] = 1;
      }
    }
    this.source.needsUpdate = true;
    const next = this.pmrem.fromEquirectangular(this.source);
    if (this.target) this.target.dispose();
    this.target = next;
    return next.texture;
  }

  dispose() { this.target?.dispose(); this.source.dispose(); this.pmrem.dispose(); }
}
