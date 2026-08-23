/**
 * Regolith kicked up by a footfall. One draw call, one buffer.
 *
 * The CPU only writes spawn attributes for a burst; the entire life of every
 * grain after that is a closed-form function of uTime evaluated in the vertex
 * shader. That matters because the interesting behaviour is a physics
 * difference, not an art choice:
 *
 *   vacuum (uDrag = 0)  grains fly hard-edged parabolas and hang for seconds
 *   air    (uDrag > 0)  the same burst billows and dies in about a second
 *
 * Only the uniforms differ between the Moon and Earth. That contrast is the
 * atmosphere argument made visible without a number on screen.
 */
import {
  BufferGeometry, BufferAttribute, Points, ShaderMaterial, NormalBlending, Color,
} from 'three';

const MAX = 12000;
const PER_BURST = 44;   // Apollo film shows a low, sharply bounded rooster tail,
                        // not a cloud — there is no air to suspend one.

const VERT = /* glsl */`
uniform float uTime, uGravity, uDrag, uSize, uLife, uViewportH;
attribute vec3 aOrigin, aVelocity;
attribute float aBirth, aSeed, aScale;
varying float vFade;

void main() {
  float age = uTime - aBirth;
  if (age < 0.0 || age > uLife) { gl_Position = vec4(2.0, 2.0, 2.0, 1.0); vFade = 0.0; return; }

  vec3 p;
  if (uDrag < 0.001) {
    // vacuum: exact ballistic solution
    p = aOrigin + aVelocity * age;
    p.y -= 0.5 * uGravity * age * age;
  } else {
    // linear drag: closed form of dv/dt = -k v - g
    float k = uDrag;
    float e = (1.0 - exp(-k * age)) / k;
    p = aOrigin + aVelocity * e;
    p.y -= (uGravity / k) * (age - e);
  }

  vec4 mv = modelViewMatrix * vec4(p, 1.0);
  gl_Position = projectionMatrix * mv;
  float t = age / uLife;
  vFade = (1.0 - t) * (1.0 - t);
  gl_PointSize = uSize * aScale * uViewportH / max(0.001, -mv.z);
}`;

const FRAG = /* glsl */`
uniform vec3 uColor;
varying float vFade;
void main() {
  vec2 d = gl_PointCoord - 0.5;
  float r = dot(d, d);
  if (r > 0.25 || vFade <= 0.0) discard;
  // soft-edged grain; regolith is not a glowing sprite, so no additive blend
  gl_FragColor = vec4(uColor, vFade * (1.0 - r * 4.0) * 0.85);
}`;

export class Dust {
  constructor() {
    this.cursor = 0;
    const g = new BufferGeometry();
    this.origin = new Float32Array(MAX * 3);
    this.velocity = new Float32Array(MAX * 3);
    this.birth = new Float32Array(MAX).fill(-1e6);
    this.seed = new Float32Array(MAX);
    this.scale = new Float32Array(MAX);
    for (let i = 0; i < MAX; i++) { this.seed[i] = Math.random(); this.scale[i] = 0.6 + Math.random() * 0.9; }
    g.setAttribute('position', new BufferAttribute(new Float32Array(MAX * 3), 3));
    g.setAttribute('aOrigin', new BufferAttribute(this.origin, 3));
    g.setAttribute('aVelocity', new BufferAttribute(this.velocity, 3));
    g.setAttribute('aBirth', new BufferAttribute(this.birth, 1));
    g.setAttribute('aSeed', new BufferAttribute(this.seed, 1));
    g.setAttribute('aScale', new BufferAttribute(this.scale, 1));
    g.boundingSphere = null;
    this.geo = g;

    this.material = new ShaderMaterial({
      vertexShader: VERT, fragmentShader: FRAG,
      uniforms: {
        uTime: { value: 0 }, uGravity: { value: 1.62 }, uDrag: { value: 0 },
        uSize: { value: 0.04 }, uLife: { value: 7.5 }, uViewportH: { value: 900 },
        uColor: { value: new Color(0.52, 0.50, 0.48) },
      },
      transparent: true, depthWrite: false, blending: NormalBlending,
    });
    this.points = new Points(g, this.material);
    this.points.frustumCulled = false;
  }

  /**
   * @param {Vector3} at        contact point
   * @param {number} strength   0..1, scales grain count and speed
   * @param {Vector3} dir       travel direction, so the plume trails the step
   */
  burst(at, strength, dir) {
    const n = Math.max(4, Math.round(PER_BURST * strength));
    const t = this.material.uniforms.uTime.value;
    for (let i = 0; i < n; i++) {
      const idx = this.cursor; this.cursor = (this.cursor + 1) % MAX;
      const a = Math.random() * Math.PI * 2;
      // Low launch angles: a footfall sprays sideways, it does not fountain.
      const elev = 0.12 + Math.random() * 0.42;
      const speed = (0.4 + Math.random() * 1.5) * (0.5 + strength);
      this.origin[idx * 3] = at.x; this.origin[idx * 3 + 1] = at.y + 0.01; this.origin[idx * 3 + 2] = at.z;
      this.velocity[idx * 3] = Math.cos(a) * speed + (dir?.x || 0) * speed * 0.5;
      this.velocity[idx * 3 + 1] = Math.sin(elev) * speed;
      this.velocity[idx * 3 + 2] = Math.sin(a) * speed + (dir?.z || 0) * speed * 0.5;
      this.birth[idx] = t;
    }
    this.geo.attributes.aOrigin.needsUpdate = true;
    this.geo.attributes.aVelocity.needsUpdate = true;
    this.geo.attributes.aBirth.needsUpdate = true;
  }

  applyEnvironment(env) {
    const u = this.material.uniforms;
    u.uGravity.value = Math.max(0.001, env.g);
    u.uDrag.value = env.dust.drag;
    u.uLife.value = env.dust.life;
    u.uSize.value = env.dust.size;   // metres; the shader converts to pixels
    u.uColor.value.copy(env.dust.color);
    this.points.visible = !!env.terrain;      // nothing to kick up in free fall
  }

  update(time, viewportH) {
    this.material.uniforms.uTime.value = time;
    this.material.uniforms.uViewportH.value = viewportH;
  }

  /** Wipe every live grain — used when the scene changes under you. */
  clear() { this.birth.fill(-1e6); this.geo.attributes.aBirth.needsUpdate = true; }
}
