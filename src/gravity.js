/**
 * Entry point for the three-gravity comparison.
 *
 * Deliberately a separate page from the main arena. The arena answers "what
 * does a G1 look like on the Moon"; this answers "what does gravity DO", and
 * the two need different framing, different camera behaviour and a different
 * readout. Sharing one page would compromise both.
 */
import { GravityCompare } from './ui/GravityCompare.js';

const canvas = document.getElementById('stage');
const ui = document.getElementById('ui');
const boot = document.getElementById('boot');

const app = new GravityCompare(canvas, ui);

function showFailure(err) {
  boot.classList.remove('gone');
  boot.innerHTML = '';
  const box = document.createElement('div');
  box.className = 'boot-error';
  box.innerHTML = `<b>Could not load the robots</b><pre></pre>
    <span>The URDF meshes are fetched at runtime from <code>public/robots/g1/</code>.
    If that folder is missing from the deploy, this is exactly how it fails.</span>`;
  box.querySelector('pre').textContent = String((err && err.message) || err);
  boot.appendChild(box);
}

const resize = () => app.resize(innerWidth, innerHeight);
addEventListener('resize', resize);
resize();

let last = performance.now();
function loop(now) {
  const dt = Math.min(0.05, (now - last) / 1000);
  last = now;
  app.frame(dt);
  requestAnimationFrame(loop);
}

app.start('walk')
  .then(() => { boot.classList.add('gone'); requestAnimationFrame(loop); })
  .catch(showFailure);

// Debug handle so a headless check can assert on the thing that matters —
// that the three robots have actually pulled apart — rather than on a picture.
window.__gravity = app;
