/**
 * The lobby. It enters from the left, and it is the only place a run is
 * configured: pick a robot, pick a field, enter. Leaving it slides it back off
 * to the left and hands the camera a cinematic move into the arena, so the
 * transition between "choosing" and "watching" is one continuous gesture.
 */
import { ROBOTS } from '../render/Robots.js';
import { ENVIRONMENTS, gRatio } from '../render/Environments.js';
import { MOTIONS } from '../sim/Gait.js';

export class Lobby {
  /** @param {(sel:{robot:string, env:string}) => void} onEnter */
  constructor(root, onEnter) {
    this.onEnter = onEnter;
    this.sel = { robot: 'g1', env: 'moon', motion: 'walk' };

    this.el = document.createElement('aside');
    this.el.className = 'lobby';
    this.el.innerHTML = `
      <h1>Multi-Gravity Arena<small>Unitree locomotion across four fields</small></h1>
      <div class="group" data-group="robot"><label>Platform</label></div>
      <div class="group" data-group="env"><label>Gravitational field</label></div>
      <div class="group" data-group="motion"><label>Motion</label></div>
      <button class="enter">Enter arena</button>`;

    this.tab = document.createElement('button');
    this.tab.className = 'lobby-tab';
    this.tab.textContent = 'Lobby';
    this.tab.addEventListener('click', () => this.show());

    const robotGroup = this.el.querySelector('[data-group="robot"]');
    for (const r of ROBOTS) {
      robotGroup.appendChild(this._card(r.id, 'robot', r.short, r.blurb,
        r.kind === 'quadruped' ? '4 legs' : '2 legs'));
    }
    const envGroup = this.el.querySelector('[data-group="env"]');
    for (const e of ENVIRONMENTS) {
      const rel = e.g === 0 ? 'free fall' : `${gRatio(e).toFixed(2)} g⊕`;
      envGroup.appendChild(this._card(e.id, 'env', e.short, e.blurb, rel));
    }

    const motionGroup = this.el.querySelector('[data-group="motion"]');
    for (const m of MOTIONS) {
      motionGroup.appendChild(this._card(m.id, 'motion', m.name, m.blurb,
        m.id === 'swim' ? 'free fall' : ''));
    }

    this.el.querySelector('.enter').addEventListener('click', () => {
      this.hide();
      this.onEnter({ ...this.sel });
    });

    root.append(this.el, this.tab);
    this._paint();
    // One frame before adding .in, so the browser has the off-screen transform
    // as a starting value and actually animates rather than snapping.
    requestAnimationFrame(() => requestAnimationFrame(() => this.show()));
  }

  _card(id, kind, title, blurb, badge) {
    const b = document.createElement('button');
    b.className = 'card';
    b.dataset.id = id; b.dataset.kind = kind;
    b.innerHTML = `<div class="row"><b></b><em></em></div><span></span>`;
    b.querySelector('b').textContent = title;
    b.querySelector('em').textContent = badge;
    b.querySelector('span').textContent = blurb;
    b.addEventListener('click', () => { this.sel[kind] = id; this._paint(); });
    return b;
  }

  _paint() {
    const iss = this.sel.env === 'iss';
    for (const c of this.el.querySelectorAll('.card')) {
      c.classList.toggle('on', this.sel[c.dataset.kind] === c.dataset.id);
      if (c.dataset.kind !== 'motion') continue;
      // In free fall a contact gait is not merely unusual, it is impossible:
      // there is no ground to push off. Show that rather than silently
      // correcting the user afterwards.
      const swim = c.dataset.id === 'swim';
      c.classList.toggle('off', iss ? !swim : swim);
    }
  }

  show() { this.el.classList.add('in'); this.tab.style.opacity = '0'; }
  hide() { this.el.classList.remove('in'); this.tab.style.opacity = '1'; }
}
