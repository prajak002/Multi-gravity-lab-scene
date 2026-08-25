/** Numbers that change only because gravity changed. */
export class HUD {
  constructor(root) {
    this.el = document.createElement('div');
    this.el.className = 'hud';
    this.el.innerHTML = `<h2>Field readout</h2><dl></dl>`;
    this.dl = this.el.querySelector('dl');
    this.rows = {};
    this.labels = {};
    root.appendChild(this.el);

    this.title = document.createElement('div');
    this.title.className = 'title-card';
    this.title.innerHTML = `<b></b><span></span>`;
    root.appendChild(this.title);
  }

  row(key, label, value, unit = '') {
    if (!this.rows[key]) {
      const dt = document.createElement('dt'); dt.textContent = label;
      const dd = document.createElement('dd');
      this.dl.append(dt, dd);
      this.rows[key] = dd;
      this.labels[key] = dt;
    }
    this.rows[key].innerHTML = `${value}${unit ? `<u>${unit}</u>` : ''}`;
  }

  /**
   * Take a row away again.
   *
   * Rows used to be write-only, so anything scene-specific outlived its scene:
   * walking on the Moon still showed the butterfly stroke's amplitude and rate
   * from the last ISS run, and a reader has no way to tell a stale number from
   * a live one.
   */
  dropRow(key) {
    if (!this.rows[key]) return;
    this.rows[key].remove();
    this.labels[key]?.remove();
    delete this.rows[key];
    delete this.labels[key];
  }

  setTitle(main, sub) {
    this.title.querySelector('b').textContent = main;
    this.title.querySelector('span').textContent = sub;
  }

  show(on) {
    this.el.classList.toggle('in', on);
    this.title.classList.toggle('in', on);
  }
}
