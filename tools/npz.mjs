/**
 * npz.mjs — read NumPy .npz / .npy without NumPy.
 *
 * The motion packets ship their authoritative data as NPZ (the CSVs are a
 * flattened re-export), so anything that wants to check what the packets
 * actually contain has to read the NPZ. There is no Python in this toolchain,
 * and an .npz is just a ZIP of .npy members, so both formats are parsed here
 * directly.
 */
import fs from 'fs';
import zlib from 'zlib';

const DTYPES = {
  '<f8': { get: (d, o) => d.getFloat64(o, true), size: 8 },
  '<f4': { get: (d, o) => d.getFloat32(o, true), size: 4 },
  '<i8': { get: (d, o) => Number(d.getBigInt64(o, true)), size: 8 },
  '<i4': { get: (d, o) => d.getInt32(o, true), size: 4 },
  '<u4': { get: (d, o) => d.getUint32(o, true), size: 4 },
  '|b1': { get: (d, o) => d.getUint8(o) !== 0, size: 1 },
};

/**
 * Fixed-width byte strings, '|S30' and friends.
 *
 * NumPy pads these with NULs to the field width rather than terminating them,
 * so the trailing NULs have to be stripped or every joint name comes back with
 * invisible padding and no lookup ever matches. The retarget schema stores its
 * joint_cols this way, so without this a demo NPZ cannot be read at all.
 */
const parseBytes = (descr) => {
  const m = /^\|S(\d+)$/.exec(descr);
  if (!m) return null;
  const size = Number(m[1]);
  return {
    size,
    get: (d, o) => {
      let s = '';
      for (let i = 0; i < size; i++) {
        const c = d.getUint8(o + i);
        if (c === 0) break;
        s += String.fromCharCode(c);
      }
      return s;
    },
  };
};

/** Parse one .npy buffer into { shape, dtype, data }. */
export function parseNPY(buf) {
  if (buf.readUInt8(0) !== 0x93 || buf.toString('latin1', 1, 6) !== 'NUMPY') {
    throw new Error('not a .npy file');
  }
  const major = buf.readUInt8(6);
  const hlen = major === 1 ? buf.readUInt16LE(8) : buf.readUInt32LE(8);
  const hstart = major === 1 ? 10 : 12;
  const header = buf.toString('latin1', hstart, hstart + hlen);

  const descr = /'descr'\s*:\s*'([^']+)'/.exec(header)?.[1];
  const fortran = /'fortran_order'\s*:\s*(True|False)/.exec(header)?.[1] === 'True';
  const shape = (/'shape'\s*:\s*\(([^)]*)\)/.exec(header)?.[1] ?? '')
    .split(',').map((s) => s.trim()).filter(Boolean).map(Number);

  const dt = DTYPES[descr] || parseBytes(descr);
  if (!dt) throw new Error(`unsupported dtype ${descr}`);
  if (fortran) throw new Error('fortran-order arrays are not supported');

  const body = buf.subarray(hstart + hlen);
  const count = shape.reduce((a, b) => a * b, 1);
  const view = new DataView(body.buffer, body.byteOffset, body.byteLength);
  const data = new Array(count);
  for (let i = 0; i < count; i++) data[i] = dt.get(view, i * dt.size);
  return { shape, dtype: descr, data };
}

/**
 * Minimal ZIP reader — enough for .npz, which uses store or deflate and never
 * spans volumes. Reading the central directory rather than scanning local
 * headers is what makes the member sizes trustworthy.
 */
function unzipEntries(buf) {
  // End of central directory: scan back for the signature.
  let eocd = -1;
  for (let i = buf.length - 22; i >= 0 && i > buf.length - 65558; i--) {
    if (buf.readUInt32LE(i) === 0x06054b50) { eocd = i; break; }
  }
  if (eocd < 0) throw new Error('not a zip archive');
  const nEntries = buf.readUInt16LE(eocd + 10);
  let p = buf.readUInt32LE(eocd + 16);

  const out = {};
  for (let e = 0; e < nEntries; e++) {
    if (buf.readUInt32LE(p) !== 0x02014b50) throw new Error('bad central directory');
    const method = buf.readUInt16LE(p + 10);
    const compSize = buf.readUInt32LE(p + 20);
    const nameLen = buf.readUInt16LE(p + 28);
    const extraLen = buf.readUInt16LE(p + 30);
    const commentLen = buf.readUInt16LE(p + 32);
    const localOff = buf.readUInt32LE(p + 42);
    const name = buf.toString('latin1', p + 46, p + 46 + nameLen);

    // The local header's own name/extra lengths are authoritative for the data
    // offset; the central directory's extra field is a different field.
    const lNameLen = buf.readUInt16LE(localOff + 26);
    const lExtraLen = buf.readUInt16LE(localOff + 28);
    const dataStart = localOff + 30 + lNameLen + lExtraLen;
    const raw = buf.subarray(dataStart, dataStart + compSize);
    out[name] = method === 0 ? raw : zlib.inflateRawSync(raw);

    p += 46 + nameLen + extraLen + commentLen;
  }
  return out;
}

/** Load an .npz as { arrayName: {shape, dtype, data} }. */
export function loadNPZ(file) {
  const entries = unzipEntries(fs.readFileSync(file));
  const out = {};
  for (const [name, buf] of Object.entries(entries)) {
    out[name.replace(/\.npy$/, '')] = parseNPY(buf);
  }
  return out;
}

/** Reshape a flat array to nested rows of the last dimension. */
export function rows(arr) {
  const [n, ...rest] = arr.shape;
  const stride = rest.reduce((a, b) => a * b, 1);
  const out = new Array(n);
  for (let i = 0; i < n; i++) out[i] = arr.data.slice(i * stride, (i + 1) * stride);
  return out;
}
