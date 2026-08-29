import { defineConfig } from 'vite';
import { resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const dir = fileURLToPath(new URL('.', import.meta.url));

export default defineConfig({
  server: { fs: { allow: ['..'] } },
  // URDF meshes are fetched at runtime from public/robots/<id>/, so nothing
  // here needs to inline them; keep the asset pipeline out of the way.
  assetsInclude: ['**/*.urdf', '**/*.STL', '**/*.stl'],
  build: {
    target: 'es2022',
    chunkSizeWarningLimit: 2400,
    // BOTH entry points, explicitly.
    //
    // Vite builds index.html and only index.html unless told otherwise. The
    // dev server happily serves any .html it finds, so arena.html worked
    // locally and would simply have 404'd in production — the build output
    // would not have contained it at all. Listing it here is the only thing
    // that puts it in dist/.
    rollupOptions: {
      input: {
        main: resolve(dir, 'index.html'),
        arena: resolve(dir, 'arena.html'),
        gravity: resolve(dir, 'gravity.html'),
        arms: resolve(dir, 'arms.html'),
      },
    },
  },
});
