import { defineConfig } from 'vite';

export default defineConfig({
  server: { fs: { allow: ['..'] } },
  // URDF meshes are fetched at runtime from public/robots/<id>/, so nothing
  // here needs to inline them; keep the asset pipeline out of the way.
  assetsInclude: ['**/*.urdf', '**/*.STL', '**/*.stl'],
  build: { target: 'es2022', chunkSizeWarningLimit: 2400 },
});
