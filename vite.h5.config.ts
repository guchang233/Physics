import { defineConfig } from 'vite';
import { viteSingleFile } from 'vite-plugin-singlefile';

/**
 * 哔哩哔哩 Toy / 单页 H5：把 JS、CSS、字体打进一个 index.html。
 * Rapier 仍按需动态加载；本构建把它一并内联，保证离线也能启用试验接触后端。
 */
export default defineConfig({
  base: './',
  plugins: [
    viteSingleFile({
      removeViteModuleLoader: true,
      deleteInlinedFiles: true,
    }),
  ],
  build: {
    outDir: 'publish/h5',
    emptyOutDir: true,
    assetsInlineLimit: 100_000_000,
    cssCodeSplit: false,
    modulePreload: false,
    target: 'es2020',
    minify: 'esbuild',
    rollupOptions: {
      output: {
        inlineDynamicImports: true,
        manualChunks: undefined,
      },
    },
  },
  optimizeDeps: {
    include: ['jquery', '@pie-framework/mathquill'],
  },
});
