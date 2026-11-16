import { defineConfig } from 'vite';

export default defineConfig({
  optimizeDeps: {
    include: ['jquery', '@pie-framework/mathquill'],
  },
  server: {
    host: '127.0.0.1',
    port: 5173,
  },
});
