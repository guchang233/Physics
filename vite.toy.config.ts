import { defineConfig, type Plugin } from 'vite';

/**
 * 哔哩哔哩 Toy 构建：入口 index.html 必须小于 1MB，
 * 因此 JS / CSS / 字体全部作为独立资源输出，而不是内联进 HTML。
 */
/** 只在 Toy 构建产物里注入官方 SDK，开发/普通构建保持干净。 */
function toySdk(): Plugin {
  return {
    name: 'toy-sdk',
    transformIndexHtml(html) {
      return html.replace(
        '</head>',
        '  <script src="//s1.hdslb.com/bfs/seed/toy/app/sdk/toy-sdk.js"></script>\n</head>',
      );
    },
  };
}

function stripLegacyFontFormats(): Plugin {
  return {
    name: 'toy-strip-legacy-fonts',
    enforce: 'post',
    generateBundle(_options, bundle) {
      for (const [fileName, item] of Object.entries(bundle)) {
        if (item.type === 'asset' && fileName.endsWith('.css')) {
          const css = typeof item.source === 'string'
            ? item.source
            : new TextDecoder().decode(item.source);
          item.source = css
            .replace(/src:\s*url\([^)]*\.eot\)\s*;?/g, '')
            .replace(/url\([^)]*\.eot\)\s*,?\s*/g, '')
            .replace(/,?\s*url\([^)]*\.svg#[^)]*\)\s*format\(["']svg["']\)/g, '');
        }
        if (item.type === 'asset' && /Symbola.*\.(eot|svg)$/i.test(fileName)) {
          delete bundle[fileName];
        }
      }
    },
  };
}

export default defineConfig({
  base: './',
  plugins: [toySdk(), stripLegacyFontFormats()],
  build: {
    outDir: 'publish/h5',
    emptyOutDir: true,
    target: 'es2020',
    assetsInlineLimit: 0,
    cssCodeSplit: false,
    modulePreload: false,
    sourcemap: false,
    minify: 'esbuild',
    rollupOptions: {
      output: {
        entryFileNames: 'assets/[name]-[hash].js',
        chunkFileNames: 'assets/[name]-[hash].js',
        assetFileNames: 'assets/[name]-[hash][extname]',
      },
    },
  },
  optimizeDeps: {
    include: ['jquery', '@pie-framework/mathquill'],
  },
});
