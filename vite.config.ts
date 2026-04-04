import { defineConfig } from 'vite';
import { resolve } from 'path';

export default defineConfig({
  resolve: {
    alias: {
      '@sim': resolve(__dirname, 'src/sim'),
      '@ui': resolve(__dirname, 'src/ui'),
      '@lib': resolve(__dirname, 'src/lib'),
      '@worker': resolve(__dirname, 'src/worker'),
    },
  },
  worker: {
    format: 'es',
  },
});
