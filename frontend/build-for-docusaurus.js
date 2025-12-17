/**
 * Script to build frontend components for Docusaurus integration
 * This creates a bundled version that can be imported into Docusaurus
 */

import { build } from 'vite';
import { resolve } from 'path';

const config = {
  build: {
    lib: {
      entry: resolve(__dirname, 'src/components/RagChatbot.tsx'),
      name: 'RagChatbot',
      fileName: 'rag-chatbot',
      formats: ['umd'] // Universal Module Definition for browser compatibility
    },
    rollupOptions: {
      external: ['react', 'react-dom'],
      output: {
        globals: {
          react: 'React',
          'react-dom': 'ReactDOM'
        }
      }
    },
    outDir: '../book/dist',
    emptyOutDir: true
  },
  define: {
    'process.env.NODE_ENV': JSON.stringify('production')
  }
};

async function buildForDocusaurus() {
  try {
    console.log('Building RagChatbot component for Docusaurus integration...');
    await build(config);
    console.log('Build completed successfully!');
    console.log('RagChatbot component is now available in ../book/dist/rag-chatbot.umd.js');
  } catch (error) {
    console.error('Build failed:', error);
    process.exit(1);
  }
}

buildForDocusaurus();