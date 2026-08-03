import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// Dev: `npm run dev` proxies /api to the Rust server on :7878.
// Prod: `npm run build` emits dist/, which studio-server serves statically.
export default defineConfig({
	plugins: [react()],
	server: {
		proxy: {
			'/api': 'http://127.0.0.1:7878',
		},
	},
})
