import { sveltekit } from '@sveltejs/kit/vite';
import { defineConfig } from 'vite';

export default defineConfig({
	plugins: [sveltekit()],
	server: {
		port: 8000,
		proxy: {
			'/ros': {
				target: 'ws://localhost:3000',
				ws: true,
			}
		}
	}
});
