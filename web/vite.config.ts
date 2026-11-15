/// <reference types="vitest/config" />
import tailwindcss from "@tailwindcss/vite";
import react from "@vitejs/plugin-react";
import { defineConfig } from "vite";

// Dev: `npm run dev` proxies /ws and /api to the gateway (VITE_GATEWAY, default the mock on localhost:8080).
const target = process.env.VITE_GATEWAY ?? "http://localhost:8080";

export default defineConfig({
  plugins: [react(), tailwindcss()],
  server: {
    host: true,
    proxy: {
      "/api": target,
      "/ws": { target: target.replace(/^http/, "ws"), ws: true },
    },
  },
  build: { outDir: "dist", emptyOutDir: true },
  test: { environment: "node", include: ["src/**/*.test.ts"] },
});
