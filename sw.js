// Minimal service worker — enables "Add to Home Screen" / installability.
// Network passthrough only (no aggressive caching, so the app is always fresh
// and Firebase realtime data is never served stale).
const CACHE = 'utm-vending-v1';

self.addEventListener('install', () => self.skipWaiting());
self.addEventListener('activate', (e) => e.waitUntil(self.clients.claim()));

// A fetch handler is required for installability. We just go to the network.
self.addEventListener('fetch', (e) => {
  e.respondWith(fetch(e.request).catch(() => caches.match(e.request)));
});
