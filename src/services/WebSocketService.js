/**
 * WebSocket service for receiving maritime sensor data from RPi backend.
 * Auto-reconnects with exponential backoff.
 *
 * Expected message format:
 * {
 *   type: "frame_update",
 *   timestamp: 1234567890.123,
 *   lidar: { distance: 1.23, points: [[dist, angle], ...] },
 *   cv_detections: [{ label: "ladder", confidence: 0.92, bbox: [x, y, w, h] }],
 *   source_resolution: { w: 640, h: 480 },
 *   camera_urls: { cam1: "http://...", cam2: "http://..." }
 * }
 */
class WebSocketService {
  constructor(url) {
    this.url = url;
    this.ws = null;
    this.listeners = [];
    this.statusListeners = [];
    this.connected = false;
    this.reconnectDelay = 1000;
    this.maxReconnectDelay = 10000;
    this.shouldReconnect = true;
    this.lastMessage = null;
  }

  connect() {
    this.shouldReconnect = true;

    try {
      this.ws = new WebSocket(this.url);

      this.ws.onopen = () => {
        this.connected = true;
        this.reconnectDelay = 1000;
        this.statusListeners.forEach((cb) => cb(true));
      };

      this.ws.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          this.lastMessage = data;
          this.listeners.forEach((cb) => cb(data));
        } catch {
          // Ignore malformed messages
        }
      };

      this.ws.onclose = () => {
        this.connected = false;
        this.statusListeners.forEach((cb) => cb(false));
        this._scheduleReconnect();
      };

      this.ws.onerror = () => {
        // onclose will fire after this
      };
    } catch {
      this._scheduleReconnect();
    }
  }

  disconnect() {
    this.shouldReconnect = false;
    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }
  }

  onMessage(callback) {
    this.listeners.push(callback);
    return () => {
      this.listeners = this.listeners.filter((cb) => cb !== callback);
    };
  }

  onStatusChange(callback) {
    this.statusListeners.push(callback);
    return () => {
      this.statusListeners = this.statusListeners.filter((cb) => cb !== callback);
    };
  }

  getLastMessage() {
    return this.lastMessage;
  }

  _scheduleReconnect() {
    if (!this.shouldReconnect) return;

    setTimeout(() => {
      if (this.shouldReconnect) this.connect();
    }, this.reconnectDelay);

    this.reconnectDelay = Math.min(
      this.reconnectDelay * 2,
      this.maxReconnectDelay
    );
  }
}

export default WebSocketService;
