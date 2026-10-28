export class Link {
  constructor(handlers = {}) {
    this.handlers = handlers;
    this.ws = null;
    this.retry = 1000;
    this.lastPong = Date.now();
    this.manualClose = false;
    this.timer = null;
  }

  connect() {
    this.manualClose = false;
    const scheme = location.protocol === "https:" ? "wss" : "ws";
    const token = localStorage.getItem("console_token") || "";
    const url = `${scheme}://${location.host}/ws?token=${encodeURIComponent(token)}`;
    try {
      this.ws = new WebSocket(url);
    } catch {
      this.scheduleReconnect();
      return;
    }
    this.ws.onopen = () => {
      this.retry = 1000;
      this.lastPong = Date.now();
      this.handlers.onStatus?.(true);
      if (this.timer) clearInterval(this.timer);
      this.timer = setInterval(() => this.heartbeat(), 4000);
    };
    this.ws.onmessage = (event) => {
      let message = null;
      try {
        message = JSON.parse(event.data);
      } catch {
        return;
      }
      if (message.type === "pong") {
        this.lastPong = Date.now();
        return;
      }
      if (message.type === "snapshot") this.handlers.onSnapshot?.(message);
      else if (message.type === "event") this.handlers.onEvent?.(message);
      else if (message.type === "role") this.handlers.onRole?.(message.role);
      else if (message.type === "detections") this.handlers.onDetections?.(message);
      else if (message.type === "error") this.handlers.onError?.(message.text);
    };
    this.ws.onclose = () => {
      this.handlers.onStatus?.(false);
      if (this.timer) clearInterval(this.timer);
      this.timer = null;
      if (!this.manualClose) this.scheduleReconnect();
    };
    this.ws.onerror = () => {
      try {
        this.ws.close();
      } catch {
        /* ignore */
      }
    };
  }

  heartbeat() {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) return;
    if (Date.now() - this.lastPong > 15000) {
      this.ws.close();
      return;
    }
    this.send({ type: "ping", t: Date.now() / 1000 });
  }

  scheduleReconnect() {
    setTimeout(() => this.connect(), this.retry);
    this.retry = Math.min(8000, Math.round(this.retry * 1.6));
  }

  reconnect() {
    this.manualClose = true;
    if (this.ws) {
      try {
        this.ws.close();
      } catch {
        /* ignore */
      }
    }
    this.retry = 500;
    setTimeout(() => this.connect(), 120);
  }

  send(payload) {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) return false;
    this.ws.send(JSON.stringify(payload));
    return true;
  }
}
