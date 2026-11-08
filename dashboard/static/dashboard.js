const fields = {
  state: document.querySelector("#vehicle-state"),
  lastSeen: document.querySelector("#last-seen"),
  speed: document.querySelector("#speed"),
  obstacle: document.querySelector("#obstacle"),
  hump: document.querySelector("#hump"),
  sequence: document.querySelector("#sequence"),
  uptime: document.querySelector("#uptime"),
  retry: document.querySelector("#retry"),
  backlog: document.querySelector("#backlog"),
  faults: document.querySelector("#faults"),
  count: document.querySelector("#message-count"),
  connection: document.querySelector("#connection"),
  error: document.querySelector("#error"),
};

function valueOrDash(value) {
  return value === null || value === undefined ? "—" : value;
}

function render(data) {
  const telemetry = data.telemetry;
  const online = data.connected && data.age_seconds !== null && data.age_seconds <= 5;
  fields.connection.textContent = online ? "MQTT online" : "MQTT offline";
  fields.connection.className = `status ${online ? "online" : "offline"}`;
  fields.count.textContent = `${data.message_count} message${data.message_count === 1 ? "" : "s"}`;
  fields.error.hidden = !data.error;
  fields.error.textContent = data.error || "";
  if (!telemetry) return;

  fields.state.textContent = telemetry.state;
  fields.lastSeen.textContent = data.age_seconds === null
    ? "No message received"
    : `Last update ${data.age_seconds}s ago`;
  fields.speed.textContent = valueOrDash(telemetry.speed_cm_s);
  fields.obstacle.textContent = valueOrDash(telemetry.obst_cm);
  fields.hump.textContent = valueOrDash(telemetry.hump_mm);
  fields.sequence.textContent = telemetry.seq;
  fields.uptime.textContent = (telemetry.t_ms / 1000).toFixed(1);
  fields.retry.textContent = telemetry.retry;
  fields.backlog.textContent = telemetry.backlog ? "Yes" : "No";
  fields.faults.textContent = telemetry.faults.length ? telemetry.faults.join(", ") : "None reported";
}

async function refresh() {
  try {
    const response = await fetch("/api/status", { cache: "no-store" });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    render(await response.json());
  } catch (error) {
    fields.connection.textContent = "Dashboard error";
    fields.connection.className = "status offline";
    fields.error.hidden = false;
    fields.error.textContent = error.message;
  }
}

refresh();
setInterval(refresh, 1000);
