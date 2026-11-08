# Pico W telemetry dashboard

This is a local-only web dashboard for the MQTT telemetry published by the
Pico W. It subscribes to `robot/is06/state` and reuses the decoder in
`tools/wifi_telemetry_tool.py`.

## Run on Windows

From the repository root:

```powershell
python -m pip install -r dashboard\requirements.txt
python dashboard\app.py
```

Open <http://127.0.0.1:5000>.

The defaults match `config/wifi_secrets.h`: broker `172.20.10.2`, port
`1884`, and topic `robot/is06/state`. Override them without editing source:

```powershell
$env:MQTT_HOST = "172.20.10.2"
$env:MQTT_PORT = "1884"
python dashboard\app.py
```

Keep Mosquitto running with `tools\mosquitto_is06.conf` before starting the
dashboard. The dashboard binds to `127.0.0.1` by default and is not intended
to be exposed beyond the local computer.
