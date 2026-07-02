# RDK X5 Edge Client (Mock)

Mock implementation for simulation testing. No real hardware connections.

## Usage

```bash
pip install -r requirements.txt
python main.py --url ws://localhost:9200/ws/board
```

## Features

- Connects to backend /ws/board WebSocket
- Reports mock sensor data (temperature, humidity, distance)
- Responds to hardware control commands (mock execution)
