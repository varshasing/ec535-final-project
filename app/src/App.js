import React, { useEffect, useRef, useState } from 'react';
import './App.css';

function App() {
  const ws = useRef(null);
  const pressed = useRef(false);
  const [videoError, setVideoError] = useState(false);
  const [blockStatus, setBlockStatus] = useState('clear');
  const [height, setHeight] = useState(-10);
  const [speed, setSpeed] = useState(10);
  const [wsLatency, setWsLatency] = useState(null);

  useEffect(() => {
    ws.current = new WebSocket('ws://172.20.10.8:8765');

    ws.current.onopen = () => {
      console.log('WebSocket connected');

      // Ping every 2 seconds
      const interval = setInterval(() => {
        const timestamp = Date.now();
        ws.current.send(JSON.stringify({ type: 'ping', timestamp }));
      }, 2000);
      ws.current.pingInterval = interval;
    };

    ws.current.onclose = () => {
      console.log('WebSocket disconnected');
      clearInterval(ws.current?.pingInterval);
    };

    ws.current.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        if (data.status) {
          setBlockStatus(data.status);
        } else if (data.type === 'pong' && data.timestamp) {
          const latency = Date.now() - data.timestamp;
          setWsLatency(latency);
        }
      } catch (e) {
        console.error("Invalid message", event.data);
      }
    };

    return () => {
      clearInterval(ws.current?.pingInterval);
      ws.current?.close();
    };
  }, []);

  const sendCommand = (command) => {
    if (ws.current?.readyState === 1) {
      const payload = { key: command };
      ws.current.send(JSON.stringify(payload));
      console.log('Sent:', payload);
    } else {
      console.warn('WebSocket not ready. Command not sent:', command);
    }
  };

  const sendHeight = (val) => {
    setHeight(val);
    ws.current?.send(JSON.stringify({ height: val }));
  };

  const sendSpeed = (val) => {
    setSpeed(val);
    ws.current?.send(JSON.stringify({ speed: val }));
  };

  useEffect(() => {
    const keys = ['w', 'a', 's', 'd'];

    const handleKeyDown = (event) => {
      const key = event.key.toLowerCase();
      if (keys.includes(key) && !pressed.current) {
        pressed.current = true;
        sendCommand(key);
      }
    };

    const handleKeyUp = (event) => {
      const key = event.key.toLowerCase();
      if (keys.includes(key)) {
        pressed.current = false;
        sendCommand('x');
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, []);

  const trickCommands = ['bow', 'lie_down', 'moonwalk', 'nod', 'spacewalk', 'stand', 'push-up'];

  return (
    <div className="App">
      <header className="App-header">
        <h1 className="App-title">PuppyPi Controller</h1>
        <p>Press W (forward), A (left), S (backward), D (right), and X (stop) to control the robot:</p>

        <img
          src="http://172.20.10.8:8081/?action=stream"
          alt="Live stream"
          width="640"
          height="480"
          onError={() => setVideoError(true)}
          style={{ border: '2px solid white' }}
        />

        {videoError && (
          <p style={{ color: 'red', marginTop: '1rem' }}>
            Error loading video stream. Check if the Raspberry Pi is online and running MJPEG streamer.
          </p>
        )}

        <p style={{ marginTop: '10px' }}>
          Block Status: <strong>{blockStatus.replace('_', ' ')}</strong>
        </p>

        <p style={{ marginTop: '10px' }}>
          WebSocket Latency: <strong>{wsLatency !== null ? `${wsLatency} ms` : 'Measuring...'}</strong>
        </p>

        <div style={{ marginTop: '20px' }}>
          <label>
            Height: {height}
            <input
              type="range"
              min="-20"
              max="0"
              step="1"
              value={height}
              onChange={(e) => sendHeight(Number(e.target.value))}
            />
          </label>
          <br />
          <label>
            Speed: {speed}
            <input
              type="range"
              min="0"
              max="20"
              step="1"
              value={speed}
              onChange={(e) => sendSpeed(Number(e.target.value))}
            />
          </label>
        </div>

        <div style={{ marginTop: '20px', display: 'flex', flexWrap: 'wrap', gap: '10px' }}>
          {trickCommands.map((cmd) => (
            <button key={cmd} onClick={() => sendCommand(cmd)} style={{ padding: '10px 20px' }}>
              {cmd.replace(/_/g, ' ').replace(/\b\w/g, (c) => c.toUpperCase())}
            </button>
          ))}
        </div>
      </header>
    </div>
  );
}

export default App;
