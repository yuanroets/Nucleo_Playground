# RTT + Plotter Quick Start

Use this when streaming data over SEGGER RTT and plotting live on your Mac.

## 1) Activate venv (both terminals)
```zsh
source venv/bin/activate
```

## 2) Make temp file
```zsh
mkfifo /tmp/imu_stream 2>/dev/null
```

## 3) Start RTT capture (Terminal 1)
From the project root:
```zsh
python3 tools/rtt_capture.py > /tmp/imu_stream 
```

## 4) Start live plotter (Terminal 2)
From the project root:
```zsh
cat /tmp/imu_stream | python3 tools/imu_live_plot_logger.py
```