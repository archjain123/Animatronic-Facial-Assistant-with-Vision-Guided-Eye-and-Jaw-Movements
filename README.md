# Animatronic Facial Assistant with Vision-Guided Eye and Jaw Movements

An integrated **vision-guided animatronic system** that combines **face
tracking**, **eye tracking**, **jaw actuation**, and an **LLM-powered
conversational voice agent**.\
Designed for robotics, expressive animatronics, and interactive
assistants.

The system uses: - Real-time inference for face and eye tracking\
- ESP32 and PlatformIO-based motor control nodes\
- A Python-based LLM voice agent for speech recognition, reasoning, and
TTS\
- Vision-driven servo motion for realistic eye, jaw, and neck movements

## Features

-   **Face Tracking Node (ESP32, PlatformIO)**
    -   Real-time detection and servo control\
    -   Lightweight optimized firmware for responsive movement
-   **Eye Tracking Module**
    -   CNN-based DNN (Caffe SSD model)\
    -   Haar cascade alternatives for low-compute systems\
    -   Vision-guided pan/tilt eye motion
-   **Jaw and Neck Animations**
    -   Test routines for blink, jaw sync, eye pan/tilt, neck actuation\
    -   Configurable servo limits and calibration tools
-   **LLM Voice Agent Node**
    -   Speech-to-text (Vosk)\
    -   TTS engine integration\
    -   Groq-powered LLM agent for chatbot-like conversation\
    -   Modular architecture for plugging in different models or
        endpoints

## Project Structure

    Animatronic-Facial-Assistant-with-Vision-Guided-Eye-and-Jaw-Movements/
    ├── doc/                     # Reference documents & servo limit charts
    ├── Eye Tracking/            # Vision-based eye tracking models & scripts
    ├── Face Tracking/           # ESP32/PlatformIO projects for head/face tracking
    ├── src/                     # Main code: LLM agent + servo controller node
    ├── test/                    # Hardware test sketches for servos/animations
    ├── LICENSE                  # License information
    └── README.md                # This documentation

## System Architecture

Camera -->|Frames| EyeTracking[Eye Tracking (DNN / Haar)]
EyeTracking -->|Eye position| ServoESP[ESP32 Servo Controller]

Camera2 -->|Frames| FaceTracker[Face Tracking Node]
FaceTracker -->|Coordinates| ServoESP

Mic --> STT[Speech-to-Text (Vosk)]
STT --> LLM[LLM Voice Agent (Groq / Python)]
LLM --> TTS[Text-to-Speech Engine]
TTS --> Speaker

LLM -->|Commands| ServoESP
ServoESP --> Hardware[Servos: Eye, Jaw, Neck]
```

## Setup Instructions

### 1. Clone the repository

``` bash
git clone git@github.com:YOUR_USERNAME/Animatronic-Facial-Assistant-with-Vision-Guided-Eye-and-Jaw-Movements.git
cd Animatronic-Facial-Assistant-with-Vision-Guided-Eye-and-Jaw-Movements
```

## Eye Tracking Module

Location: `Eye Tracking/`

Run DNN-based tracker:

``` bash
python3 DNN.py
```

## Face Tracking (ESP32)

Build & upload:

``` bash
cd Face\ Tracking/Platformio_memento_camera_node
pio run --target upload
```

## LLM Voice Agent Node

Create virtual environment:

``` bash
cd src/LLM_voice_Agent_node
python3 -m venv virEnv
source virEnv/bin/activate
pip install -r requirements.txt
```

Run:

``` bash
python3 main.py
```

## ESP32 Servo Controller Node

Upload:

``` bash
pio run --target upload
```

## Hardware Servo Tests

Tests in the `test/` folder: - blink_jaw\
- blink_test\
- eyePan_Tilt\
- neck_test\
- servo_limit_testing

## Documentation

`doc/` contains servo limit charts, reference images, and calibration
resources.

## Roadmap

-   Add ROS2 integration\
-   Add depth sensing\
-   GPU-accelerated inference\
-   Emotion-mapped facial animation\
-   Phoneme-based jaw sync\
-   Web dashboard for tuning

## License

See `LICENSE` file.
