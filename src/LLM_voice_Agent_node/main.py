import os
import json
import queue
import time
import serial
import threading
import sounddevice as sd
import numpy as np

from dotenv import load_dotenv
from vosk import Model as VoskModel, KaldiRecognizer
from piper import PiperVoice
from groq import Groq

MIC_ENABLED = True

# ================== LOAD ENV ==================
load_dotenv(".env")  # loads .env file

GROQ_API_KEY = os.getenv("GROQ_API_KEY")
if not GROQ_API_KEY:
    raise RuntimeError("GROQ_API_KEY not found in .env file")

# ================== CONFIG ==================

SAMPLE_RATE = 16000
BLOCKSIZE = 4000
SILENCE_TIMEOUT = 0.8

VOSK_MODEL_PATH = "models/vosk-model-en-us-0.22-lgraph"
TTS_MODEL_PATH = "tts/models/en_US-joe-medium/en_US-joe-medium.onnx"

GROQ_MODEL = "llama-3.1-8b-instant"


SYSTEM_PROMPT = (
    "You are Skully, a chill robot friend.\n"
    "Talk naturally like a real person, keep it simple and relaxed.\n"
    "Be funny but not try-hard, humor should feel natural not forced.\n"
    "Keep replies short, 1 to 2 sentences max.\n"
    "Don't overuse words like dude, bruh, haha, lol. Use them rarely.\n"
    "If someone asks you to sing, sing just 2 to 3 lines of a simple popular song casually.\n"
    "If someone asks you to laugh, laugh naturally but keep it short.\n"
    "Sometimes crack a dry joke or say something witty.\n"
    "You are a friend, not a performer. Stay cool and unbothered.\n"
    "Occasionally ask how they are doing but don't force it.\n"
)

# ================== SERIAL CONNECTION ==================
try:
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # Adjust port as needed
    time.sleep(2)  # Wait for connection
    print("✅ Serial connected to animatronic")
except Exception as e:
    print(f"⚠️ Serial connection failed: {e}")
    ser = None

# ============================================

# ---------- AUDIO QUEUE ----------
audio_q = queue.Queue()

def audio_callback(indata, frames, time_info, status):
    if MIC_ENABLED:
        audio_q.put(bytes(indata))


# ---------- INIT STT ----------
vosk_model = VoskModel(VOSK_MODEL_PATH)
recognizer = KaldiRecognizer(vosk_model, SAMPLE_RATE)
recognizer.SetWords(False)

# ---------- INIT TTS ----------
voice = PiperVoice.load(TTS_MODEL_PATH)

# ---------- JAW CONTROL ----------
def send_jaw_command(command):
    """Send commands to Arduino/ESP32"""
    if ser:
        try:
            ser.write(f"{command}\n".encode())
        except Exception as e:
            print(f"Serial error: {e}")

def jaw_animation_thread(audio_duration):
    """Animate jaw during speech"""
    if not ser:
        return
    
    send_jaw_command("JAW_START")
    time.sleep(audio_duration)
    send_jaw_command("JAW_STOP")

def speak(text: str):
    global MIC_ENABLED

    MIC_ENABLED = False  # 🔇 mute mic

    audio_chunks = []
    for chunk in voice.synthesize(text):
        audio_chunks.append(chunk.audio_float_array)

    if audio_chunks:
        audio = np.concatenate(audio_chunks)
        
        # Slow down the audio playback
        playback_rate = voice.config.sample_rate * 0.95  # 0.85 = 15% slower
        
        # Calculate duration based on slower playback
        duration = len(audio) / playback_rate
        
        # Start jaw animation in separate thread
        jaw_thread = threading.Thread(target=jaw_animation_thread, args=(duration,))
        jaw_thread.start()
        
        # Play audio at slower rate
        sd.play(audio, playback_rate)
        sd.wait()
        
        # Wait for jaw thread to finish
        jaw_thread.join()

    time.sleep(0.1)
    MIC_ENABLED = True  # 🎙️ unmute mic

    
# ---------- INIT GROQ ----------
client = Groq(api_key=GROQ_API_KEY)

# ---------- CONVERSATION MEMORY ----------
messages = [
    {"role": "system", "content": SYSTEM_PROMPT}
]

# ---------- MAIN LOOP ----------
print("🎙️ Listening... (Ctrl+C to stop)")

final_text = ""
last_speech_time = time.time()

with sd.RawInputStream(
    samplerate=SAMPLE_RATE,
    blocksize=BLOCKSIZE,
    dtype="int16",
    channels=1,
    callback=audio_callback,
):
    while True:
        data = audio_q.get()

        if recognizer.AcceptWaveform(data):
            result = json.loads(recognizer.Result())
            text = result.get("text", "").strip()
            if text:
                final_text += " " + text
                last_speech_time = time.time()
        else:
            partial = json.loads(recognizer.PartialResult()).get("partial", "")
            if partial:
                last_speech_time = time.time()

        # ---------- ENDPOINT ----------
        if final_text and (time.time() - last_speech_time) > SILENCE_TIMEOUT:
            user_text = final_text.strip()
            print(f"\n🧑 USER: {user_text}")

            messages.append({"role": "user", "content": user_text})

            try:
                response = client.chat.completions.create(
                    model=GROQ_MODEL,
                    messages=messages,
                    max_tokens=60,
                    temperature=0.8,
                )
                assistant_text = response.choices[0].message.content.strip()
            except Exception as e:
                assistant_text = "bruh something went wrong lol"

            if not assistant_text:
                assistant_text = "Okay."

            print(f"🤖 SKULLY: {assistant_text}\n")
            speak(assistant_text)

            messages.append({"role": "assistant", "content": assistant_text})

            # keep memory short (system + last 4 turns)
            if len(messages) > 9:
                messages = [messages[0]] + messages[-8:]

            final_text = ""
            recognizer.Reset()