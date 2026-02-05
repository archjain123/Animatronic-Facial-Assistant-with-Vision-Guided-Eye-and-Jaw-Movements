import sounddevice as sd
import numpy as np
from piper import PiperVoice

VOICE_PATH = "tts/models/en_US-joe-medium/en_US-joe-medium.onnx"

voice = PiperVoice.load(VOICE_PATH)

text = "Hello. I am Skully."

audio_chunks = []

for chunk in voice.synthesize(text):
    audio_chunks.append(chunk.audio_float_array)

audio = np.concatenate(audio_chunks)

sd.play(audio, voice.config.sample_rate)
sd.wait()
