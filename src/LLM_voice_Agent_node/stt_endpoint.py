import json
import queue
import sounddevice as sd
import time
from vosk import Model, KaldiRecognizer

MODEL_PATH = "models/vosk-model-small-en-us-0.15"
SAMPLE_RATE = 16000
SILENCE_TIMEOUT = 0.8  # seconds

q = queue.Queue()
last_speech_time = time.time()
final_text = ""

def callback(indata, frames, time_info, status):
    q.put(bytes(indata))

model = Model(MODEL_PATH)
recognizer = KaldiRecognizer(model, SAMPLE_RATE)
recognizer.SetWords(False)

with sd.RawInputStream(
    samplerate=SAMPLE_RATE,
    blocksize=4000,
    dtype="int16",
    channels=1,
    callback=callback
):
    print("🎙️ Speak...")

    while True:
        data = q.get()

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

        # silence detection
        if final_text and (time.time() - last_speech_time) > SILENCE_TIMEOUT:
            print("✅ USER SAID:", final_text.strip())
            final_text = ""
            recognizer.Reset()
