import json
import queue
import sounddevice as sd
from vosk import Model, KaldiRecognizer

MODEL_PATH = "models/vosk-model-small-en-us-0.15"
SAMPLE_RATE = 16000

q = queue.Queue()

def callback(indata, frames, time, status):
    q.put(bytes(indata))

model = Model(MODEL_PATH)
recognizer = KaldiRecognizer(model, SAMPLE_RATE)
recognizer.SetWords(False)

with sd.RawInputStream(
    samplerate=SAMPLE_RATE,
    blocksize=4000,   # smaller = faster reaction
    dtype="int16",
    channels=1,
    callback=callback
):
    print("🎙️ Listening...")

    while True:
        data = q.get()

        if recognizer.AcceptWaveform(data):
            result = json.loads(recognizer.Result())
            text = result.get("text", "").strip()
            if text:
                print("FINAL:", text)
        else:
            partial = json.loads(recognizer.PartialResult()).get("partial", "")
            if partial:
                print("...", partial, end="\r")
