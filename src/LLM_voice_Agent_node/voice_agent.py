import json
import queue
import time
import sounddevice as sd
import numpy as np

from vosk import Model as VoskModel, KaldiRecognizer
from llama_cpp import Llama
from piper import PiperVoice

# ================== CONFIG ==================

SAMPLE_RATE = 16000
BLOCKSIZE = 4000
SILENCE_TIMEOUT = 0.8

VOSK_MODEL_PATH = "models/vosk-model-small-en-us-0.15"
LLM_MODEL_PATH = "models/tinyllama-1.1b-chat-v1.0.Q2_K.gguf"  # change if needed
TTS_MODEL_PATH = "tts/models/en_US-joe-medium/en_US-joe-medium.onnx"

SYSTEM_PROMPT = (
    "You are Skully, a voice assistant.\n"
    "Answer briefly and directly.\n"
    "One sentence only.\n"
    "No examples, no rules, no explanations.\n"
)

# ===========================================

# ---------- AUDIO QUEUE ----------
audio_q = queue.Queue()

def audio_callback(indata, frames, time_info, status):
    audio_q.put(bytes(indata))

# ---------- INIT STT ----------
vosk_model = VoskModel(VOSK_MODEL_PATH)
recognizer = KaldiRecognizer(vosk_model, SAMPLE_RATE)
recognizer.SetWords(False)

# ---------- INIT LLM ----------
llm = Llama(
    model_path=LLM_MODEL_PATH,
    n_ctx=1024,
    n_threads=6,
)

# ---------- INIT TTS ----------
voice = PiperVoice.load(TTS_MODEL_PATH)

def speak(text: str):
    audio_chunks = []
    for chunk in voice.synthesize(text):
        audio_chunks.append(chunk.audio_float_array)

    audio = np.concatenate(audio_chunks)
    sd.play(audio, voice.config.sample_rate)
    sd.wait()

# ---------- CONVERSATION MEMORY ----------
messages = [{"role": "system", "content": SYSTEM_PROMPT}]

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

        # ---- ENDPOINT (user stopped talking) ----
        if final_text and (time.time() - last_speech_time) > SILENCE_TIMEOUT:
            user_text = final_text.strip()
            print(f"\n🧑 USER: {user_text}")

            messages.append({"role": "user", "content": user_text})

            response = llm.create_chat_completion(
                messages=messages,
                max_tokens=25,
                temperature=0.2,
                stop=["</s>", "\n"],
            )

            assistant_text = response["choices"][0]["message"]["content"].strip()
            print(f"🤖 SKULLY: {assistant_text}\n")

            speak(assistant_text)

            messages.append({"role": "assistant", "content": assistant_text})

            # keep memory small (system + last 4 turns)
            if len(messages) > 9:
                messages = [messages[0]] + messages[-8:]

            final_text = ""
            recognizer.Reset()
