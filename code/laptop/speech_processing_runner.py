# speech_processing_runner.py (background listener version)

import os
import time
import requests
import speech_recognition as sr
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

from speech_processing import (
    SAFETY_PREFIX,
    get_steering_values_from_text,
)

# Send control command to car
def send_to_car(angle: float, throttle: float, timeout: float):
    url = os.getenv("DONKEYCAR_ENDPOINT", 'http://ucsdrobocar-148-13.local:8887/drive')
    if not url:
        print("❌ DONKEYCAR_ENDPOINT is not set.")
        return

    json_data = {
        "angle": angle,
        "throttle": throttle,
    }

    print(f"\n▶︎ Sending command → angle: {angle:.3f}, throttle: {throttle:.3f}")
    print(f"🔗 POST to: {url}")

    try:
        response = requests.post(url, json=json_data, timeout=3)
        if response.status_code == 200:
            print("✅ Command sent successfully!\n")
        else:
            print(f"❌ Error: status code {response.status_code}\nResponse: {response.text}")
    except Exception as e:
        print(f"⚠️ Failed to send command: {e}")

# Callback function: triggered when speech is detected
def callback(recognizer, audio):
    print("🎤 Detected speech, processing...")

    try:
        text = recognizer.recognize_google(audio).lower()
        print(f"✅ Recognized text: \"{text}\"")

        if SAFETY_PREFIX and not text.startswith(SAFETY_PREFIX):
            print(f"⚠️ Missing safety prefix \"{SAFETY_PREFIX}\". Skipping.")
            return

        angle, throttle, timeout = get_steering_values_from_text(
            text,
            current_angle=0.0,
            current_throttle=0.2,
            current_timeout=10.0
        )

        send_to_car(angle, throttle, timeout)

    except sr.UnknownValueError:
        print("🤔 Could not understand audio.")
    except Exception as e:
        print(f"❌ Error during recognition or control: {e}")

# Main loop (sets up background listener)
def main():
    recognizer = sr.Recognizer()
    mic = sr.Microphone()

    with mic as source:
        recognizer.adjust_for_ambient_noise(source)
        print("🎧 Background listening started. Say a command. (Ctrl+C to exit)")

    stop_listening = recognizer.listen_in_background(mic, callback)

    try:
        while True:
            time.sleep(0.1)  # Keep the main thread alive
    except KeyboardInterrupt:
        print("\n👋 Exiting program.")
        stop_listening(wait_for_stop=False)

if __name__ == "__main__":
    main()
