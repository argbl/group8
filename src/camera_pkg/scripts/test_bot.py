#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
import pyttsx3
import whisper
import sounddevice as sd
import scipy.io.wavfile as wav
import numpy as np
import random
import time
import threading

class MathGameNode:
    def __init__(self):
        rospy.init_node('math_game_node')
        self.engine = pyttsx3.init()
        self.engine.setProperty('rate', 150)
        self.model = whisper.load_model("base")

        self.person_present = False
        self.in_game = False
        self.awaiting_gesture = False
        self.waiting_for_restart = False
        self.expected_answer = None
        self.speak_lock = threading.Lock()

        rospy.Subscriber('/person_detected', Bool, self.face_callback)
        rospy.Subscriber('/hand_gesture', Bool, self.gesture_callback)

        rospy.loginfo("🤖 Math game node initialized.")
        rospy.spin()

    def speak(self, text):
        with self.speak_lock:
            try:
                self.engine.stop()
                rospy.loginfo(f"🗣️ Speaking: {text}")
                self.engine.say(text)
                self.engine.runAndWait()
                time.sleep(len(text) * 0.06 + 0.5)
                rospy.loginfo("✅ Finished speaking.")
            except Exception as e:
                rospy.logwarn(f"⚠️ Speak failed: {e}")

    def transcribe_audio(self):
        rospy.loginfo("🎧 Recording user response...")
        sample_rate = 16000
        duration = 5
        audio = sd.rec(int(duration * sample_rate), samplerate=sample_rate, channels=1, dtype='int16')
        sd.wait()
        wav.write("temp.wav", sample_rate, audio)
        result = self.model.transcribe("temp.wav", language="en")
        text = result["text"].strip()
        rospy.loginfo(f"📝 Transcribed: {text}")
        return text

    def parse_number(self, text):
        try:
            word_to_num = {
                "zero": 0, "one": 1, "two": 2, "three": 3, "four": 4,
                "five": 5, "six": 6, "seven": 7, "eight": 8, "nine": 9,
                "ten": 10, "eleven": 11, "twelve": 12, "i": 1  # 支持 i = 1
            }
            text = text.lower()
            negative = False

            negative_keywords = ["minus", "negative", "subtract", "subtracted", "less"]
            for neg in negative_keywords:
                if neg in text:
                    negative = True
                    text = text.replace(neg, "").strip()

            for word, num in word_to_num.items():
                if word in text:
                    return -num if negative else num

            for token in text.split():
                if token.isdigit():
                    return -int(token) if negative else int(token)

            return None
        except:
            return None

    def face_callback(self, msg):
        self.person_present = msg.data
        if msg.data and not self.in_game:
            rospy.loginfo("✅ Face detected. Ready to receive gesture.")

    def gesture_callback(self, msg):
        if self.person_present and not self.in_game and msg.data:
            self.in_game = True
            threading.Thread(target=self.start_game).start()
        elif self.awaiting_gesture and not self.in_game:
            if msg.data and not self.waiting_for_restart:
                self.waiting_for_restart = True
                rospy.loginfo("🆗 OK gesture detected. Restarting game.")
                threading.Thread(target=self.start_game_wrapper).start()
            elif not msg.data:
                self.speak("Thanks for playing! See you next time.")
                rospy.signal_shutdown("Game ended.")

    def start_game_wrapper(self):
        self.start_game()
        self.waiting_for_restart = False

    def start_game(self):
        self.awaiting_gesture = False
        self.speak("Let's begin a math challenge!")

        a, b = random.randint(1, 10), random.randint(1, 10)
        op = random.choice(["+", "-"])
        # if a < b:
        #     op = "+"

        question = f"{a} {op} {b}"
        self.expected_answer = eval(question)

        self.speak(f"What is {a} {op} {b}?")
        time.sleep(0.5)

        user_text = self.transcribe_audio()
        answer = self.parse_number(user_text)

        if answer is None:
            self.speak("Sorry, I didn't catch that. Let's try again.")
        elif answer == self.expected_answer:
            self.speak("Correct! Great job!")
        else:
            self.speak(f"Oops! That's wrong. The correct answer was {self.expected_answer}.")

        self.speak("Do you want to try another one? Show OK to continue or open hand to stop.")
        self.awaiting_gesture = True
        self.in_game = False

if __name__ == '__main__':
    try:
        MathGameNode()
    except rospy.ROSInterruptException:
        pass
