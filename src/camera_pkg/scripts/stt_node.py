#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
import whisper
import pyttsx3
import threading
import time
from transformers import pipeline
import os

class InteractionNode:
    def __init__(self):
        rospy.init_node('interaction_node', anonymous=True)

        # Whisper 模型
        self.model = whisper.load_model("base")

        # TTS 播报
        self.engine = pyttsx3.init()
        self.engine.setProperty('rate', 160)

        # 本地对话模型（BlenderBot）
        rospy.loginfo("🧠 Loading BlenderBot model...")
        self.chatbot = pipeline("text2text-generation", model="facebook/blenderbot-400M-distill")
        rospy.loginfo("✅ BlenderBot ready.")

        # 状态管理
        self.listening = False
        self.last_detected = False
        self.face_last_seen_time = None
        self.face_timeout_sec = 3  # 连续缺失3秒才算“离开”

        rospy.Subscriber('person_detected', Bool, self.person_callback)
        rospy.loginfo("🤖 InteractionNode is running...")
        rospy.spin()

    def speak(self, text):
        rospy.loginfo(f"🗣️ Speaking: {text}")
        self.engine.say(text)
        self.engine.runAndWait()

    def transcribe_audio(self):
        rospy.loginfo("🎙️ Recording audio...")
        os.system("arecord -D hw:0,1 -f S16_LE -r 16000 -c 2 -d 5 temp.wav")
        os.system("ffmpeg -y -i temp.wav -ac 1 temp_mono.wav")

        rospy.loginfo("📝 Transcribing...")
        result = self.model.transcribe("temp_mono.wav")
        text = result["text"].strip()
        rospy.loginfo("🔤 Recognized: %s", text or "[empty]")
        return text

    def respond(self, user_input):
        try:
            reply = self.chatbot(user_input, max_new_tokens=100)[0]['generated_text']
            return reply
        except Exception as e:
            rospy.logwarn(f"⚠️ Chat error: {e}")
            return "Sorry, I couldn't understand."

    def person_callback(self, msg):
        current_time = time.time()
        if msg.data:
            self.face_last_seen_time = current_time

            if not self.last_detected and not self.listening:
                self.listening = True
                threading.Thread(target=self.start_interaction).start()

            self.last_detected = True
        else:
            if self.last_detected and self.face_last_seen_time and time.time() - self.face_last_seen_time > self.face_timeout_sec:
                rospy.loginfo("👋 Person left. Resetting state.")
                self.listening = False
                self.last_detected = False

    def start_interaction(self):
        self.speak("Hi! What would you like to learn today?")
        time.sleep(3)
        user_input = self.transcribe_audio()
        if user_input:
            reply = self.respond(user_input)
            self.speak(reply)
        else:
            self.speak("I didn't hear anything.")
        self.listening = False

if __name__ == "__main__":
    try:
        InteractionNode()
    except rospy.ROSInterruptException:
        pass
