#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
import pyttsx3
import whisper
import sounddevice as sd
import numpy as np
import scipy.io.wavfile

class InteractionNode:
    def __init__(self):
        rospy.init_node('interaction_node', anonymous=True)

        self.prev_state = False  # 上一次是否有人
        self.has_interacted = False  # 是否已播报和识别
        self.tts_engine = pyttsx3.init()
        self.tts_engine.setProperty('rate', 150)
        self.model = whisper.load_model("base")
        self.samplerate = 16000
        self.duration = 5

        rospy.Subscriber('/person_detected', Bool, self.callback)
        rospy.loginfo("🤖 InteractionNode is ready.")
        rospy.spin()

    def callback(self, msg):
        current_state = msg.data

        if current_state and not self.prev_state:
            # ✅ 新出现的人，开始交互
            rospy.loginfo("👤 New person detected. Speaking + Listening...")

            # 播报
            self.tts_engine.say("Hi! What would you like to learn today?")
            self.tts_engine.runAndWait()

            # 录音识别
            self.listen_and_transcribe()

            self.has_interacted = True

        elif not current_state:
            # ✅ 人离开，重置状态
            self.has_interacted = False

        self.prev_state = current_state

    def listen_and_transcribe(self):
        rospy.loginfo("🎧 Recording 5 seconds... Please speak.")
        audio = sd.rec(int(self.samplerate * self.duration), samplerate=self.samplerate, channels=1, dtype='int16')
        sd.wait()
        scipy.io.wavfile.write("temp.wav", self.samplerate, audio)

        result = self.model.transcribe("temp.wav", language=None)
        lang = result.get("language", "")
        text = result["text"].strip()

        if lang == "en" and text:
            rospy.loginfo(f"📝 You said (EN): {text}")
        else:
            rospy.logwarn(f"⛔ Ignored non-English or empty input (Detected: {lang})")

if __name__ == '__main__':
    try:
        InteractionNode()
    except rospy.ROSInterruptException:
        pass
