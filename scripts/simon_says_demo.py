"""
Explicar como hacer el dialogo
"""
import rospy
from haru_drums_ros_driver.srv import SimonSaysRequest, SimonSays, SimonSaysResponse
from idmind_tabletop_msgs.srv import TTSServiceRequest, TTSService
import random

positive_phrases = [
    "Way to go, champ! You're nailing this game.",
    "Excellent job! You're doing everything right.",
    "Bravo! You're a fantastic player.",
    "Incredible! You're quickly catching on to how to play.",
    "Fantastic! I'm impressed by your skills.",
    "You're a genius at this! You can't go wrong.",
    "Wonderful! You're following the instructions perfectly.",
    "You're amazing! I'm impressed by your abilities.",
    "Spectacular! You're crushing it in this game.",
    "You're on fire! Nothing can stop you."
]

encouraging_phrases = [
    "Don't worry, we all make mistakes. Keep trying!",
    "Oops, not this time. But I'm sure you'll do better on the next try.",
    "Don't be discouraged! I'm confident you'll get it next time.",
    "It's okay! Keep going and show me what you're made of.",
    "Ooh, you were so close. But don't worry, you have more chances.",
    "Failing is part of learning. Keep trying, and you'll improve!",
    "Don't give up. I'm sure you can do it. Give it another shot!",
    "Sometimes things don't go as planned, but don't let that discourage you.",
    "Come on, you've got this! Keep trying, and you'll achieve success.",
    "Remember, the most important thing is to have fun. Keep playing and enjoying the game!"
]

def simon_says_round():
    rospy.wait_for_service('/simon_says_service')
    srv_proxy = rospy.ServiceProxy('/simon_says_service', SimonSays)
    request = SimonSaysRequest()
    try:
        response: SimonSaysResponse = srv_proxy(request)

        return response.success, response.score
    except rospy.ServiceException as e:
        rospy.logerr("Error al llamar al servicio: %s" % str(e))

def speak(text):
    rospy.wait_for_service('/idmind_tabletop/cmd_tts_blocking_srv')
    srv_proxy = rospy.ServiceProxy('/idmind_tabletop/cmd_tts_blocking_srv', TTSService)
    request = TTSServiceRequest()
    request.message = text
    try:
        response = srv_proxy(request)
        print(response)
    except rospy.ServiceException as e:
        rospy.logerr("Error al llamar al servicio: %s" % str(e))


if __name__ == '__main__':
    speak("Hello there! I'm Haru ")
    # Some initial explanations
    speak("I will be changing my eye's color to help you!")
    speak("Try it by hitting the drums!")
    rospy.sleep(3)
    # Simon says gameplay block
    speak("Alright, let's start to play")
    success = True

    while success:
        success, score = simon_says_round()
        # if score is 10, then congratulate the player
        if score == 10:
            speak(random.choice(positive_phrases))
        # if score is higher than five, then let him try again
        elif 5 < score < 10:
            speak(random.choice(encouraging_phrases))
        # If score is less than five, then that's a failure

    speak("Well... that's a game over buddy")

