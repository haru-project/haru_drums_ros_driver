import rospkg
import os
import yaml
import sys
from pydub import AudioSegment
from pydub.playback import play

sys.stderr = open("errores_alsa.txt", "w")

package_path = rospkg.RosPack().get_path("haru_drums_ros_driver")
sound_folder = os.path.join(package_path, "src", "sounds", "sound_set_1")
# sounds_config_file = os.path.join(package_path, "config", "sound_sets_config", "sound_set_1.yaml")
drum_settings_path = os.path.join(package_path, "config", "drum_settings.yaml")

with open(drum_settings_path, "r") as file:
    settings = yaml.safe_load(file)

num_to_color_dict = settings["number_color"]
rutas = [os.path.join(sound_folder, sound) for sound in os.listdir(sound_folder)]

# Ruta al archivo de audio
sounds = dict(zip(num_to_color_dict.keys(), rutas))

def play_sound(number):
    mp3_file = sounds[number]
    sound = AudioSegment.from_mp3(mp3_file)
    play(sound)