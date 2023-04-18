import pyaudio
import wave
import rospkg
import os
import yaml
from yaml.loader import SafeLoader

package_path = rospkg.RosPack().get_path("haru_drums_ros_driver")
sounds_config_file = os.path.join(package_path, "config", "sound_sets_config", "sound_set_1.yaml")
drum_settings_path = os.path.join(package_path, "config", "drum_settings.yaml")

with open(drum_settings_path, "r") as file:
    settings = yaml.safe_load(file)

num_to_color_dict = settings["number_color"]
color_to_num_dict = settings["color_number"]
rutas = []
color_keys = []

# Ruta al archivo de audio
sounds = dict(zip(color_keys, rutas))


def play_sound(number):
    path_to_sound = sounds[number]
    with wave.open(path_to_sound, 'rb') as archivo_wav:
        datos = archivo_wav.readframes(archivo_wav.getnframes())

        # Inicializar pyaudio y crear un objeto de stream
        audio = pyaudio.PyAudio()
        stream = audio.open(format=audio.get_format_from_width(archivo_wav.getsampwidth()),
                            channels=archivo_wav.getnchannels(),
                            rate=archivo_wav.getframerate(),
                            output=True)

    # Reproducir el audio
    stream.write(datos)

    # Detener el stream y liberar los recursos
    stream.stop_stream()
    stream.close()
    audio.terminate()
