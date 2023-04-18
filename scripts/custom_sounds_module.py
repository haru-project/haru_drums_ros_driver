import pyaudio
import wave
import rospkg
import os
import yaml
from yaml.loader import SafeLoader

package_path = rospkg.RosPack().get_path("haru_drums_ros_driver")

sounds_folder = os.path.join(package_path, "src", "../src/sounds", "sound_set_1")
sounds_config_file = os.path.join(sounds_folder, "sound_set_1.yaml")
config_path = os.path.join(package_path, "config")
settings_path = os.path.join(config_path, "drum_settings.yaml")
try:
    with open(settings_path, "r") as file:
        settings = yaml.safe_load(file)
    num_to_color_dict = settings["number_color"]
    color_to_num_dict = settings["color_number"]
    rutas = []
    color_keys = []
    with open(sounds_config_file) as sounds_file:
        sounds_file_data = yaml.load(sounds_file, Loader=SafeLoader)
    for key in sounds_file_data:
        color_keys.append(color_to_num_dict[key])
        rutas.append(sounds_folder + "/" + sounds_file_data[key])
    # Ruta al archivo de audio
    sounds = dict(zip(color_keys, rutas))
    print("hols")
except:
    print("Not enough custom sounds")

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
