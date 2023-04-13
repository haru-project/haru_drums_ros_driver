import pyaudio
import wave
import rospkg
import os
import yaml

package_path = rospkg.RosPack().get_path("haru_drums_ros_driver")

sounds_folder = os.path.join(package_path, "src", "sounds", "quetevotechapote")
config_path = os.path.join(package_path, "config")
settings_path = os.path.join(config_path, "drum_settings.yaml")

with open(settings_path, "r") as file:
    settings = yaml.safe_load(file)

num_to_color_dict = settings["number_color"]

rutas = []
for sound in os.listdir(sounds_folder):
    rutas.append(os.path.join(sounds_folder, sound))

# Ruta al archivo de audio
sounds = dict(zip(num_to_color_dict.keys(), rutas))

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
