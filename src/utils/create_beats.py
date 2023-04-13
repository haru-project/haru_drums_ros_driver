import rtmidi
import mido
import subprocess
import time
import rospy
import signal

class BeatsCreator():

    def __init__(self):
        # Create a MidiIn object
        self.midi_in = rtmidi.MidiIn()
        # Crear el objeto MidiFile
        self.midi_file = mido.MidiFile()
        # Agregar una pista al objeto MidiFile
        self.track = mido.MidiTrack()
        self.midi_file.tracks.append(self.track)
        # List all available MIDI input ports
        ports = self.midi_in.get_ports()
        print("Available MIDI input ports:")
        for i, port in enumerate(ports):
            print(f"{i}: {port}")
        # Choose a MIDI input port to use
        port_index = None
        available_port = False
        print("Select the Midi port you want to use:")
        while not available_port:
            port_index = int(input())
            if port_index <= len(ports) - 1 and port_index >= 0:
                available_port = True
            else:
                print("This is not an available port. Please try again:")
        self.midi_in.open_port(port_index)
        self.now = time.time()
        self.last_note = 0
        # Set the callback function for the MidiIn object
        self.midi_in.set_callback(self.handle_message)
        signal.signal(signal.SIGINT, self.shutdown)

    # Define a callback function to handle incoming MIDI messages
    def handle_message(self, event, data):
        message, delta_time = event[0], event[1]
        if delta_time > 0.2 or delta_time==0:
            print("event: ", event)
            elapse_time = int((time.time() - self.now) * 1000) 
            self.now = time.time()
            print("elapse time:", elapse_time)
            if message[0] == 153:
                self.track.append(mido.Message("note_on", note=message[1], velocity=50, time=elapse_time, channel=9))
            elif message[0] == 137:
                self.track.append(mido.Message("note_off", note=message[1], velocity=50, time=elapse_time, channel=9))
    
    def shutdown(self, signum, frame):
        # Save the midi file and create the wav file
        self.midi_file.save('midi_file2.mid')
        with open("mi_archivo2.wav", "wb") as f:
            subprocess.call(["timidity", "-Ow", "-o", "-", "midi_file2.mid"], stdout=f)

        # Close the MIDI input stream
        self.midi_in.close_port()
        del self.midi_in
    
if __name__ == '__main__':
    rospy.init_node("midi_drum")
    drum = BeatsCreator()
    rospy.spin()