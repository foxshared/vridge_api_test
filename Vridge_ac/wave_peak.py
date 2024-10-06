import pyaudiowpatch as pyaudio
import numpy as np

maxValue = 2**16
bars = 35
CHUNK_SIZE = 512
p = pyaudio.PyAudio()
# stream=p.open(format=pyaudio.paInt16,channels=2,rate=44100,
#               input=True, frames_per_buffer=1024)


try:
    # Get default WASAPI info
    wasapi_info = p.get_host_api_info_by_type(pyaudio.paWASAPI)
except OSError:
    print("Looks like WASAPI is not available on the system. Exiting...")

    exit()

# Get default WASAPI speakers
default_speakers = p.get_device_info_by_index(
    wasapi_info["defaultOutputDevice"])

if not default_speakers["isLoopbackDevice"]:
    for loopback in p.get_loopback_device_info_generator():
        """
        Try to find loopback device with same name(and [Loopback suffix]).
        Unfortunately, this is the most adequate way at the moment.
        """
        if default_speakers["name"] in loopback["name"]:
            default_speakers = loopback
            break
    else:
        print("Default loopback output device not found.\n\nRun `python -m pyaudiowpatch` to check available devices.\nExiting...\n")
        exit()
stream = p.open(format=pyaudio.paInt16,
                channels=default_speakers["maxInputChannels"],
                rate=int(default_speakers["defaultSampleRate"]),
                frames_per_buffer=CHUNK_SIZE,
                input=True,
                input_device_index=default_speakers["index"])


while True:
    data = np.fromstring(stream.read(1024), dtype=np.int16)
    dataL = data[0::2]
    dataR = data[1::2]
    peakL = np.abs(np.max(dataL)-np.min(dataL))/maxValue
    peakR = np.abs(np.max(dataR)-np.min(dataR))/maxValue
    lString = "#"*int(peakL*bars)+"-"*int(bars-peakL*bars)
    rString = "#"*int(peakR*bars)+"-"*int(bars-peakR*bars)
    print("L=[%s]\tR=[%s]" % (lString, rString))
