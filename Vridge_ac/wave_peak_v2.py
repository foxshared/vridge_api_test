import pyaudiowpatch as pyaudio
import numpy as np

from time import sleep

from dualsense_controller import DualSenseController


maxValue = 2**16
bars = 35
CHUNK_SIZE = 512
p = pyaudio.PyAudio()
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


# ================================================================
# list availabe devices and throw exception when tzhere is no device detected
device_infos = DualSenseController.enumerate_devices()
if len(device_infos) < 1:
    raise Exception('No DualSense Controller available.')

# create an instance, use fiŕst available device
controller = DualSenseController()

controller.activate()


# ================================================================
while True:
    data = np.fromstring(stream.read(1024),dtype=np.int16)
    dataL = data[0::2]
    dataR = data[1::2]
    peakL = np.abs(np.max(dataL)-np.min(dataL))/maxValue
    peakR = np.abs(np.max(dataR)-np.min(dataR))/maxValue
    print("L:%00.02f R:%00.02f"%(peakL*100, peakR*100))

    controller.left_rumble.set(peakL)
    controller.right_rumble.set(peakR)


# disable/disconnect controller device
controller.deactivate()