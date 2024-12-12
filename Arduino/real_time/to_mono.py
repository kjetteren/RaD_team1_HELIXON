from pydub import AudioSegment
filename = "DontYouKnowHowBusyAndImportantIAm"
# Load the stereo audio file
audio = AudioSegment.from_file(filename+".mp3")

# Convert to mono
mono_audio = audio.set_channels(1)

# Save the mono audio to a file
mono_audio.export(filename+"_mono.wav", format="wav")