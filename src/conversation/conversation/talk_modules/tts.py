from gtts import gTTS 
from pydub import AudioSegment


def make_wav(text:str, output_file_path='temp.wav', slow=False) -> int:
    gTTS(text=text, lang='en', slow=slow).save('temp.mp3')
    sound = AudioSegment.from_mp3('temp.mp3')
    sound.export(output_file_path, format='wav')
    return sound.duration_seconds
