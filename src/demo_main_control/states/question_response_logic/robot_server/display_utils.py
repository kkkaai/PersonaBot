import requests
import json
import base64
import pyaudio
from io import BytesIO

TTS_URL = "http://10.192.40.202:30790/paddlespeech/tts/streaming"

def request_tts(text):
     import requests
     import json
     import base64
     import pyaudio
     from io import BytesIO

     s = requests.Session()

     p = pyaudio.PyAudio()
     stream = p.open(format=p.get_format_from_width(2),
                    channels=2,
                    rate=12000,
                    output=True)

     with s.post(TTS_URL, data=json.dumps({"text":text, "spk_id":1}), stream=True) as r:
          idx = 0
          for line in r:
               if line:
                    audio_data = base64.b64decode(line)
                    audio_stream = BytesIO(audio_data)
                    chunk_size = 1024  # 或者你可以选择其他大小的数据块
                    data = audio_stream.read(chunk_size)
                    while data:
                         stream.write(data)
                         data = audio_stream.read(chunk_size)
     stream.stop_stream()
     stream.close()
     p.terminate()

     print("播放完毕。")
     return True