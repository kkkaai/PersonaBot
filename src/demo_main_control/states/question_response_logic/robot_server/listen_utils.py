# -*- encoding: utf-8 -*-
import time
import websockets
import asyncio
import json
from multiprocessing import Process

import time
import logging

all_start_time = time.time()

logging.basicConfig(level=logging.ERROR)

# message_stop = False
messages_queue = asyncio.Queue()


async def record_microphone(exit_flag):
    try:
        import pyaudio
        global websocket # , message_stop
        FORMAT = pyaudio.paInt16
        CHANNELS = 1
        RATE = 16000
        chunk_size = 60 
        CHUNK = int(RATE / 1000 * chunk_size)

        p = pyaudio.PyAudio()

        stream = p.open(format=FORMAT,
                        channels=CHANNELS,
                        rate=RATE,
                        input=True,
                        frames_per_buffer=CHUNK)
        
        message = json.dumps({"mode": "offline", "chunk_size": [5,10,5], "chunk_interval": 10,
                            "wav_name": "microphone", "is_speaking": True, "hotwords":"", "itn": True})
        print(message)
        await websocket.send(message)
        while not exit_flag.is_set():
            # await exit_flag.wait()
            data = stream.read(CHUNK)
            message = data
            await websocket.send(message)
            await asyncio.sleep(0.005)
        print("stop record")

    except Exception as e:
        print(e, flush=True)
          
async def message(exit_flag):
    global websocket, all_start_time, message_stop
    # text_print = ""
    # tmp_end_timestamp = 0

    try:
        # while True:

        meg = await websocket.recv()
        meg = json.loads(meg)
        print(meg['text'])

        await messages_queue.put(meg['text'])
        print(messages_queue.qsize())
        # exit(0)
        
        exit_flag.set()
            # yield meg
        print("stop message")

    except Exception as e:
        print("Exception:", e, flush=True)
        #traceback.print_exc()
        #await websocket.close()
 

async def ws_client():
    try:
        global websocket
        ASR_URL = "10.192.40.202:10095"
        uri = f"ws://{ASR_URL}"
        ssl_context = None
        print("connect to", uri)
        async with websockets.connect(uri, subprotocols=["binary"], ping_interval=None, ssl=ssl_context) as websocket:
            exit_flag = asyncio.Event()
            t1 = asyncio.create_task(record_microphone(exit_flag))
            t2 = asyncio.create_task(message(exit_flag))

            await asyncio.gather(t1, t2)
        # exit(0)
    except Exception as e:
        print(e)

def get():

    global messages_queue
    messages_queue = asyncio.Queue()
    asyncio.run(ws_client())
    while not messages_queue.empty():
        msg = messages_queue.get_nowait()
        return msg
    

if __name__ == '__main__':
    while True:
        get()
