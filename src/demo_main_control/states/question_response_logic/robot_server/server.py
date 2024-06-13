# import sys
# sys.path.append("E:\moss_project\speech_demo\robot_control")

from listen_utils import get
from display_utils import request_tts


from flask import Flask, request

app = Flask(__name__)

@app.route("/listen")
def listen_one_question():
    r = get()
    # import pdb; pdb.set_trace()
    print(type(r))
    return r

@app.route("/display",  methods=['POST'])
def display():
    text = request.form['text']
    status = request_tts(text)
    return status