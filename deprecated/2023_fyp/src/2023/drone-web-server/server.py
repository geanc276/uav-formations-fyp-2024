from flask import Flask

app = Flask(__name__)


@app.route("/isDrone")
def is_drone():
    return "I am a drone"
