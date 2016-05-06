from flask import Flask, request, send_from_directory, url_for
import os.path
import rospkg


rp = rospkg.RosPack()
app = Flask(__name__, static_folder=os.path.join(rp.get_path('web_dashboard'), 'static'))


@app.route('/')
def hello_world():
	return app.send_static_file('index.html')

def init():
	app.run(host='0.0.0.0')

