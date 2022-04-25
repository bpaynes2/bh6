from flask import Flask, render_template, request, Response
from bigherotest3 import main
from robotcontroltest import main as robocontrolo
from highsea import main as hic
from ice import main as ice
from dots import main as dots
import time
from noods import main as noods
#from streamdreams import main as streamy
import cv2
app = Flask(__name__)

cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)

cap.set(cv2.CAP_PROP_FOURCC, 0x32595559)

cap.set(cv2.CAP_PROP_FPS, 25)


def generate_frames():
    while True:
            
        ## read the camera frame
        success,frame=cap.read()
        if not success:
            break
        else:
            ret,buffer=cv2.imencode('.jpg',frame)
            frame=buffer.tobytes()

        yield(b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/')
def hello():
    return render_template('newindex.html')

@app.route('/manualswitch', methods=['POST'])
def hello_again():
    return robocontrolo()

@app.route('/streams', methods=['GET'])
def stream():
    return 1

@app.route('/run', methods=['POST', 'GET'])
def run():
    if request.method == 'POST':
        x = request.form['x']
        y = request.form['y']
        z = request.form['z']
        return main(x,y,z)
    else:
        x = request.form['x']
        y = request.form['y']
        z = request.form['z']
        return main(x,y,z)

@app.route('/box', methods=['POST'])
def box():
    if request.method == 'POST':
        item = request.form['item']
        amount = int(request.form['amount'])
        #zone = int(request.form['zone'])
        for i in range(amount):
            if (item == 'hic'):
                val = hic()
            elif (item == 'noodles'):
                return noods()
            elif (item == 'ice'):
                return ice()
            elif (item == 'water'):
                return water()
            elif (item == 'dots'):
                return dots() 
            #Szone+= i

    return val
            
            
   

@app.route('/video')
def video():
    return Response(generate_frames(),mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run()