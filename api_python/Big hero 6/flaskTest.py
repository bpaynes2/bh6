from flask import Flask, render_template, request
from bigherotest3 import main
app = Flask(__name__)

@app.route('/')
def hello_world():
    return render_template('index.html')


@app.route('/run', methods=['POST'])
def run_demo():
    return main()

if __name__ == '__main__':
    app.run()