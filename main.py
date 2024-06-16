from src.demo_main_control.main_control import MainControl
from 3rdParty.inspire_hand.python.inspire-hand-RH56-demo import *
from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import os
import asyncio



# Create a Web Server
# using Flask instance and specify the template and static folder locations
#app = Flask(__name__, template_folder='web_server/templates', static_folder='web_server/static')
template_dir = os.path.abspath('web_server/templates')
static_dir = os.path.abspath('web_server/static')
print(f"Template directory: {template_dir}")
print(f"Static directory: {static_dir}")
app = Flask(__name__, template_folder=template_dir, static_folder=static_dir)
socketio = SocketIO(app)


# Create Robot Control Logic
main_control = MainControl()




@app.route('/')
def index():
    return render_template('index.html')


# Link Robot Control with Web GUI
@socketio.on('command')
def handle_command(data):
    command = data.get('command')
    button_id = data.get('buttonId')
    print(f"Received command: {command}, from button: {button_id}")

    # Let Robot controller handle the command
    robot_controller_hand_command(command)

    # Let State Machine's current State handle the command
    asyncio.run(main_control.state_control.handle_command(command))
    #socketio.start_background_task(main_control.state_control.handle_command, command)


def robot_controller_hand_command(command):
    # Also let some important command pass-through to robot controllers
    
    # Navigation
    if command == "move_forward":
        main_control.nav_controller.move(0.5, 0.0)
    elif command == "move_backward":
        main_control.nav_controller.move(-0.5, 0.0)
    elif command == "turn_left":
        main_control.nav_controller.move(0.0, 0.5)
    elif command == "turn_right":
        main_control.nav_controller.move(0.0, -0.5)
    elif command == "stop":
        main_control.nav_controller.stop()
    elif command == "move_to_A":
        main_control.nav_controller.move_to_goal('A')
    elif command == "move_to_B":
        main_control.nav_controller.move_to_goal('B')
        
    # Arm Movement
    elif command.startswith("generated_action"):
        main_control.arm_controller.handle_command(command)
        
    # Hand Actions
    elif command == "auto_grasp_on":
        auto_grasp_on()
    elif command == "auto_grasp_off":
        auto_grasp_off()
    elif command == "grab":
        grasp()
    elif command == "release":
        loose()
    elif command == "clear_err"
        clearErr()
        set_pos(0, 0, 0, 0, 0, 0)


if __name__ == '__main__':
    try:
        socketio.start_background_task(main_control.run)
        socketio.run(app, host='0.0.0.0', port=5000)
    except KeyboardInterrupt:
        print("Shutting down gracefully...")
    finally:
        main_control.shutdown()
