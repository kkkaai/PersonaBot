from .arm_controller_impl import ArmControllerImpl

class ArmController(ArmControllerImpl):

    def __init__(self):
        super().__init__()
        
        self.load_preset_positions()
        

    def load_preset_positions(self):
        # Load the preset positions
        self.preset_paths = {
            "generated_action_1": "data/arm_motions/wave_hands_kp.csv",
            "generated_action_2": "data/arm_motions/shake_hands_kp.csv",
            "generated_action_3": "data/arm_motions/goodbye.csv",
            "generated_action_4": "data/arm_motions/standby.csv",
        }

    def handle_command(self, command):
        print(f"Handling command in ArmController: {command}")
        if command == "generated_action_1":
            self.perform_sequence_from_file(self.preset_paths[command])

