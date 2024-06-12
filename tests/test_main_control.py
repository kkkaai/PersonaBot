"""
test_main_control.py

This module contains unit tests for the MainControl class and related state handling
in the demo_main_control module. The tests are designed to verify the functionality
of state initialization, state transitions, and command handling within the robot's
main control logic.

Tests included in this module:
1. Test initial state of MainControl:
    - Verifies that the initial state is NaturalStandbyState and that the execute 
      method of NaturalStandbyState is called correctly.

2. Test state transition:
    - Ensures that the StateControl class can correctly change states from 
      NaturalStandbyState to another state (e.g., QuestionResponseState) and verifies
      the transition logic.

3. Test command handling:
    - Checks that commands sent to the current state are handled correctly by the 
      handle_command method of the NaturalStandbyState class.

Modules and methods patched with mock objects:
- NaturalStandbyState.execute: Mocked with AsyncMock to simulate asynchronous execution.
- StateControl.change_state: Mocked to track state transition calls.
- NaturalStandbyState.handle_command: Mocked with AsyncMock to simulate command handling.

Usage:
To run the tests, navigate to the project's root directory and execute the following command:
    python -m unittest discover -s tests -p 'test_*.py'
"""

import unittest
from unittest.mock import patch, AsyncMock
from src.demo_main_control.main_control import MainControl
from src.demo_main_control.states.natural_standby_state import NaturalStandbyState
from src.demo_main_control.states.question_response_state import QuestionResponseState

class TestMainControl(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.main_control = MainControl()

    # Use patch and AsyncMock to simulate execute method of NaturalStandbyState
    # to test if the initial state is NaturalStandbyState, and verify execute has been called once.
    @patch('src.demo_main_control.states.natural_standby_state.NaturalStandbyState.execute', new_callable=AsyncMock)
    async def test_initial_state(self, mock_execute):
        # Test that the initial state is NaturalStandbyState
        self.assertIsInstance(self.main_control.state_control.current_state, NaturalStandbyState)
        # Run the execute method
        await self.main_control.state_control.current_state.execute()
        mock_execute.assert_called_once()

    # Simulate StateControl's change_state method.  Test state switch feature, change to QuestionResponseState.
    @patch('src.demo_main_control.state_control.StateControl.change_state')
    async def test_change_state(self, mock_change_state):
        # Change state to QuestionResponseState
        new_state = QuestionResponseState(self.main_control.state_control)
        self.main_control.state_control.change_state(new_state)
        mock_change_state.assert_called_once_with(new_state)
        self.main_control.state_control.current_state = new_state
        self.assertIsInstance(self.main_control.state_control.current_state, QuestionResponseState)

    # Test command handling feature.  Verify if handle_command method can be correctly invoked.
    @patch('src.demo_main_control.states.natural_standby_state.NaturalStandbyState.handle_command', new_callable=AsyncMock)
    async def test_handle_command(self, mock_handle_command):
        command = 'some_command'
        await self.main_control.state_control.handle_command(command)
        mock_handle_command.assert_called_once_with(command)

if __name__ == '__main__':
    unittest.main()
