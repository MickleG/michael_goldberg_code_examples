This is a digital safe I made using a rotary encoder.
It utilizes a finite state machine to define the steps needed to unlock the safe.
The diagram of this FSM can be seen in FSM_Diagram.PNG.
The safe operation start by the user pressing the encoder button.
They then must twist the encoder exactly 4 detects CW, followed by 7 detects CCW, and finally 6 detents CW to unlock the safe.
If any deviation of this pattern occurs, then the user is reset back to the starting state, state 0.
Each sequential stage of this digital safe is its own state, and a Moore state machine was used.
The inputs and outputs of the FSM can be seen in the PNG as well.
Enjoy!