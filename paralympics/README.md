Paralympics
===========

State machine to run the mini-bot-olympics for MASLAB.

# Using SensorState

The main building block for sensor feedback based states. Look at square.py for inspiration. The function loop() is called once per message. Additionally, a msg_in and msg_out input_key and output_key are automatically generated and used to pass messages from one state to the next without dropping messages.

The reason for this is that during testing for wallfollowing, certain times states would switch once per message. The naive way of dealing with messages (i.e. one state call per message) could possibly drop messages under situations like these. Using SensorState should avoid these issues.

To move on to the next state, simply return the outcome from the loop() function. Otherwise, don't return anything. Transitions and userdata work as normal.
