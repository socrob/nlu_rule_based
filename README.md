# NLU Rule Based

## Overview

The rule-based Natural Language Understanding (NLU) module is designed to interpret spoken commands by leveraging a fixed set of templates used in RoboCup competitions. By reverse engineering the RoboCup command generator, this module can convert these templates into Regular Expressions (REGEX) patterns. The NLU matches a command to one of its predefined patterns and extracts key information, such as the action verb, objects, and locations mentioned in the instruction.

## Features
- **Template-Based Command Recognition**: Works by matching commands against a fixed set of templates from a pre-built database, ensuring accuracy within the supported set of commands.
- **Action Verb and Entity Extraction**: Identifies the key elements in a command, such as action verbs (e.g., "find," "pick up") and entities (e.g., objects, locations), using REGEX patterns.
- **Structured Command Representation**: Transforms recognized commands into a structured format that can be passed to the planner module for further processing.
- **Limited to Predefined Commands**: The module can only process commands that match existing templates, making it ideal for environments where instructions follow strict patterns but unsuitable for dynamic or novel commands.

## Requirements

- ROS version: Noetic
- Dependencies:
  - [socrob_speech_msgs](https://github.com/socrob/socrob_speech_msgs) 
  - [socrob_planning_msgs](https://github.com/socrob/socrob_planning_msgs)
  - [nltk](https://www.nltk.org)

## Installation

### 0. Install the message modules
Follow the installation instructions in the [socrob_speech_msgs](https://github.com/socrob/socrob_speech_msgs) and [socrob_planning_msgs](https://github.com/socrob/socrob_planning_msgs) repositories.

### 1. Clone the repository
```bash
cd ~/<your_workspace>/src
git clone https://github.com/socrob/nlu_rule_based.git
```

### 2. Install python dependencies
Ensure that all required dependencies are installed by running:

```bash
cd nlu_rule_based
pip install -r requirements.txt
```

### 3. Build the workspace
Navigate to your catkin workspace and build the package:

```bash
cd ~/<your_workspace>
catkin_make
```

### 4. Source the setup file
After building, source the workspace to update the environment:
```bash
source ~/<your_workspace>/devel/setup.bash
```

# Usage
## Launching the Node
To launch the node, use the following command:

```bash
roslaunch nlu_rule_based new_GPSR_nlu_node.launch
```

This will launch the NLU for the current [RoboCup@Home GPSR Command Generator](https://github.com/johaq/CommandGenerator). Other launch files are available:

- `launch/GPSR_nlu_node.launch` - Launches the NLU for the [old GPSR task](https://github.com/kyordhel/GPSRCmdGen)
- `launch/EGPSR_nlu_node.launch` - Launches the NLU for the [old EGPSR task](https://github.com/kyordhel/GPSRCmdGen)
- NLU for the euROBIN Coopetition coming soon.

## Launch file arguments
- `transcript_topic` - The topic that the nlu node will subscribe for command transcripts. The topic should publish using the `socrob_speech_msgs/ASRHypothesis` message type.
- `result_topic` - The topic where the nlu node will publish the goals extracted from the command using the `socrob_planning_msgs/nlu_msg` message type.
- `use_keyword_nlu` - Enables a experimental keyword based NLU. This option will try to, in case of failure of the standard method, try to find verbs, objects and people references to use to infer the goal. Only should be turned on in very simple use cases.
