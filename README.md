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

- `GPSR_nlu_node.launch` - Launches the NLU for the [old GPSR task](https://github.com/kyordhel/GPSRCmdGen)
- `EGPSR_nlu_node.launch` - Launches the NLU for the [old EGPSR task](https://github.com/kyordhel/GPSRCmdGen)
- `eurobin_coopetition_nlu_node.launch` - Launches the NLU for the [euROBIN Coopetition](https://github.com/IRS-group/euRobinCoopetitionCmdGenerator).

## Launch file arguments
- `transcript_topic` - Topic for command transcripts. Should publish `socrob_speech_msgs/ASRHypothesis` message type.
- `result_topic` - Topic for publishing extracted goals. Uses `socrob_planning_msgs/nlu_msg` message type.
- `use_keyword_nlu` - Enables experimental keyword-based NLU. Attempts to infer goals by identifying verbs, objects, and people references if the standard method fails. Recommended for simple use cases only.
- `load_from_xml` - Loads NLU rules from an XML file if set to `true`. Should remain `false` unless using the socrob team's private API.
- `asr_error_source` - ROS parameter for common ASR errors. Helps correct common speech recognition errors. Should contain mappings of correct spellings to common ASR misspellings. Example:
  ```yaml
  pear: [bear]
  crisps: [crisp]
  pringles: [pringle]
  tictac: [tic tac]
  ```
  If using the [tiago speech recognition module](https://github.com/socrob/tiago_speech_recognition), this parameter will be published to the server when launched.
