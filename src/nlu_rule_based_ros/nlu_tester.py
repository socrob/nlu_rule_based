def import_intruction_list(path):
    instructions = []

    example_next = False

    with open(path, "r") as f:
        for example in f:
            # Examples are the first non empty line after a comment
            if "#" in example:
                example_next = True
            
            elif example_next and len(example) > 1:
                instructions.append(example)

                example_next = False

    return instructions


if __name__ == "__main__":
    from gpsr_speech_processing.speech_command_division import CommandDivision
    import json
    from rule_based_nlu import RuleBasedNLU
    import rospkg
    import rospy
    from os import listdir
    from sys import argv

    if len(argv) >= 2:
        test_type = argv[1]
    else:
        test_type = "EGPSR"

    rospy.init_node("nlu_tester", 
                    # log_level=rospy.DEBUG)
                    log_level=rospy.WARN)

    # get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()

    # get the file path for rospy_tutorials
    grammar_path = rospack.get_path('nlu_rule_based') + f'/resources/{test_type}/'
    common_path = rospack.get_path('nlu_rule_based') + '/resources/common/'
    
    # Get paths of grammar files
    file_path = [
        grammar_path + filename 
        for filename in listdir(grammar_path) 
        if filename.endswith(".txt") and filename != "Extra Verbs.txt"
        ]

    nlu = RuleBasedNLU()

    nlu.read_grammar_files(file_path, 
                           common_path + "Locations.xml", 
                           common_path + "Names.xml", 
                           common_path + "Objects.xml", 
                           common_path + "Gestures.xml",
                           grammar_path + "Extra Verbs.txt",
                           load_from_xml=test_type != "new_GPSR")
    nlu.clean_rules()

    with open(file_path[0].split(".")[0]+".json", "w") as f:
        json.dump(nlu.grammar_rules, f, indent=2)
    
    if test_type == "GPSR":
        nlu.get_generation_rules("main")
    else:
        nlu.get_generation_rules("task")
    
    with open(file_path[0].split(".")[0]+"_regex.json", "w") as f:
        json.dump(nlu.regex_patterns, f, indent=2)

    with open(file_path[0].split(".")[0]+"_patterns.json", "w") as f:
        json.dump(nlu.patterns, f, indent=2)

    nlu.load_tasks(grammar_path + "tasks.json")
    
    if len(argv) < 3:
        examples_path = f"{test_type} Examples.txt"

    else:
        examples_path = argv[2]

    result_segments = []
    result_tasks = []
    command_division = CommandDivision()

    # command_division.readRules(file_path)
    # command_division.verbs += nlu.extra_verbs
    # command_division.verbs.sort()

    instruction_list = import_intruction_list(examples_path)

    for example in instruction_list:
        # Test to see how segments are matched
        segments = command_division.split_by_verbs(example.lower().replace(",", ""), reattach_politeness = False, keep_thens = True)
        result_segments.append({
            "sentence": example,
            "results": [nlu.match_pattern(segment) for segment in segments]
            })
        
        # Test to see if tasks are correctly extracted
        result_tasks.append({
            "sentence": example,
            "results": nlu.process_sentence(example)
            })

    with open(examples_path.split(".")[0]+"_segments.json", "w") as f:
        json.dump(result_segments, f, indent=2)

    with open(examples_path.split(".")[0]+"_tasks.txt", "w") as f:
        for task in result_tasks:
            f.write("\n" + task["sentence"])
            for result in task["results"]:
                f.write(str(result))
                f.write("\n----\n")

    # Generate a JSON
    from json import dump
    result_json = []
    for task in result_tasks:
        result_json.append({
            "instruction": task["sentence"],
            "dialogue_acts": [{
                "verb": result.verb,
                "source": result.source,
                "destination": result.destination,
                "person1": result.person1,
                "person2": result.person2,
                "object1": result.object1,
                "object2": result.object2,
                "whattosay": result.whattosay,
            } for result in task["results"]],
        })

    with open(examples_path.split(".")[0]+"_tasks.json", "w") as f:
        dump(result_json, f, indent=2)