import re
from xml.dom import minidom
from gpsr_speech_processing.speech_command_division import CommandDivision
import json

import rospy
from nlu_rule_based.msg import nlu_msg
from actions_tiago_ros.tiago_api import TiagoAPI

class RuleBasedNLU():
    # Regex to detect things inside parentheses, brackets or words that start with $
    pattern = r"\{([^{}]+)\}|\$(\w+)+"
    
    def __init__(self):
        self.regex_patterns = {}
        self.repeated_words = {}
        self.tiago_api = TiagoAPI()

    def get_word_options(self, word, nlu_name):
        words = word.split(" ")
        options_list = [word]

        if len(words) > 1:
            options_list.append("".join(words))

        # Define the translation for every option
        for option in options_list:
            # Has a defined nlu name
            if nlu_name != "":
                self.repeated_words[option] = nlu_name

            # Just use the word itself
            else:
                self.repeated_words[option] = word

        return options_list

    def read_grammar_files(self, 
            path_list, 
            locations_path="",
            names_path="",
            objects_path="",
            gestures_path="",
            extra_verbs_path="",
            load_from_xml=True):
        
        self.grammar_rules = {
            # All possible identifiers for locations
            "location": [],

            # Names, questions and gestures
            "name": [],
            "question": ["question"],
            "gesture": [],

            # Object related identifiers
            "object": [],
            "category": [],
            "object where canPourIn=true": [],
            "object where canPour=true": [],
            'object where Category="food" canPour=true': ["{object where canPour=true}"],
            "object special where canPlaceIn=true": [],
            "pron": ["him", "her", "it"],
            "pron obj": ["him", "her", "it"],
            "pron sub": ["he", "she", "it"],
            "pron pos": ["my", "your", "their", "his", "her"],
            "pron posadj": ["my", "your", "their", "his", "her"],
            "pron paj": ["my", "your", "their", "his", "her"],
            "pron pab": ["mine", "yours", "theirs"],
            "pron posabs": ["mine", "yours", "theirs"],
            }
        
        # Import locations, names, objects and gestures from xml files
        if load_from_xml:
            self.read_xml(locations_path, names_path, objects_path, gestures_path)
        else:
            self.read_from_semantic_map()
            
        # Make sure locations are ordered with biggest words first
        self.grammar_rules["location"].sort(key=len, reverse=True)

        # Copy list of location to all location categories
        self.grammar_rules["beacon"] = self.grammar_rules["location"]
        self.grammar_rules["placement"] = self.grammar_rules["location"]
        self.grammar_rules["room"] = self.grammar_rules["location"]
        self.grammar_rules["location?"] = self.grammar_rules["location"]
        self.grammar_rules["placement?"] = self.grammar_rules["location"]
            
        # Make sure names are ordered with biggest words first
        self.grammar_rules["name"].sort(key=len, reverse=True)
            
        # Make sure objects are ordered with biggest words first
        self.grammar_rules["object"].sort(key=len, reverse=True)

        # Copy object list to all object categories
        self.grammar_rules['object where Category="snacks"'] = self.grammar_rules["object"]
        self.grammar_rules["object where fruit=true"] = self.grammar_rules["object"]
        self.grammar_rules['object where Category="drinks"'] = self.grammar_rules["object"]
        self.grammar_rules['object where Category="tableware"'] = self.grammar_rules["object"]
        self.grammar_rules['object where Category="cutlery"'] = self.grammar_rules["object"]
        self.grammar_rules['object where Category="tableware"'] = self.grammar_rules["object"]
        self.grammar_rules["aobject"] = self.grammar_rules["object"]
        self.grammar_rules["kobject"] = self.grammar_rules["object"]
        self.grammar_rules["sobject"] = self.grammar_rules["object"]

        self.grammar_rules["object?"] = self.grammar_rules["category"]
        self.grammar_rules["aobject?"] = self.grammar_rules["category"]
        self.grammar_rules["kobject?"] = self.grammar_rules["category"]
        self.grammar_rules["sobject?"] = self.grammar_rules["category"]
            
        # Make sure categories are ordered with biggest words first
        self.grammar_rules["category"].sort(key=len, reverse=True)
            
        # Make sure gestures are ordered with biggest words first
        self.grammar_rules["gesture"].sort(key=len, reverse=True)

        # Read extra verbs
        self.extra_verbs = []
        try:
            for line in open(extra_verbs_path, "r"):
                self.extra_verbs.append(line.replace("\n", ""))
        except FileNotFoundError:
            rospy.loginfo("No file found with extra verbs.")

        # Read rules from grammar files
        for path in path_list:
            for line in open(path, "r"):
                rospy.logdebug(f"Processing line: {line}")
                # If line starts with "$" something is being defined
                if line[0] == "$":
                    line_split = re.split(r"(\s+|)= ", line[1:].replace("\n", ""))
                    (key, definition) = (line_split[0], line_split[-1])

                    # Split definition by " | "
                    options = [definition]
                    i = 0
                    open_par = 0

                    while i < len(options[-1]):
                        # Found a | and there are not open parentheses, split string
                        if options[-1][i] == "|" and open_par == 0:
                            options.append(options[-1][i+2:])
                            options[-2] = options[-2][:i-1]
                            i = 0
                            continue

                        elif options[-1][i] == "(": open_par += 1

                        elif options[-1][i] == ")": open_par -= 1

                        i += 1

                    rospy.logdebug(f"key: {key} options: {options}")

                    # Add options to dict
                    try:
                        self.grammar_rules[key] += options
                    
                    # Key doesn't exist, create it
                    except KeyError:
                        self.grammar_rules[key] = options

        rospy.logdebug(self.grammar_rules)

        # Start command division class
        self.command_division = CommandDivision()

        # self.command_division.readRules(path_list)
        self.command_division.verbs += self.extra_verbs
        self.command_division.verbs.sort()

    def read_xml(self, locations_path, names_path, objects_path, gestures_path):
        # Import locations
        locations_xml = minidom.parse(locations_path)

        # Read rooms
        for elem in locations_xml.getElementsByTagName("room"):
            # self.grammar_rules["room"].append(elem.getAttribute("name"))

            self.grammar_rules["location"] += self.get_word_options(elem.getAttribute("name"), elem.getAttribute("nlu_name"))

        # Read locations
        for elem in locations_xml.getElementsByTagName("location"):
            self.grammar_rules["location"] += self.get_word_options(elem.getAttribute("name"), elem.getAttribute("nlu_name"))

        # Import names
        names_xml = minidom.parse(names_path)

        # Read names
        for elem in names_xml.getElementsByTagName("name"):
            self.grammar_rules["name"].append(elem.firstChild.data.lower())

        # Import objects
        objects_xml = minidom.parse(objects_path)

        # Read objects
        for elem in objects_xml.getElementsByTagName("object"):
            self.grammar_rules["object"] += self.get_word_options(elem.getAttribute("name"), elem.getAttribute("nlu_name"))

            if elem.getAttribute("canPourIn") == "true":
                self.grammar_rules["object where canPourIn=true"] += self.get_word_options(elem.getAttribute("name"), elem.getAttribute("nlu_name"))
                  
            if elem.getAttribute("canPour") == "true":
                self.grammar_rules["object where canPour=true"] += self.get_word_options(elem.getAttribute("name"), elem.getAttribute("nlu_name"))

            if elem.getAttribute("canPlaceIn") == "true" and elem.getAttribute("type") == "special":
                self.grammar_rules["object special where canPlaceIn=true"] += self.get_word_options(elem.getAttribute("name"), elem.getAttribute("nlu_name"))
                
        # Read categories
        for elem in objects_xml.getElementsByTagName("category"):
            self.grammar_rules["category"].append(elem.getAttribute("name"))

        # Import gestures
        gestures_xml = minidom.parse(gestures_path)

        # Read gestures
        for elem in gestures_xml.getElementsByTagName("gesture"):
            self.grammar_rules["gesture"].append(elem.getAttribute("name"))

    def read_from_semantic_map(self):
        semantic_map = self.tiago_api.semantic_map.get_semantic_map()
        self.grammar_rules["location"] = self.tiago_api.semantic_map.get_available_location_names()
        self.grammar_rules["name"] = semantic_map.names
        self.grammar_rules["object"] = [obj["semantic_name"] for obj in semantic_map.object_classes.values()]
        self.grammar_rules["category"] = [cat["plural_name"] for cat in semantic_map.object_categories.values()]
        self.grammar_rules["plurCat"] = [cat["plural_name"] for cat in semantic_map.object_categories.values()]
        self.grammar_rules["singCat"] = [cat["singular_name"] for cat in semantic_map.object_categories.values()]

        # Also add possible ASR errors
        for error, nlu_name in self.tiago_api.speech.get_common_errors(
            categories=["names"]):
            self.grammar_rules["name"] += self.get_word_options(error, nlu_name)

        for error, nlu_name in self.tiago_api.speech.get_common_errors(
            categories=self.grammar_rules["category"]):
            self.grammar_rules["object"] += self.get_word_options(error, nlu_name)

        # Sometimes objects are categories
        self.grammar_rules["object"] += self.grammar_rules["singCat"]

    def clean_rules(self):
        """Makes a basic cleanup of the rules, removing useless information and trying to replace some references"""

        # Remove useless information from grammar rules
        for task in self.grammar_rules:
            for _ in range(len(self.grammar_rules[task])):
                command = self.grammar_rules[task].pop(0)
                new_command = ""

                # meta information is not usefull for the nlu, remove it
                brackets_open = 0
                in_meta = False
                for i in range(len(command)):
                    # Count open and closed brackets
                    if command[i] == "{": brackets_open += 1
                    if command[i] == "}": brackets_open -= 1

                    # Start of meta information
                    if command[i:i+6] == " meta:":
                        in_meta = True

                    # Add command if not in void wildcard
                    if not in_meta:
                        new_command += command[i]

                    # End of void wildcard
                    if in_meta and brackets_open == 0:
                        in_meta = False
                        new_command += "}"
                
                # Void wildcards only serve to add meta information, remove them
                new_command = new_command.replace("{void}", "")

                # If command is empty continue and don't add it back
                if new_command == "": continue
                
                # Check if rule the whole rule is inside parentheses, if so they are redundant so we can remove them
                if new_command[0] == "(" and new_command[-1] == ")":
                    par_open = 1
                    for i in range(1, len(new_command)):
                        # Count open and closed parentheses
                        if command[i] == "(": par_open += 1
                        if command[i] == ")": par_open -= 1

                        # If all parentheses were closed, check if we are at the end of the string
                        if par_open == 0:
                            # We are at the end of the string so the outer parentheses are redundant
                            if i == len(new_command)-1: new_command = new_command[1:-1]
                            
                            else: break

                self.grammar_rules[task].append(new_command)

        # Replace rules made from a conjuction of references (ex: "$place | $deliver")
        for task in self.grammar_rules:
            for _ in range(len(self.grammar_rules[task])):
                command = self.grammar_rules[task].pop(0)

                # Check if command is conjuction of references, if not, add it back to the rules
                if re.sub(r"\$\w+ \| \$\w+", "", command) != "":
                    self.grammar_rules[task].append(command)
                    continue

                # If it is add both the references to the list of commands
                for reference in command.split(" | "):
                    self.grammar_rules[task].append(reference)

        # Replace references to rules made from 1 string (atomic rules) by that string and repeat until nothing changes
        made_changes = True
        while made_changes:
            made_changes = False
            for task in self.grammar_rules:
                for _ in range(len(self.grammar_rules[task])):
                    command = self.grammar_rules[task].pop(0)

                    # Find what rules the command makes reference
                    references = self.find_children(command)

                    # For each reference, see if it's an atomic rule and replace it if that's the case
                    for reference in references:
                        # If rule for reference does not exist remove rule
                        if self.grammar_rules.get(reference) == None:
                            rospy.logwarn(f"Removing rule '{command}' because the reference '{reference}' does not exist")
                            command = ""
                            break

                        # It is an atomic command
                        if len(self.grammar_rules[reference]) == 1:
                            # Replace simple references
                            command = re.sub(
                                r"(\$%s)|(\{%s\?*\})" % (reference, reference), 
                                self.grammar_rules[reference][0], 
                                command
                                )
                            
                            # Replace references with numbers
                            matches = re.findall(
                                r"(\{%s( \d)*\})" % reference, 
                                command
                                )
                            
                            for match in matches:
                                # We assume we are replacing it with a wildcard and put the number inside the wild card
                                command = command.replace(match[0], self.grammar_rules[reference][0][:-1] + match[1] + "}")

                            made_changes = True

                        # It is a empty rule, remove reference
                        elif len(self.grammar_rules[reference]) == 0:
                            command = re.sub(
                                r"(\$%s)|(\{%s( \d)*\})|(\{%s\?\})" % (reference, reference, reference), 
                                "", 
                                command
                                )
                            made_changes = True
                    
                    # Only add back the command if it is not an empty string
                    if command != "":
                        # If command starts with a space remove it
                        if command[0] == " ":
                            command = command[1:]

                        # If command ends with a space remove it
                        if command[-1] == " ":
                            command = command[:-1]
                        
                        # Add command back to the list
                        self.grammar_rules[task].append(command)

        # Replace command composed of only a reference
        made_changes = True
        while made_changes:
            made_changes = False
            for task in self.grammar_rules:
                for _ in range(len(self.grammar_rules[task])):
                    command = self.grammar_rules[task].pop(0)

                    # See if the command only references other list
                    if command[0] == "$" and len(command.split(" ")) == 1:
                        # If so add other list to this one
                        self.grammar_rules[task] += self.grammar_rules[command[1:]]

                        made_changes = True

                    # Check if the command is composed of only a wildcard
                    elif command[0] == "{" and command[-1] == "}" and "}" not in command[1:-1]:
                        reference = re.sub(r"\s\d", "", command[1:-1])

                        # If is not a self reference replace command by list
                        if reference != task:
                            self.grammar_rules[task] += self.grammar_rules[reference]
    
                        made_changes = True

                    # If no change was made put command back
                    else:
                        self.grammar_rules[task].append(command)

        # Sort lists of strings by size
        for rule in self.grammar_rules:
            self.grammar_rules[rule].sort(key=len, reverse=True)

    def find_children(self, command):
        """Returns a list with the children of a command"""
        return [
            re.sub(r"\s\d", "", "".join(child)) 
            for child in re.findall(self.pattern, command)
        ]

    def compile_regex(self, parent, list_to_regex = False):
        """Compiles the regular expression equivalent to a grammar rule"""
        # Check if regex was already compiled
        try:
            self.regex_patterns[parent]
            return

        # Don't compile for the second time
        except KeyError: 
            self.regex_patterns[parent] = []

        # Compile regex for every individual command
        pattern_list = []
        facts_list = []
        for command in self.grammar_rules[parent]:
            # We do this so back appears in the list of facts, so we can differentiate guide back from other kinds of guide
            command = command.replace("back", "(back)")

            # Add things so we can identify the variations of the tell task
            command = command.replace("how many", "(how many)")
            command = command.replace("three", "(three)")

            # Make it accept both "'s" and "is"
            command = command.replace("'s", "('s| is)")

            # Convert command to lower case, and remove extra spaces
            pattern = re.sub(r"\s+", " ", command.replace(" | ", "|").replace("( ", "(").replace(" )", ")")).lower()

            facts = []
            self_reference = False
            children = []

            # Check if command starts with extra verbs
            if parent[:2] != "vb":
                for verb in self.extra_verbs:
                    # Pattern is only the verb
                    if pattern == verb:
                        pattern = pattern.replace(verb, f"({verb})")
                        facts.append("vbextra" + verb)
                        break

                    # Pattern is not only the verb, check for space after verb
                    elif pattern.startswith(verb + " "):
                        pattern = pattern.replace(verb + " ", f"({verb}) ")
                        facts.append("vbextra" + verb)
                        break
            
            # Find facts that come from the rules and facts that come from references
            fact_stack = []
            open_par = []
            for i in range(len(command)):
                # The start of a match group that represents a fact
                if command[i] == "(":
                    open_par.append(i)
                    fact_stack.append(len(facts))
                    facts.append("")

                # The end of a match group that represents a fact, fact name is the text inside parentheses
                elif command[i] == ")":
                    facts[fact_stack.pop(-1)] = command[open_par.pop(-1)+1:i]

                # Start of a reference
                elif command[i] == "$":
                    # Find the end of the reference (an empty space)
                    child = command[i+1:].split(" ")[0].replace(")", "")

                    # Compile the regex for the child
                    self.compile_regex(child, list_to_regex=True)

                    # Add the facts to the list
                    facts += [child] + self.regex_patterns[child]["facts"]

                    # Add child to the list to be replaced later
                    children.append(child)

                # Start of a wildcard
                elif command[i] == "{":
                    # Find the end of the wild card (closed brackets)
                    for j in range(i, len(command)):
                        if command[j] == "}": break

                    child = command[i+1:j]

                    # Wild cards may have other information that is not the reference
                    if child in self.grammar_rules:
                        reference = child

                    else:
                        reference = child.split(" ")[0]

                    if reference != parent:
                        # Compile the regex for the child
                        self.compile_regex(reference, list_to_regex=True)

                        # Add the facts to the list
                        facts += [child] + self.regex_patterns[reference]["facts"]

                        # Add child to the list to be replaced later
                        children.append(reference)

                    # If the rule self references ignore it
                    else:
                        self_reference = True

            if not self_reference:
                # Replace each element
                for child in children:
                    pattern = re.sub(
                        r"(\$%s)|(\{%s( \d)*\})|(\{%s\?\})" % (child.lower(), child.lower(), child.lower()), 
                        self.regex_patterns[child]["pattern"], 
                        pattern
                        )
                    
                # Add pattern to the list
                pattern_list.append(pattern)
                facts_list.append(facts)


        # Compile list to regex
        if list_to_regex:
            # Remove empty strings from list
            try:
                self.regex_patterns[parent].remove("")
            
            except ValueError: pass

            # Only one element
            if len(pattern_list) == 1:
                self.regex_patterns[parent] = {"pattern": pattern_list[0],
                                               "facts": facts_list[0]}

            # A list of elements
            elif len(pattern_list) > 1:
                self.regex_patterns[parent] = {"pattern": "(" + "|".join(pattern_list) + ")",
                                               "facts": sum(facts_list, [])}

            else:
                self.regex_patterns[parent] = {"pattern": "",
                                               "facts": []}

        # Leave it as a list   
        else:
            self.regex_patterns[parent] = [
                {"pattern": pattern,
                 "facts": facts} for (pattern, facts) in zip(pattern_list, facts_list)
                ]

    def get_generation_rules(self, parent):
        """Generate the patterns for the nlu"""
        flag = True
        while flag:
            flag = False
            # Define a set so we don't have repeated rules
            divided_sentences = set()

            # Divide sentences in components
            for sentence in self.grammar_rules[parent]:
                divided_sentence = re.split(r", and |, then |, | and ", sentence.replace(".", ""))

                for sentence in divided_sentence:
                    divided_sentences.add(sentence)

            # Split rules composed two commands
            new_rules = set()
            deleted_rules = set()
            for rule in divided_sentences:
                if rule[0] == "(" and rule[-1] == ")": 
                    deleted_rules.add(rule)
                    open_brackets = 0
                    cmd_start = 1

                    for i in range(1,len(rule)):
                        if rule[i] == "(":
                            open_brackets += 1
                        if rule[i] == ")":
                            open_brackets -= 1

                        # Split rule
                        if rule[i] == "|" and open_brackets == 0:
                            new_rules.add(rule[cmd_start:i-1])
                            cmd_start = i+2

                    # Add last part
                    new_rules.add(rule[cmd_start:-1])

            # Delete required rules
            divided_sentences.difference_update(deleted_rules)

            # Add new rules
            divided_sentences.update(new_rules)

            # Convert back to list
            self.grammar_rules["commands"] = list(divided_sentences)

            self.clean_rules()

            for rule in self.grammar_rules["commands"]:
                if "," in rule or "and" in rule:
                    flag = True
                    parent = "commands"
                    break

        # Compile regex for commands
        self.compile_regex("commands")

        self.patterns = self.regex_patterns["commands"]

        # Sort commands by order of information provided
        self.patterns = sorted(self.patterns, key=lambda cmd: (len(cmd["facts"]), len(cmd["pattern"])), reverse=True)

        # Attribute an id to each pattern for debugging
        for i in range(len(self.patterns)):
            self.patterns[i]["id"] = i

    def match_pattern(self, sentence):
        """Matches a sentence to a pattern from the previously compiled list"""
        possible_matches = []

        for pattern in self.patterns:
            # Find all matches in the input text
            match = re.match(pattern["pattern"], sentence)

            # Extract facts from the match
            if match:
                facts = match.groups()

                match_information = {
                    "sentence": sentence,
                    "facts": {},
                    "pattern": pattern["pattern"],
                    "id": pattern["id"]
                }

                for fact_id, value in zip(pattern["facts"], facts):
                    if value is not None:
                        # Check for repeated words
                        if self.repeated_words.get(value) != None:
                            value = self.repeated_words.get(value)

                        # No repeat
                        if fact_id not in match_information["facts"]:
                            match_information["facts"][fact_id] = value
                        
                        # Repeat fact, add number
                        else:
                            match_information["facts"][fact_id + " 2"] = value

                possible_matches.append(match_information)

        # No match
        if len(possible_matches) == 0:
            return {
                "sentence": sentence,
            }
        
        # Return the match that gives us the most information
        possible_matches.sort(key=lambda x: len(x["facts"]), reverse=True)
        return possible_matches[0]

    def load_tasks(self, path):
        with open(path, "r") as f:
            self.tasks = json.load(f)

    def get_previous_fact(self, fact):
        for i in range(1, len(self.actions)+1):
            if getattr(self.actions[-i], fact) != "":
                return getattr(self.actions[-i], fact)
            
        # Didn't find what it wanted
        return ""
        
    def process_sentence(self, sentence):
        rospy.loginfo(f"Processing sentence: {sentence}")

        # Divide sentence by verbs
        segments = self.command_division.split_by_verbs(sentence.lower().replace(",", ""), reattach_politeness = False, keep_thens = True)

        rospy.logdebug(f"Divided sentence: {segments}")

        # Match patterns
        results = [self.match_pattern(segment) for segment in segments]

        rospy.logdebug(results)

        # Get tasks from results
        self.actions = []
        use_keyword_nlu = rospy.get_param("~use_keyword_nlu", False)

        for result in results:
            rospy.loginfo(f"Processing segment: {result['sentence']}")

            # Check if matching was sucssefull
            facts = result.get("facts")
            if facts != None:
                self.process_match(facts)

            # Failed to match a pattern, try to find a verb in the sentence
            elif use_keyword_nlu:
                self.process_match_keyword_based(result)

            else:
                rospy.logerr("Was not capable to process segment")

        return self.actions
    
    def process_match(self, facts):
        action = nlu_msg()

        # Find a match in the tasks
        for task in self.tasks:
            if task not in facts:
                continue
            
            # If there is not a verb we want to update the last task
            if "verb" not in self.tasks[task]:
                try:
                    action = self.actions.pop(-1)
                except IndexError:
                    rospy.logwarn(f"Could not detect a verb in sentence and there is no previous action.")

            # Get each fact
            for fact in self.tasks[task]:
                # Try every possible source
                for source in self.tasks[task][fact]["source"]:
                    # Simple source, try to extract the fact with the source name as a key
                    if type(source) == str:
                        if facts.get(source) == None:
                            continue
                        
                        setattr(action, fact, facts.get(source))
                        break
                    
                    # Dicts provide specific instructions to get the fact
                    if type(source) == dict:
                        # Check if the specified source is in facts, if not got to next source
                        if facts.get(source["source"]) == None:
                            continue

                        # There is a condition specified
                        if source.get("if") != None:
                            # Specified fact doesn't exist, so we ignore the rule
                            if facts.get(source["if"]) == None:
                                continue
                        
                        # If there is a previous fact specified, try to extract it from previous actions
                        if source.get("prev_fact"):
                            prev_fact = self.get_previous_fact(source["prev_fact"])

                            if prev_fact != "":
                                setattr(action, fact, prev_fact)
                                break

                        # Case statement, check which case it is
                        if source.get("case"):
                            found_match = False
                            for case_option, case_result in source["case"].items():
                                if facts[source["source"]] == case_option:
                                    setattr(action, fact, case_result)
                                    found_match = True
                                    break
                            
                            # Found match go to next fact
                            if found_match: continue

                        # Apply default if nothing else worked and it exists
                        if source.get("default"):
                            setattr(action, fact, source["default"])
                            break

                        # If there is no default simply get the fact
                        setattr(action, fact, facts.get(source["source"]))
                        break

                # Attribute is not set and is not optional
                if getattr(action, fact) == "" and not self.tasks[task][fact]["optional"]:
                    # If there is a default apply it
                    try:
                        # Simple default
                        if type(self.tasks[task][fact]["default"]) == str:
                            setattr(action, fact, self.tasks[task][fact]["default"])
                        
                        # Dicts provide more specific instructions to get the fact
                        elif type(self.tasks[task][fact]["default"]) == dict:
                            setattr(action, fact, self.get_previous_fact(self.tasks[task][fact]["default"]["prev_fact"]))

                    # If default doesn't exist show warning
                    except KeyError:
                        rospy.logwarn(f"Required fact '{fact}' could not be resolved and has no default")

            # We don't need to check other tasks if we already found a match
            break

        self.actions.append(action)

    def process_match_keyword_based(self, result):
        rospy.logwarn("Failed to match sentence, trying to resolve by keywords")
        task = ""

        for verb in [i for i in self.grammar_rules if i.startswith("vb")]:
            for word in self.grammar_rules[verb]:
                if word in result["sentence"]:
                    task = verb
                    break

            if task != "":
                break

        if self.tasks.get(task) != None:
            action = nlu_msg()

            # Get each fact
            for fact in self.tasks[task]:
                # Try every possible source
                for source in self.tasks[task][fact]["source"]:
                    # Simple source, try to find a keyword corresponding to the source
                    if type(source) == str:
                        # Get keywords from grammar rules (remove digits from source name)
                        keyword_list = self.grammar_rules.get(re.sub(r"\s\d", "", source))
                        if not keyword_list:
                            rospy.logwarn(f"{source} is not in grammar rules")
                            continue

                        for keyword in keyword_list:
                            if keyword in result["sentence"]:
                                setattr(action, fact, keyword)
                                break
                    
                    # Dicts provide specific instructions to get the fact
                    if type(source) == dict:
                        # Check if a keyword from the specified source is in facts, if not got to next source
                        source_in_sentence = False
                        keyword_list = self.grammar_rules.get(re.sub(r"\s\d", "", source["source"]))
                        if not keyword_list:
                            rospy.logwarn(f"{source['source']} is not in grammar rules")
                            continue
                        
                        for keyword in keyword_list:
                            if keyword in result["sentence"]:
                                source_in_sentence = True
                                break
                        if not source_in_sentence:
                            continue
                        
                        # If there is a previous fact specified, try to extract it from previous actions
                        if source.get("prev_fact"):
                            prev_fact = self.get_previous_fact(source["prev_fact"])

                            if prev_fact != "":
                                setattr(action, fact, prev_fact)
                                continue

                        # Apply default if nothing else worked and it exists
                        if source.get("default"):
                            setattr(action, fact, source["default"])
                            continue
                
                # Attribute is not set and is not optional
                if getattr(action, fact) == "" and not self.tasks[task][fact]["optional"]:
                    # If there is a default apply it
                    try:
                        # Simple default
                        if type(self.tasks[task][fact]["default"]) == str:
                            setattr(action, fact, self.tasks[task][fact]["default"])
                        
                        # Dicts provide more specific instructions to get the fact
                        elif type(self.tasks[task][fact]["default"]) == dict:
                            setattr(action, fact, self.get_previous_fact(self.tasks[task][fact]["default"]["prev_fact"]))

                    # If default doesn't exist show warning
                    except KeyError:
                        rospy.logwarn(f"Required fact '{fact}' could not be resolved for sentence '{result['sentence']}' and has no default")

            self.actions.append(action)
