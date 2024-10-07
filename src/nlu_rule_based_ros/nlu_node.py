import rospy
import rospkg

from nlu_rule_based.msg import nlu_msg
from google_speech_recognition.msg import ASRNBestList

from .rule_based_nlu import RuleBasedNLU

from os import listdir

class NLU_node():
    def __init__(self):
        rospy.init_node("NLU_node", 
                        # log_level=rospy.DEBUG
                        )

        # Initialize subscriber of transcript and publisher
        self.transcript_sub = rospy.Subscriber(rospy.get_param("~transcript_topic", "/mbot_speech_recognition/transcript"),
                                               ASRNBestList, self.transcript_callback, queue_size = 5)
        self.results_pub = rospy.Publisher(rospy.get_param("~result_topic", "~dialogue_acts"),
                                           nlu_msg, queue_size = 5)

        self.rate = rospy.Rate(2)

        # Initialize NLU
        self.nlu = RuleBasedNLU()

        # Get the path of the directory with the grammar files from parameter server
        grammar_path = rospy.get_param("~grammar_path")
        common_path = rospy.get_param("~common_path")
        rules_parent = rospy.get_param("~rules_parent")
        load_from_xml = rospy.get_param("~load_from_xml", True)
        
        # Get paths of grammar files
        file_path = [
            grammar_path + filename 
            for filename in listdir(grammar_path) 
            if filename.endswith(".txt") and filename != "Extra Verbs.txt"
            ]

        # Read grammar files with the information about sentence generation
        self.nlu.read_grammar_files(file_path, 
                           common_path + "Locations.xml", 
                           common_path + "Names.xml", 
                           common_path + "Objects.xml", 
                           common_path + "Gestures.xml",
                           grammar_path + "Extra Verbs.txt",
                           load_from_xml=load_from_xml)
        
        # Convert grammar files to regex patterns
        self.nlu.clean_rules()
        self.nlu.get_generation_rules(rules_parent)

        # Load task file with the information that can be extracted for each task
        self.nlu.load_tasks(grammar_path + "tasks.json")

        rospy.loginfo(f"Generated patterns and loaded tasks from {grammar_path}")

    def main(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

    def transcript_callback(self, msg):
        """Callback that is called when a new transcript is received"""

        # Process received most probable sentence
        actions = self.nlu.process_sentence(msg.hypothesis[0].transcript)

        # Warning if no action was extracted
        if len(actions) == 0: rospy.logwarn(f"No action extracted from sentence {msg.hypothesis[0].transcript}")

        # Publish every actions extracted from sentence
        for action in actions:
            self.results_pub.publish(action)

        # Publish end of speech message
        self.results_pub.publish(nlu_msg(verb="end_speech"))

def main():
    node = NLU_node()
    node.main()

if __name__ == "__main__":
    main()