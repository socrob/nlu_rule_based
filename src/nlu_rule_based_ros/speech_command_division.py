from nltk.tokenize import word_tokenize

class CommandDivision(object):
    def __init__(self, verbs, polite_terms=[]):
        #Variables
        self.pronouns = ['him','her']
        self.pronouns_objects = ['it']
        self.information_subject = ['you can', 'you will', 'you may']

        # Load the verbs
        self.verbs = []
        self.special_verbs = []
        for verb in verbs:
            tokens = word_tokenize(verb.replace('(', '').replace(')', ''))
            
            if len(tokens)==1 and ' '.join(tokens).lower() not in self.verbs:
                self.verbs.append(' '.join(tokens).lower())

            elif len(tokens) > 1:
                special_verb=[]
                for token in tokens:
                    special_verb.append(token.lower())

                if special_verb not in self.special_verbs:
                    self.special_verbs.append(special_verb)

        # Load polite terms
        self.polite_terms = polite_terms
        self.politeness_in_use = None

    #Function that checks if a sentence begins with a predefined polite statement, removes that statement and returns the new sentence
    def polite_check(self, sentence):
        for politeness in self.polite_terms:

            if len(politeness) <= len(sentence.split(' ')) and ' '.join(politeness) == ' '.join(sentence.split(' ')[0:len(politeness)]):
                self.politeness_in_use = ' '.join(politeness)
                return ' '.join(sentence.split(' ')[len(politeness):len(sentence)])

        self.politeness_in_use = None
        return sentence

    #This function splits a sentence through its verbs. Returns a list of splitted sentences
    def split_by_verbs(self, sentence, reattach_politeness, keep_thens):
        sentence = self.polite_check(sentence)

        if sentence is None:
            return "error, sentence is None"

        words = word_tokenize(sentence)

        for verb in self.special_verbs:
            special_verb = ' '.join(verb)

            for j in range(0,len(words)-len(verb)+1):
                n_gram = ' '.join(words[j:(j+len(verb))])
                if n_gram == special_verb and j>0:
                    words[j] = "| " + words[j]
                    break

        i = 0
        for word in words:
            if word in self.verbs and i != 0 and i!=len(words)-1:
                if ' '.join(words[i-2:i]) in self.information_subject:
                    words[i-2] = "| " + words[i-2]
                else:
                    words[i] = "| " + word
            elif word == 'and' or word == 'then':
                words[i] = "|"
            i += 1

        if keep_thens:
            final_sentence = ' '.join(words).replace(' .','.').replace('| |','|').replace(' \'','\'').replace("then |", "| then")
        else:
            final_sentence = ' '.join(words).replace(' .','.').replace('| |','|').replace(' \'','\'').replace(" then ", " ")

        sentence_splitted = final_sentence.split(' | ')

        if reattach_politeness and self.politeness_in_use is not None:
            sentence_splitted[0] = self.politeness_in_use + ' ' + sentence_splitted[0]

        self.politeness_in_use = None

        return sentence_splitted
