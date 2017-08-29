from collections import namedtuple
from duckietown_utils.system_cmd_imp import contract

ResultDBEntry = namedtuple('ResultDBEntry',
                           ['date',
                            'host',
                            'cpu',
                            'user',
                            'results',
                            'branch',
                            'commit'])

class ResultDB():
    
    @contract(current=ResultDBEntry, entries='seq($ResultDBEntry)')
    def __init__(self, current, entries):
        self.entries = entries
        self.current = current
        
    def get_current_results(self):
        """ 
            Returns the current results.
            A dictionary of analyzer -> logs -> statistics
        """
        return self.current.results
        
    def get_at_date(self, date):
        pass