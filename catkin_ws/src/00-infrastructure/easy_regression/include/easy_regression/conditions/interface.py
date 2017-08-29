from collections import namedtuple
from duckietown_utils.exceptions import DTException, DTConfigException
from abc import abstractmethod, ABCMeta
from duckietown_utils.system_cmd_imp import contract
from easy_regression.conditions.result_db import ResultDB
 
class RTParseError(DTConfigException):
    """ Cannot parse condition """


class RTCheck():
    __metaclass__ = ABCMeta
    
    FAIL = 'failed'
    OK = 'ok'
    WARN = 'caution'
    NODATA = 'missing' # the historical data is not there yet
    ABNORMAL = 'abnormal' # Other error in the evaluation
    
    CHECK_RESULTS = [OK, WARN, FAIL, NODATA, ABNORMAL]
    
    CheckResult = namedtuple('CheckResult',
                             ['status', # One of the above in CHECK_RESULTS 
                              'summary', # A short string
                              'details', # A long description
                              ])

    @abstractmethod
    @contract(returns=CheckResult, result_db=ResultDB)
    def check(self, result_db):
        """ 
            Returns a CheckResult.
        """

    @staticmethod
    def from_string(line):
        """
            Returns a RTCheck object.
            
            Syntaxes allowed:
            
            Simple checks:
            
                v:analyzer/log/statistics == value
                v:analyzer/log/statistics >= value
                v:analyzer/log/statistics <= value
                v:analyzer/log/statistics < value
                v:analyzer/log/statistics > value
    
            Check that it is in 10% of the value:
                
                v:analyzer/log/statistics ==[10%] value
            
            Use `@date` to reference the last value:
            
                v:analyzer/log/statistics ==[10%] v:analyzer/log/statistic@date  

            Use `#branch@date` to reference the value of a branch at a certain date
            
                v:analyzer/log/statistics ==[10%] v:analyzer/log/statistic#branch@date  
                
            Other checks:
            
                v:analyzer/log/statistics contains ![log name]  
                
            Raises DTConfigException if the syntax is not valid.
    
        """
        from .implementation import _parse_regression_test_check
        return _parse_regression_test_check(line)
