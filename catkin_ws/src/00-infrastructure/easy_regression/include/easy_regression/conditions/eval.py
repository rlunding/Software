from abc import ABCMeta, abstractmethod
from contextlib import contextmanager

from duckietown_utils.exception_utils import raise_wrapped
from duckietown_utils.instantiate_utils import indent
from duckietown_utils.system_cmd_imp import contract

from .interface import RTCheck
from .result_db import ResultDB


class EvaluationError(Exception):
    pass

class DataNotFound(Exception):
    pass

class Evaluable():
    __metaclass__ = ABCMeta
    
    @abstractmethod
    @contract(rdb=ResultDB)
    def eval(self, rdb):
        """ Raise EvaluationError or DataNotFound """


class Wrapper(RTCheck):
    
    def __init__(self, evaluable):
        self.evaluable = evaluable
        
    # @contract(returns=CheckResult, result_db=ResultDB)
    def check(self, rdb):
        """ 
            Returns a CheckResult, or raises
            RegressionTestCheckException
            if an abnormal situation is encountered.
        """
        
        try: 
            res = self.evaluable.eval(rdb)
            if not isinstance(res, bool):
                msg = 'Invalid non-boolean result obtained: %s' % res.__repr__()
                return RTCheck.CheckResult(
                   status=RTCheck.ABNORMAL,
                   summary='Invalid test',
                   details=msg) 
            if res == True:
                return RTCheck.CheckResult(
                   status=RTCheck.OK,
                   summary='OK',
                   details='')
            if res == False:
                return RTCheck.CheckResult(
                   status=RTCheck.FAIL,
                   summary='Failed',
                   details='')
            
        except DataNotFound as e:
            return RTCheck.CheckResult(
               status=RTCheck.FAIL,
               summary='No data available',
               details=str(e))
        except EvaluationError as e:
            return RTCheck.CheckResult(
               status=RTCheck.ABNORMAL,
               summary='Invalid test',
               details=str(e))
        
    
#      
#     FAIL = 'failed'
#     OK = 'ok'
#     WARN = 'caution'
#     NODATA = 'missing' # the historical data is not there yet
#     ABNORMAL = 'abnormal' # Other error in the evaluation
#     
#     CHECK_RESULTS = [OK, WARN, FAIL, NODATA, ABNORMAL]
#     
#     CheckResult = namedtuple('CheckResult',
#                              ['status', # One of the above in CHECK_RESULTS 
#                               'summary', # A short string
#                               'details', # A long description
#                               ])
class BinaryEval(Evaluable):
    
    @contract(a=Evaluable, b=Evaluable)
    def __init__(self, a, op, b):
        self.a = a
        self.op = op
        self.b = b
        
    def eval(self, test_results):
        
        @contextmanager
        def r(m):
            try:
                yield
            except EvaluationError as e:
                msg = 'Cannot evaluate binary operation: error during %s' % m
                msg += '\n' + str(self)
                raise_wrapped(EvaluationError, e, msg, compact=True)
                
        with r('first operator evaluation'):
            a = self.a.eval(test_results)
        
        with r('second operator evaluation'):
            b = self.b.eval(test_results)
        
        with r('binary evaluation'):
            return self.op(a,b)
    
    def __str__(self):
        s = "Binary operation"
        s += '\n' + indent(self.a, '', '   a: ')
        s += '\n' + indent(self.op, '', '  op: ')
        s += '\n' + indent(self.b, '', '   b: ')
        return s
    
    