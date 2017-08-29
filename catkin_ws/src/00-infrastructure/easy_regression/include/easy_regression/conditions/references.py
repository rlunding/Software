import yaml

from duckietown_utils.exception_utils import raise_wrapped, check_is_in
from duckietown_utils.system_cmd_imp import contract
from duckietown_utils.text_utils import remove_prefix, string_split
from easy_regression.conditions.eval import Evaluable, EvaluationError
from easy_regression.conditions.interface import RTParseError


def parse_reference(s):
    """
        v:analyzer/log/statistic~master@date
        
    """
    prefix = 'v:'
    
    if s.startswith(prefix):
        s = remove_prefix(s, prefix)
        
        if '@' in s:
            s, date_spec = string_split(s, '@')
            if not date_spec:
                msg = 'Invalid date spec %r.' % date_spec 
                raise RTParseError(msg)
            date = parse_date_spec(date_spec)
        else:
            date = None
            
        if '~' in s:
            s, branch_spec = string_split(s, '~')
            if not branch_spec:
                msg = 'Invalid branch spec %r.' % branch_spec 
                raise RTParseError(msg)
        else:
            branch_spec = None
            
        tokens = s.split('/')
        if not len(tokens) >= 3:
            msg = 'Expected "analyzer/log/statistic"'
            raise RTParseError(msg)
        
        analyzer = tokens[0]
        log = tokens[1]
        statistic = tuple(tokens[2:]) 
            
        return StatisticReference(analyzer=analyzer, log=log, statistic=statistic, branch=branch_spec, date=date)
    
    try:
        c = yaml.load(s)
        return Constant(c)
    except yaml.YAMLError:
        msg = 'Could not parse reference %s.' % s.__repr__()
        raise RTParseError(msg)

def parse_date_spec(d):
    from dateutil.parser import parse
    try:
        return parse(d)
    except ValueError as e:
        msg = 'Cannot parse date %s.' % d.__repr__()
        raise_wrapped(RTParseError, e, msg, compact=True)

class StatisticReference(Evaluable):
    
    @contract(statistic='seq(str)')
    def __init__(self, analyzer, log, statistic, branch, date):
        self.analyzer = analyzer
        self.log = log
        self.statistic = statistic
        self.branch = branch
        self.date = date
    
    def __str__(self):
        return ('StatisticReference(%s,%s,%s,%s,%s)' % 
                (self.analyzer, self.log, self.statistic, self.branch, self.date))
        
    def eval(self, rdb):
        if self.branch is None and self.date is None:
            results = rdb.get_current_results()
            
            check_is_in('analyzer', self.analyzer, results, EvaluationError)
            logs = results[self.analyzer]
            check_is_in('log', self.log, logs, EvaluationError)
            forlog = logs[self.log]
            val = eval_name(forlog, self.statistic)
            return val
        else:
            raise NotImplementedError()

@contract(name_tuple=tuple)
def eval_name(x, name_tuple):
    if not name_tuple:
        return x
    else:
        first = name_tuple[0]
        rest = name_tuple[1:]
        check_is_in('value', first, x, EvaluationError)
        xx = x[first]
        return eval_name(xx, rest)
    
class Constant(Evaluable):
    def __init__(self, x):
        self.x = x
    def eval(self, _test_results):
        return self.x
    def __repr__(self):
        return 'Constant(%s)' % self.x.__repr__()
    
