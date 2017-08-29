from contracts.utils import check_isinstance

from easy_regression.conditions.interface import RTParseError

from .values import parse_float
from duckietown_utils.text_utils import remove_prefix_suffix, remove_suffix
from easy_regression.conditions.eval import EvaluationError,\
    ResultWithDescription
from duckietown_utils.exception_utils import raise_wrapped

def parse_binary(s):
    """
        Syntax:
        
            ==
            >=
            >
            <=
            <
            
            ==[10%]
            !=
            
            contains
        
        Returns an object that can be called with two arguments 
        and returns True or False, or TypeError.
        
        Raise RTParseError on error.
    """
    
    simple = {
        '>': gt,
        '>=': gte,
        '<=': lte, 
        '<': lt,
        '==': eq,
        '!=': neq, 
        'contains': contains,
    }
    if s in simple:
        return simple[s]
    
    try:
        inside = remove_prefix_suffix(s, '==[', ']')
    except ValueError:
        pass
    else:        
        if inside.endswith('%'):
            val = remove_suffix(inside, '%')
            percentage = parse_float(val)
            if percentage < 0:
                msg = 'Invalid percentage %s' % percentage
                raise RTParseError(msg)
            ratio = percentage / 100.0
            return CompareApproxRelative(ratio)
        else:
            msg = 'Cannot parse "%s": expected "%%" in "%s".' % (s, inside)
            raise RTParseError(msg)
    
    msg = 'Cannot parse string "%s".' % s
    raise RTParseError(msg)


def float_op(f):
    class X():
        def __call__(self, a, b):
            try:
                expect_float(a)
                expect_float(b)
                val =  f(a, b)
                desc = '%s %s %s' % (a, f.__name__, b)
                return ResultWithDescription(val, desc)
            except EvaluationError as e:
                msg = 'While evaluating %s(%s, %s)' % (f.__name__, a, b)
                raise_wrapped(EvaluationError, e, msg, compact=True)
        def __repr__(self):
            return f.__name__
    X.__name__ = f.__name__
    return X()

def contains(a, b):
    return b in a

@float_op
def gt(a, b):
    return a > b

@float_op
def lt(a, b):
    return a < b
    
@float_op
def lte(a, b):
    return a <= b

@float_op
def gte(a, b):
    return a >= b

@float_op
def eq(a, b):
    return a == b

@float_op
def neq(a, b):
    return a != b

def expect_float(x):
    if not isinstance(x, (float,int)):
        msg = 'Expected a number, got %s.' % x.__repr__() 
        raise EvaluationError(msg)
    
class CompareApproxRelative():
    
    def __init__(self, rel_error):
        check_isinstance(rel_error, float)
        self.rel_error = rel_error
    
    def __call__(self, a, b):
        delta = self.rel_error * a
        lb = a-delta
        ub = a+delta
        res = lb <= b <= ub
        desc = 'Condition checked:\n   %s <= %s <= %s.' % (lb,b,ub)
        return ResultWithDescription(res, desc)
    
    def __str__(self):
        return 'EqualUpTo(%g%%)' % (100 * self.rel_error)
    
    