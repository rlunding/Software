from comptests.registrar import run_module_tests, comptest

from duckietown_utils.instantiate_utils import indent
from easy_regression.conditions.interface import RTCheck
from easy_regression.conditions.result_db import ResultDB, ResultDBEntry


def get_test_db():
    results = {'analyzer': {
    'log1': {
        'value2': 2,
        'value3': 3,
        }
    }} 
    current = ResultDBEntry(date='',
                           host='',
                           cpu='',
                           user='',
                           results=results,
                           branch='',
                           commit='')
    old = ResultDBEntry(date='',
                       host='',
                       cpu='',
                       user='',
                       results=results,
                       branch='',
                       commit='commit-id')
    rdb = ResultDB(current=current, entries=[])
    return rdb

def raise_error(rdb, t, res, s):
    msg = s 
    msg += '\n Obtained: %s' % res.__repr__()
    msg += '\n' + indent(str(t), '', 'test: ')
    msg += '\n' + indent(str(rdb),'','rdb: ')
    raise Exception(msg)

@comptest
def test_true():
    conditions_true = [
        'v:analyzer/log1/value2 == 2',
        'v:analyzer/log1/value2 > 1',
        'v:analyzer/log1/value2 < 3',
        'v:analyzer/log1/value2 <= 2',
    ]
    rdb = get_test_db()
    for ct in conditions_true:
        t = RTCheck.from_string(ct)
        res = t.check(rdb)
        if not res.status == RTCheck.OK:
            raise_error(rdb, t, res, 'Expected OK')

@comptest
def test_false():
    conditions_false = [
        'v:analyzer/log1/value2 == 1',
        'v:analyzer/log1/value2 > 2',
        'v:analyzer/log1/value2 < 2',
        'v:analyzer/log1/value2 <= 1',
    ]   
    rdb = get_test_db()
    
    for ct in conditions_false:
        t = RTCheck.from_string(ct)
        res = t.check(rdb)
        if not res.status == RTCheck.FAIL:
            raise_error(rdb, t, res, 'Expected FAIL')

@comptest
def test_eval_error():
    conditions_evaluation_error = [
        'v:analyzer/log1/not_exist <= 1',
        'v:analyzer/not_exist/value2 <= 1',
        'v:not_exist/log1/value2 <= 1',
    ]
    rdb = get_test_db()
    for ct in conditions_evaluation_error:
        t = RTCheck.from_string(ct)
        
        res = t.check(rdb)
        if not res.status == RTCheck.ABNORMAL:
            raise_error(rdb, t, res, 'Expected ABNORMAL') 

@comptest
def test_data_not_found():
    conditions_data_not_found = [
        'v:analyzer/log1/value2@2016-01-12 <= 1',
        'v:analyzer/log1/value2~branchname <= 1',
    ]
    rdb = get_test_db()
    for ct in conditions_data_not_found:
        t = RTCheck.from_string(ct)
        
        res = t.check(rdb)
        if not res.status == RTCheck.NODATA:
            raise_error(t, res, 'Expected NODATA') 

if __name__ == '__main__':
    run_module_tests()
