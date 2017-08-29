from duckietown_utils.instantiate_utils import indent
from duckietown_utils.system_cmd_imp import contract
from easy_regression.conditions.interface import CheckResult, RTCheck
from easy_regression.conditions.result_db import ResultDBEntry, ResultDB


def make_entry(results_all):
    current = ResultDBEntry(date='',
                           host='',
                           cpu='',
                           user='',
                           results=results_all,
                           branch='',
                           commit='')
    return current 

def compute_check_results(rt, results_all):
    current = make_entry(results_all)
    rdb = ResultDB(current=current, entries=[])
    
    res = []
    for cwc in rt.get_checks():
        for check in cwc.checks:
            r = check.check(rdb)
            assert isinstance(r, CheckResult)
            res.append(r)
    return res
        
def display_check_results(results, out):
    s = ""
    for i, r in enumerate(results):
        s += '\n' + indent(str(r), '', '%d of %d: ' % (i+1, len(results)))
    
    print(s)
    
@contract(results='list($CheckResult)')
def fail_if_not_expected(results, expect):
    statuses = [r.status for r in results]
    summary = summarize_statuses(statuses)
    if summary != expect:
        msg = 'Expected status %r, but got %r.' % (expect, summary)
        for i, r in enumerate(results):
            msg += '\n' + indent(str(r), '', '%d of %d: ' % (i+1, len(results)))
        raise Exception(msg)
    
def summarize_statuses(codes):
    # abnormal + ... = abnormal
    # fail + .. = fail
    # notfound + ... = notfound
    for c in codes:
        assert c in RTCheck.CHECK_RESULTS
    if RTCheck.ABNORMAL in codes:
        return RTCheck.ABNORMAL
    if RTCheck.FAIL in codes:
        return RTCheck.FAIL
    if RTCheck.NODATA in codes:
        return RTCheck.NODATA
    for c in codes:
        assert c == RTCheck.OK
    return RTCheck.OK
        
        