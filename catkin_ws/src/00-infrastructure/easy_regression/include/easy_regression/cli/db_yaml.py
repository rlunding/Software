from duckietown_utils.system_cmd_imp import contract
from easy_regression.conditions.result_db import ResultDBEntry
from duckietown_utils.yaml_pretty import yaml_dump_pretty, yaml_load
from duckietown_utils.constants import get_duckietown_root
import os
from duckietown_utils.path_utils import get_ros_package_path


@contract(r=ResultDBEntry)
def yaml_from_rdbe(r):
    return yaml_dump_pretty(r._asdict())

def rdbe_from_yaml(s):
    return yaml_load(s)

def get_unique_filename(rt_name, rdbe):
    commit = rdbe.commit[-8:]    
    basename = rt_name + '_%s_%s_%s.rdbe.yaml' % (rdbe.date, rdbe.branch, commit)
    
    dr = get_ros_package_path('easy_regression')
    filename = os.path.join(dr, 'db', rt_name, basename)
    return filename
