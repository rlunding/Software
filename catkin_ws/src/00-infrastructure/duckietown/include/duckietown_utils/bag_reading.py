import time

from duckietown_utils import logger
from duckietown_utils.exceptions import DTBadData


def d8n_bag_read_with_progress(bag, topic):
    bag_t0 = bag.get_start_time()
    bag_t1 = bag.get_end_time()
    length = bag_t1 - bag_t0  
    n = 0
    msg_time0 = None
    # write a message every once in a while 
    INTERVAL = 1
    first = last = time.time()
    for topic, msg, msg_time in bag.read_messages(topics=[topic]):
        # compute progess
        msg_time = msg_time.to_sec()
        if msg_time0 is None:
            msg_time0 = msg_time
        progress = float(msg_time-msg_time0) / length
        
        # current time
        n += 1 
        t = time.time()
        if t - last > INTERVAL:
            last =  t
            fps =  n / (t-first) 
            logger.debug('%6d  %4.1f%%  %5.1f fps' % (n, progress * 100, fps))
        yield msg
    if n == 0:
        s = 'Could not find any message for topic %r.' % topic
        raise DTBadData(s)
    
    fps = n / (time.time() - first)
    logger.debug('Read %d messages for %s. Processing time: %.1f fps.' % (n, topic, fps))
    bag.close()
    
