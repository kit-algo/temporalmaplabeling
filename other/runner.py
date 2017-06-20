import sys
import os
import threading
import subprocess
import time
from queue import Queue

CONFIG = {
    'K': [5, 10, -1],
    'MAP': [('/home/lukas/Downloads/maps/karlsruhe.osm', '/home/lukas/Downloads/maps/karlsruhe.pycgr')]
}

BINARY="/home/lukas/src/LabelRotation/LabelRotation"

GRAPH_OUT_DIR="/tmp/out/graphs"
INTERVAL_OUT_DIR="/tmp/out/intervals"
CSV_OUT_DIR="/tmp/out/csvs"
ERROR_OUT_DIR="/tmp/out/errors"

ILP_THREADS=1
PARALLELISM=2

PREFIX="instance"

q = Queue()

timings = []

def generate_filename(cfg):
    return PREFIX + "-" + os.path.basename(cfg['map'])[:-4] + "-seed_" + str(cfg['seed']) + "-k" + str(cfg["K"])

def worker(thread_id):
    global q
    global timings

    while True:
        cfg = q.get()
        if cfg is None:
            return

        fname = generate_filename(cfg)

        print("INFO:  Thread " + str(thread_id) + ": Starting " + fname)

        out_path = os.path.join(CSV_OUT_DIR, fname) + ".csv"
        graph_path = os.path.join(GRAPH_OUT_DIR, fname) + ".graphml"
        interval_path = os.path.join(INTERVAL_OUT_DIR, fname) + ".csv"

        start_time = time.perf_counter()
        success = True
        try:
            p = subprocess.check_output([BINARY, "--cli", "-f", "-s", str(cfg['seed']), "-m", str(cfg['map']), "-p", str(cfg['pmap']), "-t", str(ILP_THREADS), "-o", out_path, "-g", graph_path, "-v", interval_path, "-i", "1", "-k", str(cfg['K'])], stderr=subprocess.STDOUT, universal_newlines=True)
        except subprocess.CalledProcessError as e:
            p = e.output
            success = False

        end_time = time.perf_counter()

        print("INFO:  Thread " + str(thread_id) + ": Finished " + fname + " in " + str(end_time - start_time) + " seconds")
        timings.append(end_time - start_time)
        if len(timings) % 10 == 0:
            print("INFO:  -> Average computation time: " + str(sum(timings) / len(timings)))


        if not success:
            print("ERROR: Thread " + str(thread_id) + ": Failed on " + fname)
            with open(os.path.join(ERROR_OUT_DIR, fname + "-stdout.txt"), "w") as errorfile:
                errorfile.write(p)



with open(sys.argv[1], 'r') as seedfile:
    seeds = []
    for seed_str in seedfile.readlines():
        seeds.append(int(seed_str))

    while len(seeds) > 0:
        subseeds = seeds[:7]
        seeds = seeds[7:]

        for (mapfile, pmapfile) in CONFIG['MAP']:
            for k in CONFIG['K']:
                for seed in subseeds:
                    q.put({'K': k, 'map': mapfile, 'pmap': pmapfile, 'seed': seed})

threads = [ threading.Thread(target=worker, args=[i]) for i in range(PARALLELISM) ]
for thread in threads:
  thread.start()
  q.put(None)
