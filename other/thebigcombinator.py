import sys
import csv

DESC = {
    5: ["FILE-VERSION","K","SEED","ILP AM1 SCORE","ILP AM2 SCORE","ILP AM3 SCORE","ILP AM1 TIME","ILP AM2 TIME","ILP AM3 TIME","GREEDY AM1 SCORE","GREEDY AM2 SCORE","GREEDY AM3 SCORE","GREEDY AM1 TIME","GREEDY AM2 TIME","GREEDY AM3 TIME","INTGRAPH AM1 SCORE","INTGRAPH AM2 SCORE","INTGRAPH AM3 SCORE","INTGRAPH AM1 TIME","INTGRAPH AM2 TIME","INTGRAPH AM3 TIME","ROTATIONAL CONFLICT TIME","ZOOMING CONFLICT TIME","PATH CREATION TIME","INTERPOLATION TIME","GRAPH AM1 TIME","GRAPH AM2 TIME","GRAPH AM3 TIME","NUMBER OF CONFLICTS","NUMBER OF VISIBILITIES","TRAJECTORY LENGTH","TRAJECTORY SIZE","VISIBILITIES LENGTH","CONFLICTS LENGTH","AM1 GRAPH NODES","AM1 GRAPH EDGES","AM2 GRAPH NODES","AM2 GRAPH EDGES","AM3 GRAPH NODES","AM3 GRAPH EDGES"],
    6: ["FILE-VERSION","K","SEED","ILP AM1 SCORE","ILP AM2 SCORE","ILP AM3 SCORE","ILP AM1 BOUND","ILP AM2 BOUND","ILP AM3 BOUND","ILP AM1 GAP","ILP AM2 GAP","ILP AM3 GAP","ILP AM1 TIME","ILP AM2 TIME","ILP AM3 TIME","GREEDY AM1 SCORE","GREEDY AM2 SCORE","GREEDY AM3 SCORE","GREEDY AM1 TIME","GREEDY AM2 TIME","GREEDY AM3 TIME","INTGRAPH AM1 SCORE","INTGRAPH AM2 SCORE","INTGRAPH AM3 SCORE","INTGRAPH AM1 TIME","INTGRAPH AM2 TIME","INTGRAPH AM3 TIME","ROTATIONAL CONFLICT TIME","ZOOMING CONFLICT TIME","PATH CREATION TIME","INTERPOLATION TIME","GRAPH AM1 TIME","GRAPH AM2 TIME","GRAPH AM3 TIME","NUMBER OF CONFLICTS","NUMBER OF VISIBILITIES","TRAJECTORY LENGTH","TRAJECTORY SIZE","VISIBILITIES LENGTH","CONFLICTS LENGTH","AM1 GRAPH NODES","AM1 GRAPH EDGES","AM2 GRAPH NODES","AM2 GRAPH EDGES","AM3 GRAPH NODES","AM3 GRAPH EDGES"]
}
LATEST = 6
UNAVAILABLE = -3

def read_in_version(version_id, row):
    version = DESC[version_id]
    assert(len(row) == len(version))
    assert(int(row[0]) == version_id)

    res = {}
    for pair in zip(version, row):
        res[pair[0]] = pair[1]

    for field in DESC[LATEST]:
        if field not in res:
            res[field] = UNAVAILABLE

    res['FILE-VERSION'] = LATEST

    return res


rows = []

def readfile(fname):
    with open(fname, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',', quotechar='"')

        for row in reader:
            if row[0] == "FILE-VERSION":
                pass
            elif (len(row) <= 3):
                pass # Incomplete row
            elif int(row[0]) in DESC:
                rows.append(read_in_version(int(row[0]), row))
            else:
                raise "Unknown version: " + row[0]

for i in range(2, len(sys.argv)):
    readfile(sys.argv[i])

with open(sys.argv[1], 'w') as csvfile:
    writer = csv.writer(csvfile, delimiter=',', quotechar='"')

    writer.writerow(DESC[LATEST])
    for entry in rows:
        row = [ entry[key] for key in DESC[LATEST] ]
        writer.writerow(row)
