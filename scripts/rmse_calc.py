#! /bin/env python3

import csv
import math

csv_fh = open("/home/vivek/ws/NYU/ARPL/RAL2021/data/simple_kalman_data_RDp1.csv")
csv_reader = csv.reader(csv_fh)

err_x1 = 0.0
err_x2 = 0.0

err = 0.001

i = 0

for row in csv_reader:
    i += 1
    print(i)
    print(row[0])
    m1x = float(row[0])
    m1y = float(row[1])
    m2x = float(row[2])
    m2y = float(row[3])
    t1x = float(row[4])
    t1y = float(row[5])
    t2x = float(row[6])
    t2y = float(row[7])

    if(not m1x == 0):
        errA1x = math.fabs(m1x - t1x)
        errA2x = math.fabs(m1x - t2x)
    else:
        errA1x = 1000.0
        errA2x = 1000.0
    if(not m2x == 0):
        errB1x = math.fabs(m2x - t1x)
        errB2x = math.fabs(m2x - t2x)
    else:
        errABx = 1000.0
        errABx = 1000.0

    
    

    if(errA1x > errB1x):
        if(errB1x < 700):
            err_x1 += (errB1x*errB1x)
    else:
        if(errA1x < 700):
            err_x1 += (errA1x*errA1x)
    
    if(errA2x > errB2x):
        if(errB2x < 700):
            err_x2 += (errB2x*errB2x)
    else:
        if(errA2x < 700):
            err_x2 += (errA2x*errA2x)

print(math.sqrt(err_x1/i))
print(math.sqrt(err_x2/i))
    

