#!/usr/bin/env python3
import csv
import math
import statistics
from collections import defaultdict
import numpy as np
import matplotlib.pyplot as plt

infile = "a4_measurements.csv"
rows = []

with open(infile, "r") as fh:
    rdr = csv.DictReader(fh)
    for r in rdr:
        try:
            d_meas = float(r["d_meas_m"])
            th_meas = float(r["theta_meas_rad"])
            d_err = float(r["d_err_m"])
            th_err = float(r["theta_err_rad"])
            rows.append((d_meas, th_meas, d_err, th_err))
        except:
            pass

print(f"Loaded {len(rows)} rows from {infile}.")


dist_bin_size = 0.1   # 1 meter bins

def dist_bin(d):
    return int(d // dist_bin_size)

bins = defaultdict(list)

for d, th, de, the in rows:
    b = dist_bin(d)
    bins[b].append((d, th, de, the))

def var(lst):
    return statistics.pvariance(lst) if len(lst) > 1 else float("nan")

table = []
for b in sorted(bins.keys()):
    de_list  = [x[2] for x in bins[b]]
    the_list = [x[3] for x in bins[b]]
    d_center = (b + 0.5) * dist_bin_size

    sd2  = var(de_list)
    sth2 = var(the_list)

    table.append((b, d_center, sd2, sth2, len(de_list)))

with open("a4_variance_by_distance.csv", "w", newline="") as fh:
    wr = csv.writer(fh)
    wr.writerow(["bin_index", "dist_center_m", "var_d_m2", "var_theta_rad2", "count"])
    wr.writerows(table)

print("Wrote a4_variance_by_distance.csv")

D = np.array([r[1] for r in table if not math.isnan(r[2]) and not math.isnan(r[3])])
Vd = np.array([r[2] for r in table if not math.isnan(r[2]) and not math.isnan(r[3])])
Vt = np.array([r[3] for r in table if not math.isnan(r[2]) and not math.isnan(r[3])])

X = np.column_stack([np.ones_like(D), D**2])

a0, a1 = np.linalg.lstsq(X, Vd, rcond=None)[0]
b0, b1 = np.linalg.lstsq(X, Vt, rcond=None)[0]

print("\nFITTED MODELS:")
print(f"sigma_d^2(d)     = {a0:.4e} + {a1:.4e} * d^2")
print(f"sigma_theta^2(d) = {b0:.4e} + {b1:.4e} * d^2")

with open("a4_model_params.txt", "w") as fh:
    fh.write(f"sigma_d^2(d)     = {a0} + {a1} * d^2\n")
    fh.write(f"sigma_theta^2(d) = {b0} + {b1} * d^2\n")


# ----------------------------
# Plot (1) Distance variance
# ----------------------------
plt.figure(figsize=(10,5))
plt.title("Distance Measurement Variance vs Distance")
plt.scatter(D, Vd, color="blue", label="Measured Variance", s=40)
d_plot = np.linspace(min(D), max(D), 200)
plt.plot(d_plot, a0 + a1*(d_plot**2), "r-", linewidth=2, label="Model: a0 + a1 d^2")
plt.xlabel("Distance (m)")
plt.ylabel("Variance of distance (m²)")
plt.grid(True)
plt.legend()


# ----------------------------
# Plot (2) Bearing variance
# ----------------------------
plt.figure(figsize=(10,5))
plt.title("Bearing Measurement Variance vs Distance")
plt.scatter(D, Vt, color="green", label="Measured Variance", s=40)
plt.plot(d_plot, b0 + b1*(d_plot**2), "r-", linewidth=2, label="Model: b0 + b1 d^2")
plt.xlabel("Distance (m)")
plt.ylabel("Variance of bearing (rad²)")
plt.grid(True)
plt.legend()


# ----------------------------
# Plot (3) Actual error scatter (optional)
# ----------------------------
d_all  = np.array([r[0] for r in rows])
de_all = np.array([r[2] for r in rows])
te_all = np.array([r[3] for r in rows])

plt.figure(figsize=(10,5))
plt.title("Raw Distance Error vs Distance")
plt.scatter(d_all, de_all, s=5, alpha=0.4)
plt.xlabel("Distance (m)")
plt.ylabel("Distance Error (m)")
plt.grid(True)

plt.figure(figsize=(10,5))
plt.title("Raw Bearing Error vs Distance")
plt.scatter(d_all, te_all, s=5, alpha=0.4)
plt.xlabel("Distance (m)")
plt.ylabel("Bearing Error (rad)")
plt.grid(True)


plt.show()
