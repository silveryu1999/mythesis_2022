import numpy as np
import matplotlib.pyplot as plt

x = np.array([3, 6, 10])

avg_f1_detect_3 = (1.000000 + 0.999698 + 0.999654 + 0.999341 + 1.000000 + 1.000000 + 0.999670) / 7
avg_f1_track_3 = (0.694589 + 0.767686 + 0.737624 + 0.759282 + 0.787352 + 0.784576 + 0.720399) / 7
avg_f1_detect_6 = (0.603888 + 0.717483 + 0.734125 + 0.776028 + 0.774304 + 0.746455 + 0.713002) / 7
avg_f1_track_6 = (0.519112 + 0.643237 + 0.640575 + 0.611319 + 0.673510 + 0.653776 + 0.588522) / 7
avg_f1_detect_10 = (0.356927 + 0.529664 + 0.576406 + 0.501761 + 0.626032 + 0.558140 + 0.488589) / 7
avg_f1_track_10 = (0.376656 + 0.529664 + 0.571603 + 0.494795 + 0.559201 + 0.544963 + 0.474345) / 7

plt.plot(x, np.array([avg_f1_detect_3, avg_f1_detect_6, avg_f1_detect_10]), label='detect')
plt.plot(x, np.array([avg_f1_track_3, avg_f1_track_6, avg_f1_track_10]), label='track')
plt.xlabel("camera fps")
plt.ylabel("f1 score")

plt.legend()
plt.title("Constant interval, no network")
plt.savefig('performance_f1_score.png')
plt.show()

avg_frame_delay_detect_3 = (0 + 0 + 0 + 0.004464 + 0.002500 + 0 + 0.001887) / 7
avg_frame_delay_track_3 = (1.496732 + 1.501160 + 1.498856 + 1.501124 + 1.499374 + 1.500000 + 1.500473) / 7
avg_frame_delay_detect_6 = (1.012987 + 0.995392 + 1.004566 + 0.995536 + 1.000000 + 1.000000 + 1.000000) / 7
avg_frame_delay_track_6 = (2.500000 + 2.498840 + 2.500000 + 2.498876 + 2.500000 + 2.500000 + 2.500000) / 7
avg_frame_delay_detect_10 = (2.064935 + 2.032258 + 2.063927 + 2.044643 + 2.040000 + 2.074627 + 2.069811) / 7
avg_frame_delay_track_10 = (3.523490 + 3.509346 + 3.535797 + 3.518100 + 3.519497 + 3.530075 + 3.534156) / 7

plt.plot(x, np.array([avg_frame_delay_detect_3, avg_frame_delay_detect_6, avg_frame_delay_detect_10]), label='detect')
plt.plot(x, np.array([avg_frame_delay_track_3, avg_frame_delay_track_6, avg_frame_delay_track_10]), label='track')
plt.xlabel("camera fps")
plt.ylabel("frame delay")

plt.legend()
plt.title("Constant interval, no network")
plt.savefig('performance_frame_delay.png')
plt.show()
