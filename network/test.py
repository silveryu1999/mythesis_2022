import numpy as np

f = open("./result.txt", 'a', encoding='utf-8')

for i in range(10):
	f.write(str(i) + ' ' + str(i+1) + '\n')
	
f.close()

a = np.loadtxt("./result.txt", dtype=int, delimiter = ' ')
print(type(a))
print(a[:,0])
