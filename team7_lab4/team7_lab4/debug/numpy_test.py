import numpy as np
x  = np.array([[-90,0],[0,90]])
y = x.copy()
neg_indexes = np.where(y<0)
y[neg_indexes] += 360
# print(y)
# print(x)

def get_indicies_of_interest( num_ranges,min_ang,increment):
            angle_of_interest = np.array([[-90,-0.001],[0,90]])
            neg_indexes = np.where(angle_of_interest<0)
            angle_of_interest[neg_indexes] += 360
            indicies_of_interest = (angle_of_interest-min_ang)/increment
            return indicies_of_interest

inds = get_indicies_of_interest(360,0,1)
print(inds)