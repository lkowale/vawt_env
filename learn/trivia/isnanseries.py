import pandas as pd
import numpy as np

df = pd.DataFrame({'Date': [0, 1, 2, 0, 1, 2], 'Name': ['A', 'B', 'C', 'A', 'B', 'C'], 'val': [0, 1, np.nan, 3, 4, 5]})
# select row as a Series
a = df.iloc[2]
# return Series of Trues and False
b = a.isnull()
# count how many Trues are there
b_sum = b.sum()
# return Series of elements of a only where b is True
c = a[b]

pass
