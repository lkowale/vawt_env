import pandas as pd
import numpy as np

df = pd.DataFrame({'Date': [0, 1, -2, 0, 1, 2], 'A': [10, 11, -21, 10, 21, 21], 'val': [0, 1, np.nan, 3, 4, -5]})
print(df)
# select row as a Series
a = df.iloc[2]
# return Series of Trues and False
b = np.argmax(a)
print(b)

pass
