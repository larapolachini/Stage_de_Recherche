import pandas as pd
df = pd.read_feather("results/result.feather")
pd.set_option('display.max_rows', None)   # show all rows
pd.set_option('display.max_columns', None)  # if you also want every column
print(df)