import pandas as pd
df = pd.read_feather("results/result.feather")
print(df.columns)
print(df.head())
print(df)
