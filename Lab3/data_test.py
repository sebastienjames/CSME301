import numpy as np
import pandas as pd
import seaborn as sns


data = pd.read_csv("/Users/sebastienjames/Documents/CS301/Lab3/data.csv")
data.dropna(inplace=True)


def get_distance(goal, volt):
    newdata = data[["Distance (cm)", "Battery (volts)"]]
    newdata=(newdata-newdata.mean())/newdata.std()


    newdata["Distance (cm)"] -= goal
    newdata["Distance (cm)"] *= newdata["Distance (cm)"]
    newdata["Battery (volts)"] -= volt
    newdata["Battery (volts)"] *= newdata["Battery (volts)"]

    newdata["sum"] = newdata["Distance (cm)"] + newdata["Battery (volts)"]
    newdata["sum"] = np.sqrt(newdata["sum"])

    return newdata

data["weight"] = 1 / get_distance(25, 11)["sum"]
print(data)

sns.scatterplot(data=data, x="Distance (cm)", y="Battery (volts)")
