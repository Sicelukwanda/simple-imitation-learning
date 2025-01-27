import numpy as np
from sklearn.model_selection import train_test_split

def load_data(states_npy_path, actions_npy_path, test_size=0.2, random_state=123):
    """
    Load the states and actions from the given paths and split the data into training and testing sets.

    Parameters:
        states_npy_path (str): Path to the states numpy file.
        actions_npy_path (str): Path to the actions numpy file.
        test_size (float): Fraction of the data to reserve for testing.
        random_state (int): Random seed for reproducibility.

    Returns:
        tuple: (X_train, X_test, y_train, y_test) - Training and testing splits.
    """
    states = np.load(states_npy_path)
    actions = np.load(actions_npy_path)

    X_train, X_test, y_train, y_test = train_test_split(states, actions, test_size=test_size, random_state=random_state)

    return X_train, X_test, y_train, y_test