import torch
from torch.utils.data import DataLoader, TensorDataset
from imitation_learning.models import BCPolicy, train_BC_policy
from imitation_learning.utils.data_utils import load_data

import logging
import wandb

def default_float():
    return torch.float32

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    wandb.init(project="imitation_learning Cobot")

    # Load Data
    X_train, X_test, y_train, y_test = load_data("data/states_0.npy", "data/actions_0.npy")

    # Convert from numpy to torch tensors and make DataLoaders (batch training)
    train_dataset = TensorDataset(
        torch.tensor(X_train, dtype=default_float()),
        torch.tensor(y_train, dtype=default_float())
    )

    test_dataset = TensorDataset(
        torch.tensor(X_test, dtype=default_float()),
        torch.tensor(y_test, dtype=default_float())
    )

    logging.info(f"Train dataset size: {len(train_dataset)}")
    logging.info(f"Test dataset size: {len(test_dataset)}")

    train_loader = DataLoader(train_dataset, batch_size=64, shuffle=True)
    test_loader = DataLoader(test_dataset, batch_size=64, shuffle=True)

    # initialize policy
    state_dim = X_train.shape[1]
    action_dim = y_train.shape[1]
    policy = BCPolicy(state_dim, action_dim, hidden_dims=[256, 256, 256])

    # train policy
    trained_policy = train_BC_policy(
        policy, 
        train_loader, 
        test_loader, 
        epochs=100, 
        learning_rate=1e-3, 
        device='cuda' if torch.cuda.is_available() else "cpu"
    ) 
    
    # save trained policy
    torch.save(trained_policy.state_dict(), "trained_bc_policy.pth")  