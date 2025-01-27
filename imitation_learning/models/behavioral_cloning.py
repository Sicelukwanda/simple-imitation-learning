import torch
import torch.nn as nn
from torch.utils.data import DataLoader

from typing import List
import logging

class Block(nn.Module):
    def __init__(self, in_dim: int, out_dim: int):
        """
        Initialize the Block module.
        Args:
            in_dim (int): The input dimension.
            out_dim (int): The output dimension.
        """
        super(Block, self).__init__()
        # wrap these in an nn.Sequential block?
        self.linear = nn.Linear(in_dim, out_dim)
        self.relu = nn.ReLU()
    
    
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        Forward pass through the Block module.
        Args:
            x (torch.Tensor): The input tensor.
        Returns:
            torch.Tensor: The output tensor.
        """
        x = self.linear(x)
        x = self.relu(x)
        return x

class BCPolicy(nn.Module):
    def __init__(self, state_dim: int, action_dim: int, hidden_dims: List[int]):
        """
        Initialize the policy network.
        
        Args:
            state_dim (int): The dimension of the state space.
            action_dim (int): The dimension of the action space.
            hidden_dims (List[int]): The dimensions of the hidden layers.
        """
        super(BCPolicy, self).__init__()

        self.linear_blocks = nn.ModuleList()

        for i, dim in enumerate(hidden_dims):
            if i == 0:
                self.linear_blocks.append(Block(state_dim, dim))
            else:
                self.linear_blocks.append(Block(hidden_dims[i-1], dim))
            
        self.output_layer = nn.Linear(hidden_dims[-1], action_dim)

    def forward(self, state: torch.Tensor) -> torch.Tensor:
        """
        Forward pass through the policy network.
        Args:
            state (torch.Tensor): The input state tensor.
        Returns:
            torch.Tensor: The output action tensor.
        """
        x = state
        for block in self.linear_blocks:
            x = block(x)
        x = self.output_layer(x)
        return x
    

def train_BC_policy(
        policy_model: nn.Module, 
        train_loader: DataLoader, 
        test_loader: DataLoader, 
        epochs: int=100, 
        learning_rate: float=1e-3, 
        device: str='cpu'
        ) -> nn.Module:
    """
    Train BC policy.

    Args:
        policy (nn.Module): The policy network.
        train_loader (DataLoader): The training data loader.
        test_loader (DataLoader): The test data loader.
        epochs (int): The number of epochs to train for.
        learning_rate (float): The learning rate for the optimizer.
        device (str): The device to train on. Default is 'cpu'.
    
    Returns:
        nn.Module: The trained policy network.
    """

    policy_model.to(device)
    optimizer = torch.optim.Adam(policy_model.parameters(), lr=learning_rate)
    MSE_loss_fn = nn.MSELoss()

    for epoch in range(epochs):
        policy_model.train()
        train_loss = 0.0

        for states, actions in train_loader:
            states, actions = states.to(device), actions.to(device)

            optimizer.zero_grad()
            predictions = policy_model(states)

            loss = MSE_loss_fn(predictions, actions)
            loss.backward()
            optimizer.step()
            train_loss += loss.item()

        # evaluate on test set
        test_loss = 0.0
        with torch.no_grad():
            for states, actions in test_loader:  # use test_loader
                states, actions = states.to(device), actions.to(device)
                predictions = policy_model(states)
                loss = MSE_loss_fn(predictions, actions)
                test_loss += loss.item()
        
        logging.info(f"Epoch {epoch+1}/{epochs}, Train Loss: {train_loss/len(train_loader):.4f}, Test Loss: {test_loss/len(test_loader):.4f}")
    
    return policy_model

