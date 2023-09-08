import jax
import jax.numpy as jnp
import flax
import flax.linen as nn
import optax

from sklearn.metrics import r2_score
from sklearn.model_selection import train_test_split
from typing import Sequence, Tuple, NamedTuple, Any, Dict
import matplotlib.pyplot as plt
import numpy as np


# Loading trajectories

states, actions = jnp.load("data/states.npy"), jnp.load("data/actions.npy")
print("States shape:", states.shape, "Acions shape: ", actions.shape)

# Train Val Test data splitting

X_train, X_test, Y_train, Y_test = train_test_split(states, actions, train_size=0.8, random_state=42)
X_train, X_val, Y_train, Y_val = train_test_split(X_train, Y_train, train_size=0.75, random_state=42) 
X_train, X_val, X_test, Y_train, Y_val, Y_test = jnp.array(X_train, dtype=jnp.float32),\
                                                jnp.array(X_val, dtype=jnp.float32),\
                                                jnp.array(X_test, dtype=jnp.float32),\
                                                jnp.array(Y_train, dtype=jnp.float32),\
                                                jnp.array(Y_val, dtype=jnp.float32),\
                                                jnp.array(Y_test, dtype=jnp.float32)

samples, state_n = X_train.shape
samples, action_n = Y_train.shape
print("States (Train, Val, Test)", X_train.shape, X_val.shape, X_test.shape)
print("Actions (Train, Val, Test)", Y_train.shape, Y_val.shape, Y_test.shape)

# Normalizing data

mean = X_train.mean(axis=0)
std = X_train.std(axis=0)
X_train = (X_train - mean) / std
X_val = (X_val - mean) / std
X_test = (X_test - mean) / std

# Neural Network

class MultiLayerPerceptronRegressor(nn.Module):
    features: Sequence[int] = (state_n,32,32,action_n)

    def setup(self):
        self.layers = [nn.Dense(feat) for feat in self.features]

    def __call__(self, inputs):
        x = inputs
        for i, lyr in enumerate(self.layers):
            x = lyr(x)
            if i != len(self.layers) - 1:
                x = nn.relu(x)
        return x

nn_architecture = (state_n,256,256,action_n)
model = MultiLayerPerceptronRegressor(nn_architecture)

seed = jax.random.PRNGKey(0)
params = model.init(seed, X_train[:5])
for layer_params in params["params"].items():
    print("Layer Name : {}".format(layer_params[0]))
    weights, biases = layer_params[1]["kernel"], layer_params[1]["bias"]
    print("\tLayer Weights : {}, Biases : {}".format(weights.shape, biases.shape))

# Model prediction

preds = model.apply(params, X_train[:5])
print("Current model predictions")
print("states", X_train[:5])
print("predicted actions", preds)

# Loss function

def MeanSquaredErrorLoss(weights, input_data, actual):
    preds = model.apply(weights, input_data)
    return jnp.power(actual - preds.squeeze(), 2).mean()

# Training

epochs=1000

model = MultiLayerPerceptronRegressor() ## Define Model
random_arr = jax.random.normal(key=seed, shape=(5, state_n))
params = model.init(seed, random_arr) ## Initialize Model Parameters

optimizer = optax.sgd(learning_rate=1/1e3) ## Initialize SGD Optimizer using OPTAX

optimizer_state = optimizer.init(params)
loss_grad = jax.value_and_grad(MeanSquaredErrorLoss)

loss_train = []
loss_val = []
for i in range(1,epochs+1):
    loss, gradients = loss_grad(params, X_train, Y_train) ## Calculate Loss and Gradients
    loss_train.append(loss)
    loss_val.append(MeanSquaredErrorLoss(params, X_val, Y_val))
    updates, optimizer_state = optimizer.update(gradients, optimizer_state)
    params = optax.apply_updates(params, updates) ## Update weights
    if i % 100 == 0:
        print('MSE After {} Epochs : {:.2f}'.format(i, jnp.mean(jnp.array(loss_val[-100:-1]))))


# Plot of training and validation loss

plt.plot(loss_train, label='Training Loss')
plt.plot(loss_val, label='Validation Loss')
plt.xlabel('Iteration')
plt.ylabel('Loss')
plt.title('Training and Validation Loss')
plt.legend()
plt.show()

# Evaluating

train_preds = model.apply(params, X_train) ## Make Predictions on train dataset
train_preds = train_preds.ravel()
loss_val, gradients = loss_grad(params, X_train, Y_train)
print("Train Loss", loss_val)
test_preds = model.apply(params, X_test) ## Make Predictions on test dataset
test_preds = test_preds.ravel()
loss_val, gradients = loss_grad(params, X_test, Y_test)
print("Test Loss", loss_val)
