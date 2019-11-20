import torch
import torch.nn as nn
import torch.nn.functional as F
import os
import pickle
import numpy as np

from torch import optim
from os.path import expanduser



'''

CODE THAT TRAINS ONE LAYER LINEAR (NO ACTIVACTION FUNCTION) RNN, BASICALLY A LINEAR REGRESSION
WITH RECURRENCE

THE WEIGHTS OF THE NETWORK AND THE HIDDEN STATE WILL BE USED FOR ANALYSIS (SPECTRAL ANALYSIS ETC)


IN PROGRESS 

'''


class Net(nn.Module):
    def __init__(self, input_size, hidden_size, output_size):
        super(Net, self).__init__()
        self.hidden_size = hidden_size

        self.fc = nn.Linear(input_size + hidden_size, 60)



    def forward(self, x, hidden):

        combined = torch.cat((x, hidden), 1)

        x = self.fc(combined)

        return x, hidden


    def initHidden(self):
        return torch.zeros(1, self.hidden_size)





folder = os.path.dirname(os.path.abspath(__file__))
folder =  folder[:-len('scripts/analysis')] + 'data'

image_folder = folder + '/data/'

files = [img for img in os.listdir(image_folder) if img.endswith(".pickle")]

print(files)
name = ""
for i in files:
    if 'karim_2' in i:
        name = i
        break


with open(os.path.join(image_folder, name), 'rb') as f:
    data = pickle.load(f)

def swap(x):
    if x != np.inf:
        return x
    else:
        return 12




data = [(x['ranges'], x['time']) for x in data]



data = sorted(data, key=lambda x:x[1])

data = [[swap(z[1]) for z in sorted(x[0].items(), key=lambda y: y[0])] for x in data]



X = np.float32(data)

X = torch.from_numpy(X)
learning_rate  = 0.0005

rnn = Net(60, 30, 60)

optimizer = optim.SGD(rnn.parameters(), lr=learning_rate, momentum=0.9)
# create a loss function
criterion = nn.MSELoss()



print_every = 10

n_iters = 10000


run_loss = []

for iter in range(0, n_iters):

    hidden = rnn.initHidden()


    for i in range(0, X.size()[0]-3, 3):

        for j in range(3):
            output, hidden = rnn(torch.unsqueeze(X[i + j, :],0), hidden)

        loss = criterion(output, X[i+j+1,:])
        loss.backward()

        # Add parameters' gradients to their values, multiplied by learning rate
        for p in rnn.parameters():
            p.data.add_(-learning_rate, p.grad.data)
        # optimizer.step()

        rnn.zero_grad()


        run_loss.append(float(loss))
        # Print iter number, loss, name and guess
        if iter % print_every == 0:
            print('%.4f ' % (loss))


        if sum(run_loss[-5:]) < 0.1:
            break


    if sum(run_loss[-5:]) < 0.1:
        for i,p in enumerate(rnn.parameters()):
            with open(folder + '/data/weight_karim_2_' + str(i) + '.pkl', 'wb') as f:
                pickle.dump(p.data.numpy(), f)





