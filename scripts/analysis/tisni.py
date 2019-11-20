import os
import pickle
import numpy as np
from tsnecuda import TSNE
from tqdm import tqdm
from os.path import expanduser



## Code to create TSNE plot for 4 people 
## with varying perplexity
##PLEASE CHANGE THE data_folder which should contain data points for analysis


data_folder = expanduser('~') + '/dogrobo_data'


files = [img for img in os.listdir(data_folder) if img.endswith(".pickle")]

def swap(x):
    if x != np.inf:
        return x
    else:
        return 12



for fyle in tqdm(files):

    if 'kojo_1' in fyle or 'whit_2' in fyle or 'karim_2' in fyle or 'almas' in fyle or 'karim_4' in fyle:


        with open(os.path.join(data_folder, fyle), 'rb') as f:
            data = pickle.load(f)


        data = [[swap(z[1]) for z in sorted(x['ranges'].items(), key=lambda y: y[0])] for x in data]


        X = data

        print(X)
        
        X = np.float32(X)


        for perp in [2, 10, 20, 30, 50, 60, 80]:



            X_embedded = TSNE(n_components=2, perplexity=perp, learning_rate=10).fit_transform(X)


            with open('/home/almas_abdibayev/' + fyle[:-len('.pickle')] + '_embedded_perp' + str(perp) + '.pkl', 'wb') as f:
                pickle.dump(X_embedded, f)
