from mpl_toolkits import mplot3d
import re
import os
import pickle
import numpy as np
import matplotlib.pyplot as plt
from os.path import expanduser
from sklearn.cluster import KMeans
from tqdm import tqdm
import matplotlib.cm as cm
import sklearn





'''

CODE THAT GENERATE X,Y PLOT OF TISNI EMBEDDINGS
FOR EACH PERSON WE HAVE k PLOTS EACH CORRESPONDING TO DIFF VAL OF PERPLEXITY (A HYPERPARAM OF TSNE)
A PERSON MAY HAVE SEVERAL DIFFERENT DATA FILES ASSOCIATED WITH DIFFERENT DATA GATHERING INSTANCES

image_folder - TSNE EMBEDDINGS FOLDER
image_folder_2 - ACTUAL READINGS/GATHERED DATA FOLDER

PLEASE DON'T DELETE THE COMMENTED OUT CODE AS IT PRESENTS DIFFERENT FUNCTIONALITY THAT I HAD TO USE PRIOR

'''


folder = os.path.dirname(os.path.abspath(__file__))
folder =  folder[:-len('scripts/analysis')] + 'data'

image_folder = folder
image_folder_2 = folder
X = []





def cond(img):
    if img.endswith(".pkl") and 'perp' in img and 'embedded' in img and 'md' in img:
        return True
    else:
        return False

fyles = [img for img in os.listdir(image_folder) if cond(img)]

def swap(x):
    if x != np.inf:
        return x
    else:
        return 12

# temp = []
# for i in files:
#     if 'kojo_1' in i or 'whit_2' in i or 'karim_2' in i or 'almas' in i:
#         temp.append(i)

# labels = []

# print(temp)
# pos = [0]
# for i, fyle in enumerate(tqdm(temp)):


#     with open(os.path.join(image_folder, fyle), 'rb') as f:
#         data = pickle.load(f)


#     labels.extend([i]*len(data))
#     pos.append(pos[-1] + len(data))

#     X.extend([[swap(z[1]) for z in sorted(x['ranges'].items(), key=lambda y: y[0])] for x in data])


# X = np.float32(X)

# print(labels)
# kmeans = KMeans(init='k-means++', n_clusters=4, n_init=10).fit(X)



# print(sklearn.metrics.homogeneity_score(labels, kmeans.labels_))

# klabels = [mapp[i] for i in kmeans.labels_]


# summ = 0

# for i, j in zip(labels, klabels):
#     if i == j:
#         summ += 1



# print('accuracy', summ/len(labels_))





# image_folder = expanduser('~') + '/Desktop/'

# colors = cm.rainbow(np.linspace(0, 1, 4))


# colors = ["r", "b", "g", 'purple']
# for i, perp in enumerate([2,30,50,80]):
#     with open(os.path.join(image_folder, 'beatles_embedded_perp' + str(perp) + '.pkl'), 'rb') as f:
#         data = pickle.load(f)


#     kmeans = KMeans(init='k-means++', n_clusters=4, n_init=10).fit(data)
#     print('perp ', perp,  sklearn.metrics.homogeneity_score(labels, kmeans.labels_))




#     # mintime = min([y['time'] for y in ddata])

#     # times = [y['time'] - mintime for y in ddata]

#     # with open(os.path.join(image_folder, 'md_karim_4embedded.pkl'), 'rb') as f:
#     #     data = pickle.load(f)



#     x = [y[0] for y in data]

#     y = [z[1] for z in data]

#     plt.figure(i)



#     for j,i in enumerate(data):
#         plt.scatter(i[0], i[1], c=colors[labels[j]])

#     plt.savefig('true_clustering_perp_' + str(perp) + '.png')




#     plt.figure(i)



#     for j,i in enumerate(data):
#         plt.scatter(i[0], i[1], c=colors[labels[j]])

#     plt.savefig('')



files = [img for img in os.listdir(image_folder_2) if img.endswith('.pickle')]
ddata = {}
names = ['kojo_1' ,  'whit_2' ,  'karim_2' ,  'md_almas' ,  'karim_4' ,  'whit_1' ,  'karim_3', 'kojo_2', 'kizito_2', 'karim_1']


for i in files:
    print(i, 'md_almas' in i)

    if any([x in i for x in names]):
        with open(os.path.join(image_folder_2, i), 'rb') as f:
            data = pickle.load(f)
        ddata[i[:-len('.pickle')]] = data


print(ddata.keys())


for i, fyle in tqdm(enumerate(fyles)):


    with open(os.path.join(image_folder, fyle), 'rb') as f:
        data = pickle.load(f)

    

    x = re.sub('_embedded_perp\d+\.pkl$', '', fyle)

    print(x)

    mintime = min([y['time'] for y in ddata[x]])

    times = [y['time'] - mintime for y in ddata[x]]

    plt.figure(i)
    x = [z[0] for z in data]
    y = [z[1] for z in data]

    plt.scatter(x,y, c=times, cmap='Greens')

    plt.savefig('robodog/clustering' + fyle[:-4]  +'.png')



