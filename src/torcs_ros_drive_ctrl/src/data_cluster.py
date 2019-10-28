from sklearn.cluster import KMeans
from sklearn.preprocessing import MinMaxScaler
import numpy as np
import matplotlib.pyplot as plt
from logger import Logger
from minisom import MiniSom


def visualize(X, pred, index_to_show=None, centers=None):
    if index_to_show is None:
        index_to_show = [0, 1]
    K = max(pred) + 1
    for k in range(K):
        idx = [i for i, v in enumerate(pred) if v == k]
        c = [np.random.random() for i in range(3)]
        plt.plot(X[idx, index_to_show[0]], X[idx, index_to_show[1]], '.', color=c)
    if centers is not None:
        for center_batch in centers:
            for c in center_batch:
                plt.plot(c[0], c[1], 'x', markersize=12)
    plt.show()

def norm(v1, v2):
    s = 0.
    for x, y in zip(v1, v2):
        s += (x - y) ** 2
    return np.sqrt(s)


def remap_cluster_id(pred, from_idx, to_idx):
    for i in range(len(pred)):
        if pred[i] == from_idx:
            pred[i] = to_idx


def run_k_means(data, num_of_clusters):
    # Runs in parallel 4 CPUs
    kmeans = KMeans(n_clusters=num_of_clusters, n_jobs=4)
    # Train K-Means.
    y_pred_kmeans = kmeans.fit_predict(data)
    centers = kmeans.cluster_centers_
    return kmeans, y_pred_kmeans, centers


def test_on_simple_database():
    def create_database(centers, size=100, cov=None):
        if cov is None:
            cov = np.eye(len(centers[0]))
        X = np.zeros((size * len(centers), len(centers[0])))
        for k in range(len(centers)):
            X[(k * size):(k + 1) * size, :] = np.random.multivariate_normal(centers[k], cov, size)
        return X
    c1 = [1, 1]
    c2 = [3, 3]
    c3 = [1, 3]
    c4 = [3, 1]
    cs = [c1, c2, c3, c4]
    X = create_database(cs, size=100, cov=np.array([[0.2, 0], [0, 0.2]]))
    # plt.plot(X[0, :], X[1, :], '.')
    # plt.show()
    kmeans, y_pred_kmeans, centers = run_k_means(X, len(cs))
    print centers
    return kmeans, y_pred_kmeans, centers, X


def combine_clusters(centers, y_pred_kmeans, threshold=0.3):
    similar_clusters = []
    for i in range(len(centers)):
        for j in range(i + 1, len(centers)):
            if norm(centers[i], centers[j]) < threshold:
                similar_clusters.append((i, j))
    for pair in similar_clusters:
        print "Pair: {}: ({:.2f}, {:.2f}), {}: ({:.2f}, {:.2f})".format(pair[0], centers[pair[0]][0], centers[pair[0]][1], pair[1], centers[pair[1]][0], centers[pair[1]][1])
        remap_cluster_id(y_pred_kmeans, pair[0], pair[1])

def test_mini_som():
    def create_database(centers, size=100, cov=None):
        if cov is None:
            cov = np.eye(len(centers[0]))
        X = np.zeros((size * len(centers), len(centers[0])))
        for k in range(len(centers)):
            X[(k * size):(k + 1) * size, :] = np.random.multivariate_normal(centers[k], cov, size)
        return X
    som_x = 10
    som_y = 10
    max_it = 500
    sigma = 0.3
    learning_rate = 0.5
    X = create_database([[1, 3], [3, 1], [3, 3], [1, 1]], size=100, cov=np.array([[0.2, 0], [0, 0.2]]))
    import time
    now = time.time()
    som = MiniSom(x=som_x, y=som_y, input_len=2, sigma=sigma, learning_rate=learning_rate)
    som.random_weights_init(X)
    som.train_random(X, max_it)
    print 'Elapsed {}'.format(time.time() - now)
    res = np.zeros((len(X), 1))
    centers = som.get_weights()
    for i, x in enumerate(X):
        w = som.winner(x)

        res[i] = w[0] + w[1] * som_y
    return som, res, centers, X




if __name__ == '__main__':
    # kmeans, y_pred_kmeans, centers, X = test_on_simple_database()
    som, res, centers, X = test_mini_som()
    visualize(X, res, centers=centers)
    # logger = Logger(read_or_write='r')
    # data = logger.load()
    # data_np = np.array(data)
    # what_to_use = [3, -2, -5]  #7, 8, 9, 10, 11, 12, 13, 14]
    # X = data_np[:, what_to_use]
    # X_scaled = MinMaxScaler(feature_range=(0, 1)).fit_transform(X)
    # kmeans, y_pred_kmeans, centers = run_k_means(X_scaled, 10)
    # print centers
    # combine_clusters(centers, y_pred_kmeans, 0.2)
    #
    # visualize(data_np, y_pred_kmeans, index_to_show=[1, 2])