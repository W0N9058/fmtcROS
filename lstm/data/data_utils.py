import numpy as np
import torch
from torch.utils.data import Dataset

def load_data():
    """
    더미 데이터를 생성합니다.
    :return: X, y (각각 입력 데이터와 타겟 데이터)
    """
    X = np.random.randn(1000, 16, 4)  # 1000개의 샘플, 각각 16개의 슬라이드, 각 슬라이드에 4개의 입력 데이터
    y = np.random.randn(1000, 1)      # 1000개의 타겟 값
    return X, y

def preprocess_data(X, y):
    """
    데이터 전처리를 수행합니다.
    :param X: 입력 데이터
    :param y: 타겟 데이터
    :return: 전처리된 데이터 (X, y)
    """
    X = (X - np.mean(X, axis=0)) / np.std(X, axis=0)
    return X, y

class CustomDataset(Dataset):
    def __init__(self, X, y):
        self.X = torch.tensor(X, dtype=torch.float32)
        self.y = torch.tensor(y, dtype=torch.float32)

    def __len__(self):
        return len(self.X)

    def __getitem__(self, idx):
        return self.X[idx], self.y[idx]
