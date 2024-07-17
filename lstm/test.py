import torch
from torch.utils.data import DataLoader
from torch import nn
from data import load_data, preprocess_data, CustomDataset
from models import LSTMModel

# 하이퍼파라미터 설정
input_size = 4
hidden_size = 50
num_layers = 2
output_size = 1

# 데이터 준비
X, y = load_data()
X, y = preprocess_data(X, y)
test_dataset = CustomDataset(X, y)
test_loader = DataLoader(test_dataset, batch_size=32, shuffle=False)

# 모델 초기화 및 로드
model = LSTMModel(input_size, hidden_size, num_layers, output_size)
model.load_state_dict(torch.load('trained_model.pth'))
print("Model loaded from 'trained_model.pth'")

# 손실 함수 초기화
criterion = nn.MSELoss()

# 모델 평가
model.eval()
total_loss = 0.0
with torch.no_grad():
    for X_batch, y_batch in test_loader:
        outputs = model(X_batch)
        loss = criterion(outputs, y_batch)
        total_loss += loss.item()

avg_loss = total_loss / len(test_loader)
print(f'Test Loss: {avg_loss:.4f}')
