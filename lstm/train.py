import torch
from torch.utils.data import DataLoader
from torch import nn, optim
from data import load_data, preprocess_data, CustomDataset
from models import LSTMModel

# 하이퍼파라미터 설정
input_size = 4
hidden_size = 50
num_layers = 2
output_size = 1
num_epochs = 100
learning_rate = 0.001

# 데이터 준비
X, y = load_data()
X, y = preprocess_data(X, y)
train_dataset = CustomDataset(X, y)
train_loader = DataLoader(train_dataset, batch_size=32, shuffle=True)

# 모델, 손실 함수 및 옵티마이저 초기화
model = LSTMModel(input_size, hidden_size, num_layers, output_size)
criterion = nn.MSELoss()
optimizer = optim.Adam(model.parameters(), lr=learning_rate)

# 학습 루프
for epoch in range(num_epochs):
    model.train()
    running_loss = 0.0
    for X_batch, y_batch in train_loader:
        outputs = model(X_batch)
        loss = criterion(outputs, y_batch)
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        running_loss += loss.item()

    if (epoch+1) % 10 == 0:
        avg_loss = running_loss / len(train_loader)
        print(f'Epoch [{epoch+1}/{num_epochs}], Loss: {avg_loss:.4f}')

# 모델 저장
torch.save(model.state_dict(), 'trained_model.pth')
print("Model saved to 'trained_model.pth'")
