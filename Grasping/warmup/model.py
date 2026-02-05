import torch
import torch.nn as nn
import torch.nn.functional as F

# class SimpleModel(torch.nn.Module):
    
#     def __init__(self, img_size = (28,28), label_num = 10):
#         super().__init__()
#         self.label_num = label_num
        
#         ## Modify the code below ##

#         self.layers = torch.nn.Sequential(
#             torch.nn.Flatten(),
#             torch.nn.Linear(img_size[0]*img_size[1], label_num),
#             torch.nn.Softmax(dim=1)
#         )


#     def forward(self, x: torch.Tensor) -> torch.Tensor:
#         return self.layers(x)



class SimpleModel(torch.nn.Module):
    def __init__(self, img_size=(28, 28), label_num=10):
        super().__init__()
        self.label_num = label_num
        
        # Convolutional layers
        self.conv1 = nn.Conv2d(1, 32, kernel_size=3, stride=1, padding=1)  # Assuming input is grayscale
        self.conv2 = nn.Conv2d(32, 64, kernel_size=3, stride=1, padding=1)
        
        # Pooling layer
        self.pool = nn.MaxPool2d(kernel_size=2, stride=2)
        
        # Calculate the size of the flattened features after the convolutional and pooling layers
        # This depends on the input size and the architecture of your network
        # For a 28x28 input and the layers defined above, the size will be 7*7*64 after two conv and pool operations
        feature_size = 64 * (img_size[0] // 4) * (img_size[1] // 4)  # Divide by 4 because of 2 pooling layers
        
        # Fully connected layers
        self.fc1 = nn.Linear(feature_size, 128)
        self.fc2 = nn.Linear(128, label_num)
        
    def forward(self, x):
        # Convolutional layers with ReLU and pooling
        x = self.pool(F.relu(self.conv1(x)))
        x = self.pool(F.relu(self.conv2(x)))
        
        # Flattening the output for the fully connected layer
        x = x.view(-1, self.num_flat_features(x))
        
        # Fully connected layers with ReLU activation for the first one
        x = F.relu(self.fc1(x))
        x = self.fc2(x)
        
        # Apply softmax to the output
        x = F.softmax(x, dim=1)

        return x
    
    def num_flat_features(self, x):
        size = x.size()[1:]  # all dimensions except the batch dimension
        num_features = 1
        for s in size:
            num_features *= s
        return num_features
