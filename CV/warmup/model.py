import torch

class SimpleModel(torch.nn.Module):
    
    def __init__(self, img_size = (28,28), label_num = 10):
        super().__init__()
        self.label_num = label_num
        
        ## Modify the code below ##

class SimpleModel(torch.nn.Module):
    def __init__(self, img_size=(28, 28), label_num=10):
        super().__init__()
        self.label_num = label_num

        self.layers = torch.nn.Sequential(
            # First Convolutional Block
            torch.nn.Conv2d(in_channels=1, out_channels=32, kernel_size=3, stride=1, padding=1),
            torch.nn.ReLU(),
            torch.nn.MaxPool2d(kernel_size=2, stride=2),
            # Second Convolutional Block
            torch.nn.Conv2d(in_channels=32, out_channels=64, kernel_size=3, stride=1, padding=1),
            torch.nn.ReLU(),
            torch.nn.MaxPool2d(kernel_size=2, stride=2),
            # Flattening the output for the Fully Connected Layer
            torch.nn.Flatten(),
            torch.nn.Linear(64 * 7 * 7, label_num),
            torch.nn.Softmax(dim=1)
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.layers(x)
