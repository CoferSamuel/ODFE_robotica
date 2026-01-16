import torch
import torch.nn as nn
import torchvision
import torchvision.transforms as transforms
import torch.nn.functional as F
from train_mnist import Net

def test():
    # Transformations
    transform = transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize((0.1307,), (0.3081,))
    ])

    # Load dataset
    testset = torchvision.datasets.MNIST(root='./data', train=False,
                                       download=True, transform=transform)
    testloader = torch.utils.data.DataLoader(testset, batch_size=64,
                                           shuffle=False)

    # Load model
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = Net().to(device)
    try:
        model.load_state_dict(torch.load('my_network.pt'))
    except FileNotFoundError:
        print("Error: my_network.pt not found. Run train_mnist.py first.")
        return

    model.eval()

    correct = 0
    total = 0
    with torch.no_grad():
        for data in testloader:
            images, labels = data
            images, labels = images.to(device), labels.to(device)
            outputs = model(images)
            _, predicted = torch.max(outputs.data, 1)
            total += labels.size(0)
            correct += (predicted == labels).sum().item()

    accuracy = 100 * correct / total
    print(f'Accuracy of the network on the 10000 test images: {accuracy:.2f}%')

    if accuracy > 95:
        print("SUCCESS: Accuracy is over 95%")
    else:
        print("FAILURE: Accuracy is under 95%")

if __name__ == "__main__":
    test()
